'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');
const perception=require('../sensor-perception.js');
const cameraGeometry=require('../camera-geometry.js');

function bootSimulator(options={}){
 const root=path.join(__dirname,'..');
 const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
 const source=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 const values={};
 for(const match of html.matchAll(/<([a-z][a-z0-9-]*)\b[^>]*\bid="([^"]+)"[^>]*>/gi)){
  const value=match[1].toLowerCase()==='input'?match[0].match(/\bvalue="([^"]*)"/):null;
  values[match[2]]=value?value[1]:'';
 }
 values.model='hybrid';
 const elements={};
 const frames=[];
 let drawCalls=0;
 const ctx={};
 for(const name of ['beginPath','moveTo','lineTo','setLineDash','stroke','closePath','fill','fillText','arc','clearRect','fillRect']){
  ctx[name]=()=>{if(name==='clearRect')drawCalls++;};
 }
 ctx.save=()=>{};
 ctx.restore=()=>{};
 const makeElement=(id,value='')=>({
  id,value,textContent:'',style:{},onclick:null,oninput:null,onchange:null,
  children:[],appendChild(child){this.children.push(child);return child;}
 });
 const getElementById=id=>{
  if(!elements[id])elements[id]=makeElement(id,values[id]??'');
  return elements[id];
 };
 for(const id of Object.keys(values))getElementById(id);
 elements.gatePositionUncertainty.value='0.05';
 elements.gateHeadingUncertaintyDeg.value='1';
 elements.gatePostWidthUncertainty.value='0.02';
 elements.gateDepthUncertainty.value='0.03';
 const canvas=getElementById('view');
 canvas.width=1000;
 canvas.height=650;
 canvas.getContext=()=>ctx;
 const windowObject={TruckVisionPhysics:physics,TruckVisionPlanner:planner,TruckVisionPerception:perception,addEventListener:()=>{}};
 const sandbox={
  window:windowObject,
  document:{getElementById,createElement:tag=>makeElement(tag,'')},
  requestAnimationFrame:callback=>frames.push(callback),
  setTimeout:()=>1,
  clearTimeout:()=>{},
  console
 };
 if(options.workerResult){
  sandbox.Worker=class TestWorker{
   constructor(url){this.url=url;this.terminated=false;}
   postMessage(message){
    Promise.resolve().then(()=>{
     if(!this.terminated&&typeof this.onmessage==='function'){
      this.onmessage({data:{id:message.id,type:'result',result:options.workerResult}});
     }
    });
   }
   terminate(){this.terminated=true;}
  };
 }
 vm.runInNewContext(source,sandbox,{filename:'truckvision.js'});
 return {elements,frames,windowObject,get drawCalls(){return drawCalls;}};
}

test('planner goal pose checks wrapped heading and trailer articulation',()=>{
 const q={x:0.2,y:-0.1,psi:-179*Math.PI/180,psiT:171*Math.PI/180};
 const goal={x:0,y:0,heading:179*Math.PI/180};
 const errors=planner.goalErrors(q,goal);
 assert.ok(Math.abs(errors.headingError-2*Math.PI/180)<1e-10);
 assert.ok(Math.abs(errors.articulation-10*Math.PI/180)<1e-10);
 assert.equal(planner.goalReached(q,goal,{
  positionTolerance:0.5,
  headingTolerance:5*Math.PI/180,
  maxTerminalArticulation:12*Math.PI/180
 }),true);
});

test('planner rejects a positional match with incorrect terminal course or trailer alignment',()=>{
 const goal={x:0,y:0,heading:0};
 const tolerances={positionTolerance:0.5,headingTolerance:10*Math.PI/180,maxTerminalArticulation:10*Math.PI/180};
 assert.equal(planner.goalReached({x:0.1,y:0,psi:30*Math.PI/180,psiT:30*Math.PI/180},goal,tolerances),false);
 assert.equal(planner.goalReached({x:0.1,y:0,psi:0,psiT:-20*Math.PI/180},goal,tolerances),false);
});

test('live perception frame enters the scene and triggers whole-vehicle collision warning',()=>{
 const {elements,windowObject}=bootSimulator();
 const result=windowObject.TruckVisionSetPerceptionFrame({
  frameId:'smoke-perception-001',
  timestamp:Date.now(),
  coordinateFrame:'vehicle-local',
  vehiclePose:{x:0,y:0,heading:0},
  objects:[{
   id:'obstacle-at-tractor',
   type:'pallet',
   pose:{x:0,y:0,heading:0},
   dimensions:{length:2,width:2,height:1},
   confidence:0.99,
   source:'fused'
  }]
 });
 assert.equal(result.ok,true);
 assert.equal(result.accepted,1);
 assert.match(elements.perceptionStatus.textContent,/smoke-perception-001/);
 assert.match(elements.assistantAdvice.textContent,/пересекаются с препятствием/);
 assert.match(elements.assistantAdvice.textContent,/pallet · obstacle-at-tractor/);
});

test('simulator starts and exposes route controls',()=>{
 const {elements}=bootSimulator();
 assert.equal(typeof elements.planRoute.onclick,'function');
 assert.equal(elements.goalHeading.value,'0');
 assert.equal(elements.goalHeadingTolerance.value,'12');
 assert.equal(elements.goalArticulationLimit.value,'12');
 assert.equal(typeof elements.routeStart.onclick,'function');
 assert.equal(typeof elements.routePause.onclick,'function');
 assert.equal(typeof elements.routeConfirmGear.onclick,'function');
 assert.equal(typeof elements.routeStop.onclick,'function');
 const html=fs.readFileSync(path.join(__dirname,'..','index.html'),'utf8');
 assert.match(html,/<button id="routeConfirmGear"[^>]*hidden[^>]*disabled/);
});

test('simulator exposes measured gate geometry and invalidates a route when gate obstacles are enabled',()=>{
 const {elements}=bootSimulator();
 assert.equal(elements.gateEnabled.checked,undefined);
 assert.equal(elements.gateCenterX.value,'12');
 assert.equal(elements.gateCenterY.value,'0');
 assert.equal(elements.gateHeadingDeg.value,'90');
 assert.equal(typeof elements.gateEnabled.onchange,'function');
 elements.gateEnabled.checked=true;
 elements.gateEnabled.onchange();
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
});

test('gate jamb uncertainty controls advertise positive minima',()=>{
 const html=fs.readFileSync(path.join(__dirname,'..','index.html'),'utf8');
 for(const id of ['gatePostWidthUncertainty','gateDepthUncertainty']){
  const input=html.match(new RegExp('<input[^>]*id="'+id+'"[^>]*>'));
  assert.ok(input,'missing input '+id);
  assert.match(input[0],/min="0\.001"/,'input '+id+' must reject zero uncertainty');
 }
});

test('route planning is blocked when any enabled gate dimension or pose bound is zero',()=>{
 for(const field of ['gateWidthUncertainty','gatePostWidthUncertainty','gateDepthUncertainty','gatePositionUncertainty','gateHeadingUncertaintyDeg']){
  const {elements}=bootSimulator();
  elements.gateEnabled.checked=true;
  elements.gateWidthUncertainty.value='0.08';
  elements.gatePostWidthUncertainty.value='0.02';
  elements.gateDepthUncertainty.value='0.03';
  elements.gatePositionUncertainty.value='0.05';
  elements.gateHeadingUncertaintyDeg.value='1';
  elements[field].value='0';
  elements.planRoute.onclick();
  assert.ok(elements.routeAdvice.textContent.includes(field+': при включённых воротах задайте положительную границу неопределённости'));
  assert.match(elements.difficultyAdvice.textContent,/Сложность не рассчитана/);
 }
});

test('camera gate registration rejects zero jamb dimension or pose uncertainty atomically',()=>{
 const {elements,windowObject}=bootSimulator();
 const fields=['gatePostWidthUncertainty','gateDepthUncertainty','gatePositionUncertainty','gateHeadingUncertaintyDeg'];
 const before=[elements.gateWidth.value,elements.gateWidthUncertainty.value,elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value,elements.gatePostWidthUncertainty.value,elements.gateDepthUncertainty.value];
 for(const field of fields){
  elements[field].value='0';
  assert.equal(windowObject.TruckVisionApplyGateRegistration({centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:0.08}),false,field+' must reject registration');
  assert.deepEqual([elements.gateWidth.value,elements.gateWidthUncertainty.value,elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value,elements.gatePostWidthUncertainty.value,elements.gateDepthUncertainty.value],before);
  elements[field].value=field==='gatePostWidthUncertainty'?'0.02':field==='gateDepthUncertainty'?'0.03':field==='gatePositionUncertainty'?'0.05':'1';
 }
});

test('route planning is blocked when an enabled gate lintel lacks alignment confirmation',()=>{
 const {elements}=bootSimulator();
 elements.gateEnabled.checked=true;
 elements.gateWidthUncertainty.value='0.08';
 elements.gatePositionUncertainty.value='0.05';
 elements.gateHeadingUncertaintyDeg.value='1';
 elements.gateLintelEnabled.checked=true;
 elements.gateLintelAlignmentConfirmed.checked=false;
 elements.planRoute.onclick();
 assert.match(elements.routeAdvice.textContent,/gateLintelAlignmentConfirmed: независимо подтвердите центрирование, параллельность и горизонтальность/);
});

test('route planning is blocked when an enabled lintel has any zero uncertainty bound',()=>{
 const fields=['gateLintelCameraHeightUncertainty','gateLintelFloorUncertainty','gateLintelThicknessUncertainty','gateLintelSpanUncertainty','gateLintelDepthUncertainty'];
 for(const field of fields){
  const {elements}=bootSimulator();
  elements.gateEnabled.checked=true;
  elements.gateWidthUncertainty.value='0.08';
  elements.gatePositionUncertainty.value='0.05';
  elements.gateHeadingUncertaintyDeg.value='1';
  elements.gateLintelEnabled.checked=true;
  elements.gateLintelAlignmentConfirmed.checked=true;
  for(const key of fields)elements[key].value='0.02';
  elements[field].value='0';
  elements.planRoute.onclick();
  assert.ok(elements.routeAdvice.textContent.includes(field+': для верхней перемычки задайте положительную границу неопределённости'));
 }
});

test('route planning is blocked for physically invalid lintel geometry even with positive bounds',()=>{
 const {elements}=bootSimulator();
 elements.gateEnabled.checked=true;
 elements.gateWidthUncertainty.value='0.08';
 elements.gatePositionUncertainty.value='0.05';
 elements.gateHeadingUncertaintyDeg.value='1';
 elements.gateLintelEnabled.checked=true;
 elements.gateLintelAlignmentConfirmed.checked=true;
 elements.gateLintelClearHeight.value='0.4';
 elements.gateLintelCameraHeightUncertainty.value='0.02';
 elements.gateLintelFloorUncertainty.value='0.02';
 elements.gateLintelThickness.value='0.25';
 elements.gateLintelThicknessUncertainty.value='0.01';
 elements.gateLintelSpan.value='4.6';
 elements.gateLintelSpanUncertainty.value='0.1';
 elements.gateLintelDepth.value='0.45';
 elements.gateLintelDepthUncertainty.value='0.02';
 elements.planRoute.onclick();
 assert.match(elements.routeAdvice.textContent,/gate: Для верхней перемычки нужны измеренные размеры/);
});

test('automatic route remains blocked when obstacle checks are disabled',()=>{
 const {elements}=bootSimulator();
 elements.obstaclesToggle.onclick();
 elements.planRoute.onclick();
 assert.match(elements.difficultyAdvice.textContent,/проверка препятствий отключена/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд заблокирован/);
});

test('route planning reports missing Web Worker support without blanking the simulator',async()=>{
 const {elements}=bootSimulator();
 elements.planRoute.onclick();
 await new Promise(resolve=>setImmediate(resolve));
 assert.match(elements.routeAdvice.textContent,/не поддерживает фоновый расчёт маршрута/);
 assert.equal(elements.planRoute.disabled,false);
 assert.equal(typeof elements.routeStart.onclick,'function');
});


test('canvas keeps rendering after a route-planning worker error',async()=>{
 const simulator=bootSimulator();
 const before=simulator.drawCalls;
 simulator.elements.planRoute.onclick();
 await new Promise(resolve=>setImmediate(resolve));
 assert.match(simulator.elements.routeAdvice.textContent,/не поддерживает фоновый расчёт маршрута/);
 assert.ok(simulator.drawCalls>before,'route planning should redraw the existing simulator scene');
 const frame=simulator.frames.shift();
 assert.equal(typeof frame,'function','the animation frame should remain scheduled');
 const afterPlanning=simulator.drawCalls;
 frame(16);
 assert.ok(simulator.drawCalls>afterPlanning,'the animation loop should continue drawing after the worker error');
 assert.ok(simulator.frames.length>0,'the next animation frame should be scheduled');
});

test('invalid model settings fail closed without replacing the last valid geometry',()=>{
 const {elements}=bootSimulator();
 elements.wheelbase.value='0';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/Параметры не применены/);
 assert.match(elements.routeAdvice.textContent,/wheelbase/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд заблокирован/);
});

test('changing a valid model setting invalidates the old route before playback',()=>{
 const {elements}=bootSimulator();
 elements.wheelbase.value='4.00';
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/параметры изменились/);
 assert.match(elements.difficultyAdvice.textContent,/прежняя оценка маршрута недействительна/);
});

test('negative measurement uncertainty blocks route playback instead of weakening clearance checks',()=>{
 const {elements}=bootSimulator();
 elements.measurementUncertainty.value='-0.01';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/measurementUncertainty/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд заблокирован/);
});

test('kinematic uncertainty settings include Ackermann, axle tracks and tire dimensions',()=>{
 const {elements}=bootSimulator();
 const ids=[
  'trailerAxleSpacingUncertainty','trailerAxleSpacing12Uncertainty','trailerAxleSpacing23Uncertainty',
  'trailerAxle1TrackUncertainty','trailerAxle2TrackUncertainty','trailerAxle3TrackUncertainty',
  'tractorSteeringTrackUncertainty','tractorFrontTrackUncertainty','tractorRearTrackUncertainty',
  'trailerTrackUncertainty','tractorRearTireGroupWidthUncertainty','trailerTireGroupWidthUncertainty',
  'tireLengthUncertainty','tireWidthUncertainty',
  'cabOffsetUncertainty','cabFrontUncertainty','cabRearUncertainty',
  'cabWidthUncertainty','mirrorOutboardUncertainty',
  'mirrorLongitudinalUncertainty','mirrorLengthUncertainty'
 ];
 for(const id of ids)assert.equal(elements[id].value,'0',id+' should start as not-varied');
 elements.trailerAxleSpacing.value='1.3';
 elements.trailerAxleSpacing12.value='1.2';
 elements.trailerAxleSpacing23.value='1.4';
 elements.trailerAxle1Track.value='2.7';
 elements.trailerAxle2Track.value='2.8';
 elements.trailerAxle3Track.value='2.85';
 elements.trailerAxleSpacingUncertainty.value='0.02';
 elements.trailerAxleSpacing12Uncertainty.value='0.02';
 elements.trailerAxleSpacing23Uncertainty.value='0.02';
 elements.trailerAxle1TrackUncertainty.value='0.02';
 elements.trailerAxle2TrackUncertainty.value='0.02';
 elements.trailerAxle3TrackUncertainty.value='0.02';
 elements.tractorSteeringTrackUncertainty.value='0.03';
 elements.tractorFrontTrackUncertainty.value='0.02';
 elements.tractorRearTrackUncertainty.value='0.02';
 elements.trailerTrackUncertainty.value='0.03';
 elements.tractorRearTireGroupWidth.value='0.72';
 elements.trailerTireGroupWidth.value='0.68';
 elements.tractorRearTireGroupWidthUncertainty.value='0.02';
 elements.trailerTireGroupWidthUncertainty.value='0.02';
 elements.tireLengthUncertainty.value='0.02';
 elements.tireWidthUncertainty.value='0.01';
 elements.cabOffsetUncertainty.value='0.05';
 elements.cabFrontUncertainty.value='0.05';
 elements.cabRearUncertainty.value='0.05';
 elements.cabWidthUncertainty.value='0.05';
 elements.mirrorOutboardUncertainty.value='0.15';
 elements.mirrorLongitudinalUncertainty.value='0.10';
 elements.mirrorLengthUncertainty.value='0.10';
 elements.apply.onclick();
 assert.doesNotMatch(elements.routeAdvice.textContent,/Параметры не применены/);
 const source=fs.readFileSync(path.join(__dirname,'..','truckvision.js'),'utf8');
 for(const mapping of [
  /trailerAxleSpacingUncertainty:'trailerAxleSpacing'/,
  /trailerAxleSpacing12Uncertainty:'trailerAxleSpacing12'/,
  /trailerAxleSpacing23Uncertainty:'trailerAxleSpacing23'/,
  /trailerAxle1TrackUncertainty:'trailerAxle1Track'/,
  /trailerAxle2TrackUncertainty:'trailerAxle2Track'/,
  /trailerAxle3TrackUncertainty:'trailerAxle3Track'/,
  /tractorSteeringTrackUncertainty:'tractorSteeringTrack'/,
  /tractorFrontTrackUncertainty:'tractorFrontTrack'/,
  /tractorRearTrackUncertainty:'tractorRearTrack'/,
  /trailerTrackUncertainty:'trailerTrack'/,
  /tractorRearTireGroupWidthUncertainty:'tractorRearTireGroupWidth'/,
  /trailerTireGroupWidthUncertainty:'trailerTireGroupWidth'/,
  /tireLengthUncertainty:'tireLength'/,
  /tireWidthUncertainty:'tireWidth'/,
  /cabOffsetUncertainty:'cabOffset'/,
  /cabFrontUncertainty:'cabFront'/,
  /cabRearUncertainty:'cabRear'/,
  /cabWidthUncertainty:'cabWidth'/,
  /mirrorOutboardUncertainty:'mirrorOutboard'/,
  /mirrorLongitudinalUncertainty:'mirrorLongitudinal'/,
  /mirrorLengthUncertainty:'mirrorLength'/
 ])assert.match(source,mapping);
});

test('tire-group uncertainty is rejected until a measured nominal group width is entered',()=>{
 const {elements}=bootSimulator();
 elements.tractorRearTireGroupWidthUncertainty.value='0.10';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/tractorRearTireGroupWidthUncertainty/);
 assert.match(elements.routeAdvice.textContent,/сначала задайте измеренную номинальную ширину группы задних шин тягача/);
});

test('live assistant identifies the nearest vehicle part and named obstacle',()=>{
 const {elements}=bootSimulator();
 assert.match(elements.assistantAdvice.textContent,/Ближайший объект:/);
 assert.match(elements.assistantAdvice.textContent,/Часть автопоезда: (тягач|кабина|полуприцеп)\./);
 assert.match(elements.assistantAdvice.textContent,/Расчётный зазор/);
});

test('current-pose and per-part clearance include vehicle and obstacle uncertainty envelopes',()=>{
 const {elements,windowObject}=bootSimulator();
 const getDiagnostics=windowObject.TruckVisionGetClearanceDiagnostics;
 assert.equal(typeof getDiagnostics,'function');
 const baseline=getDiagnostics();
 assert.equal(baseline.parts.length,3);
 assert.ok(baseline.parts.every(part=>Number.isFinite(part.effective)));
 assert.ok(baseline.parts.every(part=>part.geometryLowerBound<=part.distance+1e-9));
 elements.obstaclePositionUncertainty.value='0.80';
 elements.obstacleSizeUncertainty.value='0.40';
 elements.vehicleFootprintUncertainty.value='0.20';
 elements.measurementUncertainty.value='0.30';
 elements.apply.onclick();
 const bounded=getDiagnostics();
 assert.equal(bounded.uncertainty.obstaclePosition,0.8);
 assert.equal(bounded.uncertainty.obstacleSize,0.4);
 assert.equal(bounded.uncertainty.vehicleFootprint,0.2);
 assert.ok(bounded.parts.every((part,index)=>part.effective<=baseline.parts[index].effective+1e-9));
 assert.ok(bounded.scene.nearest.effectiveDistance<=baseline.scene.nearest.effectiveDistance+1e-9);
 assert.ok(bounded.parts.every(part=>['тягач','кабина','полуприцеп'].includes(part.vehiclePart)));
});

test('uncertainty-envelope overlap is not misreported as a nominal collision',()=>{
 const {elements,windowObject}=bootSimulator();
 elements.measurementUncertainty.value='5';
 elements.apply.onclick();
 const diagnostics=windowObject.TruckVisionGetClearanceDiagnostics();
 assert.equal(diagnostics.scene.collision,false);
 assert.ok(diagnostics.scene.nearest.effectiveDistance<0);
 assert.match(elements.assistantAdvice.textContent,/оболочка погрешности пересекается с контуром/);
 assert.doesNotMatch(elements.assistantAdvice.textContent,/Стоп! Расчётные контуры/);
});

test('live assistant fails closed when obstacle checks are disabled',()=>{
 const {elements}=bootSimulator();
 elements.obstaclesToggle.onclick();
 assert.match(elements.assistantAdvice.textContent,/Проверка препятствий отключена/);
 assert.match(elements.assistantAdvice.textContent,/Свободный путь не подтверждён/);
 assert.doesNotMatch(elements.assistantAdvice.textContent,/Путь по текущему положению свободен/);
});

test('simulator explains joint endpoint and one-factor midpoint parameter coverage',()=>{
 const root=path.join(__dirname,'..');
 const source=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 assert.match(source,/stable-across-joint-endpoint-combinations/);
 assert.match(source,/stable-across-pairwise-endpoint-scenarios/);
 assert.match(source,/середины отдельных диапазонов по одному параметру/);
 assert.match(source,/не полный перебор всех комбинаций/);
});


test('camera obstacle registration requires three repeatable measurements and applies their conservative envelope',()=>{
 const {elements,windowObject}=bootSimulator();
 const measurement={
  width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01,
  conditioningWarning:false,shapeWarning:false,
  nonlinearSensitivityAvailable:true,nonlinearWarning:false
 };
 const anchor={x:3,y:4},heading=Math.PI/2,name='Опорный столб';
 const poseUncertainty={positionX:0.1,positionY:0.15,headingDeg:2};
 const verticalMeasurement={
  height:2.5,heightUncertainty:0.03,baseDatumUncertainty:0.02,totalUncertainty:0.05,
  datum:'local-road-surface',source:'manual-independent-measurement',confirmed:true
 };
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty),false);
 const readings=[
  {width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01},
  {width:2.005,height:1.004,sigmaWidth:0.012,sigmaHeight:0.012},
  {width:1.998,height:0.998,sigmaWidth:0.011,sigmaHeight:0.011}
 ];
 const repeatability=cameraGeometry.summarizeRepeatability(readings,{widthTolerance:0.02,heightTolerance:0.02});
 assert.equal(repeatability.withinTolerance,true);
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,repeatability),false,'a camera obstacle without independent height confirmation must be rejected');
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,'',poseUncertainty,repeatability,verticalMeasurement),false);
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,{positionX:0,positionY:0.15,headingDeg:2},repeatability,verticalMeasurement),false,'zero X uncertainty must be rejected');
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,{positionX:0.1,positionY:0,headingDeg:2},repeatability,verticalMeasurement),false,'zero Y uncertainty must be rejected');
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,{positionX:0.1,positionY:0.15,headingDeg:0},repeatability,verticalMeasurement),false,'zero heading uncertainty must be rejected');
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,repeatability,verticalMeasurement),true);
 assert.match(windowObject.TruckVisionCameraObstacleVerticalStatus,/только по горизонтальной 2D-оболочке/);
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
 assert.equal(windowObject.TruckVisionClearCameraObstacles(),true);
});

test('camera obstacle registration rejects a two-frame or unstable repeatability result',()=>{
 const {windowObject}=bootSimulator();
 const measurement={
  width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01,
  conditioningWarning:false,shapeWarning:false,
  nonlinearSensitivityAvailable:true,nonlinearWarning:false
 };
 const anchor={x:3,y:4},heading=Math.PI/2,name='Опорный столб';
 const poseUncertainty={positionX:0.1,positionY:0.15,headingDeg:2};
 const verticalMeasurement={height:2.5,heightUncertainty:0.03,baseDatumUncertainty:0.02,totalUncertainty:0.05,datum:'local-road-surface',source:'manual-independent-measurement',confirmed:true};
 const validRepeatability=cameraGeometry.summarizeRepeatability([
  {width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01},
  {width:2.005,height:1.004,sigmaWidth:0.012,sigmaHeight:0.012},
  {width:1.998,height:0.998,sigmaWidth:0.011,sigmaHeight:0.011}
 ],{widthTolerance:0.02,heightTolerance:0.02});
 const twoFrames={...validRepeatability,count:2};
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,twoFrames,verticalMeasurement),false);
 const unstable=cameraGeometry.summarizeRepeatability([
  {width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01},
  {width:2.2,height:1.1,sigmaWidth:0.01,sigmaHeight:0.01},
  {width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01}
 ],{widthTolerance:0.02,heightTolerance:0.02});
 assert.equal(unstable.withinTolerance,false);
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,unstable,verticalMeasurement),false);
});

test('camera obstacle registration rejects inconsistent repeatability uncertainty summaries',()=>{
 const {windowObject}=bootSimulator();
 const measurement={
  width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01,
  conditioningWarning:false,shapeWarning:false,
  nonlinearSensitivityAvailable:true,nonlinearWarning:false
 };
 const anchor={x:3,y:4},heading=Math.PI/2,name='Опорный столб';
 const poseUncertainty={positionX:0.1,positionY:0.15,headingDeg:2};
 const verticalMeasurement={height:2.5,heightUncertainty:0.03,baseDatumUncertainty:0.02,totalUncertainty:0.05,datum:'local-road-surface',source:'manual-independent-measurement',confirmed:true};
 const valid=cameraGeometry.summarizeRepeatability([
  {width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01},
  {width:2.005,height:1.004,sigmaWidth:0.012,sigmaHeight:0.012},
  {width:1.998,height:0.998,sigmaWidth:0.011,sigmaHeight:0.011}
 ],{widthTolerance:0.02,heightTolerance:0.02});
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,{...valid,maxSigmaWidth:0.001},verticalMeasurement),false);
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,{...valid,widthEnvelopeBound:0.001},verticalMeasurement),false);
 assert.equal(windowObject.TruckVisionAddCameraObstacle(measurement,anchor,heading,name,poseUncertainty,{...valid,maxDeviationWidth:valid.rangeWidth+0.01},verticalMeasurement),false);
});
 
test('camera registration atomically applies measured gate pose and enables jamb obstacles',()=>{
 const {elements,windowObject}=bootSimulator();
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:0.08,
  clearHeight:4.1,heightUncertaintyBound:0.12
 });
 assert.equal(applied,true);
 assert.equal(elements.gateWidth.value,'3.80');
 assert.equal(elements.gateWidthUncertainty.value,'0.080');
 assert.equal(elements.gateCenterX.value,'14.50');
 assert.equal(elements.gateCenterY.value,'-2.25');
 assert.equal(elements.gateHeadingDeg.value,'37.00');
 assert.equal(elements.gateLintelClearHeight.value,'4.100');
 assert.equal(elements.gateLintelCameraHeightUncertainty.value,'0.120');
 assert.equal(elements.gateEnabled.checked,true);
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
});

test('camera gate registration activates a measured lintel only with explicit thickness and depth',()=>{
 const {elements,windowObject}=bootSimulator();
 elements.gateLintelEnabled.checked=true;
 elements.gateLintelAlignmentConfirmed.checked=true;
 elements.gateLintelFloorUncertainty.value='0.02';
 elements.gateLintelThickness.value='0.25';
 elements.gateLintelThicknessUncertainty.value='0.01';
 elements.gateLintelSpan.value='4.6';
 elements.gateLintelSpanUncertainty.value='0.1';
 elements.gateLintelDepth.value='0.45';
 elements.gateLintelDepthUncertainty.value='0.02';
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:0.08,
  clearHeight:4.1,heightUncertaintyBound:0.12
 });
 assert.equal(applied,true);
 assert.equal(elements.gateLintelEnabled.checked,true);
 assert.equal(elements.gateLintelClearHeight.value,'4.100');
 assert.equal(elements.gateLintelCameraHeightUncertainty.value,'0.120');
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
});

test('camera gate registration rejects a lintel without explicit alignment confirmation',()=>{
 const {elements,windowObject}=bootSimulator();
 elements.gateLintelEnabled.checked=true;
 elements.gateLintelFloorUncertainty.value='0.02';
 elements.gateLintelThickness.value='0.25';
 elements.gateLintelThicknessUncertainty.value='0.01';
 elements.gateLintelSpan.value='4.6';
 elements.gateLintelSpanUncertainty.value='0.1';
 elements.gateLintelDepth.value='0.45';
 elements.gateLintelDepthUncertainty.value='0.02';
 const before=elements.gateWidth.value;
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:0.08,
  clearHeight:4.1,heightUncertaintyBound:0.12
 });
 assert.equal(applied,false);
 assert.equal(elements.gateWidth.value,before);
});

test('camera gate registration rejects an enabled lintel without measured beam dimensions',()=>{
 const {elements,windowObject}=bootSimulator();
 elements.gateLintelEnabled.checked=true;
 elements.gateLintelAlignmentConfirmed.checked=true;
 const before=elements.gateWidth.value;
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:0.08,
  clearHeight:4.1,heightUncertaintyBound:0.12
 });
 assert.equal(applied,false);
 assert.equal(elements.gateWidth.value,before);
});

test('camera registration rejects invalid gate width without changing current map settings',()=>{
 const {elements,windowObject}=bootSimulator();
 const before=[elements.gateWidth.value,elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value];
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:0.4,headingDeg:37
 });
 assert.equal(applied,false);
 assert.deepEqual(
  [elements.gateWidth.value,elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value],
  before
 );
 assert.notEqual(elements.gateEnabled.checked,true);
});

test('camera registration rejects an uncertainty envelope that collapses the gate opening atomically',()=>{
 const {elements,windowObject}=bootSimulator();
 const before=[
  elements.gateWidth.value,elements.gateWidthUncertainty.value,
  elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value
 ];
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:3.8
 });
 assert.equal(applied,false);
 assert.deepEqual([
  elements.gateWidth.value,elements.gateWidthUncertainty.value,
  elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value
 ],before);
 assert.notEqual(elements.gateEnabled.checked,true);
});

test('gate map uncertainty settings are forwarded to both jamb polygons',()=>{
 const root=path.join(__dirname,'..');
 const source=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 assert.match(source,/widthUncertainty:cfg\.gateWidthUncertainty/);
 assert.match(source,/positionUncertainty:cfg\.gatePositionUncertainty/);
 assert.match(source,/headingUncertaintyDeg:cfg\.gateHeadingUncertaintyDeg/);
});

test('camera registration rejects invalid manual gate pose uncertainty without partial mutation',()=>{
 const {elements,windowObject}=bootSimulator();
 const before=[
  elements.gateWidth.value,elements.gateWidthUncertainty.value,
  elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value
 ];
 elements.gateHeadingUncertaintyDeg.value='46';
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37,widthUncertaintyBound:0.08
 });
 assert.equal(applied,false);
 assert.deepEqual([
  elements.gateWidth.value,elements.gateWidthUncertainty.value,
  elements.gateCenterX.value,elements.gateCenterY.value,elements.gateHeadingDeg.value
 ],before);
 assert.notEqual(elements.gateEnabled.checked,true);
});

test('ambiguous planner results preserve candidate states and simulator draws them without enabling playback',()=>{
 const root=path.join(__dirname,'..');
 const plannerSource=fs.readFileSync(path.join(root,'articulated-planner.js'),'utf8');
 const simulatorSource=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 assert.match(plannerSource,/candidateRoutes:candidates\.map\(\(candidate,index\)=>\(/);
 assert.match(plannerSource,/states:candidate\.states/);
 assert.match(simulatorSource,/alternativeRoutes=Array\.isArray\(result\.candidateRoutes\)/);
 assert.match(simulatorSource,/drawLine\(candidate\.states,colors\[index%colors\.length\],4,\[9,6\]\)/);
 assert.match(simulatorSource,/geometrySensitivity\.status==='nominal-ambiguous'/);
 assert.match(simulatorSource,/Совместная оболочка погрешности ворот не проверялась/);
 assert.match(simulatorSource,/if\(plannedRoute\.length<2\|\|!routeAssessment\)/);
});


function perPartClearanceWorkerResult(){
 const states=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0},
  {x:1,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:1}
 ];
 const trajectoryClearance={
  statesAnalyzed:2,segmentsAnalyzed:1,distanceAlongRoute:1,
  parts:[
   {vehiclePart:'тягач',minimumSampledClearance:0.8,minimumClearanceLowerBound:0.7,
    limitingDetail:{kind:'segment-lower-bound',obstacleAtReference:'front pallet',distanceAlongRoute:0.4,gear:1}},
   {vehiclePart:'кабина',minimumSampledClearance:0.9,minimumClearanceLowerBound:0.8,
    limitingDetail:{kind:'segment-lower-bound',obstacleAtReference:'cab-side pallet',distanceAlongRoute:0.4,gear:1}},
   {vehiclePart:'полуприцеп',minimumSampledClearance:0.6,minimumClearanceLowerBound:0.5,
    limitingDetail:{kind:'segment-lower-bound',obstacleAtReference:'rear trailer pallet',distanceAlongRoute:0.7,gear:-1}}
  ],
  minimumClearanceLowerBound:0.5,limitingPart:'полуприцеп'
 };
 const recommendation={initialGear:1,firstTurn:'left',firstTurnGear:1,firstTurnDistance:0.4,tractorYawDirection:'left'};
 return {
  ok:false,reason:'ambiguous-start',
  candidateRoutes:[{index:0,label:'проверенный тестовый вариант',cost:1,states,
   metrics:{minimumClearanceLowerBound:0.65,sweptClearanceDetail:null},
   trajectoryClearance,recommendation}],
  candidateSummaries:[{label:'проверенный тестовый вариант',cost:1,initialGear:1,firstTurn:'left',
   firstTurnGear:1,tractorYawDirection:'left'}]
 };
}

function conservativeSelectionWorkerResult(){
 const states=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0},
  {x:11,y:0,psi:0,psiT:0,gear:1,steer:0},
  {x:22,y:0,psi:0,psiT:0,gear:1,steer:0}
 ];
 return {
  ok:false,
  reason:'uncertainty-sensitive-start',
  conservativeCandidateRoutes:[
   {index:0,label:'проверенный резервный кандидат',cost:22,states,metrics:{
    minimumClearanceLowerBound:1.2,
    sweptClearanceDetail:null
   }},
   {index:1,label:'второй резервный кандидат',cost:23,states:states.map(state=>({...state})),metrics:{
    minimumClearanceLowerBound:1.1,
    sweptClearanceDetail:null
   }}
  ],
  sensitivity:{
   measurementUncertainty:0.15,
   nominalRequiredClearance:0.65,
   reserveRequiredClearance:0.95,
   comparison:{
    nominalAction:{initialGear:1,firstTurn:'left'},
    reserveAction:{initialGear:-1,firstTurn:'right'}
   }
  }
 };
}

function incompleteWheelSamplingWorkerResult(){
 const result=conservativeSelectionWorkerResult();
 const states=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0,stepDistance:0},
  {x:0.5,y:0,psi:0.1,psiT:0,gear:1,steer:0,stepDistance:0.5},
  {x:1,y:0,psi:0.2,psiT:0,gear:1,steer:0,stepDistance:0.5}
 ];
 return {
  ...result,
  conservativeCandidateRoutes:result.conservativeCandidateRoutes.map(candidate=>({
   ...candidate,states:states.map(state=>({...state}))
  }))
 };
}

function routeControlBoundaryWorkerResult(states){
 const result=conservativeSelectionWorkerResult();
 return {
  ...result,
  conservativeCandidateRoutes:result.conservativeCandidateRoutes.map(candidate=>({
   ...candidate,states:states.map(state=>({...state}))
  }))
 };
}

test('selected alternative displays per-part swept clearance, limiting obstacles and route segment direction',async()=>{
 const simulator=bootSimulator({workerResult:perPartClearanceWorkerResult()});
 const {elements}=simulator;
 elements.goalX.value='1';
 elements.goalY.value='0';
 elements.goalHeading.value='0';
 await elements.planRoute.onclick();
 assert.equal(elements.alternativeRouteSelect.children.length,1);
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.difficultyAdvice.textContent,/Зазоры по частям вдоль маршрута/);
 assert.match(elements.difficultyAdvice.textContent,/Ограничивающее колесо:/);
 assert.match(elements.difficultyAdvice.textContent,/промежуточная выборка/);
 assert.match(elements.difficultyAdvice.textContent,/тягач: 0\.70 м, препятствие у начала сегмента «front pallet»/);
 assert.match(elements.difficultyAdvice.textContent,/кабина: 0\.80 м, препятствие у начала сегмента «cab-side pallet»/);
 assert.match(elements.difficultyAdvice.textContent,/полуприцеп: 0\.50 м, препятствие у начала сегмента «rear trailer pallet»/);
 assert.match(elements.difficultyAdvice.textContent,/выборочные длины траекторий \(центр\/левое\/правое, м\)/);
 assert.match(elements.difficultyAdvice.textContent,/ось 1/);
 assert.match(elements.difficultyAdvice.textContent,/внутренняя сторона/);
 assert.match(elements.difficultyAdvice.textContent,/задний ход/);
 assert.match(elements.difficultyAdvice.textContent,/Автопроезд остаётся заблокирован/);
});

test('clearing the alternative-route placeholder clears the selected route instead of coercing an empty value to index zero',async()=>{
 const simulator=bootSimulator({workerResult:conservativeSelectionWorkerResult()});
 const {elements}=simulator;
 elements.goalX.value='22';
 elements.goalY.value='0';
 elements.goalHeading.value='0';
 await elements.planRoute.onclick();
 assert.equal(elements.alternativeRouteSelect.style.display,'block');
 assert.equal(elements.alternativeRouteSelect.children.length,2);
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.routeAdvice.textContent,/Выбран вариант 1/);
 elements.alternativeRouteSelect.value='';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.routeAdvice.textContent,/Выбор альтернативной траектории снят/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Сначала нажмите «Построить путь»/);
});

test('invalid alternative-route indices fail closed and never keep a stale candidate',async()=>{
 const simulator=bootSimulator({workerResult:conservativeSelectionWorkerResult()});
 const {elements}=simulator;
 elements.goalX.value='22';
 elements.goalY.value='0';
 elements.goalHeading.value='0';
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='99';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.routeAdvice.textContent,/такого варианта нет/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Сначала нажмите «Построить путь»/);
});

test('simulator applies measured mirror geometry and reports a mirror-side obstacle component',()=>{
 const {elements,windowObject}=bootSimulator();
 assert.equal(elements.mirrorOutboard.value,'0');
 assert.match(elements.assistantAdvice.textContent,/Геометрия зеркал не задана/);
 elements.mirrorOutboard.value='0.45';
 elements.mirrorLongitudinal.value='3.00';
 elements.mirrorLength.value='0.80';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
 const frame=windowObject.TruckVisionSetPerceptionFrame({
  frameId:'mirror-calibration-smoke',
  timestamp:Date.now(),
  coordinateFrame:'vehicle-local',
  vehiclePose:{x:0,y:0,heading:0},
  objects:[{
   id:'left-mirror-post',type:'post',
   pose:{x:3,y:1.75,heading:0},
   dimensions:{length:0.2,width:0.2,height:1.0},
   confidence:0.99,source:'fused'
  }]
 });
 assert.equal(frame.ok,true);
 const diagnostics=windowObject.TruckVisionGetClearanceDiagnostics();
 assert.equal(diagnostics.uncertainty.mirror.enabled,true);
 assert.equal(diagnostics.parts[1].component,'левое зеркало');
 assert.match(elements.assistantAdvice.textContent,/левое зеркало/);
 const collisionFrame=windowObject.TruckVisionSetPerceptionFrame({
  frameId:'mirror-collision-smoke',
  timestamp:Date.now(),
  coordinateFrame:'vehicle-local',
  vehiclePose:{x:0,y:0,heading:0},
  objects:[{
   id:'left-mirror-contact',type:'post',
   pose:{x:3,y:1.45,heading:0},
   dimensions:{length:0.2,width:0.2,height:1.0},
   confidence:0.99,source:'fused'
  }]
 });
 assert.equal(collisionFrame.ok,true);
 assert.match(elements.assistantAdvice.textContent,/левое зеркало/);
 assert.match(elements.assistantAdvice.textContent,/пересекаются с препятствием/);
});


test('simulator exposes calibrated wheel tracks and invalidates a route when they change',()=>{
 const {elements}=bootSimulator();
 assert.equal(elements.tractorFrontTrack.value,'2.05');
 assert.equal(elements.tractorSteeringTrack.value,'2.05');
 assert.equal(elements.tractorRearTrack.value,'2.05');
 assert.equal(elements.trailerTrack.value,'2.79');
 assert.equal(elements.tireLength.value,'0.72');
 assert.equal(elements.tireWidth.value,'0.44');
 elements.tractorSteeringTrack.value='2.12';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
 elements.trailerTrack.value='2.65';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
});


test('route assessment names the individual trailer wheel that limits the swept clearance',async()=>{
 const simulator=bootSimulator({workerResult:perPartClearanceWorkerResult()});
 const {elements,windowObject}=simulator;
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const firstAxle=physics.trailerAxlePoints({x:0,y:0,psi:0,psiT:0},p)[0];
 const frame=windowObject.TruckVisionSetPerceptionFrame({
  frameId:'wheel-clearance-smoke-001',
  timestamp:Date.now(),
  coordinateFrame:'vehicle-local',
  vehiclePose:{x:0,y:0,heading:0},
  objects:[{
   id:'outer-wheel-post',type:'post',
   pose:{x:firstAxle.x,y:1.66,heading:0},
   dimensions:{length:0.1,width:0.1,height:1.0},
   confidence:0.99,source:'fused'
  }]
 });
 assert.equal(frame.ok,true);
 elements.goalX.value='1';
 elements.goalY.value='0';
 elements.goalHeading.value='0';
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.difficultyAdvice.textContent,/Ограничивающее колесо: левое колесо 1-й оси полуприцепа/);
 assert.match(elements.difficultyAdvice.textContent,/консервативная геометрическая оценка/);
 assert.match(elements.difficultyAdvice.textContent,/Автопроезд остаётся заблокирован/);
});

test('route assessment labels the default Volvo FH and tri-axle geometry as preliminary when bounds are zero',async()=>{
 const simulator=bootSimulator({workerResult:perPartClearanceWorkerResult()});
 const {elements}=simulator;
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.difficultyAdvice.textContent,/геометрия модели Volvo FH 4×2 и трёхосного полуприцепа предварительная/);
 assert.match(elements.difficultyAdvice.textContent,/Нулевой допуск означает «не задан», а не точность/);
 assert.match(elements.difficultyAdvice.textContent,/рекомендация стороны и направления начала манёвра не подтверждена/);
});

test('route assessment fails closed when per-wheel swept clearance cannot validate integration steps',async()=>{
 const simulator=bootSimulator({workerResult:conservativeSelectionWorkerResult()});
 const {elements}=simulator;
 elements.goalX.value='22';
 elements.goalY.value='0';
 elements.goalHeading.value='0';
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.difficultyAdvice.textContent,/Проверка отдельных колёс не подтверждена/);
 assert.match(elements.difficultyAdvice.textContent,/stepDistance/);
 assert.match(elements.difficultyAdvice.textContent,/Автопроезд остаётся заблокирован/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд не запущен: проверка отдельных колёс не завершена или не подтверждена/);
});


test('route assessment blocks virtual driving for incomplete wheel sampling even when the route reaches its goal',async()=>{
 const simulator=bootSimulator({workerResult:incompleteWheelSamplingWorkerResult()});
 const {elements}=simulator;
 elements.goalX.value='1';
 elements.goalY.value='0';
 elements.goalHeading.value=String(0.2*180/Math.PI);
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.match(elements.difficultyAdvice.textContent,/Промежуточная проверка не завершена на 2 сегментах/);
 assert.match(elements.difficultyAdvice.textContent,/Автопроезд остаётся заблокирован/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд не запущен: проверка отдельных колёс не завершена или не подтверждена/);
});


test('simulator collision diagnostics use the current Ackermann steering angle for a protruding front tire',()=>{
 const {elements,windowObject}=bootSimulator();
 elements.tractorFrontTrack.value='2.4';
 elements.tractorSteeringTrack.value='2.4';
 elements.tireLength.value='1.2';
 elements.tireWidth.value='0.8';
 elements.trailerTrack.value='2.5';
 elements.apply.onclick();
 elements.steer.value='35';
 elements.steer.oninput();
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorFrontTrack:2.4,tractorSteeringTrack:2.4,tractorRearTrack:2.05,
  trailerTrack:2.5,tireLength:1.2,tireWidth:0.8,footprintUncertainty:0.03
 };
 const wheel=physics.wheelFootprints({x:0,y:0,psi:0,psiT:0},p,35*Math.PI/180).find(item=>
  item.name==='левое переднее колесо тягача'
 );
 assert.ok(wheel);
 const corner=wheel.polygon[0];
 const frame=windowObject.TruckVisionSetPerceptionFrame({
  frameId:'ackermann-front-wheel-smoke-001',timestamp:Date.now(),
  coordinateFrame:'vehicle-local',vehiclePose:{x:0,y:0,heading:0},
  objects:[{
   id:'steered-front-wheel-post',type:'post',
   pose:{x:corner.x,y:corner.y,heading:0},
   dimensions:{length:0.1,width:0.1,height:1.0},
   confidence:0.99,source:'fused'
  }]
 });
 assert.equal(frame.ok,true);
 const diagnostics=windowObject.TruckVisionGetClearanceDiagnostics();
 const tractor=diagnostics.parts.find(part=>part.vehiclePart==='тягач');
 assert.ok(tractor);
 assert.equal(tractor.component,'левое переднее колесо тягача');
});

test('route metrics does not count an unknown initial reverse selection as a transmission change',async()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
  trailerTrack:2.11,tireLength:0.72,tireWidth:0.44
 };
 const start={
  x:0,y:0,psi:0,psiT:0,gear:1,steer:0,
  gearKnown:false,steerKnown:false,stepDistance:0
 };
 const steer=-0.16,step=0.5;
 const first=physics.rk4(start,-0.7,steer,step/0.7,p);
 const second=physics.rk4(first,-0.7,steer,step/0.7,p);
 const states=[
  start,
  {...first,gear:-1,steer,stepDistance:step},
  {...second,gear:-1,steer,stepDistance:step}
 ];
 const simulator=bootSimulator({workerResult:routeControlBoundaryWorkerResult(states)});
 const {elements}=simulator;
 elements.goalX.value=String(second.x);
 elements.goalY.value=String(second.y);
 elements.goalHeading.value=String(second.psi*180/Math.PI);
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 const advice=elements.difficultyAdvice.textContent;
 assert.match(advice,/задний ход/);
 assert.equal(/переключений передачи: 1/.test(advice),false);
 assert.match(advice,/первое рулевое действие при начале маршрута/);
});

test('route metrics still counts a measured in-route forward-to-reverse gear change',async()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
  trailerTrack:2.11,tireLength:0.72,tireWidth:0.44
 };
 const start={
  x:0,y:0,psi:0,psiT:0,gear:1,steer:0,
  gearKnown:true,steerKnown:true,stepDistance:0
 };
 const step=0.5;
 const forward=physics.rk4(start,0.7,0,step/0.7,p);
 const reverse=physics.rk4(forward,-0.7,0,step/0.7,p);
 const states=[
  start,
  {...forward,gear:1,steer:0,stepDistance:step},
  {...reverse,gear:-1,steer:0,stepDistance:step}
 ];
 const simulator=bootSimulator({workerResult:routeControlBoundaryWorkerResult(states)});
 const {elements}=simulator;
 elements.goalX.value=String(reverse.x);
 elements.goalY.value=String(reverse.y);
 elements.goalHeading.value=String(reverse.psi*180/Math.PI);
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 const advice=elements.difficultyAdvice.textContent;
 assert.match(advice,/переключений передачи: 1/);
 assert.equal(/первое рулевое действие при начале маршрута/.test(advice),false);
});

test('virtual playback stops at the last waypoint before a gear change and requires explicit confirmation',()=>{
 const source=fs.readFileSync(path.join(__dirname,'..','truckvision.js'),'utf8');
 const start=source.indexOf('const nextGearTransition=planner.nextGearTransitionBoundary');
 const end=source.indexOf('let nearest=Infinity',start);
 assert.ok(start>=0&&end>start);
 const block=source.slice(start,end);
 assert.match(block,/shiftBoundaryIndex/);
 assert.match(block,/index<shiftBoundaryIndex/);
 assert.match(block,/const gear=routePlayback\.currentGear/);
 const stopCheck=block.indexOf('if(Math.abs(speed)>0.02)');
 const hold=block.indexOf('routePlayback.awaitingGearConfirmation=true');
 assert.ok(stopCheck>=0&&hold>stopCheck);
 assert.match(block,/boundaryDistance<0\.025/);
 assert.match(block,/drive=0;braking=true/);
 assert.match(block,/routePlayback.pendingGear=nextGearTransition\.to/);
 assert.match(block,/routeConfirmGear/);
 assert.match(block,/длительность переключения не измеряются/);
 const targetStart=source.indexOf('let targetIndex=Math.min(plannedRoute.length-1,index+6,shiftBoundaryIndex)');
 assert.ok(targetStart>0,'look-ahead target must not cross the gear-change boundary');
 const confirmStart=source.indexOf("$('routeConfirmGear').onclick");
 const confirmEnd=source.indexOf("$('routeStop').onclick",confirmStart);
 assert.ok(confirmStart>=0&&confirmEnd>confirmStart);
 const confirmBlock=source.slice(confirmStart,confirmEnd);
 assert.match(confirmBlock,/routePlayback.currentGear=confirmedGear/);
 assert.match(confirmBlock,/routePlayback.awaitingGearConfirmation=false/);
 assert.match(confirmBlock,/Реальная передача, сцепление и время переключения не измеряются/);
 const pauseStart=source.indexOf("$('routePause').onclick");
 const pauseEnd=source.indexOf("$('routeConfirmGear').onclick",pauseStart);
 assert.ok(pauseStart>=0&&pauseEnd>pauseStart);
 assert.match(source.slice(pauseStart,pauseEnd),/routePlayback.awaitingGearConfirmation/);
 const startHandler=source.slice(source.indexOf("$('routeStart').onclick"),pauseStart);
 assert.match(startHandler,/routePlayback.awaitingGearConfirmation/);
});

test('virtual playback holds at the gear boundary until the explicit confirmation button is pressed',async()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const start={
  x:0,y:0,psi:0,psiT:0,gear:1,steer:0,
  gearKnown:true,steerKnown:true,stepDistance:0
 };
 const forwardStep=0.5,reverseStep=2.0;
 const forward=physics.rk4(start,0.7,0,forwardStep/0.7,p);
 const reverse=physics.rk4(forward,-0.7,0,reverseStep/0.7,p);
 const states=[
  start,
  {...forward,gear:1,steer:0,gearKnown:true,steerKnown:true,stepDistance:forwardStep},
  {...reverse,gear:-1,steer:0,gearKnown:true,steerKnown:true,stepDistance:reverseStep}
 ];
 const simulator=bootSimulator({workerResult:routeControlBoundaryWorkerResult(states)});
 const {elements,frames}=simulator;
 elements.goalX.value=String(reverse.x);
 elements.goalY.value=String(reverse.y);
 elements.goalHeading.value='0';
 await elements.planRoute.onclick();
 elements.alternativeRouteSelect.value='0';
 elements.alternativeRouteSelect.onchange();
 assert.equal(elements.routeStart.disabled,undefined);
 assert.equal(elements.difficultyAdvice.textContent.includes('Автопроезд остаётся заблокирован'),false);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Виртуальный автопроезд начат/);
 let timestamp=1000;
 for(let index=0;index<200&&elements.routeConfirmGear.hidden!==false;index++){
  const callback=frames.shift();
  assert.equal(typeof callback,'function');
  callback(timestamp);
  timestamp+=40;
 }
 assert.equal(elements.routeConfirmGear.hidden,false,'playback must stop at the exact gear boundary');
 assert.equal(elements.routeConfirmGear.disabled,false);
 assert.match(elements.routeAdvice.textContent,/Нажмите «Подтвердить виртуальное переключение»/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Сначала подтвердите виртуальное переключение/);
 assert.equal(elements.routeConfirmGear.hidden,false);
 elements.routePause.onclick();
 assert.match(elements.routeAdvice.textContent,/ожидает отдельного подтверждения/);
 assert.equal(elements.routeConfirmGear.hidden,false);
 elements.routeConfirmGear.onclick();
 assert.equal(elements.routeConfirmGear.hidden,true);
 assert.equal(elements.routeConfirmGear.disabled,true);
 assert.match(elements.routeAdvice.textContent,/Виртуальное переключение подтверждено для симуляции/);
});


test('per-axle uncertainty cannot be entered when the individual geometry field inherits a common value',()=>{
 const {elements}=bootSimulator();
 elements.trailerAxleSpacing12Uncertainty.value='0.02';
 elements.trailerAxle1TrackUncertainty.value='0.02';
 elements.apply.onclick();
 assert.match(elements.routeAdvice.textContent,/сначала задайте индивидуальное измеренное значение/);
 assert.match(elements.routeAdvice.textContent,/trailerAxleSpacing12/);
 assert.match(elements.routeAdvice.textContent,/trailerAxle1Track/);
});

test('camera obstacle vertical range is enabled only after a valid road-surface calibration is applied',()=>{
 const {elements,windowObject}=bootSimulator();
 const surface={
  originX:0,originY:0,originZ:0,gradeX:0.01,gradeY:-0.005,
  originZUncertainty:0.01,gradeXUncertainty:0.001,gradeYUncertainty:0.001,
  surfaceResidualUncertainty:0.01
 };
 elements.roadSurface.value=JSON.stringify(surface);
 elements.apply.onclick();
 assert.doesNotMatch(elements.routeAdvice.textContent,/Параметры не применены/);
 const measurement={
  width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01,
  conditioningWarning:false,shapeWarning:false,
  nonlinearSensitivityAvailable:true,nonlinearWarning:false
 };
 const readings=[
  {width:2,height:1,sigmaWidth:0.01,sigmaHeight:0.01},
  {width:2.005,height:1.004,sigmaWidth:0.012,sigmaHeight:0.012},
  {width:1.998,height:0.998,sigmaWidth:0.011,sigmaHeight:0.011}
 ];
 const repeatability=cameraGeometry.summarizeRepeatability(readings,{widthTolerance:0.02,heightTolerance:0.02});
 const verticalMeasurement={
  height:2.5,heightUncertainty:0.03,baseDatumUncertainty:0.02,totalUncertainty:0.05,
  datum:'local-road-surface',source:'manual-independent-measurement',confirmed:true
 };
 elements.roadSurface.value='null';
 assert.equal(windowObject.TruckVisionAddCameraObstacle(
  measurement,{x:3,y:4},Math.PI/2,'Опорный столб',
  {positionX:0.1,positionY:0.15,headingDeg:2},repeatability,verticalMeasurement
 ),true);
 assert.match(windowObject.TruckVisionCameraObstacleVerticalStatus,/только по горизонтальной 2D-оболочке/);
 assert.equal(windowObject.TruckVisionClearCameraObstacles(),true);
 elements.roadSurface.value=JSON.stringify(surface);
 elements.apply.onclick();
 assert.equal(windowObject.TruckVisionAddCameraObstacle(
  measurement,{x:3,y:4},Math.PI/2,'Опорный столб',
  {positionX:0.1,positionY:0.15,headingDeg:2},repeatability,verticalMeasurement
 ),true);
 assert.match(windowObject.TruckVisionCameraObstacleVerticalStatus,/Вертикальный диапазон привязан к откалиброванной плоскости дороги/);
});
