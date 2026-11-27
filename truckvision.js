
(()=>{
'use strict';

const $=id=>document.getElementById(id);
const canvas=$('view');
const ctx=canvas.getContext('2d');
const G=9.81;
let SCALE=26;
const ARTICULATION_WARNING_DEG=45;
const ARTICULATION_STOP_DEMO_DEG=55;

const cfg={
 wheelbase:3.8,
 hitchOffset:0.575,
 tractorMass:8500,
 engineForce:24000,
 trailerLength:13.6,
 trailerKingpinToFront:1.5,
 trailerWidth:2.55,
 kingpinAxles:8,
 trailerAxleSpacing:1.3,
 trailerAxleSpacing12:0,
 trailerAxleSpacing23:0,
 cabOffset:2.1,
 cabFront:1.35,
 cabRear:0.9,
 cabWidth:2.25,
 tractorFrontOverhang:5.0,
 tractorRearOverhang:1.2,
 tractorBodyWidth:2.55,
 mirrorOutboard:0,
 mirrorLongitudinal:3.0,
 mirrorLength:0.8,
 tractorFrontTrack:2.05,
 tractorSteeringTrack:2.05,
 tractorRearTrack:2.05,
 trailerTrack:2.79,
 trailerAxle1Track:0,
 trailerAxle2Track:0,
 trailerAxle3Track:0,
 tractorRearTireGroupWidth:0,
 trailerTireGroupWidth:0,
 trailerAxle1TireGroupWidth:0,
 trailerAxle2TireGroupWidth:0,
 trailerAxle3TireGroupWidth:0,
 tractorRearTireCenterSpacing:0,
 trailerAxle1TireCenterSpacing:0,
 trailerAxle2TireCenterSpacing:0,
 trailerAxle3TireCenterSpacing:0,
 tireLength:0.72,
 tireWidth:0.44,
 auxiliaryEnvelopes:[],
 vehicleVerticalProfiles:{},
 roadSurface:null,
 suspensionTravel:null,
 verticalObstacles:[],
 trailerMass:30000,
 mu:0.7,
 crr:0.008,
 brakeDecel:4.5,
 maxAccel:0.8,
 gateWidth:3.5,
 gateCenterX:12,
 gateCenterY:0,
 gateHeadingDeg:90,
 gateWidthUncertainty:0,
 gatePositionUncertainty:0,
 gateHeadingUncertaintyDeg:0,
 gatePostWidth:0.25,
 gateDepth:0.6,
 gateLintelEnabled:false,
 gateLintelClearHeight:0,
 gateLintelCameraHeightUncertainty:0,
 gateLintelFloorUncertainty:0,
 gateLintelThickness:0,
 gateLintelThicknessUncertainty:0,
 gateLintelSpan:0,
 gateLintelSpanUncertainty:0,
 gateLintelDepth:0,
 gateLintelDepthUncertainty:0,
 margin:0.2,
 measurementUncertainty:0.15,
 obstaclePositionUncertainty:0.05,
 obstacleSizeUncertainty:0.02,
 vehicleFootprintUncertainty:0.03,
 wheelbaseUncertainty:0,
 hitchOffsetUncertainty:0,
 kingpinAxlesUncertainty:0,
 trailerLengthUncertainty:0,
 trailerKingpinToFrontUncertainty:0,
 trailerWidthUncertainty:0,
 trailerAxleSpacingUncertainty:0,
 trailerAxleSpacing12Uncertainty:0,
 trailerAxleSpacing23Uncertainty:0,
 tractorSteeringTrackUncertainty:0,
 tractorFrontTrackUncertainty:0,
 tractorRearTrackUncertainty:0,
 trailerTrackUncertainty:0,
 trailerAxle1TrackUncertainty:0,
 trailerAxle2TrackUncertainty:0,
 trailerAxle3TrackUncertainty:0,
 tractorRearTireGroupWidthUncertainty:0,
 trailerTireGroupWidthUncertainty:0,
 trailerAxle1TireGroupWidthUncertainty:0,
 trailerAxle2TireGroupWidthUncertainty:0,
 trailerAxle3TireGroupWidthUncertainty:0,
 tractorRearTireCenterSpacingUncertainty:0,
 trailerAxle1TireCenterSpacingUncertainty:0,
 trailerAxle2TireCenterSpacingUncertainty:0,
 trailerAxle3TireCenterSpacingUncertainty:0,
 tireLengthUncertainty:0,
 tireWidthUncertainty:0,
 cabOffsetUncertainty:0,
 cabFrontUncertainty:0,
 cabRearUncertainty:0,
 cabWidthUncertainty:0,
 tractorFrontOverhangUncertainty:0,
 tractorRearOverhangUncertainty:0,
 tractorBodyWidthUncertainty:0,
 mirrorOutboardUncertainty:0,
 mirrorLongitudinalUncertainty:0,
 mirrorLengthUncertainty:0
};

let state,steer=0,targetSpeed=2,speed=0,drive=0;
let braking=false,distance=0,trace=[],showTrace=true;
let camera={x:5.5,y:0},lastTime=0;
let showObstacles=true,helperEnabled=true;
let plannedRoute=[],alternativeRoutes=[],routeGoal=null,routeAssessment=null;
let routePlayback={active:false,paused:false,index:1,stopReason:'',currentGear:1,pendingGear:null,awaitingGearConfirmation:false,confirmedGearChanges:0};
let plannerWorker=null,plannerRequestId=0,plannerReject=null,routeBuildToken=0;

// Учебный коридор: стены параллельны направлению движения (+X).
// Между внутренними гранями стен остаётся 9.3 м; паллеты стоят
// снаружи коридора и не перекрывают прямой тестовый маршрут к цели (22, 0).
const cameraObstacles=[]; // Session-local obstacles registered through the camera measurement UI.
let perceptionObstacles=[];
let perceptionFrameId=null;
let perceptionFrameTimestamp=0;
const obstacles=[
 {x:8.0,y:3.3,w:1.2,h:1.2,name:'Паллет A'},
 {x:16.0,y:-3.3,w:1.0,h:1.0,name:'Паллет B'},
 {x:13.0,y:5.0,w:16.0,h:0.7,name:'Стена слева'},
 {x:13.0,y:-5.0,w:16.0,h:0.7,name:'Стена справа'}
 ];

function gateGeometryForPlanner(){
 if(!$('gateEnabled')||!$('gateEnabled').checked)return null;
 const geometry={
  centerX:cfg.gateCenterX,
  centerY:cfg.gateCenterY,
  clearWidth:cfg.gateWidth,
  postWidth:cfg.gatePostWidth,
  depth:cfg.gateDepth,
  heading:cfg.gateHeadingDeg*Math.PI/180,
  widthUncertainty:cfg.gateWidthUncertainty,
  positionUncertainty:cfg.gatePositionUncertainty,
  headingUncertaintyDeg:cfg.gateHeadingUncertaintyDeg
 };
 if(cfg.gateLintelEnabled&&$('gateLintelEnabled')&&$('gateLintelEnabled').checked&&$('gateLintelAlignmentConfirmed')&&$('gateLintelAlignmentConfirmed').checked){
  geometry.lintel={
   clearHeight:cfg.gateLintelClearHeight,
   clearHeightUncertainty:cfg.gateLintelCameraHeightUncertainty,
   floorDatumUncertainty:cfg.gateLintelFloorUncertainty,
   thickness:cfg.gateLintelThickness,
   thicknessUncertainty:cfg.gateLintelThicknessUncertainty,
   span:cfg.gateLintelSpan,
   spanUncertainty:cfg.gateLintelSpanUncertainty,
   depth:cfg.gateLintelDepth,
   depthUncertainty:cfg.gateLintelDepthUncertainty
  };
 }
 try{
  planner.createGateObstacles(geometry);
 }catch(error){
  const status=$('gateResult');
  if(status)status.textContent='Ворота не включены в карту: '+(error&&error.message?error.message:String(error))+' Исправьте положительные допуски и повторите проверку.';
  return null;
 }
 return geometry;
}
function sceneObstacles(){
 if(perceptionFrameTimestamp&&Date.now()-perceptionFrameTimestamp>1500){
  perceptionObstacles=[];perceptionFrameId=null;perceptionFrameTimestamp=0;
  updatePerceptionStatus('Данные датчиков устарели. Объекты удалены из активной карты; требуется новый кадр.');
  if(typeof invalidateRouteForSettingsChange==='function')invalidateRouteForSettingsChange();
 }
 const result=perceptionFrameTimestamp?[]:obstacles.map(obstacle=>({...obstacle}));
 // User-registered camera obstacles remain part of the map while live perception is active.
 result.push(...cameraObstacles.map(obstacle=>({...obstacle,polygon:obstacle.polygon.map(point=>({...point})),nominalPolygon:obstacle.nominalPolygon.map(point=>({...point})),anchor:{...obstacle.anchor},mapPoseUncertainty:{...obstacle.mapPoseUncertainty}})));
 result.push(...(cfg.verticalObstacles||[]).map(obstacle=>({...obstacle,
  polygon:Array.isArray(obstacle.polygon)?obstacle.polygon.map(point=>({...point})):undefined
 })));
 result.push(...perceptionObstacles.map(obstacle=>({...obstacle,polygon:obstacle.polygon.map(point=>({...point}))})));
 const gateGeometry=gateGeometryForPlanner();
 if(gateGeometry)result.push(...planner.createGateObstacles(gateGeometry));
 return result;
}

const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const norm=a=>Math.atan2(Math.sin(a),Math.cos(a));
const physics=window.TruckVisionPhysics;
if(!physics)throw new Error('TruckVision physics module failed to load.');
const planner=window.TruckVisionPlanner;
if(!planner)throw new Error('TruckVision articulated planner module failed to load.');
const perception=window.TruckVisionPerception;
if(!perception)throw new Error('TruckVision perception adapter failed to load.');
function updatePerceptionStatus(message){
 const el=$('perceptionStatus');
 if(el)el.textContent=message;
}
window.TruckVisionSetPerceptionFrame=frame=>{
 const errors=perception.validateFrame(frame);
 if(errors.length){updatePerceptionStatus('Кадр отклонён: '+errors.join(' '));return {ok:false,errors};}
 let converted;
 try{converted=perception.toObstacles(frame);}
 catch(error){const list=[error.message||'Не удалось преобразовать координаты.'];updatePerceptionStatus('Кадр отклонён: '+list.join(' '));return {ok:false,errors:list};}
 perceptionObstacles=converted;
 perceptionFrameId=frame.frameId;
 perceptionFrameTimestamp=frame.timestamp;
 invalidateRouteForSettingsChange();
 updatePerceptionStatus('Датчики передали '+converted.length+' объектов · кадр '+frame.frameId+' · '+frame.objects.map(o=>o.source).filter((v,i,a)=>a.indexOf(v)===i).join(', '));
 draw();
 return {ok:true,accepted:converted.length,frameId:perceptionFrameId};
};
window.TruckVisionClearPerceptionFrame=()=>{
 perceptionObstacles=[];perceptionFrameId=null;perceptionFrameTimestamp=0;
 invalidateRouteForSettingsChange();
 updatePerceptionStatus('Нет активного кадра датчиков. Ожидание данных LiDAR/камер.');
 draw();return true;
};


function readSettings(){
 const candidate={...cfg};
 for(const key of Object.keys(candidate)){
  if(key==='gateLintelEnabled'){
   const input=$('gateLintelEnabled');
   candidate[key]=!!(input&&input.checked);
   continue;
  }
  if(['auxiliaryEnvelopes','vehicleVerticalProfiles','roadSurface','suspensionTravel','verticalObstacles'].includes(key)){
   const input=$(key);
   if(input){
    const fallback=key==='roadSurface'||key==='suspensionTravel'?'null':key==='vehicleVerticalProfiles'?'{}':'[]';
    try{candidate[key]=JSON.parse(String(input.value||fallback));}
    catch(error){return {ok:false,changed:false,errors:[key+': введите корректный JSON.']};}
   }
   continue;
  }
  const input=$(key);
  if(input){
   if(String(input.value).trim()==='')return {ok:false,changed:false,errors:[key+': поле не должно быть пустым.']};
   candidate[key]=Number(input.value);
  }
 }
 const errors=physics.validateVehicleParams(candidate);
 const gateEnabled=!!($('gateEnabled')&&$('gateEnabled').checked);
 if(gateEnabled){
  const requiredGateBounds=[
   ['gateWidthUncertainty',candidate.gateWidthUncertainty,0.001,2],
   ['gatePositionUncertainty',candidate.gatePositionUncertainty,0.001,2],
   ['gateHeadingUncertaintyDeg',candidate.gateHeadingUncertaintyDeg,0.001,45]
  ];
  for(const [key,value,min,max] of requiredGateBounds){
   if(!Number.isFinite(value)||value<min||value>max){
    errors.push(key+': при включённых воротах задайте положительную границу неопределённости от '+min+' до '+max+'. Ноль означает «не задано».');
   }
  }
  if(candidate.gateLintelEnabled){
   const lintelBounds=[
    ['gateLintelCameraHeightUncertainty',candidate.gateLintelCameraHeightUncertainty,0.001,2],
    ['gateLintelFloorUncertainty',candidate.gateLintelFloorUncertainty,0.001,1],
    ['gateLintelThicknessUncertainty',candidate.gateLintelThicknessUncertainty,0.001,1],
    ['gateLintelSpanUncertainty',candidate.gateLintelSpanUncertainty,0.001,2],
    ['gateLintelDepthUncertainty',candidate.gateLintelDepthUncertainty,0.001,2]
   ];
   for(const [key,value,min,max] of lintelBounds){
    if(!Number.isFinite(value)||value<min||value>max){
     errors.push(key+': для верхней перемычки задайте положительную границу неопределённости от '+min+' до '+max+' м.');
    }
   }
  }
 }
 if(!Array.isArray(candidate.verticalObstacles)){
  errors.push('verticalObstacles: ожидается JSON-массив вертикальных препятствий.');
 }else{
  errors.push(...planner.validateObstacles(candidate.verticalObstacles));
  candidate.verticalObstacles.forEach((obstacle,index)=>{
   if(!obstacle||!Number.isFinite(obstacle.zMin)||!Number.isFinite(obstacle.zMax)){
    errors.push('verticalObstacles['+index+']: задайте измеренные zMin и zMax.');
   }
  });
 }
 const limits={
  tractorMass:[1000,50000],
  engineForce:[1000,200000],
  trailerMass:[1000,100000],
  mu:[0.05,1.5],
  crr:[0,0.1],
  brakeDecel:[0.1,12],
  maxAccel:[0.1,3],
  tractorFrontTrack:[1,3.5],
  tractorSteeringTrack:[1,3.5],
  tractorRearTrack:[1,3.5],
  trailerTrack:[1,4.5],
  trailerAxle1Track:[0,4.5],
  trailerAxle2Track:[0,4.5],
  trailerAxle3Track:[0,4.5],
  trailerAxleSpacing12:[0,5],
  trailerAxleSpacing23:[0,5],
  tractorRearTireGroupWidth:[0,1.5],
  trailerTireGroupWidth:[0,1.5],
  trailerAxle1TireGroupWidth:[0,1.5],
  trailerAxle2TireGroupWidth:[0,1.5],
  trailerAxle3TireGroupWidth:[0,1.5],
  tractorRearTireCenterSpacing:[0,1.5],
  trailerAxle1TireCenterSpacing:[0,1.5],
  trailerAxle2TireCenterSpacing:[0,1.5],
  trailerAxle3TireCenterSpacing:[0,1.5],
  tireLength:[0.2,1.5],
  tireWidth:[0.1,0.8],
  tractorFrontOverhang:[1,10],
  tractorRearOverhang:[0.2,5],
  tractorBodyWidth:[1.8,3.2],
  gateWidth:[1,20],
  gateCenterX:[-100,100],
  gateCenterY:[-100,100],
  gateHeadingDeg:[-180,180],
  gateWidthUncertainty:[0,2],
  gatePositionUncertainty:[0,2],
  gateHeadingUncertaintyDeg:[0,45],
  gatePostWidth:[0.05,2],
  gateDepth:[0.05,10],
  gateLintelClearHeight:[0,15],
  gateLintelCameraHeightUncertainty:[0,2],
  gateLintelFloorUncertainty:[0,1],
  gateLintelThickness:[0,2],
  gateLintelThicknessUncertainty:[0,1],
  gateLintelSpan:[0,30],
  gateLintelSpanUncertainty:[0,2],
  gateLintelDepth:[0,10],
  gateLintelDepthUncertainty:[0,2],
  margin:[0,5],
  measurementUncertainty:[0,5],
  obstaclePositionUncertainty:[0,1],
  obstacleSizeUncertainty:[0,1],
  vehicleFootprintUncertainty:[0,0.5],
  wheelbaseUncertainty:[0,2],
  hitchOffsetUncertainty:[0,1],
  kingpinAxlesUncertainty:[0,5],
  trailerLengthUncertainty:[0,5],
  trailerKingpinToFrontUncertainty:[0,2],
  trailerWidthUncertainty:[0,1.5],
  trailerAxleSpacingUncertainty:[0,2],
  trailerAxleSpacing12Uncertainty:[0,2],
  trailerAxleSpacing23Uncertainty:[0,2],
  trailerAxle1TrackUncertainty:[0,1.5],
  trailerAxle2TrackUncertainty:[0,1.5],
  trailerAxle3TrackUncertainty:[0,1.5],
  tractorSteeringTrackUncertainty:[0,1.25],
  tractorFrontTrackUncertainty:[0,1.25],
  tractorRearTrackUncertainty:[0,1.25],
  trailerTrackUncertainty:[0,1.5],
  tractorRearTireGroupWidthUncertainty:[0,0.5],
  trailerTireGroupWidthUncertainty:[0,0.5],
  trailerAxle1TireGroupWidthUncertainty:[0,0.5],
  trailerAxle2TireGroupWidthUncertainty:[0,0.5],
  trailerAxle3TireGroupWidthUncertainty:[0,0.5],
  tractorRearTireCenterSpacingUncertainty:[0,0.5],
  trailerAxle1TireCenterSpacingUncertainty:[0,0.5],
  trailerAxle2TireCenterSpacingUncertainty:[0,0.5],
  trailerAxle3TireCenterSpacingUncertainty:[0,0.5],
  tireLengthUncertainty:[0,0.5],
  tireWidthUncertainty:[0,0.3],
  cabOffsetUncertainty:[0,2],
  cabFrontUncertainty:[0,1],
  cabRearUncertainty:[0,1],
  cabWidthUncertainty:[0,0.5],
  tractorFrontOverhangUncertainty:[0,2],
  tractorRearOverhangUncertainty:[0,1.5],
  tractorBodyWidthUncertainty:[0,1],
  mirrorOutboardUncertainty:[0,0.5],
  mirrorLongitudinalUncertainty:[0,1],
  mirrorLengthUncertainty:[0,0.5]
 };
 for(const [key,range] of Object.entries(limits)){
  const value=candidate[key];
  if(!Number.isFinite(value)||value<range[0]||value>range[1]){
   errors.push(key+': допустимый диапазон '+range[0]+'–'+range[1]+'.');
  }
 }
 if(Number.isFinite(candidate.gateWidthUncertainty)&&
    candidate.gateWidthUncertainty>=candidate.gateWidth-0.05){
  errors.push('gateWidthUncertainty: консервативная ширина проёма должна оставаться больше 0,05 м.');
 }
 if(candidate.gateLintelEnabled){
  if(!$('gateLintelAlignmentConfirmed')||!$('gateLintelAlignmentConfirmed').checked){
   errors.push('gateLintelAlignmentConfirmed: подтвердите, что перемычка центрирована, параллельна линии ворот и горизонтальна; иначе эту упрощённую призму использовать нельзя.');
  }
  if(!$('gateEnabled')||!$('gateEnabled').checked){
   errors.push('gateLintelEnabled: сначала включите учёт косяков ворот на карте.');
  }else{
   try{
    planner.createGateObstacles({
     centerX:candidate.gateCenterX,centerY:candidate.gateCenterY,
     clearWidth:candidate.gateWidth,postWidth:candidate.gatePostWidth,
     depth:candidate.gateDepth,heading:candidate.gateHeadingDeg*Math.PI/180,
     widthUncertainty:candidate.gateWidthUncertainty,
     positionUncertainty:candidate.gatePositionUncertainty,
     headingUncertaintyDeg:candidate.gateHeadingUncertaintyDeg,
     lintel:{
      clearHeight:candidate.gateLintelClearHeight,
      clearHeightUncertainty:candidate.gateLintelCameraHeightUncertainty,
      floorDatumUncertainty:candidate.gateLintelFloorUncertainty,
      thickness:candidate.gateLintelThickness,
      thicknessUncertainty:candidate.gateLintelThicknessUncertainty,
      span:candidate.gateLintelSpan,
      spanUncertainty:candidate.gateLintelSpanUncertainty,
      depth:candidate.gateLintelDepth,
      depthUncertainty:candidate.gateLintelDepthUncertainty
     }
    });
   }catch(error){
    errors.push('Верхняя перемычка: '+(error&&error.message?error.message:'некорректные измерения.'));
   }
  }
 }
 const parameterUncertaintyMap={
  wheelbaseUncertainty:'wheelbase',
  hitchOffsetUncertainty:'hitchOffset',
  kingpinAxlesUncertainty:'kingpinAxles',
  trailerLengthUncertainty:'trailerLength',
  trailerKingpinToFrontUncertainty:'trailerKingpinToFront',
  trailerWidthUncertainty:'trailerWidth',
  trailerAxleSpacingUncertainty:'trailerAxleSpacing',
  trailerAxleSpacing12Uncertainty:'trailerAxleSpacing12',
  trailerAxleSpacing23Uncertainty:'trailerAxleSpacing23',
  trailerAxle1TrackUncertainty:'trailerAxle1Track',
  trailerAxle2TrackUncertainty:'trailerAxle2Track',
  trailerAxle3TrackUncertainty:'trailerAxle3Track',
  tractorSteeringTrackUncertainty:'tractorSteeringTrack',
  tractorFrontTrackUncertainty:'tractorFrontTrack',
  tractorRearTrackUncertainty:'tractorRearTrack',
  trailerTrackUncertainty:'trailerTrack',
  tractorRearTireGroupWidthUncertainty:'tractorRearTireGroupWidth',
  trailerTireGroupWidthUncertainty:'trailerTireGroupWidth',
  trailerAxle1TireGroupWidthUncertainty:'trailerAxle1TireGroupWidth',
  trailerAxle2TireGroupWidthUncertainty:'trailerAxle2TireGroupWidth',
  trailerAxle3TireGroupWidthUncertainty:'trailerAxle3TireGroupWidth',
  tractorRearTireCenterSpacingUncertainty:'tractorRearTireCenterSpacing',
  trailerAxle1TireCenterSpacingUncertainty:'trailerAxle1TireCenterSpacing',
  trailerAxle2TireCenterSpacingUncertainty:'trailerAxle2TireCenterSpacing',
  trailerAxle3TireCenterSpacingUncertainty:'trailerAxle3TireCenterSpacing',
  tireLengthUncertainty:'tireLength',
  tireWidthUncertainty:'tireWidth',
  cabOffsetUncertainty:'cabOffset',
  cabFrontUncertainty:'cabFront',
  cabRearUncertainty:'cabRear',
  cabWidthUncertainty:'cabWidth',
  tractorFrontOverhangUncertainty:'tractorFrontOverhang',
  tractorRearOverhangUncertainty:'tractorRearOverhang',
  tractorBodyWidthUncertainty:'tractorBodyWidth',
  mirrorOutboardUncertainty:'mirrorOutboard',
  mirrorLongitudinalUncertainty:'mirrorLongitudinal',
  mirrorLengthUncertainty:'mirrorLength'
 };
 for(const [uncertaintyKey,parameterKey] of Object.entries(parameterUncertaintyMap)){
  const uncertainty=candidate[uncertaintyKey];
  if(!Number.isFinite(uncertainty)||uncertainty<0)continue;
  if(uncertainty===0)continue;
  const fallbackOnlyParameters=new Set([
   'trailerAxleSpacing12','trailerAxleSpacing23',
   'trailerAxle1Track','trailerAxle2Track','trailerAxle3Track',
   'tractorRearTireCenterSpacing','trailerAxle1TireCenterSpacing',
   'trailerAxle2TireCenterSpacing','trailerAxle3TireCenterSpacing'
  ]);
  if(fallbackOnlyParameters.has(parameterKey)&&candidate[parameterKey]===0){
   errors.push(uncertaintyKey+': сначала задайте индивидуальное измеренное значение '+parameterKey+'.');
   continue;
  }
  const dimensionLimits=physics.VEHICLE_PARAMETER_LIMITS[parameterKey];
  const lower=dimensionLimits?Math.max(dimensionLimits.min,candidate[parameterKey]-uncertainty):candidate[parameterKey]-uncertainty;
  const upper=dimensionLimits?Math.min(dimensionLimits.max,candidate[parameterKey]+uncertainty):candidate[parameterKey]+uncertainty;
  for(const endpoint of [lower,upper]){
   const endpointErrors=physics.validateVehicleParams({...candidate,[parameterKey]:endpoint});
   endpointErrors.forEach(message=>errors.push(uncertaintyKey+': крайнее значение '+parameterKey+'='+endpoint.toFixed(3)+' м: '+message));
  }
 }
 physics.validateTireGroupWidthUncertainty(candidate).forEach(message=>errors.push(message));
 if(errors.length)return {ok:false,changed:false,errors};
 const changed=Object.keys(cfg).some(key=>{
  const before=cfg[key],after=candidate[key];
  if((before&&typeof before==='object')||(after&&typeof after==='object'))return JSON.stringify(before)!==JSON.stringify(after);
  return !Object.is(before,after);
 });
 Object.assign(cfg,candidate);
 return {ok:true,changed,errors:[]};
}

function vehicleParameterBoundsFromSettings(){
 const mapping={
  wheelbaseUncertainty:'wheelbase',
  hitchOffsetUncertainty:'hitchOffset',
  kingpinAxlesUncertainty:'kingpinAxles',
  trailerLengthUncertainty:'trailerLength',
  trailerKingpinToFrontUncertainty:'trailerKingpinToFront',
  trailerWidthUncertainty:'trailerWidth',
  trailerAxleSpacingUncertainty:'trailerAxleSpacing',
  trailerAxleSpacing12Uncertainty:'trailerAxleSpacing12',
  trailerAxleSpacing23Uncertainty:'trailerAxleSpacing23',
  trailerAxle1TrackUncertainty:'trailerAxle1Track',
  trailerAxle2TrackUncertainty:'trailerAxle2Track',
  trailerAxle3TrackUncertainty:'trailerAxle3Track',
  tractorSteeringTrackUncertainty:'tractorSteeringTrack',
  tractorFrontTrackUncertainty:'tractorFrontTrack',
  tractorRearTrackUncertainty:'tractorRearTrack',
  trailerTrackUncertainty:'trailerTrack',
  tractorRearTireGroupWidthUncertainty:'tractorRearTireGroupWidth',
  trailerTireGroupWidthUncertainty:'trailerTireGroupWidth',
  trailerAxle1TireGroupWidthUncertainty:'trailerAxle1TireGroupWidth',
  trailerAxle2TireGroupWidthUncertainty:'trailerAxle2TireGroupWidth',
  trailerAxle3TireGroupWidthUncertainty:'trailerAxle3TireGroupWidth',
  tractorRearTireCenterSpacingUncertainty:'tractorRearTireCenterSpacing',
  trailerAxle1TireCenterSpacingUncertainty:'trailerAxle1TireCenterSpacing',
  trailerAxle2TireCenterSpacingUncertainty:'trailerAxle2TireCenterSpacing',
  trailerAxle3TireCenterSpacingUncertainty:'trailerAxle3TireCenterSpacing',
  tireLengthUncertainty:'tireLength',
  tireWidthUncertainty:'tireWidth',
  cabOffsetUncertainty:'cabOffset',
  cabFrontUncertainty:'cabFront',
  cabRearUncertainty:'cabRear',
  cabWidthUncertainty:'cabWidth',
  mirrorOutboardUncertainty:'mirrorOutboard',
  mirrorLongitudinalUncertainty:'mirrorLongitudinal',
  mirrorLengthUncertainty:'mirrorLength'
 };
 const bounds={};
 for(const [uncertaintyKey,parameterKey] of Object.entries(mapping)){
  const uncertainty=cfg[uncertaintyKey];
  if(uncertainty>0){
   const limits=physics.VEHICLE_PARAMETER_LIMITS[parameterKey];
   bounds[parameterKey]={
    min:limits?Math.max(limits.min,cfg[parameterKey]-uncertainty):cfg[parameterKey]-uncertainty,
    max:limits?Math.min(limits.max,cfg[parameterKey]+uncertainty):cfg[parameterKey]+uncertainty
   };
  }
 }
 return bounds;
}

function invalidateRouteForSettingsChange(){
 routeBuildToken++;
 cancelPlannerWorker('Параметры модели изменены; прежний расчёт отменён.');
 $('planRoute').disabled=false;
 if(routePlayback.active)stopRoutePlayback('Автопроезд остановлен: параметры модели изменились.');
 plannedRoute=[];
 alternativeRoutes=[];
 $('alternativeRouteSelect').style.display='none';
 $('alternativeRouteSelect').value='';
 routeGoal=null;
 routeAssessment=null;
 $('routeAdvice').textContent='Параметры модели изменены. Постройте маршрут заново.';
 $('difficultyAdvice').textContent='Сложность не рассчитана: прежняя оценка маршрута недействительна.';
}

function resetState(){
 state={x:0,y:0,psi:0,psiT:0};
 speed=0;
 drive=0;
 braking=false;
 distance=0;
 trace=[{...state,ax:axlePoint(state).x,ay:axlePoint(state).y}];
 camera={x:5.5,y:0};
}

function hitchPoint(q){return physics.hitchPoint(q,cfg);}

function axlePoint(q){return physics.axlePoint(q,cfg);}

function derivative(q,v,delta){
 return physics.derivative(q,v,delta,cfg);
}

function rk4(q,v,delta,dt){
 return physics.rk4(q,v,delta,dt,cfg);
}

function toScreen(p){
 return{
  x:canvas.width/2+(p.x-camera.x)*SCALE,
  y:canvas.height/2+(p.y-camera.y)*SCALE
 };
}

function localPoint(origin,angle,longitudinal,lateral=0){
 return{
  x:origin.x+longitudinal*Math.cos(angle)-lateral*Math.sin(angle),
  y:origin.y+longitudinal*Math.sin(angle)+lateral*Math.cos(angle)
 };
}

function drawLine(points,color,width=2,dash=[]){
 ctx.beginPath();
 points.forEach((p,i)=>{
  const z=toScreen(p);
  if(i===0)ctx.moveTo(z.x,z.y);
  else ctx.lineTo(z.x,z.y);
 });
 ctx.strokeStyle=color;
 ctx.lineWidth=width;
 ctx.setLineDash(dash);
 ctx.stroke();
 ctx.setLineDash([]);
}

function drawPolygon(points,fill){
 const p=points.map(toScreen);
 ctx.beginPath();
 p.forEach((z,i)=>{
  if(i===0)ctx.moveTo(z.x,z.y);
  else ctx.lineTo(z.x,z.y);
 });
 ctx.closePath();
 ctx.fillStyle=fill;
 ctx.fill();
 ctx.strokeStyle='#dceaf5';
 ctx.lineWidth=2;
 ctx.stroke();
}

function drawBody(origin,angle,front,rear,width,fill){
 drawPolygon([
  localPoint(origin,angle,front,width/2),
  localPoint(origin,angle,rear,width/2),
  localPoint(origin,angle,rear,-width/2),
  localPoint(origin,angle,front,-width/2)
 ],fill);
}

function bodyCorners(origin,angle,front,rear,width){
 return[
  localPoint(origin,angle,front,width/2),
  localPoint(origin,angle,rear,width/2),
  localPoint(origin,angle,rear,-width/2),
  localPoint(origin,angle,front,-width/2)
 ];
}

function vehiclePolygons(q){return physics.vehiclePolygons(q,{...cfg,footprintUncertainty:cfg.vehicleFootprintUncertainty});}
function nominalVehiclePolygons(q){return physics.vehiclePolygons(q,{...cfg,footprintUncertainty:0});}
function vehicleCollisionParts(q,uncertainty=cfg.vehicleFootprintUncertainty,
 steeringAngle=Number.isFinite(q&&q.steer)?q.steer:steer){
 return physics.vehicleCollisionParts(q,{...cfg,footprintUncertainty:uncertainty},steeringAngle);
}

function obstaclePolygon(o){
 if(Array.isArray(o.nominalPolygon))return o.nominalPolygon.map(point=>({x:point.x,y:point.y}));
 if(Array.isArray(o.polygon))return o.polygon.map(point=>({x:point.x,y:point.y}));
 return[
  {x:o.x-o.w/2,y:o.y-o.h/2},
  {x:o.x+o.w/2,y:o.y-o.h/2},
  {x:o.x+o.w/2,y:o.y+o.h/2},
  {x:o.x-o.w/2,y:o.y+o.h/2}
 ];
}

function polygonsIntersect(a,b){
 const axes=[];
 for(const p of [a,b]){
  for(let i=0;i<p.length;i++){
   const n=p[(i+1)%p.length];
   const c=p[i];
   const dx=n.x-c.x;
   const dy=n.y-c.y;
   const len=Math.hypot(dx,dy)||1;
   axes.push({x:-dy/len,y:dx/len});
  }
 }
 for(const axis of axes){
  const pa=a.map(p=>p.x*axis.x+p.y*axis.y);
  const pb=b.map(p=>p.x*axis.x+p.y*axis.y);
  if(Math.max(...pa)<Math.min(...pb)||
     Math.max(...pb)<Math.min(...pa))return false;
 }
 return true;
}

function pointSegmentDistance(p,a,b){
 const dx=b.x-a.x;
 const dy=b.y-a.y;
 const l2=dx*dx+dy*dy;
 const t=l2?clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0,1):0;
 return Math.hypot(p.x-(a.x+t*dx),p.y-(a.y+t*dy));
}

function polygonDistance(a,b){
 if(polygonsIntersect(a,b))return 0;
 let best=Infinity;
 for(const p of a)
  for(let i=0;i<b.length;i++)
   best=Math.min(best,pointSegmentDistance(p,b[i],b[(i+1)%b.length]));
 for(const p of b)
  for(let i=0;i<a.length;i++)
   best=Math.min(best,pointSegmentDistance(p,a[i],a[(i+1)%a.length]));
 return best;
}

function obstacleEnvelopePolygon(obstacle){
 const nominal=obstaclePolygon(obstacle);
 const expansion=Math.max(0,cfg.obstaclePositionUncertainty)+Math.max(0,cfg.obstacleSizeUncertainty)/2;
 if(obstacle.kind==='camera-obstacle'&&Array.isArray(obstacle.nominalPolygon)){
  const pose=planner.inflateObstaclePoseUncertainty(nominal,obstacle.anchor,obstacle.mapPoseUncertainty||{});
  const totalInflation=Math.max(0,obstacle.measurementInflation||0)+pose.totalInflation+Math.SQRT2*expansion;
  return planner.inflateConvexPolygon(nominal,totalInflation);
 }
 if(expansion<=0)return nominal;
 return Array.isArray(obstacle.polygon)?
  planner.inflateConvexPolygon(nominal,Math.SQRT2*expansion):
  obstaclePolygon({...obstacle,w:obstacle.w+2*expansion,h:obstacle.h+2*expansion});
}
function clearanceMeasurementReserve(){return 2*Math.max(0,cfg.measurementUncertainty);}

function assessScene(q=state){
 const nominalParts=vehicleCollisionParts(q,0);
 const boundedParts=vehicleCollisionParts(q,cfg.vehicleFootprintUncertainty);
 let nearest={
  distance:Infinity,geometryLowerBound:Infinity,effectiveDistance:Infinity,
  name:'нет заданных препятствий',vehiclePart:null,bodyIndex:null,component:null
 };
 let collision=false,collisionDetail=null;

 if(showObstacles){
  for(const obstacle of sceneObstacles()){
   const nominalObstacle=obstaclePolygon(obstacle);
   const boundedObstacle=obstacleEnvelopePolygon(obstacle);
   for(let i=0;i<boundedParts.length;i++){
    for(let j=0;j<boundedParts[i].components.length;j++){
     const nominalComponent=nominalParts[i].components[j];
     const boundedComponent=boundedParts[i].components[j];
     const nominalDistance=polygonDistance(nominalComponent.polygon,nominalObstacle);
     const geometryLowerBound=polygonDistance(boundedComponent.polygon,boundedObstacle);
     const effectiveDistance=geometryLowerBound-clearanceMeasurementReserve();
     const detail={
      distance:nominalDistance,geometryLowerBound,effectiveDistance,
      name:obstacle.name,vehiclePart:boundedParts[i].vehiclePart,
      bodyIndex:boundedParts[i].bodyIndex,component:boundedComponent.name
     };
     if(effectiveDistance<nearest.effectiveDistance)nearest=detail;
     if(nominalDistance===0){
      collision=true;
      if(!collisionDetail)collisionDetail=detail;
     }
    }
   }
  }
 }
 return{
  nearest,collision,collisionDetail,
  obstacleChecksEnabled:showObstacles,
  beta:Math.abs(norm(q.psi-q.psiT))*180/Math.PI
 };
}

function updateAssistant(){
 const el=$('assistantAdvice');
 let message='';
 let color='#83d6a8';

 if(!helperEnabled){
  message='Помощник выключен. Контроль препятствий остаётся доступен только визуально.';
  color='#a9bacb';
 }else if(!showObstacles){
  message='Проверка препятствий отключена. Свободный путь не подтверждён; предупреждения по объектам неактивны. Включите препятствия перед оценкой или виртуальным проездом.';
  color='#ffd18b';
 }else{
  const a=assessScene();
  const effectiveClearance=a.nearest.effectiveDistance;
  const partText=a.nearest.vehiclePart?
   'Часть автопоезда: '+a.nearest.vehiclePart+'. '+
   (a.nearest.component?'Компонент: '+a.nearest.component+'. ':''):'';

  if(a.collision){
   const hit=a.collisionDetail||a.nearest;
   message='Стоп! Расчётные контуры '+(hit.vehiclePart||'автопоезда')+
    (hit.component?' · '+hit.component:'')+
    ' пересекаются с препятствием '+hit.name+'. Остановитесь и измените траекторию.';
   color='#ff8585';
  }else if(Number.isFinite(effectiveClearance)&&effectiveClearance<=0){
   message='Заданная оболочка погрешности пересекается с контуром. '+partText+
    'Объект: '+a.nearest.name+'. Номинальный зазор '+a.nearest.distance.toFixed(2)+
    ' м; нижняя оценка после допусков '+effectiveClearance.toFixed(2)+
    ' м. Свободный проход не подтверждён; проверьте фактическое пространство.';
   color='#ff8585';
  }else if(a.beta>=ARTICULATION_STOP_DEMO_DEG){
   message='Осторожно! Большой угол сочленения '+
    a.beta.toFixed(0)+
    ' градусов. Уменьшите угол складывания. Не продолжайте резкий задний ход.';
   color='#ffd18b';
  }else if(a.beta>=ARTICULATION_WARNING_DEG){
   message='Предупреждение: угол сочленения достиг '+a.beta.toFixed(0)+
    '°. Это программный порог внимания, а не предел устойчивости. Снизьте скорость и проверьте положение полуприцепа.';
   color='#ffd18b';
  }else if(effectiveClearance<0.45){
   message='Очень близко к объекту '+a.nearest.name+'. '+partText+
    'Геометрический зазор '+a.nearest.distance.toFixed(2)+' м; после заданного резерва неопределённости остаётся '+
    Math.max(0,effectiveClearance).toFixed(2)+' м. Остановитесь и проверьте пространство.';
   color='#ff8585';
  }else if(effectiveClearance<1.0){
   message='Внимание! Препятствие '+a.nearest.name+'. '+partText+
    'Расчётный зазор '+a.nearest.distance.toFixed(2)+
    ' м; после заданного резерва остаётся '+Math.max(0,effectiveClearance).toFixed(2)+' м. Маневрируйте медленно.';
   color='#ffd18b';
  }else if(Number.isFinite(a.nearest.distance)){
   message='В текущей позе расчётные контуры не пересекают заданные объекты. Ближайший объект: '+
    a.nearest.name+'. '+partText+'Расчётный зазор '+a.nearest.distance.toFixed(2)+
    ' м; после заданного резерва неопределённости остаётся '+
    Math.max(0,effectiveClearance).toFixed(2)+' м. Контролируйте зеркала и фактическое пространство.';
  }else{
   message='В тестовой карте нет заданных препятствий. Проверка не подтверждает свободный путь в реальном пространстве.';
   color='#ffd18b';
  }
 }

 if(helperEnabled&&showObstacles&&cfg.mirrorOutboard<=0){
  message+=' Геометрия зеркал не задана: зеркала не входят в проверку зазоров. Измерьте их вынос и настройте оболочку в параметрах.';
  if(color==='#83d6a8')color='#ffd18b';
 }
 if(el){
  el.textContent=message;
  el.style.color=color;
 }
}
function sweptClearanceColor(pathIndex,polygons,obstaclePolygons){
 let nearest=Infinity;
 for(const body of polygons){
  for(const obstacle of obstaclePolygons){
   nearest=Math.min(nearest,polygonDistance(body,obstacle));
  }
 }
 const reserve=2*Math.max(0,cfg.measurementUncertainty);
 if(!showObstacles)return 'rgba(125,165,205,0.28)';
 const effective=nearest-reserve;
 if(effective<0.45)return 'rgba(255,92,92,0.42)';
 if(effective<1.0)return 'rgba(255,190,82,0.38)';
 return 'rgba(52,190,160,0.34)';
}

function clearanceForVehicleParts(q=state){
 const nominalParts=vehicleCollisionParts(q,0);
 const boundedParts=vehicleCollisionParts(q,cfg.vehicleFootprintUncertainty);
 const obstacles=showObstacles?sceneObstacles().map(obstacle=>({
  name:obstacle.name,
  nominalPolygon:obstaclePolygon(obstacle),
  boundedPolygon:obstacleEnvelopePolygon(obstacle)
 })):[];
 return boundedParts.map((group,index)=>{
  let nearest={
   distance:Infinity,geometryLowerBound:Infinity,effective:Infinity,
   name:'нет препятствий',vehiclePart:group.vehiclePart,component:null
  };
  for(const obstacle of obstacles){
   for(let componentIndex=0;componentIndex<group.components.length;componentIndex++){
    const nominalComponent=nominalParts[index].components[componentIndex];
    const boundedComponent=group.components[componentIndex];
    const distance=polygonDistance(nominalComponent.polygon,obstacle.nominalPolygon);
    const geometryLowerBound=polygonDistance(boundedComponent.polygon,obstacle.boundedPolygon);
    const effective=geometryLowerBound-clearanceMeasurementReserve();
    if(effective<nearest.effective){
     nearest={
      distance,geometryLowerBound,effective,name:obstacle.name,
      vehiclePart:group.vehiclePart,component:boundedComponent.name
     };
    }
   }
  }
  const color=!showObstacles?'#8ba5b8':nearest.effective<0.45?'#ff6262':nearest.effective<1?'#ffc45f':'#42d6a0';
  return{index,bodyIndex:group.bodyIndex,...nearest,color};
 });
}

// Read-only diagnostics for regression tests and external inspection; it never changes simulator state.
window.TruckVisionGetClearanceDiagnostics=()=>({
 obstacleChecksEnabled:showObstacles,
 parts:clearanceForVehicleParts(state),
 scene:assessScene(state),
 verticalClearance:{
  obstacleCount:sceneObstacles().filter(obstacle=>Number.isFinite(obstacle.zMin)&&Number.isFinite(obstacle.zMax)).length,
  manualObstacleCount:(cfg.verticalObstacles||[]).length,
  configuredProfileClasses:Object.keys(cfg.vehicleVerticalProfiles||{}),
  suspensionTravelStatus:physics.suspensionTravelStatus(cfg)
 },
 cameraObstacles:cameraObstacles.map(obstacle=>({
  name:obstacle.name,
  measuredWidth:obstacle.measuredWidth,
  measuredDepth:obstacle.measuredDepth,
  sizeUncertainty:{sigmaWidth:obstacle.sigmaWidth,sigmaDepth:obstacle.sigmaDepth,radial2Sigma:obstacle.measurementInflation},
  mapPoseUncertainty:{positionX:obstacle.positionUncertaintyX,positionY:obstacle.positionUncertaintyY,headingDeg:obstacle.headingUncertaintyDeg},
  envelopeInflation:{position:obstacle.positionInflation,heading:obstacle.headingInflation,totalPose:obstacle.poseInflation,measurement:obstacle.measurementInflation}
 })),
 uncertainty:{
  measurement:cfg.measurementUncertainty,
  obstaclePosition:cfg.obstaclePositionUncertainty,
  obstacleSize:cfg.obstacleSizeUncertainty,
  vehicleFootprint:cfg.vehicleFootprintUncertainty,
  cab:{offset:cfg.cabOffset,front:cfg.cabFront,rear:cfg.cabRear,width:cfg.cabWidth},
  mirror:{outboard:cfg.mirrorOutboard,longitudinal:cfg.mirrorLongitudinal,length:cfg.mirrorLength,enabled:cfg.mirrorOutboard>0}
 }
});

function drawSweptCorridor(path,kind){
 if(!Array.isArray(path)||path.length<2)return;
 const stride=Math.max(1,Math.ceil(path.length/240));
 const samples=[];
 for(let i=0;i<path.length;i+=stride){
  const q=path[i];
  if(q&&[q.x,q.y,q.psi,q.psiT].every(Number.isFinite))samples.push(q);
 }
 const last=path[path.length-1];
 if(last&&samples[samples.length-1]!==last&&[last.x,last.y,last.psi,last.psiT].every(Number.isFinite))samples.push(last);
 if(samples.length<2)return;
 const obstaclePolygons=showObstacles?sceneObstacles().map(obstacleEnvelopePolygon):[];
 let previous=null;
 for(const q of samples){
  const polygons=vehicleCollisionParts(q).flatMap(group=>group.components.map(component=>component.polygon));
  const fill=sweptClearanceColor(0,polygons,obstaclePolygons);
  polygons.forEach((poly,index)=>{
   const points=poly.map(toScreen);
   ctx.beginPath();
   points.forEach((point,i)=>i?ctx.lineTo(point.x,point.y):ctx.moveTo(point.x,point.y));
   ctx.closePath();
   ctx.fillStyle=fill;
   ctx.fill();
   if(previous&&previous[index]&&previous[index].length===poly.length){
    const before=previous[index];
    for(let corner=0;corner<poly.length;corner++){
     const next=(corner+1)%poly.length;
     const quad=[before[corner],before[next],poly[next],poly[corner]].map(toScreen);
     ctx.beginPath();
     quad.forEach((point,i)=>i?ctx.lineTo(point.x,point.y):ctx.moveTo(point.x,point.y));
     ctx.closePath();
     ctx.fillStyle=fill;
     ctx.fill();
    }
   }
   ctx.beginPath();
   points.forEach((point,i)=>i?ctx.lineTo(point.x,point.y):ctx.moveTo(point.x,point.y));
   ctx.closePath();
   ctx.strokeStyle=kind==='planned'?'rgba(112,238,205,0.95)':'rgba(135,190,225,0.68)';
   ctx.lineWidth=kind==='planned'?2.4:1.6;
   ctx.stroke();
  });
  previous=polygons;
 }
 const label=toScreen(samples[Math.floor(samples.length*0.55)]);
 ctx.save();
 ctx.font='bold 12px system-ui';
 ctx.fillStyle=kind==='planned'?'#7df0c9':'#a9cde5';
 ctx.fillText(kind==='planned'?'РАСЧЁТНЫЙ КОРИДОР':'ПРОЙДЕННЫЙ ГАБАРИТ',label.x+8,label.y-18);
 ctx.restore();
}

function drawEnvelopePolygon(poly,color){
 const points=poly.map(toScreen);
 ctx.beginPath();
 points.forEach((point,index)=>index?ctx.lineTo(point.x,point.y):ctx.moveTo(point.x,point.y));
 ctx.closePath();
 ctx.strokeStyle=color;
 ctx.lineWidth=1.5;
 ctx.setLineDash([5,4]);
 ctx.stroke();
 ctx.setLineDash([]);
}

function drawObstacles(){
 if(!showObstacles)return;
 for(const o of sceneObstacles()){
  const nominal=obstaclePolygon(o);
  const expanded=obstacleEnvelopePolygon(o);
  drawEnvelopePolygon(expanded,'#ffd18b');
  const p=nominal.map(toScreen);
  ctx.beginPath();
  p.forEach((z,i)=>i?ctx.lineTo(z.x,z.y):ctx.moveTo(z.x,z.y));
  ctx.closePath();
  ctx.fillStyle=o.kind==='gate-jamb'?'#9a6635':'#8b4c43';
  ctx.fill();
  ctx.strokeStyle=o.kind==='gate-jamb'?'#ffe0a8':'#ffc0a8';
  ctx.lineWidth=2;
  ctx.stroke();
  const center=nominal.reduce((sum,point)=>({x:sum.x+point.x/nominal.length,y:sum.y+point.y/nominal.length}),{x:0,y:0});
  const label=toScreen(center);
  ctx.fillStyle='#fff';
  ctx.font='12px system-ui';
  const confidenceText=o.kind==='perception-obstacle'?' · '+Math.round(o.confidence*100)+'% · '+o.source:'';
  ctx.fillText(o.name+confidenceText,label.x+5,label.y-5);
 }
}

function routeBlocked(ix,iy){
 if(!showObstacles)return false;
 const x=ix*0.5;
 const y=iy*0.5;
 const pad=2.2;

 const point={x,y};
 return sceneObstacles().some(o=>{
  const poly=obstaclePolygon(o);
  let inside=false;
  for(let i=0,j=poly.length-1;i<poly.length;j=i++){
   const a=poly[i],b=poly[j];
   const crosses=((a.y>y)!==(b.y>y))&&
    x<(b.x-a.x)*(y-a.y)/(b.y-a.y)+a.x;
   if(crosses)inside=!inside;
  }
  if(inside)return true;
  for(let i=0;i<poly.length;i++){
   if(pointSegmentDistance(point,poly[i],poly[(i+1)%poly.length])<=pad)return true;
  }
  return false;
 });
}

function planAStar(start,goal){
 const step=0.5;
 const cell=p=>({
  x:Math.round(p.x/step),
  y:Math.round(p.y/step)
 });

 const s=cell(start);
 const g=cell(goal);
 const key=p=>p.x+','+p.y;

 const open=[{
  ...s,
  f:Math.hypot(g.x-s.x,g.y-s.y)
 }];

 const came=new Map();
 const cost=new Map([[key(s),0]]);
 const dirs=[
  [-1,-1],[-1,0],[-1,1],
  [0,-1],[0,1],
  [1,-1],[1,0],[1,1]
 ];

 let found=false;
 let loops=0;

 while(open.length&&loops++<40000){
  open.sort((a,b)=>a.f-b.f);
  const cur=open.shift();
  const ck=key(cur);

  if(cur.x===g.x&&cur.y===g.y){
   found=true;
   break;
  }

  for(const [dx,dy] of dirs){
   const n={x:cur.x+dx,y:cur.y+dy};

   // Prevent diagonal moves that squeeze through the corner of blocked cells.
   if(dx!==0&&dy!==0&&
      (routeBlocked(cur.x+dx,cur.y)||routeBlocked(cur.x,cur.y+dy)))continue;

   if(n.x< -24||n.x>120||n.y< -56||n.y>56||
      routeBlocked(n.x,n.y))continue;

   const nk=key(n);
   const ng=cost.get(ck)+(dx&&dy?Math.SQRT2:1);

   if(ng<(cost.get(nk)??Infinity)){
    cost.set(nk,ng);
    came.set(nk,ck);
    open.push({
     ...n,
     f:ng+Math.hypot(g.x-n.x,g.y-n.y)
    });
   }
  }
 }

 if(!found)return null;

 let k=key(g);
 const points=[];

 while(k){
  const [x,y]=k.split(',').map(Number);
  points.push({x:x*step,y:y*step});
  if(k===key(s))break;
  k=came.get(k);
 }

 points.reverse();

 const compact=[];

 for(let i=0;i<points.length;i++){
  if(i===0||i===points.length-1){
   compact.push(points[i]);
   continue;
  }

  const a=points[i-1];
  const b=points[i];
  const c=points[i+1];

  if(
   Math.sign(b.x-a.x)!==Math.sign(c.x-b.x)||
   Math.sign(b.y-a.y)!==Math.sign(c.y-b.y)
  )compact.push(b);
 }

 return compact;
}

function routeMetrics(path,minimumClearanceLowerBound=Infinity,sweptClearanceDetail=null,trajectoryClearance=null){
 let length=0,gearChanges=0,steeringChanges=0,initialSteeringActions=0,reverseDistance=0,reverseSegments=0;
 let minClearance=Infinity,minClearanceDetail=null,maxBeta=0,collision=false;
 const routeObstacles=showObstacles?sceneObstacles():[];
 const checkClearance=showObstacles?planner.createClearanceChecker(routeObstacles,cfg):null;
 let previousGear=path[0]?.gear||1,previousSteer=path[0]?.steer||0;
 let previousGearKnown=path[0]?.gearKnown!==false;
 let previousSteerKnown=path[0]?.steerKnown!==false,wasReverse=false;
 for(let i=0;i<path.length;i++){
  const q=path[i];
  const currentGear=q.gear===-1?-1:1;
  const currentSteer=Number.isFinite(q.steer)?q.steer:0;
  if(i>0){
   const prev=path[i-1];
   const segmentLength=Math.hypot(q.x-prev.x,q.y-prev.y);
   length+=segmentLength;
   const reversing=currentGear<0;
   if(reversing){
    reverseDistance+=segmentLength;
    if(!wasReverse)reverseSegments++;
   }
   if(previousGearKnown&&currentGear!==previousGear)gearChanges++;
   if(Math.abs(currentSteer-previousSteer)>0.12){
    if(previousSteerKnown)steeringChanges++;
    else initialSteeringActions++;
   }
   wasReverse=reversing;
   previousGearKnown=true;
   previousSteerKnown=true;
  }else wasReverse=currentGear<0;
  previousGear=currentGear;
  previousSteer=currentSteer;
  if(showObstacles){
   const c=checkClearance(q);
   if(c.distance<minClearance){
    minClearance=c.distance;
    minClearanceDetail=Number.isFinite(c.distance)?
     {...c,state:{x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear:q.gear,steer:q.steer}}:null;
   }
   if(c.distance<=1e-9)collision=true;
  }
 }
 const articulationPeak=planner.articulationExposure(path);
 maxBeta=articulationPeak.maxAngleDegrees;
 const scrubExposure=physics.trailerAxleScrubExposure(path,cfg,0.7);
 const wheelTrajectory=physics.trailerWheelTrajectoryDiagnostics(path,cfg);
 let tireTrajectory=null,tireTrajectoryError=null;
 try{
  if(typeof physics.wheelTireTrajectoryDiagnostics==='function'){
   tireTrajectory=physics.wheelTireTrajectoryDiagnostics(path,cfg);
  }else tireTrajectoryError='Индивидуальная диагностика траекторий покрышек недоступна.';
 }catch(error){
  tireTrajectoryError=error&&error.message?error.message:String(error);
 }
 const wheelTrackSummary=wheelTrajectory.axles.map((axle,index)=>{
  const segmentAxles=wheelTrajectory.segments.map(segment=>segment.axles[index]);
  const differences=segmentAxles.map(segment=>segment?.outerMinusInner).filter(Number.isFinite);
  const innerSides=[...new Set(segmentAxles.map(segment=>segment?.innerSide).filter(Boolean))];
  const innerSideSummary=innerSides.length===0?'не определяется на прямых сегментах':
   (innerSides.length===1?(innerSides[0]==='left'?'слева':'справа'):'меняется по участкам');
  return {
   axleIndex:axle.axleIndex,
   centerDistance:axle.center.sampledTravel,
   leftWheelDistance:axle.left.sampledTravel,
   rightWheelDistance:axle.right.sampledTravel,
   integratedCenterDistance:axle.center.integratedTravelEstimate,
   integratedLeftWheelDistance:axle.left.integratedTravelEstimate,
   integratedRightWheelDistance:axle.right.integratedTravelEstimate,
   integratedCoveredDistance:axle.center.integratedTravelCoveredDistance,
   innerSideSummary,
   maxOuterMinusInnerPerSample:differences.length?Math.max(0,...differences):0
  };
 });
 let wheelClearance=null,wheelClearanceError=null;
 if(showObstacles){
  try{
   const wheelObstacles=routeObstacles.map(obstacle=>({
    name:obstacle.name||'препятствие',polygon:obstacleEnvelopePolygon(obstacle),
    ...(Number.isFinite(obstacle.zMin)&&Number.isFinite(obstacle.zMax)?{
     zMin:obstacle.zMin,zMax:obstacle.zMax,zUncertainty:obstacle.zUncertainty
    }: {})
   }));
   wheelClearance=planner.analyzeWheelTrajectoryClearance(path,wheelObstacles,{
    ...cfg,footprintUncertainty:cfg.vehicleFootprintUncertainty
   });
  }catch(error){
   wheelClearanceError=error&&error.message?error.message:String(error);
  }
 }
 const limitingWheelClearance=wheelClearance?.limitingWheel||null;
 const intermediateWheelContact=Array.isArray(wheelClearance?.wheels)&&
  wheelClearance.wheels.some(wheel=>
   Number.isFinite(wheel.minimumIntermediatePolygonClearance)&&
   wheel.minimumIntermediatePolygonClearance<=1e-9
  );
 if(intermediateWheelContact)collision=true;
 const wheelLowerBounds=Array.isArray(wheelClearance?.wheels)?
  wheelClearance.wheels.map(wheel=>wheel.minimumClearanceLowerBound).filter(Number.isFinite):[];
 const turns=gearChanges+steeringChanges+initialSteeringActions;
 const clearanceByPart=Array.isArray(trajectoryClearance?.parts)?trajectoryClearance.parts:[];
 const partLowerBounds=clearanceByPart.map(part=>part.minimumClearanceLowerBound).filter(Number.isFinite);
 const clearanceForAssessment=Math.min(minClearance,minimumClearanceLowerBound,...partLowerBounds,...wheelLowerBounds);
 const limitingPartClearance=clearanceByPart.reduce((best,part)=>
  !best||part.minimumClearanceLowerBound<best.minimumClearanceLowerBound?part:best,null);
 const effectiveClearance=clearanceForAssessment-2*Math.max(0,cfg.measurementUncertainty);
 const finalErrors=planner.goalErrors(path[path.length-1],routeGoal);
 const terminalHeadingMismatch=finalErrors.headingError!==null&&finalErrors.headingError>routeGoal.headingTolerance;
 const terminalArticulationMismatch=finalErrors.articulation>routeGoal.maxTerminalArticulation;
 const reached=path.length>1&&finalErrors.position<=routeGoal.positionTolerance&&
  !terminalHeadingMismatch&&!terminalArticulationMismatch;
 const finalHeadingErrorDeg=(finalErrors.headingError??0)*180/Math.PI;
 const finalArticulationDeg=finalErrors.articulation*180/Math.PI;
 const wheelClearanceIncomplete=!!wheelClearance&&wheelClearance.segmentsWithoutIntermediateKinematicSamples>0;
 const clearanceInsufficient=!!wheelClearanceError||wheelClearanceIncomplete||
  (Number.isFinite(effectiveClearance)&&effectiveClearance<0.35);
 const articulationWarning=maxBeta>=ARTICULATION_WARNING_DEG;
 const articulationBlocked=maxBeta>=ARTICULATION_STOP_DEMO_DEG;
 const score=gearChanges*1.2+(steeringChanges+initialSteeringActions)*0.4+(reverseDistance>0?1.5:0)+(articulationWarning?3:maxBeta>30?1.5:0)+
  (effectiveClearance<0.35?4:effectiveClearance<0.75?2:effectiveClearance<1.5?1:0)+(!reached?3:0)+(collision?8:0);
 const reasons=[];
 const vehicleGeometryCalibration=typeof physics.vehicleGeometryCalibrationStatus==='function'?
  physics.vehicleGeometryCalibrationStatus(cfg):
  {status:'diagnostic-unavailable',missingBounds:[],positiveBoundsPresent:false,measurementProvenanceVerified:false};
 if(vehicleGeometryCalibration.missingBounds.length){
  const missingNames=vehicleGeometryCalibration.missingBounds.map(item=>item.label).filter(Boolean);
  const missingSummary=missingNames.slice(0,8).join(', ')+
   (missingNames.length>8?' и ещё '+(missingNames.length-8)+' параметров':'');
  reasons.push('геометрия модели Volvo FH 4×2 и трёхосного полуприцепа предварительная: нет положительных измерительных допусков для '+missingSummary+
   '. Нулевой допуск означает «не задан», а не точность; рекомендация стороны и направления начала манёвра не подтверждена для реального автопоезда.');
 }else if(vehicleGeometryCalibration.status==='positive-bounds-present'){
  reasons.push('положительные границы неопределённости ключевой геометрии введены оператором, но источник и методика измерений приложением не проверяются');
 }else{
  reasons.push('диагностика калибровки геометрии недоступна; рекомендация остаётся предварительной');
 }
 if(Array.isArray(cfg.auxiliaryEnvelopes)&&cfg.auxiliaryEnvelopes.length){
  reasons.push('дополнительные измеренные оболочки осевых узлов: '+cfg.auxiliaryEnvelopes.length+'; их размеры и опорные точки заданы вручную');
 }else{
  reasons.push('оболочки ступиц, подвески и кронштейнов не откалиброваны и не входят отдельно в проверку столкновений');
 }
 const verticalObstacleCount=sceneObstacles().filter(obstacle=>Number.isFinite(obstacle.zMin)&&Number.isFinite(obstacle.zMax)).length;
 if(verticalObstacleCount){
  const unverifiedVerticalChecks=(trajectoryClearance?.verticalUnverifiedChecks||0)+(wheelClearance?.verticalUnverifiedChecks||0);
  if(wheelClearance?.roadSurfaceStatus==='road-surface-unverified-horizontal-only'){
   reasons.push('вертикальные препятствия: '+verticalObstacleCount+'; плоскость дороги не откалибрована, поэтому вертикальное разделение отключено и все такие препятствия учитываются только по горизонтальной 2D-проекции');
  }else if(unverifiedVerticalChecks>0){
   reasons.push('вертикальные препятствия: '+verticalObstacleCount+'; для части компонентов высота не откалибрована, поэтому используется только консервативная нижняя оценка по горизонтальной проекции');
  }else{
   const suspensionStatus=physics.suspensionTravelStatus(cfg);
   if(suspensionStatus==='bounded-rigid-plane-support-envelope'){
    reasons.push('вертикальные препятствия проверены по измеренным профилям; учтён ограниченный статический диапазон heave/pitch/roll по независимым интервалам опорных точек; динамика, демпфирование, нелинейная кинематика подвески и изменение под нагрузкой не моделируются');
   }else{
    reasons.push('вертикальные препятствия проверены по измеренным профилям, но диапазоны хода подвески не откалиброваны; изменение высоты и наклона кузова не подтверждено');
   }
  }
  if(wheelClearance?.roadSurfaceStatus==='road-surface-unverified-horizontal-only'){
   reasons.push('вертикальное разделение отключено: нет откалиброванной плоскости дороги; препятствия остаются горизонтальными 2D-оболочками');
  }else if(wheelClearance?.roadSurfaceStatus==='configured-planar-surface-with-bounds'){
   reasons.push('учтена заданная плоскость дороги с указанными границами неопределённости; локальные неровности вне этого диапазона не моделируются');
  }
 }
 if(reverseDistance>0)reasons.push('задний ход: '+reverseSegments+' участков, '+reverseDistance.toFixed(1)+' м');
 if(gearChanges)reasons.push('переключений передачи: '+gearChanges);
 if(steeringChanges)reasons.push('изменений рулевого режима: '+steeringChanges);
 if(initialSteeringActions)reasons.push('первое рулевое действие при начале маршрута');
 if(collision)reasons.push('пересечение с препятствием');
 if(intermediateWheelContact)reasons.push('промежуточная RK4-выборка контура шины пересекает препятствие; виртуальный автопроезд заблокирован');
 if(!reached)reasons.push('конечное положение не достигнуто в заданных допусках');
 if(terminalHeadingMismatch)reasons.push('конечный курс тягача вне допуска');
 if(terminalArticulationMismatch)reasons.push('полуприцеп не выровнен в заданном допуске');
 if(articulationBlocked){
  reasons.push('пиковый угол сочленения достиг программного порога '+ARTICULATION_STOP_DEMO_DEG+
   '° на '+articulationPeak.distanceAlongRoute.toFixed(1)+' м пути');
 }else if(articulationWarning){
  reasons.push('повышенный пиковый угол сочленения '+maxBeta.toFixed(1)+'° на '+
   articulationPeak.distanceAlongRoute.toFixed(1)+' м пути');
 }
 if(clearanceInsufficient)reasons.push('недостаточный зазор после резерва погрешности');
 if(wheelClearanceIncomplete)reasons.push('промежуточная кинематическая проверка колёс не завершена для '+wheelClearance.segmentsWithoutIntermediateKinematicSamples+' сегментов; виртуальный автопроезд заблокирован');
 if(wheelClearanceError)reasons.push('индивидуальная проверка колёс не подтверждена: '+wheelClearanceError);
 if(limitingWheelClearance&&Number.isFinite(limitingWheelClearance.minimumClearanceLowerBound)){
  const wheelDetail=limitingWheelClearance.limitingDetail||{};
  const wheelObstacle=wheelDetail.obstacleAtReference||wheelDetail.obstacle||'не выделено';
  reasons.push('ограничивающее колесо: '+limitingWheelClearance.name+'; нижняя оценка описанной оболочки '+
   limitingWheelClearance.minimumClearanceLowerBound.toFixed(2)+' м; препятствие «'+wheelObstacle+'»');
 }
 if(scrubExposure.maxAbsLateralVelocity>1e-3){
  reasons.push('геометрический индикатор бокового перемещения крайних осей: до '+scrubExposure.maxAbsLateralVelocity.toFixed(3)+' м/с; интегральный proxy '+scrubExposure.totalAbsoluteLateralTravel.toFixed(2)+' м, не модель шин');
 }
 if(cfg.mirrorOutboard<=0)reasons.push('геометрия зеркал не задана; зеркала не входят в проверку зазоров');
 if(!reasons.length)reasons.push('в проверенной модели препятствия не пересечены');
 return {length,turns,gearChanges,steeringChanges,initialSteeringActions,reverseDistance,reverseSegments,
  maxAxleScrubSpeed:scrubExposure.maxAbsLateralVelocity,
  axleScrubTravelProxy:scrubExposure.totalAbsoluteLateralTravel,
  axleScrubTravelByAxle:scrubExposure.cumulativeAbsoluteLateralTravelByAxle,
  axleScrubSpeedByAxle:scrubExposure.maxAbsLateralVelocityByAxle,
  axleScrubDistanceMethod:scrubExposure.distanceMethod,
  axleScrubSegmentsUsingStepDistance:scrubExposure.segmentsUsingStepDistance,
  axleScrubSegmentsUsingChordFallback:scrubExposure.segmentsUsingChordFallback,
  wheelTrackTravelMethod:wheelTrajectory.wheelTravelMethod,
  wheelTrackSegmentsWithKinematicEstimate:wheelTrajectory.segmentsWithKinematicEstimate,
  wheelTrackSegmentsWithoutKinematicEstimate:wheelTrajectory.segmentsWithoutKinematicEstimate,
  wheelTrackSummary,tireTrajectory,tireTrajectoryError,
  level:collision||!reached||articulationBlocked||clearanceInsufficient?'Не пройден':score>=7?'Высокая':score>=3?'Средняя':'Низкая',
  maxBeta,articulationPeak,articulationWarning,clearance:minClearance,clearanceDetail:minClearanceDetail,
  clearanceLowerBound:clearanceForAssessment,effectiveClearance,
  clearanceByPart,limitingPartClearance,trajectoryClearance,sweptClearanceDetail,
  wheelClearance,limitingWheelClearance,wheelClearanceError,intermediateWheelContact,wheelClearanceIncomplete,
  vehicleGeometryCalibration,
  gearTransitionEvents:Array.isArray(wheelClearance?.temporalEvents)?wheelClearance.temporalEvents:[],
  timeModelComplete:false,totalDurationSeconds:null,
  collision,reached,finalHeadingErrorDeg,finalArticulationDeg,clearanceInsufficient,articulationBlocked,reasons};
}


function wheelClearanceSummaryText(metrics){
 if(metrics?.wheelClearanceError){
  return 'Проверка отдельных колёс не подтверждена: '+metrics.wheelClearanceError;
 }
 const wheelClearance=metrics?.wheelClearance||null;
 const adaptive=wheelClearance?
  ' Адаптивная проверка: '+wheelClearance.intermediateSamplesAnalyzed+
  ' промежуточных поз на '+wheelClearance.segmentsWithIntermediateKinematicSamples+
  ' сегментах; максимальная модельная граница перемещения центра колеса между выборками '+
  Number(wheelClearance.maximumWheelCenterDisplacementBound||0).toFixed(3)+' м.':'';
 const cap=wheelClearance?.segmentsAtAdaptiveSamplingCap>0?
  ' На '+wheelClearance.segmentsAtAdaptiveSamplingCap+' сегментах достигнут предел '+
  wheelClearance.maxAdaptiveIntervals+' интервалов; плотность выборки ограничена.':'';
 const incomplete=wheelClearance?.segmentsWithoutIntermediateKinematicSamples>0?
  ' Промежуточная проверка не завершена на '+wheelClearance.segmentsWithoutIntermediateKinematicSamples+
  ' сегментах; виртуальный автопроезд заблокирован.':'';
 const surfaceReference=wheelClearance?.roadSurfaceStatus==='road-surface-unverified-horizontal-only'?
  ' Плоскость дороги не откалибрована: вертикальное разделение отключено, все вертикальные препятствия учитываются как горизонтальные 2D-оболочки.':
  wheelClearance?.roadSurfaceStatus==='configured-planar-surface-with-bounds'?
   ' Учтена заданная плоскость дороги с границами неопределённости; это не 3D-карта рельефа.':'';
 const controlTransitions=wheelClearance?[
  wheelClearance.segmentsWithInitialGearSelection>0?
   ' Исходная передача не измерялась; первое направление рассматривается как выбранное действие планировщика.':'',
  wheelClearance.segmentsWithGearChangeAtStart>0?
   ' Переключений передачи на границах пространственных сегментов: '+wheelClearance.segmentsWithGearChangeAtStart+
   '; пространственная модель не задаёт их длительность, а виртуальное воспроизведение требует отдельного подтверждения после модельной остановки.':'',
  wheelClearance.controlStartPosesAnalyzed>0?
   ' Проверено статических положений при выборе рулевого воздействия: '+wheelClearance.controlStartPosesAnalyzed+
   ' (проверок колёс: '+wheelClearance.controlStartWheelChecks+').':''
 ].join(''):'';
 const wheelItems=Array.isArray(wheelClearance?.wheels)?wheelClearance.wheels:[];
 const perWheelLabel=wheelItems.length===10?
  ' Нижние оценки по каждому из 10 колёс: ':
  ' Нижние оценки по каждой из '+wheelItems.length+' расчётных шинных оболочек: ';
 const perWheel=wheelItems.length>=10?
  perWheelLabel+wheelItems.map(item=>
   item.name+' — '+(Number.isFinite(item.minimumClearanceLowerBound)?item.minimumClearanceLowerBound.toFixed(2)+' м':'не ограничен')+
   (Number.isFinite(item.minimumIntermediatePolygonClearance)?
    '; промежуточная выборка '+item.minimumIntermediatePolygonClearance.toFixed(2)+' м':'')
  ).join('; ')+'.':'';
 const wheel=metrics?.limitingWheelClearance;
 if(!wheel||!Number.isFinite(wheel.minimumClearanceLowerBound)){
  return 'Индивидуальный зазор колёс не определён.'+perWheel+adaptive+cap+incomplete+controlTransitions+surfaceReference;
 }
 const detail=wheel.limitingDetail||{};
 const obstacle=detail.obstacleAtReference||detail.obstacle||'не выделено';
 const location=detail.kind==='segment-circumscribed-envelope-lower-bound'||
  detail.kind==='adaptive-segment-circumscribed-envelope-lower-bound'?
  ' на сегменте от '+Number(detail.distanceAlongRoute).toFixed(1)+' м, '+(detail.gear<0?'задний ход':'вперёд'):
  detail.kind==='sampled-circumscribed-envelope'?
   ' в позе около '+Number(detail.distanceAlongRoute).toFixed(1)+' м пути':'';
 return 'Ограничивающее колесо: '+wheel.name+'; нижняя оценка по описанной оболочке '+
  wheel.minimumClearanceLowerBound.toFixed(2)+' м; препятствие «'+obstacle+'»'+location+
  ' (консервативная геометрическая оценка).'+perWheel+adaptive+cap+incomplete+controlTransitions+surfaceReference;
}

function wheelTrackSummaryText(metrics){
 const items=Array.isArray(metrics?.wheelTrackSummary)?metrics.wheelTrackSummary:[];
 if(!items.length)return 'выборочные траектории колёс трёх осей не определены';
 const sampled='выборочные длины траекторий (центр/левое/правое, м): '+
  items.map(item=>'ось '+item.axleIndex+' '+
   item.centerDistance.toFixed(2)+'/'+item.leftWheelDistance.toFixed(2)+'/'+item.rightWheelDistance.toFixed(2)+
   '; внутренняя сторона '+item.innerSideSummary+
   '; максимальная разница внешнего и внутреннего колеса по хордам на сегменте '+
   item.maxOuterMinusInnerPerSample.toFixed(3)).join('; ');
 const integratedAvailable=items.every(item=>
  [item.integratedCenterDistance,item.integratedLeftWheelDistance,item.integratedRightWheelDistance].every(Number.isFinite)
 );
 if(!integratedAvailable){
  const reason=metrics.wheelTrackTravelMethod==='sampled-chords-only'?
   'stepDistance отсутствует':
   'не все сегменты прошли проверку реконструкции RK4';
  return sampled+'; интегрированная оценка длины пути не выдана ('+reason+').';
 }
 const integrated='оценка длины пути по RK4 и Симпсону (центр/левое/правое, м): '+
  items.map(item=>'ось '+item.axleIndex+' '+
   item.integratedCenterDistance.toFixed(2)+'/'+item.integratedLeftWheelDistance.toFixed(2)+'/'+item.integratedRightWheelDistance.toFixed(2)).join('; ');
 return sampled+'; '+integrated+'; численная оценка внутри кинематической модели, не измерение шин.';
}

function tireTrajectorySummaryText(metrics){
 const result=metrics?.tireTrajectory;
 if(metrics?.tireTrajectoryError)return 'не рассчитаны: '+metrics.tireTrajectoryError;
 if(!result||!Array.isArray(result.tires)||result.tires.length===0)return 'не определены';
 const tires=result.tires.map(tire=>{
  const sampled=Number.isFinite(tire.sampledTravel)?tire.sampledTravel.toFixed(2):'не определена';
  const integrated=Number.isFinite(tire.integratedTravelEstimate)?
   tire.integratedTravelEstimate.toFixed(2):'не выдана';
  return tire.name+' '+sampled+'/'+integrated;
 });
 const integration=result.integratedTravelComplete?
  'RK4 + составной Симпсон, все сегменты реконструированы':
  'интегральная оценка не выдана: не все сегменты прошли проверку RK4 или не имеют stepDistance';
 return result.tires.length+' оболочек (выборочная хорда / интегральная оценка, м): '+tires.join('; ')+
  '; '+integration+'; максимальный индикатор уточнения Симпсона '+
  Number(result.maxSegmentRefinementDelta||0).toFixed(6)+' м (не граница ошибки).';
}

function updateRecommendationCard(data={}){
 const fields={
  recommendationStatus:data.status||'Ожидает расчёта',
  recommendationLevel:data.level||'Не рассчитана',
  recommendationStart:data.start||'Сначала постройте маршрут',
  recommendationTurn:data.turn||'Пока не определено',
  recommendationReasons:data.reasons||'Нужен расчёт полной траектории и зазоров',
  recommendationCheck:data.check||'Рекомендация относится только к виртуальной модели'
 };
 for(const [id,value] of Object.entries(fields)){
  const element=$(id);
  if(element)element.textContent=value;
 }
}

function cancelPlannerWorker(reason='Расчёт маршрута отменён.'){
 const reject=plannerReject;
 plannerReject=null;
 if(plannerWorker){plannerWorker.terminate();plannerWorker=null;}
 plannerRequestId++;
 if(reject)reject(new Error(reason));
}

function runPlannerWorker(options,onProgress){
 cancelPlannerWorker('Запущен новый расчёт маршрута.');
 if(typeof Worker==='undefined')return Promise.reject(new Error('Этот браузер не поддерживает фоновый расчёт маршрута. Сцена симулятора сохранена; путь не построен.'));
 const requestId=plannerRequestId;
 return new Promise((resolve,reject)=>{
  let settled=false;
  let worker;
  let timeout;
  const finish=(error,value)=>{
   if(settled)return;
   settled=true;
   clearTimeout(timeout);
   if(worker)worker.terminate();
   if(plannerWorker===worker)plannerWorker=null;
   if(plannerReject===cancel)plannerReject=null;
   if(error)reject(error);else resolve(value);
  };
  const cancel=reason=>finish(new Error(reason));
  try{
   worker=new Worker('./planner-worker.js?v=20261127-7');
   plannerWorker=worker;
   plannerReject=cancel;
   timeout=setTimeout(()=>finish(new Error('Фоновый расчёт превысил 90 секунд и был остановлен. Попробуйте более близкую конечную точку.')),90000);
   worker.onmessage=event=>{
    const data=event.data||{};
    if(data.id!==requestId)return;
    if(data.type==='progress'){
     if(onProgress)onProgress(data.progress);
    }else if(data.type==='result')finish(null,data.result);
    else if(data.type==='error')finish(new Error(data.message||'Ошибка фонового планировщика.'));
   };
   worker.onerror=event=>finish(new Error('Ошибка фонового расчёта: '+(event.message||'не удалось запустить worker.')));
   worker.onmessageerror=()=>finish(new Error('Не удалось прочитать ответ планировщика.'));
   worker.postMessage({id:requestId,options});
  }catch(error){
   finish(error);
  }
 });
}

async function buildRoute(){
 const settings=readSettings();
 const token=++routeBuildToken;
 cancelPlannerWorker('Предыдущий расчёт заменён новым.');
 $('planRoute').disabled=false;
 if(!settings.ok){
  plannedRoute=[];routeGoal=null;routeAssessment=null;
  $('routeAdvice').textContent='Расчёт заблокирован: проверьте параметры модели. '+settings.errors.join(' ');
  $('difficultyAdvice').textContent='Сложность не рассчитана из-за некорректных параметров.';
  draw();
  return;
 }
 if(!showObstacles){
  routeAssessment=null;
  $('routeAdvice').textContent='Нельзя рассчитать готовность к автопроезду: включите «Препятствия» и повторно постройте маршрут.';
  $('difficultyAdvice').textContent='Сложность не рассчитана: проверка препятствий отключена.';
  draw();
  return;
 }
 const gx=Number($('goalX').value);
 const gy=Number($('goalY').value);
 const headingDegrees=Number($('goalHeading').value);
 const headingToleranceDegrees=Number($('goalHeadingTolerance').value);
 const articulationLimitDegrees=Number($('goalArticulationLimit').value);
 if(![gx,gy,headingDegrees,headingToleranceDegrees,articulationLimitDegrees].every(Number.isFinite)||
    headingDegrees< -180||headingDegrees>180||
    headingToleranceDegrees<1||headingToleranceDegrees>45||
    articulationLimitDegrees<1||articulationLimitDegrees>45){
  $('routeAdvice').textContent='Проверьте координаты и допуски конечного положения: курс от −180° до 180°, допуски от 1° до 45°.';
  return;
 }

 alternativeRoutes=[];
 $('alternativeRouteSelect').style.display='none';
 $('alternativeRouteSelect').value='';
 const startState={...state};
 const heading=headingDegrees*Math.PI/180;
 const headingTolerance=headingToleranceDegrees*Math.PI/180;
 const maxTerminalArticulation=articulationLimitDegrees*Math.PI/180;
 const goal={x:gx,y:gy,heading};
 routeGoal={...goal,positionTolerance:0.9,headingTolerance,maxTerminalArticulation};
 plannedRoute=[];
 routeAssessment=null;
 $('planRoute').disabled=true;
 $('routeAdvice').textContent='Запускаю фоновый расчёт траектории…';
 $('difficultyAdvice').textContent='Проверяю полный контур тягача и полуприцепа, включая задний ход…';
 updateRecommendationCard({status:'Идёт расчёт',start:'Сравниваю варианты движения вперёд и назад',turn:'Определяется планировщиком',reasons:'Проверяются контуры тягача, кабины и полуприцепа',check:'Окончательная рекомендация появится после проверки зазоров и погрешностей'});
 draw();

 const mobile=window.matchMedia&&window.matchMedia('(max-width: 600px)').matches;
 try{
  const result=await runPlannerWorker({
   start:startState,
   goal,
   positionTolerance:routeGoal.positionTolerance,
   headingTolerance:routeGoal.headingTolerance,
   maxTerminalArticulation:routeGoal.maxTerminalArticulation,
   params:{...cfg},
   obstacles:sceneObstacles(),
   gateGeometry:gateGeometryForPlanner(),
   maxExpansions:mobile?4500:12000,
   alternativeMaxExpansions:mobile?2200:4500,
   costTolerance:0.25,
   minClearance:0.65,
   measurementUncertainty:cfg.measurementUncertainty,
   obstaclePositionUncertainty:cfg.obstaclePositionUncertainty,
   obstacleSizeUncertainty:cfg.obstacleSizeUncertainty,
   vehicleFootprintUncertainty:cfg.vehicleFootprintUncertainty,
   vehicleParameterBounds:vehicleParameterBoundsFromSettings(),
   bounds:{minX:-20,maxX:42,minY:-20,maxY:20}
  },progress=>{
   if(token!==routeBuildToken)return;
   const parameterText=progress.parameterScenario?'параметры: '+progress.parameterScenario+' ('+progress.parameterScenarioIndex+'/'+progress.parameterScenarioCount+'); ':'';
   const geometryText=progress.geometryScenario==='expanded-geometry-envelope'?'расширенная геометрия; ':progress.geometryScenario==='nominal-geometry'?'номинальная геометрия; ':'';
   const scenarioText=parameterText+geometryText+(progress.scenario==='reserve'?'поиск с резервом неопределённости: ':'базовый поиск: ');
   const candidateText=progress.candidateIndex?'кандидат '+progress.candidateIndex+'/'+progress.candidateCount+'; суммарно проверено ':'проверено ';
   $('routeAdvice').textContent='Рассчитываю варианты траектории… '+scenarioText+candidateText+progress.expanded+
    ' из '+progress.maxExpansions+' состояний; до цели '+
    progress.goalDistance.toFixed(1)+' м; ошибка курса '+
    (Number.isFinite(progress.headingError)?(progress.headingError*180/Math.PI).toFixed(1)+'°':'не задана')+
    '; угол сочленения '+(progress.articulationAngle*180/Math.PI).toFixed(1)+'°.';
  });
  if(token!==routeBuildToken)return;

  const positionChange=Math.hypot(state.x-startState.x,state.y-startState.y);
  const headingChange=Math.abs(norm(state.psi-startState.psi));
  if(positionChange>0.05||headingChange>0.01){
   $('routeAdvice').textContent='Расчёт отменён: положение или курс тягача изменились во время поиска. Остановите модель и постройте путь заново.';
   $('difficultyAdvice').textContent='Сложность не рассчитана: исходное положение изменилось.';
   return;
  }


  
if(result.reason==='search-incomplete'||result.reason==='geometry-search-incomplete'){
 if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: поиск начальных действий не завершён.');
 plannedRoute=[];
 alternativeRoutes=[];
 routeAssessment=null;
 const chooser=$('alternativeRouteSelect');
 if(chooser){chooser.innerHTML='<option value="">Поиск не завершён</option>';chooser.style.display='none';}
 const incomplete=result.incompleteActions||
  result.reserveFailure?.incompleteActions||
  result.envelopeFailure?.incompleteActions||[];
 const actionNames=incomplete.map(item=>item.label).filter(Boolean);
 const actionNote=actionNames.length?
  ' Не завершены проверки: '+actionNames.join('; ')+'.':'';
 const geometrySearch=result.reason==='geometry-search-incomplete'||
  result.geometrySensitivity?.status==='envelope-search-incomplete';
 $('routeAdvice').textContent=geometrySearch?
  'Поиск в расширенной геометрической оболочке не завершён: достигнут лимит расширений до проверки всех необходимых начальных действий. Рекомендация и виртуальный автопроезд заблокированы.'+actionNote:
  'Поиск не завершён: достигнут лимит расширений до проверки всех необходимых начальных действий. Найденные варианты не считаются полным сравнением; сторона и передача не рекомендованы.'+actionNote;
 $('difficultyAdvice').textContent='Это ограничение вычислительного поиска, а не доказательство отсутствия маршрута. Увеличьте допустимый вычислительный бюджет или уточните сцену и повторите расчёт. Не уменьшайте геометрические и измерительные резервы ради получения рекомендации.';
 draw();
 return;
}

if(result.reason==='geometry-sensitive-start'||result.reason==='geometry-envelope-no-route'){
 if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: геометрическая неопределённость не подтверждает выбранный старт.');
 plannedRoute=[];
 routeAssessment=null;
 const geometry=result.geometrySensitivity||{};
 const gate=geometry.gateGeometryUncertainty;
 const gateNote=gate&&gate.enabled?
  ' Совместная оболочка ворот: ширина '+Number(gate.nominalClearWidth).toFixed(2)+' → '+Number(gate.conservativeClearWidth).toFixed(2)+' м; центр ±'+Number(gate.positionUncertaintyPerAxis).toFixed(2)+' м по X/Y; курс ±'+Number(gate.headingUncertaintyDeg).toFixed(2)+'°.':
  '';
 const perObstacleCount=Math.max(0,Number(geometry.perObstaclePoseEnvelopeCount)||0);
 const perObstacleNote=perObstacleCount?
  ' Отдельно сравнивались номинальные и консервативные контуры '+perObstacleCount+' препятствий с индивидуальной погрешностью.':
  '';
 const describeStartAction=action=>{
  if(!action)return 'не определено';
  const gear=action.initialGear===1?'вперёд':action.initialGear===-1?'назад':'передача не определена';
  const steering=action.steeringWheelDirection??action.firstTurn;
  const turn=steering==='left'?'руль влево':
   steering==='right'?'руль вправо':'направление руля не определено';
  const yaw=action.tractorYawDirection==='left'?'курс тягача меняется влево':
   action.tractorYawDirection==='right'?'курс тягача меняется вправо':'направление курса не определено';
  const distance=Number.isFinite(action.firstTurnDistance)?
   ' через '+action.firstTurnDistance.toFixed(2)+' м':'';
  const turnGear=action.firstTurnGear===1?'вперёд':
   action.firstTurnGear===-1?'назад':'';
  const after=action.firstTurnAfterGearChange?' после переключения передачи':'';
  return gear+', '+turn+distance+(turnGear?' (на '+turnGear+after+')':'')+'; '+yaw;
 };
 const comparison=geometry.comparison;
 const comparisonLabels=geometry.status==='start-action-changed-under-clearance-reserve'?
  {nominal:'В расширенной геометрии при обычном требуемом зазоре',
   reserve:'В расширенной геометрии с резервом зазора'}:
  {nominal:'Номинально',reserve:'С учётом ограниченной геометрии'};
 const comparisonNote=comparison&&comparison.nominalAction&&comparison.reserveAction?
  ' '+comparisonLabels.nominal+': '+describeStartAction(comparison.nominalAction)+
  '. '+comparisonLabels.reserve+': '+describeStartAction(comparison.reserveAction)+
  '. Изменились поля: '+(comparison.changedFields||[]).map(field=>({
    firstTurn:'сторона поворота руля',
    tractorYawDirection:'направление изменения курса тягача',
    firstTurnDistance:'расстояние до первого рулевого действия',
    initialGear:'начальная передача',
    turnGear:'передача при первом рулевом действии'
   }[field]||field)).join(', ')+'.':
  '';
 const clearanceEvidence=comparison?.clearanceEvidence;
 const describeClearanceEvidence=(label,evidence)=>{
  if(!evidence)return '';
  const obstacle=evidence.limitingObstacle;
  const part=evidence.limitingVehiclePart;
  const lowerBound=evidence.minimumClearanceLowerBound;
  if(!obstacle&&!part&&!Number.isFinite(lowerBound))return '';
  return ' '+label+': ближайшее ограничение — '+
   (obstacle?'«'+obstacle+'»':'препятствие не идентифицировано')+
   (part?' ('+part+')':'')+
   (Number.isFinite(lowerBound)?', нижняя оценка зазора '+lowerBound.toFixed(2)+' м':'')+'.';
 };
 const nominalClearanceNote=describeClearanceEvidence('Номинальный маршрут',clearanceEvidence?.nominal);
 const reserveClearanceNote=describeClearanceEvidence('Маршрут с учётом погрешности',clearanceEvidence?.reserve);
 const obstacleChangeNote=clearanceEvidence?.limitingObstacleChanged&&
  clearanceEvidence.nominal?.limitingObstacle&&clearanceEvidence.reserve?.limitingObstacle?
  ' Ограничивающее препятствие в двух сценариях различается: «'+
   clearanceEvidence.nominal.limitingObstacle+'» → «'+
   clearanceEvidence.reserve.limitingObstacle+'».':'';
 const clearanceNote=nominalClearanceNote+reserveClearanceNote+obstacleChangeNote;
 if(result.reason==='geometry-sensitive-start'){
  $('routeAdvice').textContent='Начальное действие меняется при одновременном учёте заданной геометрической неопределённости. Сторона и передача не выбраны; номинальный и консервативный маршруты требуют разных действий.'+comparisonNote+clearanceNote+gateNote+perObstacleNote;
  $('difficultyAdvice').textContent='Рекомендация неустойчива к заданным границам погрешности препятствий и внешних габаритов автопоезда. Уточните геометрию рулеткой и повторите расчёт. Проверяется ограниченная оболочка, не все возможные комбинации ошибок.'+comparisonNote+clearanceNote+gateNote+perObstacleNote;
 }else if(geometry.status==='ambiguous-under-envelope'){
  $('routeAdvice').textContent='В расширенной геометрической оболочке остаются несколько близких вариантов с разными начальными действиями. Однозначная сторона и передача не выбраны; виртуальный автопроезд заблокирован.'+gateNote+perObstacleNote;
  $('difficultyAdvice').textContent='Погрешность оставляет неоднозначность между вариантами манёвра. Уточните положение препятствий и геометрию автопоезда; отсутствие единственной рекомендации не означает, что другие траектории невозможны.'+gateNote+perObstacleNote;
 }else{
  $('routeAdvice').textContent='Поиск в расширенной геометрической оболочке не подтвердил маршрут в заданном лимите вычислений. Это не доказывает невозможность других маршрутов; рекомендация не выдаётся.'+gateNote+perObstacleNote;
  $('difficultyAdvice').textContent='Автопроезд заблокирован: выбранный маршрут не подтверждён при расширении геометрии препятствий и автопоезда. Не уменьшайте резерв только ради получения маршрута.'+gateNote+perObstacleNote;
 }
 $('difficultyAdvice').textContent+=' Границы проверки: положение препятствия ±'+Number(geometry.obstaclePositionUncertainty||0).toFixed(2)+' м по каждой оси; ошибка полного размера препятствия ±'+Number(geometry.obstacleSizeUncertainty||0).toFixed(2)+' м; расширение каждой внешней грани автопоезда '+Number(geometry.vehicleFootprintUncertainty||0).toFixed(2)+' м. Кинематические размеры проверяются отдельно при ненулевых допусках: совместные крайние сценарии плюс середины диапазонов по одному параметру; это не полный перебор всех комбинаций и не непрерывный worst-case анализ.';
 draw();
 return;
}

if(result.reason==='parameter-sensitive-start'||result.reason==='parameter-scenario-unavailable'){
 if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: совместная неопределённость физических размеров не подтверждает начальное действие.');
 plannedRoute=[];
 routeAssessment=null;
 const sensitivity=result.parameterSensitivity||{};
 const scenarioName=sensitivity.failedScenario||'совместная комбинация параметров';
 const values=(sensitivity.failedParameters||[]).map(item=>
  item.key+'='+Number(item.value).toFixed(3)+' м ('+item.side+')'
 ).join('; ');
 const coverageText=sensitivity.coverage==='pairwise-endpoint-covering-array'?
  'Проверено парное покрытие крайних значений для каждой пары активных параметров; дополнительно проверены середины отдельных диапазонов по одному параметру. Все многопараметрические внутренние сочетания не проверялись.':
  sensitivity.coverage==='full-endpoint-combinations'?
  'Проверены все сочетания нижних и верхних границ активных параметров (до трёх параметров), а также середина каждого диапазона отдельно. Совместные внутренние сочетания не проверялись.':
  'Покрытие неопределённости физических размеров не подтверждено.';
 if(result.reason==='parameter-sensitive-start'){
  $('routeAdvice').textContent='Начальное действие меняется в сценарии «'+scenarioName+'»'+
   (values?': '+values:'')+'. Изменились поля: '+(sensitivity.changedFields||[]).join(', ')+
   '. Сторона и передача не выбраны; виртуальный автопроезд заблокирован.';
  $('difficultyAdvice').textContent='Рекомендация зависит от совместного изменения физических размеров тягача/полуприцепа. Уточните геометрию по документам и контрольным измерениям, затем повторите расчёт.';
 }else{
  const validation=(sensitivity.validationErrors||[]).join(' ');
  $('routeAdvice').textContent='Для сценария «'+scenarioName+'»'+
   (values?' ('+values+')':'')+' планировщик не подтвердил допустимую геометрию или маршрут. Причина: '+
   (validation||sensitivity.failureReason||'нет результата')+
   '. Это не доказывает невозможность других траекторий; рекомендация не выдаётся.';
  $('difficultyAdvice').textContent='Проверка диапазона физических параметров не завершена для всех сценариев. Уточните совместимость размеров по документации/измерениям и повторите расчёт; номинальный маршрут не считается устойчивым.';
 }
 $('difficultyAdvice').textContent+=' '+coverageText;
 draw();
 return;
}

if(result.reason==='uncertainty-sensitive-start'){
   if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: рекомендация меняется при учёте погрешности измерений.');
   plannedRoute=[];
   routeAssessment=null;
   alternativeRoutes=Array.isArray(result.conservativeCandidateRoutes)?result.conservativeCandidateRoutes:[];
   const chooser=$('alternativeRouteSelect');
   chooser.innerHTML='<option value="">Выберите консервативный вариант…</option>';
   alternativeRoutes.forEach((candidate,index)=>{
    const option=document.createElement('option');
    option.value=String(index);
    option.textContent='Консервативный вариант '+(index+1)+' — '+candidate.label;
    chooser.appendChild(option);
   });
   chooser.style.display=alternativeRoutes.length?'block':'none';
   const sensitivity=result.sensitivity||{};
   const describeAction=action=>{
    if(!action)return 'не выбрано';
    const gear=action.initialGear<0?'назад':'вперёд';
    const steering=action.steeringWheelDirection??action.firstTurn;
    const turn=steering==='left'?'руль влево':
     steering==='right'?'руль вправо':'без значимого поворота руля';
    const yaw=action.tractorYawDirection==='left'?'курс тягача влево':
     action.tractorYawDirection==='right'?'курс тягача вправо':'курс не определён';
    return gear+', '+turn+'; '+yaw;
   };
   $('routeAdvice').textContent='Рекомендация чувствительна к погрешности: базовый поиск ('+
    Number(sensitivity.nominalRequiredClearance||0).toFixed(2)+' м требуемого зазора) дал '+describeAction(sensitivity.comparison?.nominalAction)+
    ', а повторный поиск с резервом '+Number(sensitivity.measurementUncertainty||0).toFixed(2)+
    ' м на каждую сторону ('+Number(sensitivity.reserveRequiredClearance||0).toFixed(2)+
    ' м требуемого зазора) дал '+describeAction(sensitivity.comparison?.reserveAction)+
    '. Однозначная сторона и передача не выбраны.';
   $('difficultyAdvice').textContent=alternativeRoutes.length?
    'Найден консервативный вариант с увеличенным запасом зазора. Выберите его в списке: после выбора программа отдельно проверит траекторию. Различие базового и резервного поиска останется отмеченным; это ограниченная проверка погрешности, а не полный анализ распределения ошибок камеры и геометрии автомобиля.':
    'Маршрут не подтверждён даже консервативным поиском. Уточните размеры ворот и препятствий и повторите расчёт. Это проверка резерва зазора, а не полный анализ распределения ошибок камеры и геометрии автомобиля.';
   draw();
   return;
  }
  if(result.reason==='uncertainty-no-route'){
   if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: маршрут не подтверждён с резервом неопределённости.');
   plannedRoute=[];
   routeAssessment=null;
   const sensitivity=result.sensitivity||{};
   $('routeAdvice').textContent='В базовом сценарии маршрут-кандидат найден, но повторный поиск с резервом '+
    Number(sensitivity.measurementUncertainty||0).toFixed(2)+' м на каждую сторону потребовал зазор '+
    Number(sensitivity.reserveRequiredClearance||result.requiredClearance||0).toFixed(2)+
    ' м и не нашёл допустимого пути в заданном лимите вычислений. Это не доказывает невозможность других маршрутов. Маршрут не выбран.';
   $('difficultyAdvice').textContent='Сложность и рекомендация не подтверждены с учётом заданной погрешности. Увеличивать допуски или уменьшать резерв только ради получения маршрута нельзя считать подтверждением безопасности.';
   draw();
   return;
  }
  if(result.reason==='ambiguous-start'){
   if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: начальные варианты манёвра неоднозначны.');
   plannedRoute=[];
   alternativeRoutes=Array.isArray(result.candidateRoutes)?result.candidateRoutes.filter(candidate=>Array.isArray(candidate.states)&&candidate.states.length>=2):[];
   routeAssessment=null;
   const chooser=$('alternativeRouteSelect');
   chooser.innerHTML='<option value="">Выберите вариант траектории…</option>';
   alternativeRoutes.forEach((candidate,index)=>{
    const option=document.createElement('option');
    option.value=String(index);
    option.textContent='Вариант '+(index+1)+' — '+candidate.label+' · балл '+candidate.cost.toFixed(2);
    chooser.appendChild(option);
   });
   chooser.style.display=alternativeRoutes.length?'block':'none';
   const alternatives=(result.candidateSummaries||[]).map(candidate=>{
    const gear=candidate.initialGear<0?'задний ход':'вперёд';
    const steering=candidate.steeringWheelDirection??candidate.firstTurn;
    const turn=steering==='left'?'руль влево':
     steering==='right'?'руль вправо':'без значимого поворота';
    const yaw=candidate.tractorYawDirection==='left'?'курс тягача влево':
     candidate.tractorYawDirection==='right'?'курс тягача вправо':'курс не определён';
    return candidate.label+': '+gear+', '+turn+', '+yaw+', эвристический балл '+candidate.cost.toFixed(2);
   }).join('; ');
   const geometrySensitivity=result.geometrySensitivity||{};
   const gateUncertainty=geometrySensitivity.gateGeometryUncertainty;
   const gateEnvelopeNote=gateUncertainty?.enabled&&geometrySensitivity.status==='nominal-ambiguous'?
    ' Совместная оболочка погрешности ворот не проверялась: номинальный поиск уже неоднозначен. Сначала нужно устранить неоднозначность стартовых действий; отсутствие проверки оболочки не трактуется как устойчивость.':'';
   $('routeAdvice').textContent='Найдено несколько близких по эвристической стоимости вариантов с разными начальными действиями. Однозначная сторона и передача не выбраны. '+alternatives+'. Балл объединяет длину пути, переключения, пиковый угол и запас зазора; это не вероятность безопасности и не доказательство глобального оптимума.'+gateEnvelopeNote;
   const reserveNote=result.sensitivity?.status==='reserve-ambiguous'?' Повторный поиск с резервом неопределённости также дал неоднозначные варианты.':'';
   $('difficultyAdvice').textContent='Показаны '+alternativeRoutes.length+' найденных траекторий разными линиями. Выбор и автопроезд пока заблокированы: сначала нужно сравнить варианты и подтвердить один маршрут. '+reserveNote+gateEnvelopeNote;
   updateRecommendationCard({status:'Несколько вариантов',level:'Не определена',start:'Сторона начала не выбрана: варианты близки по оценке',turn:alternatives||'Сравните варианты в списке',reasons:'Разные варианты требуют разных первых действий; автоматически выбирать сторону нельзя',check:'Выберите вариант вручную; после выбора программа повторно оценит маршрут'});
   draw();
   return;
  }
  plannedRoute=result.ok?result.states:[];
  if(plannedRoute.length<2){
   routeAssessment=null;
   const closestText=result.closest?
    ' Ближайшее найденное положение: '+result.goalDistance.toFixed(1)+' м до цели'+
    (Number.isFinite(result.headingError)?', ошибка курса '+(result.headingError*180/Math.PI).toFixed(1)+'°':'')+
    ', угол сочленения '+(result.articulationAngle*180/Math.PI).toFixed(1)+'°.':'';
   $('routeAdvice').textContent=result.reason==='start-collision'?
    'Маршрут не построен: текущая модель уже пересекается с препятствием. Сначала измените начальное положение.':
    result.reason==='start-clearance-insufficient'?
    'Маршрут не построен: начальный зазор '+result.startClearance.toFixed(2)+' м меньше требуемого резерва '+result.requiredClearance.toFixed(2)+' м. Увеличьте расстояние до препятствий и повторите расчёт.':
    'Не найден путь, который одновременно достигает координат, заданного курса тягача и допустимого угла полуприцепа. Проверено '+result.expanded+
    ' состояний.'+closestText+' Попробуйте изменить конечный курс, допуски или начальное положение.';
   $('difficultyAdvice').textContent='Автопроезд не разрешён: кинематический планировщик не подтвердил траекторию.';
   draw();
   return;
  }

  const m=routeMetrics(plannedRoute,result.minimumClearanceLowerBound,result.sweptClearanceDetail,result.trajectoryClearance);
  routeAssessment=m;
  const recommendationComparison=result.recommendationComparison||
   planner.compareStartCandidates([{states:plannedRoute,cost:result.length,label:'одиночный маршрут'}],{costTolerance:0.25});
  const recommendation=result.recommendation||
   (recommendationComparison.ok?
    (recommendationComparison.recommendation||recommendationComparison.bestCandidate.recommendation):
    planner.recommendStart(plannedRoute));
  const ambiguousFields=recommendationComparison.ok?recommendationComparison.ambiguousFields:[];
  const firstGear=recommendation.initialGear;
  const gearText=ambiguousFields.includes('initialGear')?
   'передача начала не выбрана: близкие варианты расходятся':
   firstGear<0?'начать задним ходом':'начать движением вперёд';
  let firstTurnText='первый участок пройти прямо; значимого поворота руля в рассчитанном маршруте не найдено';
  if(ambiguousFields.includes('firstTurn')){
   firstTurnText='сторона первого рулевого действия не выбрана: близкие по стоимости варианты расходятся';
  }else if(recommendation.firstTurn){
   const steering=recommendation.steeringWheelDirection??recommendation.firstTurn;
   const side=steering==='left'?'повернуть руль влево':'повернуть руль вправо';
   const timing=recommendation.firstTurnDistance<=0.3?'сразу':'примерно через '+recommendation.firstTurnDistance.toFixed(1)+' м';
   const turnGear=ambiguousFields.includes('turnGear')?'при неоднозначной передаче':
    recommendation.firstTurnGear<0?'задним ходом':'вперёд';
   const gearNote=recommendation.firstTurnAfterGearChange?' после переключения на движение '+turnGear:' при движении '+turnGear;
   const yawSide=ambiguousFields.includes('tractorYawDirection')?'направление изменения курса неоднозначно':
    recommendation.tractorYawDirection==='left'?'курс тягача начнёт меняться влево':'курс тягача начнёт меняться вправо';
   firstTurnText=side+' '+timing+gearNote+'; '+yawSide;
  }
  const recommendationNote=recommendationComparison.status==='single-candidate'?
   ' Найден только '+(result.candidatesFound||1)+' из '+(result.candidatesAttempted||1)+' проверенных стартовых вариантов; остальные не подтвердили путь в заданном лимите поиска, поэтому преимущество стороны не доказано.':
   recommendationComparison.status==='preferred'?
   ' Выбран меньший эвристический балл среди '+(result.candidatesFound||1)+' найденных стартовых вариантов из '+(result.candidatesAttempted||1)+'; это ограниченное сравнение, не доказательство глобального оптимума.':
   ' Показано начальное действие найденного маршрута без подтверждённого сравнения альтернатив.';
  const sensitivity=result.sensitivity||{};
  const sensitivityNote=sensitivity.status==='stable-start-action'?
   ' При повторном поиске с дополнительным резервом '+sensitivity.measurementUncertainty.toFixed(2)+
   ' м на каждую сторону (требуемый зазор '+sensitivity.nominalRequiredClearance.toFixed(2)+' → '+
   sensitivity.reserveRequiredClearance.toFixed(2)+' м) начальное действие совпало среди найденных кандидатов.'+
   (sensitivity.candidateCoverageChanged?' Число найденных кандидатов при этом изменилось; сравнение остаётся ограниченным.':''):
   sensitivity.status==='zero-reserve'?' Дополнительный резерв неопределённости равен нулю; повторный поиск не выполнялся.':'';
const geometrySensitivity=result.geometrySensitivity||{};
const gateUncertainty=geometrySensitivity.gateGeometryUncertainty;
const gateGeometryNote=gateUncertainty&&gateUncertainty.enabled?
 ' Геометрия ворот проверена совместной ограниченной оболочкой: чистая ширина '+Number(gateUncertainty.nominalClearWidth).toFixed(2)+' → '+Number(gateUncertainty.conservativeClearWidth).toFixed(2)+' м; центр ±'+Number(gateUncertainty.positionUncertaintyPerAxis).toFixed(2)+' м по X/Y; курс ±'+Number(gateUncertainty.headingUncertaintyDeg).toFixed(2)+'°. Это сравнение номинального проёма с консервативно расширенными косяками, а не перебор всех независимых реализаций ошибок.':'';
const geometryNote=geometrySensitivity.status==='stable-within-bounded-envelope'?
 ' Дополнительно проверена расширенная геометрия: положение препятствий ±'+geometrySensitivity.obstaclePositionUncertainty.toFixed(2)+' м по каждой оси, ошибка полного размера ±'+geometrySensitivity.obstacleSizeUncertainty.toFixed(2)+' м, внешний контур автопоезда расширен на '+geometrySensitivity.vehicleFootprintUncertainty.toFixed(2)+' м с каждой грани. Параметры wheelbase, hitchOffset и kingpinAxles не варьировались; это ограниченная проверка оболочки, не полный анализ неопределённости.'+gateGeometryNote:
 geometrySensitivity.status==='zero-geometry-bounds'?' Геометрические границы неопределённости заданы нулевыми; расширенная проверка не выполнялась.'+gateGeometryNote:'';
const parameterSensitivity=result.parameterSensitivity||{};
const parameterRanges=Object.entries(parameterSensitivity.suppliedBounds||{}).map(([key,bound])=>
 key+' '+Number(bound.min).toFixed(3)+'–'+Number(bound.max).toFixed(3)+' м'
).join('; ');
const stableParameterStatuses=[
 'stable-across-joint-endpoint-combinations',
 'stable-across-pairwise-endpoint-scenarios',
 'stable-across-one-at-a-time-bounds'
];
const parameterNote=stableParameterStatuses.includes(parameterSensitivity.status)?
 ' Проверено '+parameterSensitivity.testedScenarioCount+' сценариев физических размеров ('+parameterRanges+'); начальное действие совпало среди найденных кандидатов. '+
 (parameterSensitivity.note||'Проверка ограничена заданными сценариями.') :
 parameterSensitivity.status==='not-varied'?
 ' Неопределённости физических размеров не заданы (нулевые допуски означают «не оценивалось», а не доказанную точность).':
 ' Устойчивость к диапазонам физических размеров не подтверждена.';

  const peakGearText=m.articulationPeak.gear<0?'задним ходом':'вперёд';
  const articulationPeakText='пик угла сочленения '+m.articulationPeak.maxAngleDegrees.toFixed(1)+
   '° на '+m.articulationPeak.distanceAlongRoute.toFixed(1)+' м пути (X '+
   m.articulationPeak.state.x.toFixed(1)+' м, Y '+m.articulationPeak.state.y.toFixed(1)+' м), '+peakGearText;
  $('routeAdvice').textContent=
   'Начало манёвра: '+gearText+'; первое рулевое действие: '+firstTurnText+
   '. Сторона определяется по первому значимому управляющему углу планировщика (порог 0,08 рад), а не по одному лишь направлению на цель.'+recommendationNote+sensitivityNote+parameterNote+' Маршрут до точки ('+
   gx.toFixed(1)+', '+gy.toFixed(1)+') м; длина '+m.length.toFixed(1)+
   ' м; переключений передачи: '+m.gearChanges+'; изменений руля: '+m.steeringChanges+
   '; задний ход: '+m.reverseDistance.toFixed(1)+' м. Заданный конечный курс '+headingDegrees.toFixed(0)+
   '°; ошибка курса '+m.finalHeadingErrorDeg.toFixed(1)+'°; угол сочленения у цели '+
   m.finalArticulationDeg.toFixed(1)+'° (допуск '+articulationLimitDegrees.toFixed(0)+'°); '+articulationPeakText+'.'+geometryNote+parameterNote;

  const closestLocationText=m.clearanceDetail?
   m.clearanceDetail.vehiclePart+(m.clearanceDetail.component?' · '+m.clearanceDetail.component:'')+
   ' рядом с препятствием «'+m.clearanceDetail.obstacle+
   '» около X '+m.clearanceDetail.state.x.toFixed(1)+' м, Y '+
   m.clearanceDetail.state.y.toFixed(1)+' м':
   'не определена (препятствия выключены или зазор бесконечен)';
  const sweptLocationText=m.sweptClearanceDetail&&m.sweptClearanceDetail.kind==='segment-lower-bound'?
   'сегмент X '+m.sweptClearanceDetail.from.x.toFixed(1)+', Y '+m.sweptClearanceDetail.from.y.toFixed(1)+
   ' → X '+m.sweptClearanceDetail.to.x.toFixed(1)+', Y '+m.sweptClearanceDetail.to.y.toFixed(1):
   'отдельный ограничивающий сегмент не выделен';
  const routePartClearanceText=m.clearanceByPart.length?
   m.clearanceByPart.map(part=>{
    const lower=Number.isFinite(part.minimumClearanceLowerBound)?
     part.minimumClearanceLowerBound.toFixed(2)+' м':'не определён';
    const sampled=Number.isFinite(part.minimumSampledClearance)?
     part.minimumSampledClearance.toFixed(2)+' м':'не определён';
    const detail=part.limitingDetail;
    const obstacle=detail?.obstacleAtReference||detail?.obstacle||'препятствие не выделено';
    const component=detail?.componentAtReference||detail?.component;
    const componentText=component?' · '+component:'';
    const obstacleText=detail?.kind==='segment-lower-bound'?
     'препятствие у начала сегмента «'+obstacle+'»'+componentText:
     'препятствие «'+obstacle+'»'+componentText;
    const location=detail?.kind==='segment-lower-bound'?
     'сегмент около '+Number(detail.distanceAlongRoute).toFixed(1)+' м, '+(detail.gear<0?'задний ход':'вперёд'):
     detail?.kind==='sampled-pose'?'поза около '+Number(detail.distanceAlongRoute).toFixed(1)+' м пути':'положение не определено';
    return part.vehiclePart+': выборочный минимум '+sampled+', нижняя оценка '+lower+
     ', '+obstacleText+', '+location;
   }).join('; '):'персональная оценка частей маршрута отсутствует';
  const startActionText=ambiguousFields.includes('initialGear')?
   'Передача начала не выбрана: близкие варианты расходятся':
   firstGear<0?'Начать задним ходом':'Начать движением вперёд';
  const confidenceText=recommendationComparison.status==='single-candidate'?
   'Найден только один подтверждённый вариант; преимущество стороны не доказано.':
   recommendationComparison.status==='preferred'?
   'Сторона выбрана сравнением '+recommendationComparison.candidatesCompared+' найденных вариантов; сравнение ограничено эвристической оценкой.':
   'Сторона показана по найденному маршруту без полного сравнения альтернатив.';
  const clearanceText=Number.isFinite(m.effectiveClearance)?
   'Минимальный эффективный зазор '+m.effectiveClearance.toFixed(2)+' м.':
   'Минимальный эффективный зазор не определён.';
  updateRecommendationCard({
   status:recommendationComparison.status==='single-candidate'?'Предварительная рекомендация':
    recommendationComparison.status==='preferred'?'Сравнение вариантов выполнено':'Оценка маршрута',
   level:m.level,
   start:startActionText,
   turn:firstTurnText,
   reasons:m.reasons.slice(0,4).join('; ')+'. Пиковый угол сочленения '+m.maxBeta.toFixed(1)+'°. '+clearanceText,
   check:confidenceText+' Это учебный расчёт виртуальной модели, не гарантия безопасности.'
  });
  $('difficultyAdvice').textContent=
   'Сложность: '+m.level+'. Причины: '+m.reasons.join('; ')+
   '. Максимальный угол сочленения '+m.maxBeta.toFixed(1)+'°; '+articulationPeakText+'; конечная ошибка курса '+m.finalHeadingErrorDeg.toFixed(1)+
   '°; конечный угол тягач–полуприцеп '+m.finalArticulationDeg.toFixed(1)+'°; геометрический индикатор боковой скорости крайних осей при расчётной скорости 0,7 м/с '+m.maxAxleScrubSpeed.toFixed(3)+' м/с; по осям (передняя/средняя/задняя): '+m.axleScrubSpeedByAxle.map(value=>value.toFixed(3)).join('/')+' м/с; накопленный proxy по осям: '+m.axleScrubTravelByAxle.map(value=>value.toFixed(2)).join('/')+' м; суммарный proxy '+m.axleScrubTravelProxy.toFixed(2)+' м (не модель шин; база длины сегмента: '+(m.axleScrubDistanceMethod==='planner-step-distance'?'stepDistance планировщика':m.axleScrubDistanceMethod==='chord-fallback'?'резервная хорда между узлами':'stepDistance при наличии, иначе хорда')+'); выборочные траектории колёс трёх осей (центр/левое/правое, м): '+m.wheelTrackSummary.map(item=>'ось '+item.axleIndex+' '+item.centerDistance.toFixed(2)+'/'+item.leftWheelDistance.toFixed(2)+'/'+item.rightWheelDistance.toFixed(2)+'; внутренняя сторона '+item.innerSideSummary+'; max разница внешнего и внутреннего колеса на сегменте '+item.maxOuterMinusInnerPerSample.toFixed(3)).join('; ')+'; отдельные траектории покрышек: '+tireTrajectorySummaryText(m)+'; длины групп — хорды между узлами маршрута, не непрерывная оценка; минимальный зазор до тестовых препятствий '+
   (Number.isFinite(m.clearance)?m.clearance.toFixed(2)+' м':'не определён')+
   '; ближайшая проверенная часть автопоезда: '+closestLocationText+
   '; нижняя оценка между узлами '+(Number.isFinite(m.clearanceLowerBound)?m.clearanceLowerBound.toFixed(2)+' м':'не ограничена')+
   ' на '+sweptLocationText+' (общая оценка движения контура; точная часть в середине сегмента не атрибутируется)'+
   '; оценки частей вдоль маршрута: '+routePartClearanceText+
   '; '+wheelClearanceSummaryText(m)+
   '; после резерва погрешности: '+(Number.isFinite(m.effectiveClearance)?m.effectiveClearance.toFixed(2)+' м':'не определено')+
   '; устойчивость стартового действия: '+(sensitivity.status==='stable-start-action'?'сохранилось при повторном поиске с увеличенным требуемым зазором':sensitivity.status==='zero-reserve'?'не проверялась из-за нулевого резерва':'не подтверждена')+
   '; столкновение: '+(m.collision?'да':'не выявлено')+
   '. Курс у цели проверен относительно заданного угла и допуска; учебный расчёт, не гарантия безопасности.'+geometryNote;
  draw();
 }catch(error){
  if(token!==routeBuildToken)return;
  routeAssessment=null;
  plannedRoute=[];
  $('routeAdvice').textContent='Фоновый расчёт не завершён: '+(error&&error.message?error.message:String(error));
  $('difficultyAdvice').textContent='Сложность не рассчитана; автоматический проезд заблокирован.';
  draw();
 }finally{
  if(token===routeBuildToken)$('planRoute').disabled=false;
 }
}
function drawRoute(){
 if(alternativeRoutes.length){
  const colors=['#35c8ef','#ffb74d','#ce93d8','#80cbc4','#ef9a9a'];
  alternativeRoutes.forEach((candidate,index)=>{
   drawLine(candidate.states,colors[index%colors.length],4,[9,6]);
   const point=toScreen(candidate.states[0]);
   ctx.fillStyle=colors[index%colors.length];
   ctx.font='bold 14px system-ui';
   ctx.fillText('Вариант '+(index+1),point.x+10,point.y-10);
  });
  return;
 }
 if(plannedRoute.length<2)return;

 drawLine(plannedRoute,'#ffe082',4,[9,6]);

 // Overlay the sampled center paths of individual tire footprints.
 const tireTracks=routeAssessment?.tireTrajectory?.tires||[];
 tireTracks.forEach(track=>{
  if(!Array.isArray(track.points)||track.points.length<2)return;
  ctx.beginPath();
  track.points.forEach((point,index)=>{
   const screen=toScreen(point);
   if(index===0)ctx.moveTo(screen.x,screen.y);
   else ctx.lineTo(screen.x,screen.y);
  });
  const axleColors=['#81c784','#ffb74d','#ce93d8'];
  ctx.strokeStyle=track.vehiclePart==='тягач'?'#4fc3f7':axleColors[(track.axleIndex-1)%axleColors.length];
  ctx.lineWidth=1.35;
  ctx.globalAlpha=0.72;
  ctx.setLineDash(track.pairPosition==='inner'?[3,3]:[]);
  ctx.stroke();
 });
 ctx.setLineDash([]);
 ctx.globalAlpha=1;

 const swept=routeAssessment&&routeAssessment.sweptClearanceDetail;
 if(swept&&swept.kind==='segment-lower-bound'&&swept.from&&swept.to){
  drawLine([swept.from,swept.to],'#ff8585',3,[4,4]);
 }
 const peak=routeAssessment&&routeAssessment.articulationPeak;
 if(peak&&peak.state){
  const marker=toScreen(peak.state);
  ctx.beginPath();
  ctx.moveTo(marker.x,marker.y-10);
  ctx.lineTo(marker.x+10,marker.y);
  ctx.lineTo(marker.x,marker.y+10);
  ctx.lineTo(marker.x-10,marker.y);
  ctx.closePath();
  ctx.fillStyle='#ffb74d';
  ctx.fill();
  ctx.strokeStyle='#172536';
  ctx.lineWidth=2;
  ctx.stroke();
  ctx.fillStyle='#fff';
  ctx.font='12px system-ui';
  ctx.fillText('β MAX '+peak.maxAngleDegrees.toFixed(1)+'°',marker.x+12,marker.y+16);
 }
 const closest=routeAssessment&&routeAssessment.clearanceDetail;
 if(closest&&closest.state&&Number.isFinite(closest.distance)){
  const marker=toScreen(closest.state);
  ctx.beginPath();
  ctx.arc(marker.x,marker.y,8,0,Math.PI*2);
  ctx.fillStyle='#ff8585';
  ctx.fill();
  ctx.strokeStyle='#fff';
  ctx.lineWidth=2;
  ctx.stroke();
  ctx.fillStyle='#fff';
  ctx.font='12px system-ui';
  ctx.fillText('MIN '+closest.distance.toFixed(2)+' м',marker.x+11,marker.y-9);
 }

 if(routeGoal){
  const z=toScreen(routeGoal);
  ctx.beginPath();
  ctx.arc(z.x,z.y,9,0,Math.PI*2);
  ctx.fillStyle='#ffe082';
  ctx.fill();
  ctx.strokeStyle='#172536';
  ctx.lineWidth=2;
  ctx.stroke();

  ctx.fillStyle='#fff';
  ctx.font='13px system-ui';
  ctx.fillText('ЦЕЛЬ',z.x+12,z.y-10);
  if(Number.isFinite(routeGoal.heading)){
   drawLine([routeGoal,{x:routeGoal.x+1.5*Math.cos(routeGoal.heading),y:routeGoal.y+1.5*Math.sin(routeGoal.heading)}],'#ffe082',3);
  }
 }
}

function drawVehicleDetails(){
 // Rendering and collision checks share the same calibrated tire footprints.
 physics.wheelFootprints(state,cfg,steer).forEach(wheel=>{
  drawPolygon(wheel.polygon,'#050b11');
 });
 // Отдельные зеркальные оболочки отображаются только при введённом измеренном выносе.
 physics.mirrorPolygons(state,{...cfg,footprintUncertainty:0}).forEach(mirror=>{
  drawPolygon(mirror.polygon,'#143b50');
 });
 // Кабина: контрастное ветровое стекло и нос автомобиля.
 const windshield=bodyCorners(
  localPoint(state,state.psi,cfg.cabOffset),state.psi,
  cfg.cabFront-0.2,cfg.cabFront-1.1,cfg.cabWidth-0.05
 ).map(toScreen);
 ctx.beginPath();
 windshield.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));
 ctx.closePath();
 ctx.fillStyle='#a7e8f7';
 ctx.globalAlpha=.82;
 ctx.fill();
 ctx.globalAlpha=1;
 ctx.strokeStyle='#e4fbff';
 ctx.lineWidth=2;
 ctx.stroke();
 // Направление движения вперёд.
 const nose=toScreen(localPoint(state,state.psi,5.7));
 const arrowBase=toScreen(localPoint(state,state.psi,4.4));
 ctx.strokeStyle='#fff';
 ctx.fillStyle='#fff';
 ctx.lineWidth=3;
 ctx.beginPath();ctx.moveTo(arrowBase.x,arrowBase.y);ctx.lineTo(nose.x,nose.y);ctx.stroke();
 const left=toScreen(localPoint(state,state.psi,4.7,.28));
 const right=toScreen(localPoint(state,state.psi,4.7,-.28));
 ctx.beginPath();ctx.moveTo(nose.x,nose.y);ctx.lineTo(left.x,left.y);ctx.lineTo(right.x,right.y);ctx.closePath();ctx.fill();
}

function fitSceneCamera(){
 const points=[];
 const add=poly=>{if(Array.isArray(poly))poly.forEach(p=>{if(Number.isFinite(p.x)&&Number.isFinite(p.y))points.push(p);});};
 add(vehicleCollisionParts(state).flatMap(group=>group.components.map(component=>component.polygon)));
 if(showObstacles)sceneObstacles().forEach(o=>add(obstaclePolygon(o)));
 if(plannedRoute.length>1)plannedRoute.forEach(p=>points.push({x:p.x,y:p.y}));
 const tireTracks=routeAssessment?.tireTrajectory?.tires||[];
 tireTracks.forEach(track=>track.points?.forEach(point=>points.push({x:point.x,y:point.y})));
 if(!points.length)return;
 const xs=points.map(p=>p.x),ys=points.map(p=>p.y);
 const minX=Math.min(...xs)-2.5,maxX=Math.max(...xs)+2.5;
 const minY=Math.min(...ys)-2.5,maxY=Math.max(...ys)+2.5;
 const spanX=Math.max(8,maxX-minX),spanY=Math.max(5,maxY-minY);
 camera={x:(minX+maxX)/2,y:(minY+maxY)/2};
 SCALE=Math.max(8,Math.min(36,canvas.width/spanX,canvas.height/spanY));
}

function draw(){
 ctx.clearRect(0,0,canvas.width,canvas.height);
 ctx.fillStyle='#102235';
 ctx.fillRect(0,0,canvas.width,canvas.height);

 fitSceneCamera();
 const left=camera.x-canvas.width/(2*SCALE);
 const top=camera.y-canvas.height/(2*SCALE);

 ctx.strokeStyle='#20394e';
 ctx.lineWidth=1;

 for(let x=Math.floor(left/5)*5;
     x<left+canvas.width/SCALE;x+=5){
  const p=toScreen({x,y:camera.y});
  ctx.beginPath();
  ctx.moveTo(p.x,0);
  ctx.lineTo(p.x,canvas.height);
  ctx.stroke();
 }

 for(let y=Math.floor(top/5)*5;
     y<top+canvas.height/SCALE;y+=5){
  const p=toScreen({x:camera.x,y});
  ctx.beginPath();
  ctx.moveTo(0,p.y);
  ctx.lineTo(canvas.width,p.y);
  ctx.stroke();
 }

 drawObstacles();
 drawRoute();
 if(trace.length>1)drawSweptCorridor(trace,'actual');
 if(plannedRoute.length>1)drawSweptCorridor(plannedRoute,'planned');

 if(showTrace&&trace.length>1){
  drawLine(
   trace.map(p=>({x:p.x,y:p.y})),
   '#35c8ef',2
  );
  drawLine(
   trace.map(p=>({x:p.ax,y:p.ay})),
   '#f6bd60',2,[6,4]
  );
 }

 const h=hitchPoint(state);

 physics.vehiclePolygons(state,{...cfg,footprintUncertainty:cfg.vehicleFootprintUncertainty})
  .forEach(poly=>drawEnvelopePolygon(poly,'#ffd18b'));
 const trailerKingpinToFront=cfg.trailerKingpinToFront??1.5;
 drawBody(h,state.psiT,trailerKingpinToFront,-(cfg.trailerLength-trailerKingpinToFront),cfg.trailerWidth,'#315b78');
 drawBody(state,state.psi,cfg.tractorFrontOverhang,-cfg.tractorRearOverhang,cfg.tractorBodyWidth,'#087fa5');
 drawBody(localPoint(state,state.psi,cfg.cabOffset),state.psi,cfg.cabFront,-cfg.cabRear,cfg.cabWidth,'#35c8ef');
 drawVehicleDetails();

 const hp=toScreen(h);
 ctx.beginPath();
 ctx.arc(hp.x,hp.y,5,0,Math.PI*2);
 ctx.fillStyle='#ffd166';
 ctx.fill();

 const rp=toScreen(state);
 ctx.beginPath();
 ctx.arc(rp.x,rp.y,4,0,Math.PI*2);
 ctx.fillStyle='#fff';
 ctx.fill();

 const partClearances=clearanceForVehicleParts(state);
 vehicleCollisionParts(state).forEach(group=>group.components.forEach(component=>
  drawEnvelopePolygon(component.polygon,partClearances[group.bodyIndex]?.color||'#8ba5b8')
 ));
 ctx.save();
 ctx.textAlign='center';
 ctx.font='bold 16px system-ui';
 ctx.fillStyle='#f3fbff';
 ctx.fillText('VOLVO FH 4×2',rp.x+20,rp.y-27);
 ctx.font='bold 14px system-ui';
 const trailerLabel=toScreen(localPoint(h,state.psiT,-cfg.trailerLength*.48));
 ctx.fillStyle='#f3fbff';
 ctx.fillText('ПОЛУПРИЦЕП · 3 ОСИ',trailerLabel.x,trailerLabel.y+24);
 ctx.restore();
 const mapAlert=partClearances.reduce((best,item)=>item.effective<best.effective?item:best,partClearances[0]||{effective:Infinity,name:'нет данных'});
 ctx.save();
 ctx.font='bold 15px system-ui';
 ctx.fillStyle=mapAlert.effective<0.45?'#ff7777':mapAlert.effective<1?'#ffd078':'#83e5bb';
 const clearanceLabel=Number.isFinite(mapAlert.effective)?
  mapAlert.effective.toFixed(2)+' м · '+mapAlert.vehiclePart+
  (mapAlert.component?' / '+mapAlert.component:'')+' / '+mapAlert.name:
  'Зазор не рассчитан';
 ctx.fillText(showObstacles?'МИН. ЗАЗОР: '+clearanceLabel:'ПРЕПЯТСТВИЯ НЕ ПРОВЕРЯЮТСЯ',18,28);
 ctx.restore();

 updateReadouts();
 updateAssistant();
}

function updateReadouts(){
 const beta=norm(state.psi-state.psiT);
 const axleDiagnostics=physics.trailerAxleDiagnostics(state,speed,steer,cfg);
 const lateral=Math.abs(
  speed*speed/cfg.wheelbase*Math.tan(steer)
 );
 const gripLimit=cfg.mu*G;
 const brakeLimit=Math.max(
  0.1,Math.min(cfg.brakeDecel,gripLimit)
 );
 const brakeDistance=speed*speed/(2*brakeLimit);

 $('vOut').textContent=Math.abs(speed).toFixed(2)+' м/с';
 $('betaOut').textContent=(beta*180/Math.PI).toFixed(1)+'°';
 $('steerOut').textContent=(steer*180/Math.PI).toFixed(0)+'°';
 $('accelOut').textContent=
  (braking?-brakeLimit:(drive?cfg.maxAccel:0)).toFixed(2)+' м/с²';
 $('latOut').textContent=lateral.toFixed(2)+' м/с²';
 $('stopOut').textContent=brakeDistance.toFixed(1)+' м';
 $('gripOut').textContent=
  lateral>gripLimit?'Предел превышен':'В пределах заданного μ';
 $('distanceOut').textContent=distance.toFixed(1)+' м';
 $('axleScrubOut').textContent=axleDiagnostics.maxAbsLateralVelocity.toFixed(3)+' м/с';
 const cockpitValues={
  cockpitSpeed:Math.abs(speed).toFixed(1)+' м/с',
  cockpitSpeedLarge:Math.abs(speed).toFixed(1)+' м/с',
  cockpitBeta:(beta*180/Math.PI).toFixed(1)+'°',
  cockpitBetaLarge:(beta*180/Math.PI).toFixed(1)+'°',
  cockpitSteer:(steer*180/Math.PI).toFixed(0)+'°'
 };
 Object.entries(cockpitValues).forEach(([id,value])=>{const el=$(id);if(el)el.textContent=value;});
 const cockpitClearance=$('cockpitClearance');
 if(cockpitClearance){
  const assessment=assessScene();
  const partClearances=clearanceForVehicleParts(state);
  const nearestPart=partClearances.reduce((best,item)=>item.effective<best.effective?item:best,partClearances[0]||{effective:Infinity});
  const effectiveClearance=nearestPart.effective;
  cockpitClearance.textContent=Number.isFinite(effectiveClearance)?effectiveClearance.toFixed(2)+' м':'—';
  cockpitClearance.style.color=!showObstacles?'#9bb2c3':assessment.collision||effectiveClearance<=0||effectiveClearance<0.45?'#ff6262':effectiveClearance<1?'#ffc45f':'#42d6a0';
  const nearestLabel=$('cockpitNearest');
  if(nearestLabel)nearestLabel.textContent=showObstacles&&Number.isFinite(effectiveClearance)?nearestPart.name+' · '+nearestPart.vehiclePart+' · после допусков':'Проверка препятствий отключена';
 }
 const cockpitStatus=$('cockpitStatus');
 if(cockpitStatus)cockpitStatus.textContent=perceptionFrameTimestamp?'Датчики передали '+perceptionObstacles.length+' объектов':'Система готова · демонстрационный режим';
 const cockpitDifficulty=$('cockpitDifficulty');
 if(cockpitDifficulty)cockpitDifficulty.textContent=$('recommendationLevel')?.textContent||'Не рассчитана';

 $('mode').textContent=braking?'Торможение':
  drive>0?'Движение вперёд':
  drive<0?'Движение задним ходом':'Остановлен';

 $('speedLabel').textContent=targetSpeed.toFixed(1)+' м/с';
 $('steerLabel').textContent=(steer*180/Math.PI).toFixed(0)+'°';

 const angle=Math.abs(beta*180/Math.PI);

 if(angle>=ARTICULATION_STOP_DEMO_DEG){
  $('mode').textContent='ВНИМАНИЕ: большой угол сочленения';
  $('mode').style.color='#ff8585';
 }else if(angle>=ARTICULATION_WARNING_DEG){
  $('mode').textContent='Предупреждение: угол сочленения';
  $('mode').style.color='#ffd18b';
 }else{
  $('mode').style.color='#83d6a8';
 }
}

function updateSpeed(dt){
 if($('model').value==='kinematic'){
  speed=drive*targetSpeed;
  if(braking)speed=0;
  return;
 }

 const totalMass=cfg.tractorMass+cfg.trailerMass;
 const rolling=cfg.crr*G;
 const brakeLimit=Math.min(cfg.brakeDecel,cfg.mu*G);
 const driveLimit=Math.min(
  cfg.maxAccel,cfg.engineForce/Math.max(1000,totalMass)
 );

 if(braking){
  const amount=Math.min(Math.abs(speed),brakeLimit*dt);
  speed-=Math.sign(speed)*amount;
  if(Math.abs(speed)<0.01)speed=0;
  return;
 }

 if(drive!==0){
  const desired=drive*targetSpeed;
  const error=desired-speed;
  const accel=clamp(error*1.5,-brakeLimit,driveLimit);

  const lateral=Math.abs(
   speed*speed/cfg.wheelbase*Math.tan(steer)
  );
  const available=Math.sqrt(
   Math.max(0,(cfg.mu*G)**2-lateral*lateral)
  );

  speed+=clamp(accel,-available,available)*dt;
 }else{
  const coast=Math.min(Math.abs(speed),rolling*dt);
  speed-=Math.sign(speed)*coast;
 }

 if(Math.abs(speed)<0.005)speed=0;
}

function stopRoutePlayback(reason){
 routePlayback.active=false;
 routePlayback.paused=false;
 routePlayback.awaitingGearConfirmation=false;
 routePlayback.pendingGear=null;
 $('routeConfirmGear').hidden=true;
 $('routeConfirmGear').disabled=true;
 drive=0;
 braking=false;
 speed=0;
 steer=0;
 $('steer').value='0';
 $('steerLabel').textContent='0°';
 $('routePause').textContent='Ⅱ Пауза';
 $('mode').textContent='Автопроезд остановлен';
 if(reason){
  routePlayback.stopReason=reason;
  $('routeAdvice').textContent=reason;
 }
 draw();
}

function updateRoutePlayback(){
 if(!routePlayback.active||routePlayback.paused)return;
 if(plannedRoute.length<2){stopRoutePlayback('Нет рассчитанной траектории автопоезда.');return;}
 let index=clamp(routePlayback.index||1,1,plannedRoute.length-1);
 const nextGearTransition=planner.nextGearTransitionBoundary(
  plannedRoute,index,routePlayback.currentGear
 );
 const shiftBoundaryIndex=nextGearTransition?
  nextGearTransition.boundaryIndex:plannedRoute.length-1;
 if(nextGearTransition&&shiftBoundaryIndex<index){
  stopRoutePlayback('Автопроезд остановлен: граница смены передачи уже оказалась позади текущего индекса траектории.');
  return;
 }
 while(index<plannedRoute.length-2&&index<shiftBoundaryIndex&&
  Math.hypot(plannedRoute[index].x-state.x,plannedRoute[index].y-state.y)<0.22)index++;
 routePlayback.index=index;
 const gear=routePlayback.currentGear;
 if(nextGearTransition&&index>=shiftBoundaryIndex){
  const boundary=plannedRoute[shiftBoundaryIndex];
  const boundaryDistance=Math.hypot(boundary.x-state.x,boundary.y-state.y);
  if(boundaryDistance<0.025){
   if(Math.abs(speed)>0.02){
    drive=0;braking=true;steer=0;$('steer').value='0';
    $('routeAdvice').textContent='Переключение передачи: виртуальная модель тормозит у границы сегмента до скорости не выше 0,02 м/с. После остановки потребуется отдельное подтверждение; длительность реального переключения неизвестна.';
    return;
   }
   routePlayback.awaitingGearConfirmation=true;
   routePlayback.pendingGear=nextGearTransition.to;
   routePlayback.paused=true;
   drive=0;braking=false;speed=0;steer=0;
   $('steer').value='0';
   $('routeConfirmGear').hidden=false;
   $('routeConfirmGear').disabled=false;
   $('routeAdvice').textContent='Модельная остановка достигнута на границе смены передачи. Нажмите «Подтвердить виртуальное переключение», чтобы продолжить только симуляцию. Реальная передача, сцепление и длительность переключения не измеряются.';
   return;
  }
 }
 let nearest=Infinity;
 for(let i=Math.max(1,index-8);i<Math.min(plannedRoute.length,index+18);i++){
  nearest=Math.min(nearest,Math.hypot(plannedRoute[i].x-state.x,plannedRoute[i].y-state.y));
 }
 if(nearest>1.25){
  stopRoutePlayback('Автопроезд остановлен: модель отклонилась от рассчитанной траектории автопоезда.');
  return;
 }
 const goal=plannedRoute[plannedRoute.length-1];
 if(Math.hypot(goal.x-state.x,goal.y-state.y)<0.65&&nearest<0.8){
  const terminal=planner.goalErrors(state,routeGoal);
  if(terminal.headingError!==null&&terminal.headingError>routeGoal.headingTolerance){
   stopRoutePlayback('Автопроезд остановлен: конечная точка близка, но курс тягача не попал в заданный допуск ('+(terminal.headingError*180/Math.PI).toFixed(1)+'° ошибки).');
  }else if(terminal.articulation>routeGoal.maxTerminalArticulation){
   stopRoutePlayback('Автопроезд остановлен: конечная точка близка, но полуприцеп не выровнен ('+(terminal.articulation*180/Math.PI).toFixed(1)+'°).');
  }else{
   stopRoutePlayback('Виртуальный автопроезд завершён: конечные координаты и ориентация достигнуты в заданных допусках.');
  }
  return;
 }
 let targetIndex=Math.min(plannedRoute.length-1,index+6,shiftBoundaryIndex);
 const target=plannedRoute[targetIndex];
 const dx=target.x-state.x,dy=target.y-state.y;
 const distance=Math.max(0.5,Math.hypot(dx,dy));
 const desiredHeading=Math.atan2(dy,dx)+(gear<0?Math.PI:0);
 const alpha=norm(desiredHeading-state.psi);
 const command=gear*Math.atan2(2*cfg.wheelbase*Math.sin(alpha),distance);
 steer=clamp(command,-28*Math.PI/180,28*Math.PI/180);
 $('steer').value=String(Math.round(steer*180/Math.PI));
 drive=gear;
 braking=false;
 targetSpeed=0.35;
 $('speedSet').value='0.35';
 const safety=assessScene(state);
 const effective=safety.nearest.distance-2*Math.max(0,cfg.measurementUncertainty);
 if(safety.collision||safety.beta>55||(Number.isFinite(effective)&&effective<0.35)){
  stopRoutePlayback(safety.collision?
   'Автопроезд остановлен перед расчётным столкновением с '+safety.nearest.name+'.':
   safety.beta>55?'Автопроезд остановлен: угол сочленения превысил 55°.':
   'Автопроезд остановлен: зазор меньше установленного резерва.');
 }
}

function simulationStep(dt){
 if(routePlayback.active&&!routePlayback.paused)updateRoutePlayback();
 updateSpeed(dt);

 const oldX=state.x;
 const oldY=state.y;
 const steps=Math.max(1,Math.ceil(dt/0.01));
 const h=dt/steps;

 for(let i=0;i<steps;i++){
  const previous=state;
  const next=rk4(state,speed,steer,h);
  state=next;
  if(routePlayback.active){
   const safety=assessScene(state);
   const effective=safety.nearest.distance-2*Math.max(0,cfg.measurementUncertainty);
   if(safety.collision||safety.beta>55||(Number.isFinite(effective)&&effective<0.35)){
    state=previous;
    stopRoutePlayback(safety.collision?
     'Автопроезд остановлен перед расчётным столкновением с препятствием '+safety.nearest.name+'.':
     safety.beta>55?
     'Автопроезд остановлен: опасный угол сочленения '+safety.beta.toFixed(1)+'°.':
     'Автопроезд остановлен перед зоной недостаточного расчётного зазора.');
    break;
   }
  }
 }

 distance+=Math.hypot(state.x-oldX,state.y-oldY);

 const a=axlePoint(state);
 trace.push({x:state.x,y:state.y,psi:state.psi,psiT:state.psiT,ax:a.x,ay:a.y});
 if(trace.length>2500)trace.shift();

 camera={x:state.x,y:state.y};
}

function frame(timestamp){
 if(!lastTime)lastTime=timestamp;
 const dt=Math.min((timestamp-lastTime)/1000,0.04);
 lastTime=timestamp;

 try{
  if(drive!==0||braking||(routePlayback.active&&!routePlayback.paused))simulationStep(dt);
  draw();
 }catch(error){
  console.error('TruckVision render loop failed:',error);
  routePlayback.active=false;
  drive=0;
  braking=false;
  const advice=$('routeAdvice');
  if(advice)advice.textContent='Ошибка отрисовки: '+(error&&error.message?error.message:String(error))+'. Перезагрузите страницу; сообщение сохранено для диагностики.';
  const status=$('mode');
  if(status){status.textContent='Ошибка симулятора';status.style.color='#ff8585';}
  return;
 }
 requestAnimationFrame(frame);
}

function setSteer(){
 steer=Number($('steer').value)*Math.PI/180;
 draw();
}

function startDrive(direction){
 drive=direction;
 braking=false;
}

function applyCameraGateRegistration(registration){
 const centerX=registration&&registration.centerX;
 const centerY=registration&&registration.centerY;
 const clearWidth=registration&&registration.clearWidth;
 const headingDeg=registration&&registration.headingDeg;
 const widthUncertaintyBound=registration&&registration.widthUncertaintyBound!==undefined?
  registration.widthUncertaintyBound:0;
 const clearHeight=registration&&registration.clearHeight;
 const heightUncertaintyBound=registration&&registration.heightUncertaintyBound;
 const readGateSetting=(id,key)=>{
  const input=$(id);
  if(!input)return cfg[key];
  if(String(input.value).trim()==='')return NaN;
  return Number(input.value);
 };
 const postWidth=readGateSetting('gatePostWidth','gatePostWidth');
 const depth=readGateSetting('gateDepth','gateDepth');
 const positionUncertainty=readGateSetting('gatePositionUncertainty','gatePositionUncertainty');
 const headingUncertaintyDeg=readGateSetting('gateHeadingUncertaintyDeg','gateHeadingUncertaintyDeg');
 const lintelEnabled=!!($('gateLintelEnabled')&&$('gateLintelEnabled').checked);
 const lintelAlignmentConfirmed=!!($('gateLintelAlignmentConfirmed')&&$('gateLintelAlignmentConfirmed').checked);
 const lintel=lintelEnabled?{
  clearHeight,
  clearHeightUncertainty:heightUncertaintyBound,
  floorDatumUncertainty:readGateSetting('gateLintelFloorUncertainty','gateLintelFloorUncertainty'),
  thickness:readGateSetting('gateLintelThickness','gateLintelThickness'),
  thicknessUncertainty:readGateSetting('gateLintelThicknessUncertainty','gateLintelThicknessUncertainty'),
  span:readGateSetting('gateLintelSpan','gateLintelSpan'),
  spanUncertainty:readGateSetting('gateLintelSpanUncertainty','gateLintelSpanUncertainty'),
  depth:readGateSetting('gateLintelDepth','gateLintelDepth'),
  depthUncertainty:readGateSetting('gateLintelDepthUncertainty','gateLintelDepthUncertainty')
 }:null;
 if(lintelEnabled&&!lintelAlignmentConfirmed)return false;
 if(![centerX,centerY,clearWidth,headingDeg,widthUncertaintyBound,
      postWidth,depth,positionUncertainty,headingUncertaintyDeg].every(Number.isFinite)||
    Math.abs(centerX)>100||Math.abs(centerY)>100||
    clearWidth<1||clearWidth>20||headingDeg<-180||headingDeg>180||
    widthUncertaintyBound<0.001||widthUncertaintyBound>2||
    widthUncertaintyBound>=clearWidth-0.05||
    postWidth<0.05||postWidth>2||depth<0.05||depth>10||
    positionUncertainty<0.001||positionUncertainty>2||
    headingUncertaintyDeg<0.001||headingUncertaintyDeg>45||
    !Number.isFinite(clearHeight)||clearHeight<=0.5||clearHeight>15||
    !Number.isFinite(heightUncertaintyBound)||heightUncertaintyBound<=0||heightUncertaintyBound>2){
  return false;
 }
 try{
  const gate={
   centerX,centerY,clearWidth,
   postWidth,depth,
   heading:headingDeg*Math.PI/180,
   widthUncertainty:widthUncertaintyBound,
   positionUncertainty,
   headingUncertaintyDeg
  };
  if(lintelEnabled)gate.lintel=lintel;
  planner.createGateObstacles(gate);
 }catch(error){
  return false;
 }
 cfg.gateWidth=clearWidth;
 cfg.gateWidthUncertainty=widthUncertaintyBound;
 cfg.gateCenterX=centerX;
 cfg.gateCenterY=centerY;
 cfg.gateHeadingDeg=headingDeg;
 cfg.gatePostWidth=postWidth;
 cfg.gateDepth=depth;
 cfg.gatePositionUncertainty=positionUncertainty;
 cfg.gateHeadingUncertaintyDeg=headingUncertaintyDeg;
 cfg.gateLintelEnabled=lintelEnabled;
 cfg.gateLintelClearHeight=clearHeight;
 cfg.gateLintelCameraHeightUncertainty=heightUncertaintyBound;
 if(lintelEnabled){
  cfg.gateLintelFloorUncertainty=lintel.floorDatumUncertainty;
  cfg.gateLintelThickness=lintel.thickness;
  cfg.gateLintelThicknessUncertainty=lintel.thicknessUncertainty;
  cfg.gateLintelSpan=lintel.span;
  cfg.gateLintelSpanUncertainty=lintel.spanUncertainty;
  cfg.gateLintelDepth=lintel.depth;
  cfg.gateLintelDepthUncertainty=lintel.depthUncertainty;
 }
 $('gateWidth').value=clearWidth.toFixed(2);
 $('gateWidthUncertainty').value=widthUncertaintyBound.toFixed(3);
 $('gateCenterX').value=centerX.toFixed(2);
 $('gateCenterY').value=centerY.toFixed(2);
 $('gateHeadingDeg').value=headingDeg.toFixed(2);
 $('gateLintelClearHeight').value=clearHeight.toFixed(3);
 $('gateLintelCameraHeightUncertainty').value=heightUncertaintyBound.toFixed(3);
 $('gateEnabled').checked=true;
 invalidateRouteForSettingsChange();
 draw();
 return true;
}
window.TruckVisionApplyGateRegistration=applyCameraGateRegistration;

function applyCameraObstacleRegistration(measurement,anchor,headingRadians,name,poseUncertainty,repeatability,verticalMeasurement){
 const sigmaWidth=Number(measurement?.sigmaWidth);
 const sigmaDepth=Number(measurement?.sigmaHeight);
 const mapPoseUncertainty={
  positionX:poseUncertainty?.positionX,
  positionY:poseUncertainty?.positionY,
  headingDeg:poseUncertainty?.headingDeg
 };
 const verticalHeight=verticalMeasurement?.height;
 const verticalHeightUncertainty=verticalMeasurement?.heightUncertainty;
 const verticalBaseDatumUncertainty=verticalMeasurement?.baseDatumUncertainty;
 const verticalTotalUncertainty=verticalMeasurement?.totalUncertainty;
 if(!measurement||!Number.isFinite(measurement.width)||!Number.isFinite(measurement.height)||
    measurement.width<0.08||measurement.width>20||measurement.height<0.08||measurement.height>20||
    !Number.isFinite(sigmaWidth)||!Number.isFinite(sigmaDepth)||sigmaWidth<=0||sigmaDepth<=0||
    sigmaWidth>2||sigmaDepth>2||
    measurement.conditioningWarning===true||measurement.shapeWarning===true||
    measurement.nonlinearSensitivityAvailable!==true||measurement.nonlinearWarning===true||
    !repeatability||repeatability.withinTolerance!==true||
    repeatability.status!=='repeatable-within-operator-tolerance'||
    !Number.isInteger(repeatability.count)||repeatability.count<3||repeatability.count>10||
    ![
     repeatability.meanWidth,repeatability.meanHeight,
     repeatability.maxDeviationWidth,repeatability.maxDeviationHeight,
     repeatability.rangeWidth,repeatability.rangeHeight,
     repeatability.widthTolerance,repeatability.heightTolerance,
     repeatability.maxSigmaWidth,repeatability.maxSigmaHeight,
     repeatability.widthEnvelopeBound,repeatability.heightEnvelopeBound
    ].every(Number.isFinite)||
    repeatability.meanWidth<0.08||repeatability.meanWidth>20||
    repeatability.meanHeight<0.08||repeatability.meanHeight>20||
    repeatability.maxDeviationWidth<0||repeatability.maxDeviationHeight<0||
    repeatability.widthTolerance<0.001||repeatability.widthTolerance>0.5||
    repeatability.heightTolerance<0.001||repeatability.heightTolerance>0.5||
    repeatability.maxSigmaWidth<=0||repeatability.maxSigmaWidth>2||
    repeatability.maxSigmaHeight<=0||repeatability.maxSigmaHeight>2||
    repeatability.maxSigmaWidth+1e-9<sigmaWidth||repeatability.maxSigmaHeight+1e-9<sigmaDepth||
    repeatability.widthEnvelopeBound+1e-9<2*repeatability.maxSigmaWidth||
    repeatability.heightEnvelopeBound+1e-9<2*repeatability.maxSigmaHeight||
    repeatability.maxDeviationWidth>repeatability.rangeWidth+1e-8||
    repeatability.maxDeviationHeight>repeatability.rangeHeight+1e-8||
    repeatability.maxDeviationWidth>repeatability.widthTolerance||
    repeatability.maxDeviationHeight>repeatability.heightTolerance||
    repeatability.rangeWidth<0||repeatability.rangeWidth>repeatability.widthTolerance||
    repeatability.rangeHeight<0||repeatability.rangeHeight>repeatability.heightTolerance||
    repeatability.widthEnvelopeBound<=0||repeatability.widthEnvelopeBound>2||
    repeatability.heightEnvelopeBound<=0||repeatability.heightEnvelopeBound>2||
    Math.abs(repeatability.meanWidth-measurement.width)+2*sigmaWidth>repeatability.widthEnvelopeBound+1e-9||
    Math.abs(repeatability.meanHeight-measurement.height)+2*sigmaDepth>repeatability.heightEnvelopeBound+1e-9||
    Math.abs(repeatability.meanWidth-measurement.width)>repeatability.maxDeviationWidth+1e-8||
    Math.abs(repeatability.meanHeight-measurement.height)>repeatability.maxDeviationHeight+1e-8||
    repeatability.widthEnvelopeBound>=repeatability.meanWidth-0.05||
    repeatability.heightEnvelopeBound>=repeatability.meanHeight-0.05||
    repeatability.meanWidth<0.08||repeatability.meanHeight<0.08||
    typeof name!=='string'||!name.trim()||name.trim().length>48||
    !anchor||![anchor.x,anchor.y,headingRadians].every(Number.isFinite)||
    Math.abs(anchor.x)>100||Math.abs(anchor.y)>100||
    Math.abs(headingRadians)>Math.PI+1e-9||
    ![mapPoseUncertainty.positionX,mapPoseUncertainty.positionY,mapPoseUncertainty.headingDeg].every(Number.isFinite)||
    mapPoseUncertainty.positionX<0.001||mapPoseUncertainty.positionX>2||
    mapPoseUncertainty.positionY<0.001||mapPoseUncertainty.positionY>2||
    mapPoseUncertainty.headingDeg<0.001||mapPoseUncertainty.headingDeg>45||
    !verticalMeasurement||verticalMeasurement.confirmed!==true||
    verticalMeasurement.datum!=='local-road-surface'||
    verticalMeasurement.source!=='manual-independent-measurement'||
    ![verticalHeight,verticalHeightUncertainty,verticalBaseDatumUncertainty,verticalTotalUncertainty].every(Number.isFinite)||
    verticalHeight<0.05||verticalHeight>15||
    verticalHeightUncertainty<0.001||verticalHeightUncertainty>1||
    verticalBaseDatumUncertainty<0.001||verticalBaseDatumUncertainty>1||
    verticalTotalUncertainty>2||
    Math.abs(verticalTotalUncertainty-(verticalHeightUncertainty+verticalBaseDatumUncertainty))>1e-9||
    cameraObstacles.length>=30)return false;
 const meanWidth=repeatability.meanWidth,meanDepth=repeatability.meanHeight;
 const ux=Math.cos(headingRadians),uy=Math.sin(headingRadians);
 const nx=-uy,ny=ux,hw=meanWidth/2,hh=meanDepth/2;
 const nominalPolygon=[
  {x:anchor.x-hw*ux-hh*nx,y:anchor.y-hw*uy-hh*ny},
  {x:anchor.x+hw*ux-hh*nx,y:anchor.y+hw*uy-hh*ny},
  {x:anchor.x+hw*ux+hh*nx,y:anchor.y+hw*uy+hh*ny},
  {x:anchor.x-hw*ux+hh*nx,y:anchor.y-hw*uy+hh*ny}
 ];
 // The repeatability envelope is a bound on each full dimension. A uniform
 // half-margin using the larger bound is conservative for both axes.
 const measurementInflation=Math.max(repeatability.widthEnvelopeBound,repeatability.heightEnvelopeBound)/2;
 let poseEnvelope,conservativePolygon;
 try{
  poseEnvelope=planner.inflateObstaclePoseUncertainty(nominalPolygon,anchor,mapPoseUncertainty);
  conservativePolygon=planner.inflateConvexPolygon(poseEnvelope.polygon,measurementInflation);
 }catch(_){return false;}
 const roadSurfaceText=$('roadSurface')?.value.trim()||'';
 let enteredRoadSurface=null;
 try{enteredRoadSurface=JSON.parse(roadSurfaceText||'null');}catch(_){}
 const roadSurfaceKeys=[
  'originX','originY','originZ','gradeX','gradeY',
  'originZUncertainty','gradeXUncertainty','gradeYUncertainty','surfaceResidualUncertainty'
 ];
 const roadSurfaceMatchesApplied=!!cfg.roadSurface&&!!enteredRoadSurface&&
  roadSurfaceKeys.every(key=>Number.isFinite(enteredRoadSurface[key])&&
   Math.abs(enteredRoadSurface[key]-cfg.roadSurface[key])<=1e-12);
 const roadSurfaceErrors=cfg.roadSurface&&typeof physics.validateRoadSurface==='function'?
  physics.validateRoadSurface(cfg.roadSurface):['Калибровка поверхности дороги отсутствует.'];
 const verticalDatumCalibrated=roadSurfaceMatchesApplied&&
  Array.isArray(roadSurfaceErrors)&&roadSurfaceErrors.length===0;
 const obstacle={
  kind:'camera-obstacle',
  name:String(name||'Препятствие камеры').trim().slice(0,48)||'Препятствие камеры',
  polygon:conservativePolygon,
  nominalPolygon,
  anchor:{x:anchor.x,y:anchor.y},
  mapPoseUncertainty,
  positionUncertaintyX:mapPoseUncertainty.positionX,
  positionUncertaintyY:mapPoseUncertainty.positionY,
  headingUncertaintyDeg:mapPoseUncertainty.headingDeg,
  positionInflation:poseEnvelope.positionInflation,
  headingInflation:poseEnvelope.headingInflation,
  poseInflation:poseEnvelope.totalInflation,
  measuredWidth:meanWidth,
  measuredDepth:meanDepth,
  sigmaWidth,
  sigmaDepth,
  widthUncertaintyBound:repeatability.widthEnvelopeBound,
  depthUncertaintyBound:repeatability.heightEnvelopeBound,
  repeatability:{
   count:repeatability.count,
   widthTolerance:repeatability.widthTolerance,
   depthTolerance:repeatability.heightTolerance,
   rangeWidth:repeatability.rangeWidth,
   rangeDepth:repeatability.rangeHeight
  },
  measurementInflation,
  verticalHeightMeasurement:{
   height:verticalHeight,
   heightUncertainty:verticalHeightUncertainty,
   baseDatumUncertainty:verticalBaseDatumUncertainty,
   totalUncertainty:verticalTotalUncertainty,
   datum:'local-road-surface',
   source:'manual-independent-measurement',
   confirmed:true
  },
  verticalClearanceMode:verticalDatumCalibrated?
   'calibrated-road-surface-relative-height-range':
   'horizontal-only-until-road-surface-calibrated',
  ...(verticalDatumCalibrated?{
   zMin:0,zMax:verticalHeight,zUncertainty:verticalTotalUncertainty
  }:{}),
  uncertaintyModel:'observed-range-plus-per-frame-2sigma-screening-plus-bounded-map-pose-plus-manual-height'
 };
 try{if(planner.validateObstacles([obstacle]).length)return false;}catch(_){return false;}
 window.TruckVisionCameraObstacleVerticalStatus=verticalDatumCalibrated?
  'Высота введена вручную. Вертикальный диапазон привязан к откалиброванной плоскости дороги; вертикальное разделение проверяется только для компонентов с измеренными профилями.':
  'Высота введена вручную, но плоскость дороги не откалибрована. Препятствие учитывается только по горизонтальной 2D-оболочке; вертикальное разделение не используется.';
 cameraObstacles.push(obstacle);
 invalidateRouteForSettingsChange();
 draw();
 return true;
}
window.TruckVisionAddCameraObstacle=applyCameraObstacleRegistration;
window.TruckVisionClearCameraObstacles=()=>{
 if(!cameraObstacles.length)return true;
 cameraObstacles.length=0;
 invalidateRouteForSettingsChange();
 draw();
 return true;
};

function applySettings(){
 const result=readSettings();
 if(!result.ok){
  $('routeAdvice').textContent='Параметры не применены: '+result.errors.join(' ');
  $('difficultyAdvice').textContent='Построение маршрута заблокировано до исправления параметров.';
  draw();
  return;
 }
 if(result.changed)invalidateRouteForSettingsChange();
 else $('routeAdvice').textContent='Параметры модели проверены. Предыдущий маршрут сохраняет актуальность.';
 draw();
}

$('forward').onpointerdown=()=>startDrive(1);
$('reverse').onpointerdown=()=>startDrive(-1);

['forward','reverse'].forEach(id=>{
 const button=$(id);
 button.onpointerup=()=>{drive=0;};
 button.onpointerleave=()=>{drive=0;};
 button.onpointercancel=()=>{drive=0;};
});

$('brake').onclick=()=>{
 drive=0;
 braking=true;
};

$('left').onclick=()=>{
 $('steer').value=clamp(Number($('steer').value)-3,-35,35);
 setSteer();
};

$('right').onclick=()=>{
 $('steer').value=clamp(Number($('steer').value)+3,-35,35);
 setSteer();
};

$('straight').onclick=()=>{
 $('steer').value=0;
 setSteer();
};

$('steer').oninput=setSteer;

$('speedSet').oninput=()=>{
 targetSpeed=Number($('speedSet').value);
 draw();
};

$('reset').onclick=()=>{
 routeBuildToken++;cancelPlannerWorker('Расчёт отменён сбросом симулятора.');$('planRoute').disabled=false;
 stopRoutePlayback('');
 resetState();
 plannedRoute=[];
 routeGoal=null;
 routeAssessment=null;
 draw();
};

$('center').onclick=()=>{
 camera={x:state.x,y:state.y};
 draw();
};

$('trace').onclick=()=>{
 showTrace=!showTrace;
 $('trace').textContent='Траектория: '+(showTrace?'вкл.':'выкл.');
 draw();
};

$('apply').onclick=applySettings;

$('gateEnabled').onchange=()=>{
 invalidateRouteForSettingsChange();
 draw();
};
$('gateLintelEnabled').onchange=()=>{
 const result=readSettings();
 invalidateRouteForSettingsChange();
 if(!result.ok){
  $('routeAdvice').textContent='Параметры верхней перемычки не применены: '+result.errors.join(' ');
  $('difficultyAdvice').textContent='Построение маршрута заблокировано до проверки измерений перемычки.';
 }
 draw();
};
$('gateLintelAlignmentConfirmed').onchange=()=>{
 const result=readSettings();
 invalidateRouteForSettingsChange();
 if(!result.ok){
  $('routeAdvice').textContent='Подтверждение геометрии перемычки отсутствует или настройки невалидны: '+result.errors.join(' ');
  $('difficultyAdvice').textContent='Построение маршрута заблокировано до повторной проверки вертикальной геометрии.';
 }
 draw();
};

$('obstaclesToggle').onclick=()=>{
 showObstacles=!showObstacles;
 $('obstaclesToggle').textContent=
  'Препятствия: '+(showObstacles?'вкл.':'выкл.');
 if(!showObstacles){
  routeBuildToken++;cancelPlannerWorker('Расчёт отменён: препятствия отключены.');$('planRoute').disabled=false;
  stopRoutePlayback('');
  routeAssessment=null;
  $('routeAdvice').textContent='Препятствия отключены. Автопроезд заблокирован; включите препятствия и заново постройте маршрут.';
  $('difficultyAdvice').textContent='Сложность не подтверждена: препятствия отключены.';
 }
 draw();
};

$('helperToggle').onclick=()=>{
 helperEnabled=!helperEnabled;
 $('helperToggle').textContent=
  'Помощник: '+(helperEnabled?'вкл.':'выкл.');
 updateAssistant();
};

$('scenarioReset').onclick=()=>{
 routeBuildToken++;cancelPlannerWorker('Расчёт отменён сбросом площадки.');$('planRoute').disabled=false;
 stopRoutePlayback('');
 resetState();
 showObstacles=true;
 helperEnabled=true;
 if($('gateEnabled'))$('gateEnabled').checked=false;
 $('obstaclesToggle').textContent='Препятствия: вкл.';
 $('helperToggle').textContent='Помощник: вкл.';
 draw();
};

$('assess').onclick=()=>{
 const settings=readSettings();
 if(!settings.ok){
  $('gateResult').textContent='Оценка заблокирована: '+settings.errors.join(' ');
  return;
 }
 if(settings.changed)invalidateRouteForSettingsChange();

 const uncertainty=Math.max(0,cfg.measurementUncertainty);
 const usable=cfg.gateWidth-2*(cfg.margin+uncertainty);
 const difference=usable-cfg.trailerWidth;

 let result;

 if(difference<0)result='Проём уже ширины полуприцепа';
 else if(difference<0.3)result='Очень малый боковой запас';
 else if(difference<0.8)result='Ограниченный запас: нужен точный манёвр';
 else result='По ширине имеется предварительный запас';

 $('gateResult').textContent=
  result+'. Полезная ширина: '+usable.toFixed(2)+
  ' м; ширина полуприцепа: '+cfg.trailerWidth.toFixed(2)+
  ' м; дополнительный резерв неопределённости: '+uncertainty.toFixed(2)+' м с каждой стороны; остаточный запас: '+difference.toFixed(2)+
  ' м. '+($('gateEnabled').checked?
   'Косяки ворот включены в карту как два ориентированных препятствия; они проверяются вместе со всем автопоездом.'+
   (cfg.gateLintelEnabled?' Верхняя перемычка также добавлена как отдельная вертикальная призма; вертикальное разделение подтверждается только для компонентов с измеренными высотными профилями.':' Верхняя перемычка не включена в проверку.') :
   'Косяки ворот не включены в карту: включите переключатель и задайте положение/курс проёма. Камера не определяет мировую позу ворот автоматически.')+' Это не гарантия проезда.';
};

$('alternativeRouteSelect').onchange=()=>{
 const selectedValue=String($('alternativeRouteSelect').value??'').trim();
 const clearSelection=message=>{
  if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: '+message);
  plannedRoute=[];
  routeAssessment=null;
  $('routeAdvice').textContent=message;
  $('difficultyAdvice').textContent='Сложность не рассчитана. Выберите допустимый вариант и постройте маршрут заново.';
  draw();
 };
 // The empty placeholder must never coerce to index 0 (Number('') === 0).
 if(selectedValue===''){
  clearSelection('Выбор альтернативной траектории снят. Маршрут очищен; выберите вариант заново.');
  return;
 }
 if(!/^\d+$/.test(selectedValue)){
  clearSelection('Выбор отклонён: индекс варианта имеет неверный формат. Маршрут очищен.');
  return;
 }
 const index=Number(selectedValue);
 if(!Number.isSafeInteger(index)||index<0||index>=alternativeRoutes.length){
  clearSelection('Выбор отклонён: такого варианта нет. Маршрут очищен; выполните расчёт заново.');
  return;
 }
 const candidate=alternativeRoutes[index];
 if(!candidate||!Array.isArray(candidate.states)||candidate.states.length<2){
  clearSelection('Выбор отклонён: траектория кандидата неполная. Маршрут очищен.');
  return;
 }
 if(routePlayback.active){
  stopRoutePlayback('Виртуальный автопроезд остановлен: выбран другой вариант траектории.');
 }
 plannedRoute=candidate.states;
 routeAssessment=routeMetrics(
  plannedRoute,
  Number(candidate.metrics?.minimumClearanceLowerBound??Infinity),
  candidate.metrics?.sweptClearanceDetail??null,
  candidate.trajectoryClearance??null
 );
 const passed=!routeAssessment.collision&&routeAssessment.reached&&
  !routeAssessment.articulationBlocked&&!routeAssessment.clearanceInsufficient&&
  Number.isFinite(routeAssessment.effectiveClearance);
 $('routeAdvice').textContent='Выбран вариант '+(index+1)+'. '+(passed?'Повторная проверка пройдена по текущей модели.':'Проверка не пройдена: '+routeAssessment.reasons.join('; '));
 const selectedPartClearanceText=routeAssessment.clearanceByPart.map(part=>{
  const lower=Number.isFinite(part.minimumClearanceLowerBound)?part.minimumClearanceLowerBound.toFixed(2)+' м':'не определён';
  const detail=part.limitingDetail;
  const obstacle=detail?.obstacleAtReference||detail?.obstacle||'препятствие не выделено';
  const obstacleText=detail?.kind==='segment-lower-bound'?
   'препятствие у начала сегмента «'+obstacle+'»':'препятствие «'+obstacle+'»';
  const location=detail?.kind==='segment-lower-bound'?
   'сегмент '+Number(detail.distanceAlongRoute).toFixed(1)+' м, '+(detail.gear<0?'задний ход':'вперёд'):
   detail?.kind==='sampled-pose'?'поза '+Number(detail.distanceAlongRoute).toFixed(1)+' м':'положение не определено';
  return part.vehiclePart+': '+lower+', '+obstacleText+', '+location;
 }).join('; ');
 $('difficultyAdvice').textContent='Сложность: '+routeAssessment.level+'. Длина '+routeAssessment.length.toFixed(1)+' м; пиковый угол сочленения '+routeAssessment.maxBeta.toFixed(1)+'°; минимальный эффективный зазор '+(Number.isFinite(routeAssessment.effectiveClearance)?routeAssessment.effectiveClearance.toFixed(2)+' м':'не определён')+'. Зазоры по частям вдоль маршрута: '+selectedPartClearanceText+'. '+wheelClearanceSummaryText(routeAssessment)+'. '+wheelTrackSummaryText(routeAssessment)+'. Причины: '+routeAssessment.reasons.join('; ')+'. '+(passed?'Автопроезд разрешён только в виртуальной модели.':'Автопроезд остаётся заблокирован.');
 const selectedRecommendation=planner.recommendStart(plannedRoute);
 updateRecommendationCard({
  status:'Вариант '+(index+1)+' выбран',
  level:routeAssessment.level,
  start:selectedRecommendation.initialGear<0?'Начать задним ходом':'Начать движением вперёд',
  turn:selectedRecommendation.firstTurn?
   (selectedRecommendation.steeringWheelDirection==='left'?'Руль влево':'Руль вправо')+' примерно через '+selectedRecommendation.firstTurnDistance.toFixed(1)+' м'+(selectedRecommendation.firstTurnGear<0?' задним ходом':' вперёд')+'; '+(selectedRecommendation.tractorYawDirection==='left'?'курс тягача влево':'курс тягача вправо'):
   'Значимого первого поворота руля не найдено',
  reasons:routeAssessment.reasons.slice(0,4).join('; ')+'. Пиковый угол '+routeAssessment.maxBeta.toFixed(1)+'°; эффективный зазор '+(Number.isFinite(routeAssessment.effectiveClearance)?routeAssessment.effectiveClearance.toFixed(2)+' м':'не определён')+'.',
  check:passed?'Выбранный вариант повторно проверен; автопроезд возможен только в виртуальной модели.':'Проверка не пройдена; виртуальный автопроезд остаётся заблокирован.'
 });

 draw();
};

$('routeStart').onclick=()=>{
 const settings=readSettings();
 if(!settings.ok){
  stopRoutePlayback('Автопроезд заблокирован: '+settings.errors.join(' '));
  $('difficultyAdvice').textContent='Сложность не подтверждена из-за некорректных параметров модели.';
  return;
 }
 if(settings.changed){
  invalidateRouteForSettingsChange();
  $('routeAdvice').textContent='Автопроезд не запущен: параметры изменились. Постройте маршрут заново.';
  return;
 }
 if(!showObstacles){
  $('routeAdvice').textContent='Автопроезд заблокирован: сначала включите отображение и проверку препятствий.';
  return;
 }
 if(plannedRoute.length<2||!routeAssessment){
  $('routeAdvice').textContent='Сначала нажмите «Построить путь» и дождитесь оценки маршрута.';
  return;
 }
 const clearance=routeAssessment.effectiveClearance;
 const wheelCheckUnverified=!routeAssessment.wheelClearance||
  !!routeAssessment.wheelClearanceError||!!routeAssessment.wheelClearanceIncomplete;
 if(routeAssessment.collision||!routeAssessment.reached||routeAssessment.clearanceInsufficient||
    wheelCheckUnverified||routeAssessment.maxBeta>=ARTICULATION_STOP_DEMO_DEG||
    (Number.isFinite(clearance)&&clearance<0.35)){
  $('routeAdvice').textContent=wheelCheckUnverified?
   'Автопроезд не запущен: проверка отдельных колёс не завершена или не подтверждена. Исправьте входные данные и постройте маршрут заново.':
   'Автопроезд не запущен: предварительная проверка маршрута выявила риск или не смогла подтвердить прохождение. Перестройте маршрут и проверьте препятствия, угол сочленения и зазоры.';
  return;
 }
 if(routePlayback.active&&routePlayback.awaitingGearConfirmation){
  $('routeAdvice').textContent='Сначала подтвердите виртуальное переключение отдельной кнопкой. Кнопка «Старт» не обходит это подтверждение.';
  return;
 }
 if(routePlayback.active&&routePlayback.paused){
  routePlayback.paused=false;
  $('routePause').textContent='Ⅱ Пауза';
  return;
 }
 routePlayback={active:true,paused:false,index:1,stopReason:'',currentGear:plannedRoute[1].gear||1,
  pendingGear:null,awaitingGearConfirmation:false,confirmedGearChanges:0};
 $('routeConfirmGear').hidden=true;
 $('routeConfirmGear').disabled=true;
 speed=0;drive=0;braking=false;
 targetSpeed=0.6;
 $('speedSet').value='0.6';
 $('routeAdvice').textContent='Виртуальный автопроезд начат. Начальная передача выбрана только для симуляции; фактическая КПП автомобиля не подключена.';
};
$('routePause').onclick=()=>{
 if(!routePlayback.active)return;
 if(routePlayback.awaitingGearConfirmation){
  $('routeAdvice').textContent='Переключение ожидает отдельного подтверждения. Используйте кнопку «Подтвердить виртуальное переключение».';
  return;
 }
 routePlayback.paused=!routePlayback.paused;
 if(routePlayback.paused){drive=0;braking=false;speed=0;$('routePause').textContent='▶ Продолжить';}
 else $('routePause').textContent='Ⅱ Пауза';
};
$('routeConfirmGear').onclick=()=>{
 if(!routePlayback.active||!routePlayback.awaitingGearConfirmation||
    ![-1,1].includes(routePlayback.pendingGear))return;
 const confirmedGear=routePlayback.pendingGear;
 routePlayback.currentGear=confirmedGear;
 routePlayback.pendingGear=null;
 routePlayback.awaitingGearConfirmation=false;
 routePlayback.paused=false;
 routePlayback.confirmedGearChanges=(routePlayback.confirmedGearChanges||0)+1;
 $('routeConfirmGear').hidden=true;
 $('routeConfirmGear').disabled=true;
 $('routePause').textContent='Ⅱ Пауза';
 $('routeAdvice').textContent='Виртуальное переключение подтверждено для симуляции ('+
  (confirmedGear<0?'задний ход':'движение вперёд')+
  '). Реальная передача, сцепление и время переключения не измеряются.';
};
$('routeStop').onclick=()=>stopRoutePlayback('Виртуальный автопроезд остановлен пользователем.');
$('planRoute').onclick=()=>{
 stopRoutePlayback('');
 return buildRoute();
};

$('clearRoute').onclick=()=>{
 routeBuildToken++;cancelPlannerWorker('Расчёт отменён: маршрут удалён.');$('planRoute').disabled=false;
 stopRoutePlayback('');
 plannedRoute=[];
 alternativeRoutes=[];
 $('alternativeRouteSelect').style.display='none';
 $('alternativeRouteSelect').value='';
 routeGoal=null;
 routeAssessment=null;
 $('routeAdvice').textContent='Маршрут удалён.';
 $('difficultyAdvice').textContent='Сложность: пока не рассчитана.';
 updateRecommendationCard({status:'Маршрут удалён'});
 draw();
};

$('model').onchange=draw;

window.addEventListener('keydown',event=>{
 if(event.key==='ArrowLeft'){
  $('steer').value=clamp(Number($('steer').value)-1,-35,35);
  setSteer();
 }
 if(event.key==='ArrowRight'){
  $('steer').value=clamp(Number($('steer').value)+1,-35,35);
  setSteer();
 }
 if(event.key==='ArrowUp')startDrive(1);
 if(event.key==='ArrowDown')startDrive(-1);
 if(event.code==='Space'){
  event.preventDefault();
  drive=0;
  braking=true;
 }
});

window.addEventListener('keyup',event=>{
 if(event.key==='ArrowUp'||event.key==='ArrowDown')drive=0;
});

const initialSettings=readSettings();
if(!initialSettings.ok)console.error('Invalid initial TruckVision configuration:',initialSettings.errors);
targetSpeed=Number($('speedSet').value);
resetState();
draw();
requestAnimationFrame(frame);

})();
