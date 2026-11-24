
(()=>{
'use strict';

const $=id=>document.getElementById(id);
const canvas=$('view');
const ctx=canvas.getContext('2d');
const G=9.81;
const SCALE=22;
const ARTICULATION_WARNING_DEG=45;
const ARTICULATION_STOP_DEMO_DEG=55;

const cfg={
 wheelbase:3.8,
 hitchOffset:0.575,
 tractorMass:8500,
 engineForce:24000,
 trailerLength:13.6,
 trailerWidth:2.55,
 kingpinAxles:8,
 trailerAxleSpacing:1.3,
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
 margin:0.2,
 measurementUncertainty:0.15,
 obstaclePositionUncertainty:0.05,
 obstacleSizeUncertainty:0.02,
 vehicleFootprintUncertainty:0.03,
 wheelbaseUncertainty:0,
 hitchOffsetUncertainty:0,
 kingpinAxlesUncertainty:0,
 trailerLengthUncertainty:0,
 trailerWidthUncertainty:0
};

let state,steer=0,targetSpeed=2,speed=0,drive=0;
let braking=false,distance=0,trace=[],showTrace=true;
let camera={x:0,y:0},lastTime=0;
let showObstacles=true,helperEnabled=true;
let plannedRoute=[],alternativeRoutes=[],routeGoal=null,routeAssessment=null;
let routePlayback={active:false,paused:false,index:1,stopReason:'',currentGear:1};
let plannerWorker=null,plannerRequestId=0,plannerReject=null,routeBuildToken=0;

// Учебный коридор: стены параллельны направлению движения (+X).
// Между внутренними гранями стен остаётся 9.3 м; паллеты стоят
// снаружи коридора и не перекрывают прямой тестовый маршрут к цели (22, 0).
const cameraObstacles=[]; // Legacy diagnostic API; not part of the normal driver workflow.
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
 return {
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
}
function sceneObstacles(){
 if(perceptionFrameTimestamp&&Date.now()-perceptionFrameTimestamp>1500){
  perceptionObstacles=[];perceptionFrameId=null;perceptionFrameTimestamp=0;
  updatePerceptionStatus('Данные датчиков устарели. Объекты удалены из активной карты; требуется новый кадр.');
  if(typeof invalidateRouteForSettingsChange==='function')invalidateRouteForSettingsChange();
 }
 const result=perceptionFrameTimestamp?[]:obstacles.map(obstacle=>({...obstacle}));
 if(!perceptionFrameTimestamp)result.push(...cameraObstacles.map(obstacle=>({...obstacle,polygon:obstacle.polygon.map(point=>({...point}))})));
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
  const input=$(key);
  if(input){
   if(String(input.value).trim()==='')return {ok:false,changed:false,errors:[key+': поле не должно быть пустым.']};
   candidate[key]=Number(input.value);
  }
 }
 const errors=physics.validateVehicleParams(candidate);
 const limits={
  tractorMass:[1000,50000],
  engineForce:[1000,200000],
  trailerMass:[1000,100000],
  mu:[0.05,1.5],
  crr:[0,0.1],
  brakeDecel:[0.1,12],
  maxAccel:[0.1,3],
  gateWidth:[1,20],
  gateCenterX:[-100,100],
  gateCenterY:[-100,100],
  gateHeadingDeg:[-180,180],
  gateWidthUncertainty:[0,2],
  gatePositionUncertainty:[0,2],
  gateHeadingUncertaintyDeg:[0,45],
  gatePostWidth:[0.05,2],
  gateDepth:[0.05,10],
  margin:[0,5],
  measurementUncertainty:[0,5],
  obstaclePositionUncertainty:[0,1],
  obstacleSizeUncertainty:[0,1],
  vehicleFootprintUncertainty:[0,0.5],
  wheelbaseUncertainty:[0,2],
  hitchOffsetUncertainty:[0,1],
  kingpinAxlesUncertainty:[0,5],
  trailerLengthUncertainty:[0,5],
  trailerWidthUncertainty:[0,1.5]
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
 const parameterUncertaintyMap={
  wheelbaseUncertainty:'wheelbase',
  hitchOffsetUncertainty:'hitchOffset',
  kingpinAxlesUncertainty:'kingpinAxles',
  trailerLengthUncertainty:'trailerLength',
  trailerWidthUncertainty:'trailerWidth'
 };
 for(const [uncertaintyKey,parameterKey] of Object.entries(parameterUncertaintyMap)){
  const uncertainty=candidate[uncertaintyKey];
  if(!Number.isFinite(uncertainty)||uncertainty<0)continue;
  if(uncertainty===0)continue;
  for(const endpoint of [candidate[parameterKey]-uncertainty,candidate[parameterKey]+uncertainty]){
   const endpointErrors=physics.validateVehicleParams({...candidate,[parameterKey]:endpoint});
   endpointErrors.forEach(message=>errors.push(uncertaintyKey+': крайнее значение '+parameterKey+'='+endpoint.toFixed(3)+' м: '+message));
  }
 }
 if(errors.length)return {ok:false,changed:false,errors};
 const changed=Object.keys(cfg).some(key=>!Object.is(cfg[key],candidate[key]));
 Object.assign(cfg,candidate);
 return {ok:true,changed,errors:[]};
}

function vehicleParameterBoundsFromSettings(){
 const mapping={
  wheelbaseUncertainty:'wheelbase',
  hitchOffsetUncertainty:'hitchOffset',
  kingpinAxlesUncertainty:'kingpinAxles',
  trailerLengthUncertainty:'trailerLength',
  trailerWidthUncertainty:'trailerWidth'
 };
 const bounds={};
 for(const [uncertaintyKey,parameterKey] of Object.entries(mapping)){
  const uncertainty=cfg[uncertaintyKey];
  if(uncertainty>0){
   bounds[parameterKey]={
    min:cfg[parameterKey]-uncertainty,
    max:cfg[parameterKey]+uncertainty
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
 camera={x:0,y:0};
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

function vehiclePolygons(q){return physics.vehiclePolygons(q,cfg);}

function obstaclePolygon(o){
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

function assessScene(q=state){
 const vehicle=vehiclePolygons(q);
 const bodyNames=['тягач','кабина','полуприцеп'];
 let nearest={distance:Infinity,name:'нет заданных препятствий',vehiclePart:null,bodyIndex:null};
 let collision=false;

 if(showObstacles){
  for(const o of sceneObstacles()){
   const op=obstaclePolygon(o);
   for(let i=0;i<vehicle.length;i++){
    const d=polygonDistance(vehicle[i],op);
    if(d<nearest.distance){
     nearest={distance:d,name:o.name,vehiclePart:bodyNames[i]||('часть '+(i+1)),bodyIndex:i};
    }
    if(d===0)collision=true;
   }
  }
 }
 return{
  nearest,
  collision,
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
  const effectiveClearance=a.nearest.distance-2*Math.max(0,cfg.measurementUncertainty);
  const partText=a.nearest.vehiclePart?'Часть автопоезда: '+a.nearest.vehiclePart+'. ':'';

  if(a.collision){
   message='Стоп! Расчётные контуры '+(a.nearest.vehiclePart||'автопоезда')+
    ' пересекаются с препятствием '+a.nearest.name+'. Остановитесь и измените траекторию.';
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
 const obstaclePolygons=showObstacles?sceneObstacles().map(obstacle=>{
  const nominal=obstaclePolygon(obstacle);
  const expansion=cfg.obstaclePositionUncertainty+cfg.obstacleSizeUncertainty/2;
  return Array.isArray(obstacle.polygon)?
   planner.inflateConvexPolygon(nominal,Math.SQRT2*expansion):
   obstaclePolygon({...obstacle,w:obstacle.w+2*expansion,h:obstacle.h+2*expansion});
 }):[];
 let previous=null;
 for(const q of samples){
  const polygons=physics.vehiclePolygons(q,{...cfg,footprintUncertainty:cfg.vehicleFootprintUncertainty});
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
 const expansion=cfg.obstaclePositionUncertainty+cfg.obstacleSizeUncertainty/2;
 for(const o of sceneObstacles()){
  const nominal=obstaclePolygon(o);
  const expanded=Array.isArray(o.polygon)?
   planner.inflateConvexPolygon(nominal,Math.SQRT2*expansion):
   obstaclePolygon({...o,w:o.w+2*expansion,h:o.h+2*expansion});
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

function routeMetrics(path,minimumClearanceLowerBound=Infinity,sweptClearanceDetail=null){
 let length=0,gearChanges=0,steeringChanges=0,reverseDistance=0,reverseSegments=0;
 let minClearance=Infinity,minClearanceDetail=null,maxBeta=0,collision=false;
 const checkClearance=showObstacles?planner.createClearanceChecker(sceneObstacles(),cfg):null;
 let previousGear=path[0]?.gear||1,previousSteer=path[0]?.steer||0,wasReverse=false;
 for(let i=0;i<path.length;i++){
  const q=path[i];
  if(i>0){
   const prev=path[i-1];
   const segmentLength=Math.hypot(q.x-prev.x,q.y-prev.y);
   length+=segmentLength;
   const reversing=q.gear<0;
   if(reversing){
    reverseDistance+=segmentLength;
    if(!wasReverse)reverseSegments++;
   }
   if(q.gear!==previousGear)gearChanges++;
   if(Math.abs(q.steer-previousSteer)>0.12)steeringChanges++;
   wasReverse=reversing;
  }else wasReverse=q.gear<0;
  previousGear=q.gear||previousGear;
  previousSteer=q.steer||0;
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
 const turns=gearChanges+steeringChanges;
 const clearanceForAssessment=Math.min(minClearance,minimumClearanceLowerBound);
 const effectiveClearance=clearanceForAssessment-2*Math.max(0,cfg.measurementUncertainty);
 const finalErrors=planner.goalErrors(path[path.length-1],routeGoal);
 const terminalHeadingMismatch=finalErrors.headingError!==null&&finalErrors.headingError>routeGoal.headingTolerance;
 const terminalArticulationMismatch=finalErrors.articulation>routeGoal.maxTerminalArticulation;
 const reached=path.length>1&&finalErrors.position<=routeGoal.positionTolerance&&
  !terminalHeadingMismatch&&!terminalArticulationMismatch;
 const finalHeadingErrorDeg=(finalErrors.headingError??0)*180/Math.PI;
 const finalArticulationDeg=finalErrors.articulation*180/Math.PI;
 const clearanceInsufficient=Number.isFinite(effectiveClearance)&&effectiveClearance<0.35;
 const articulationWarning=maxBeta>=ARTICULATION_WARNING_DEG;
 const articulationBlocked=maxBeta>=ARTICULATION_STOP_DEMO_DEG;
 const score=gearChanges*1.2+steeringChanges*0.4+(reverseDistance>0?1.5:0)+(articulationWarning?3:maxBeta>30?1.5:0)+
  (effectiveClearance<0.35?4:effectiveClearance<0.75?2:effectiveClearance<1.5?1:0)+(!reached?3:0)+(collision?8:0);
 const reasons=[];
 if(reverseDistance>0)reasons.push('задний ход: '+reverseSegments+' участков, '+reverseDistance.toFixed(1)+' м');
 if(gearChanges)reasons.push('переключений передачи: '+gearChanges);
 if(steeringChanges)reasons.push('изменений рулевого режима: '+steeringChanges);
 if(collision)reasons.push('пересечение с препятствием');
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
 if(scrubExposure.maxAbsLateralVelocity>1e-3){
  reasons.push('геометрический индикатор бокового перемещения крайних осей: до '+scrubExposure.maxAbsLateralVelocity.toFixed(3)+' м/с; интегральный proxy '+scrubExposure.totalAbsoluteLateralTravel.toFixed(2)+' м, не модель шин');
 }
 if(!reasons.length)reasons.push('в проверенной модели препятствия не пересечены');
 return {length,turns,gearChanges,steeringChanges,reverseDistance,reverseSegments,
  maxAxleScrubSpeed:scrubExposure.maxAbsLateralVelocity,
  axleScrubTravelProxy:scrubExposure.totalAbsoluteLateralTravel,
  axleScrubTravelByAxle:scrubExposure.cumulativeAbsoluteLateralTravelByAxle,
  axleScrubSpeedByAxle:scrubExposure.maxAbsLateralVelocityByAxle,
  level:collision||!reached||articulationBlocked||clearanceInsufficient?'Не пройден':score>=7?'Высокая':score>=3?'Средняя':'Низкая',
  maxBeta,articulationPeak,articulationWarning,clearance:minClearance,clearanceDetail:minClearanceDetail,
  clearanceLowerBound:clearanceForAssessment,effectiveClearance,
  sweptClearanceDetail,
  collision,reached,finalHeadingErrorDeg,finalArticulationDeg,clearanceInsufficient,articulationBlocked,reasons};
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
   worker=new Worker('./planner-worker.js?v=20261124-14');
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


  
if(result.reason==='geometry-sensitive-start'||result.reason==='geometry-envelope-no-route'){
 if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: геометрическая неопределённость не подтверждает выбранный старт.');
 plannedRoute=[];
 routeAssessment=null;
 const geometry=result.geometrySensitivity||{};
 const gate=geometry.gateGeometryUncertainty;
 const gateNote=gate&&gate.enabled?
  ' Совместная оболочка ворот: ширина '+Number(gate.nominalClearWidth).toFixed(2)+' → '+Number(gate.conservativeClearWidth).toFixed(2)+' м; центр ±'+Number(gate.positionUncertaintyPerAxis).toFixed(2)+' м по X/Y; курс ±'+Number(gate.headingUncertaintyDeg).toFixed(2)+'°.':
  '';
 if(result.reason==='geometry-sensitive-start'){
  $('routeAdvice').textContent='Начальное действие меняется при одновременном учёте заданной геометрической неопределённости. Сторона и передача не выбраны; номинальный и консервативный маршруты требуют разных действий.'+gateNote;
  $('difficultyAdvice').textContent='Рекомендация неустойчива к заданным границам погрешности препятствий и внешних габаритов автопоезда. Уточните геометрию рулеткой и повторите расчёт. Проверяется ограниченная оболочка, не все возможные комбинации ошибок.'+gateNote;
 }else{
  $('routeAdvice').textContent='Поиск в расширенной геометрической оболочке не подтвердил маршрут в заданном лимите вычислений. Это не доказывает невозможность других маршрутов; рекомендация не выдаётся.'+gateNote;
  $('difficultyAdvice').textContent='Автопроезд заблокирован: выбранный маршрут не подтверждён при расширении геометрии препятствий и автопоезда. Не уменьшайте резерв только ради получения маршрута.'+gateNote;
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
    const turn=action.firstTurn==='left'?'руль влево':
     action.firstTurn==='right'?'руль вправо':'без значимого поворота руля';
    return gear+', '+turn;
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
    const turn=candidate.firstTurn==='left'?'руль влево':
     candidate.firstTurn==='right'?'руль вправо':'без значимого поворота';
    return candidate.label+': '+gear+', '+turn+', эвристический балл '+candidate.cost.toFixed(2);
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

  const m=routeMetrics(plannedRoute,result.minimumClearanceLowerBound,result.sweptClearanceDetail);
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
   firstTurnText='сторона первого поворота не выбрана: близкие по стоимости варианты расходятся';
  }else if(recommendation.firstTurn){
   const side=recommendation.firstTurn==='left'?'повернуть руль влево':'повернуть руль вправо';
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
   m.clearanceDetail.vehiclePart+' рядом с препятствием «'+m.clearanceDetail.obstacle+
   '» около X '+m.clearanceDetail.state.x.toFixed(1)+' м, Y '+
   m.clearanceDetail.state.y.toFixed(1)+' м':
   'не определена (препятствия выключены или зазор бесконечен)';
  const sweptLocationText=m.sweptClearanceDetail&&m.sweptClearanceDetail.kind==='segment-lower-bound'?
   'сегмент X '+m.sweptClearanceDetail.from.x.toFixed(1)+', Y '+m.sweptClearanceDetail.from.y.toFixed(1)+
   ' → X '+m.sweptClearanceDetail.to.x.toFixed(1)+', Y '+m.sweptClearanceDetail.to.y.toFixed(1):
   'отдельный ограничивающий сегмент не выделен';
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
   '°; конечный угол тягач–полуприцеп '+m.finalArticulationDeg.toFixed(1)+'°; геометрический индикатор боковой скорости крайних осей при расчётной скорости 0,7 м/с '+m.maxAxleScrubSpeed.toFixed(3)+' м/с; по осям (передняя/средняя/задняя): '+m.axleScrubSpeedByAxle.map(value=>value.toFixed(3)).join('/')+' м/с; накопленный proxy по осям: '+m.axleScrubTravelByAxle.map(value=>value.toFixed(2)).join('/')+' м; суммарный proxy '+m.axleScrubTravelProxy.toFixed(2)+' м (не модель шин); минимальный зазор до тестовых препятствий '+
   (Number.isFinite(m.clearance)?m.clearance.toFixed(2)+' м':'не определён')+
   '; ближайшая проверенная часть автопоезда: '+closestLocationText+
   '; нижняя оценка между узлами '+(Number.isFinite(m.clearanceLowerBound)?m.clearanceLowerBound.toFixed(2)+' м':'не ограничена')+
   ' на '+sweptLocationText+' (общая оценка движения контура; точная часть в середине сегмента не атрибутируется)'+
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

function draw(){
 ctx.clearRect(0,0,canvas.width,canvas.height);
 ctx.fillStyle='#102235';
 ctx.fillRect(0,0,canvas.width,canvas.height);

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
 drawBody(
  h,state.psiT,1.5,
  -(cfg.trailerLength-1.5),
  cfg.trailerWidth,'#315b78'
 );

 for(const p of physics.trailerAxlePoints(state,cfg)){
  drawLine([
   localPoint(p,state.psiT,0,cfg.trailerWidth/2+0.13),
   localPoint(p,state.psiT,0,-cfg.trailerWidth/2-0.13)
  ],'#101820',7);
 }

 physics.vehiclePolygons(state,{...cfg,footprintUncertainty:cfg.vehicleFootprintUncertainty})
  .forEach(poly=>drawEnvelopePolygon(poly,'#ffd18b'));
 drawBody(state,state.psi,5,-1.2,2.55,'#087fa5');
 drawBody(
  localPoint(state,state.psi,2.1),
  state.psi,1.35,-0.9,2.25,'#35c8ef'
 );

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

 ctx.fillStyle='#dceaf5';
 ctx.font='14px system-ui';
 ctx.fillText('Volvo FH 4×2',rp.x+10,rp.y-16);
 ctx.fillText('Полуприцеп · 3 оси',hp.x-10,hp.y+32);

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
 while(index<plannedRoute.length-2&&
  Math.hypot(plannedRoute[index].x-state.x,plannedRoute[index].y-state.y)<0.22)index++;
 routePlayback.index=index;
 const gear=plannedRoute[Math.min(index+1,plannedRoute.length-1)].gear||1;
 if(routePlayback.currentGear!==gear){
  if(Math.abs(speed)>0.02){
   drive=0;braking=true;steer=0;$('steer').value='0';
   return;
  }
  routePlayback.currentGear=gear;
  braking=false;
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
 let targetIndex=Math.min(plannedRoute.length-1,index+6);
 for(let i=index+1;i<=targetIndex;i++){
  if((plannedRoute[i].gear||1)!==gear){targetIndex=Math.max(index+1,i-1);break;}
 }
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
 if(![centerX,centerY,clearWidth,headingDeg,widthUncertaintyBound,
      postWidth,depth,positionUncertainty,headingUncertaintyDeg].every(Number.isFinite)||
    Math.abs(centerX)>100||Math.abs(centerY)>100||
    clearWidth<1||clearWidth>20||headingDeg<-180||headingDeg>180||
    widthUncertaintyBound<0||widthUncertaintyBound>2||
    widthUncertaintyBound>=clearWidth-0.05||
    postWidth<0.05||postWidth>2||depth<0.05||depth>10||
    positionUncertainty<0||positionUncertainty>2||
    headingUncertaintyDeg<0||headingUncertaintyDeg>45){
  return false;
 }
 try{
  planner.createGateObstacles({
   centerX,centerY,clearWidth,
   postWidth,depth,
   heading:headingDeg*Math.PI/180,
   widthUncertainty:widthUncertaintyBound,
   positionUncertainty,
   headingUncertaintyDeg
  });
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
 $('gateWidth').value=clearWidth.toFixed(2);
 $('gateWidthUncertainty').value=widthUncertaintyBound.toFixed(3);
 $('gateCenterX').value=centerX.toFixed(2);
 $('gateCenterY').value=centerY.toFixed(2);
 $('gateHeadingDeg').value=headingDeg.toFixed(2);
 $('gateEnabled').checked=true;
 invalidateRouteForSettingsChange();
 draw();
 return true;
}

window.TruckVisionApplyGateRegistration=applyCameraGateRegistration;

function applyCameraObstacleRegistration(measurement,anchor,headingRadians,name){
 const sigmaWidth=Number(measurement?.sigmaWidth)||0;
 const sigmaDepth=Number(measurement?.sigmaHeight)||0;
 if(!measurement||!Number.isFinite(measurement.width)||!Number.isFinite(measurement.height)||
    !Number.isFinite(sigmaWidth)||!Number.isFinite(sigmaDepth)||sigmaWidth<0||sigmaDepth<0||
    sigmaWidth>2||sigmaDepth>2||
    measurement.width<0.08||measurement.height<0.08||
    measurement.width>20||measurement.height>20||
    !anchor||![anchor.x,anchor.y,headingRadians].every(Number.isFinite)||
    Math.abs(anchor.x)>100||Math.abs(anchor.y)>100||
    Math.abs(headingRadians)>Math.PI+1e-9||
    measurement.conditioningWarning===true||measurement.shapeWarning===true||
    cameraObstacles.length>=30)return false;
 const ux=Math.cos(headingRadians),uy=Math.sin(headingRadians);
 const nx=-uy,ny=ux,hw=measurement.width/2,hh=measurement.height/2;
 const polygon=[
  {x:anchor.x-hw*ux-hh*nx,y:anchor.y-hw*uy-hh*ny},
  {x:anchor.x+hw*ux-hh*nx,y:anchor.y+hw*uy-hh*ny},
  {x:anchor.x+hw*ux+hh*nx,y:anchor.y+hw*uy+hh*ny},
  {x:anchor.x-hw*ux+hh*nx,y:anchor.y-hw*uy+hh*ny}
 ];
 const measurementInflation=2*Math.max(sigmaWidth,sigmaDepth);
 let conservativePolygon=polygon;
 try{if(measurementInflation>0)conservativePolygon=planner.inflateConvexPolygon(polygon,measurementInflation);}catch(_){return false;}
 const obstacle={kind:'camera-obstacle',name:String(name||'Препятствие камеры').trim().slice(0,48)||'Препятствие камеры',
  polygon:conservativePolygon,nominalPolygon:polygon,
  measuredWidth:measurement.width,measuredDepth:measurement.height,
  sigmaWidth,sigmaDepth,measurementInflation,uncertaintyModel:'2-sigma-isotropic-envelope'};
 try{if(planner.validateObstacles([obstacle]).length)return false;}catch(_){return false;}
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
  ' м. '+($('gateEnabled').checked?'Косяки ворот также включены в карту как два ориентированных препятствия; построение маршрута проверяет их вместе со всем автопоездом.':'Косяки ворот пока не включены в карту: включите соответствующий переключатель и задайте положение/курс проёма. Камера измеряет ширину, но не регистрирует положение ворот на карте автоматически.')+' Это не гарантия проезда.';
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
  candidate.metrics?.sweptClearanceDetail??null
 );
 const passed=!routeAssessment.collision&&routeAssessment.reached&&
  !routeAssessment.articulationBlocked&&!routeAssessment.clearanceInsufficient&&
  Number.isFinite(routeAssessment.effectiveClearance);
 $('routeAdvice').textContent='Выбран вариант '+(index+1)+'. '+(passed?'Повторная проверка пройдена по текущей модели.':'Проверка не пройдена: '+routeAssessment.reasons.join('; '));
 $('difficultyAdvice').textContent='Сложность: '+routeAssessment.level+'. Длина '+routeAssessment.length.toFixed(1)+' м; пиковый угол сочленения '+routeAssessment.maxBeta.toFixed(1)+'°; минимальный эффективный зазор '+(Number.isFinite(routeAssessment.effectiveClearance)?routeAssessment.effectiveClearance.toFixed(2)+' м':'не определён')+'. '+(passed?'Автопроезд разрешён только в виртуальной модели.':'Автопроезд остаётся заблокирован.');
 const selectedRecommendation=planner.recommendStart(plannedRoute);
 updateRecommendationCard({
  status:'Вариант '+(index+1)+' выбран',
  level:routeAssessment.level,
  start:selectedRecommendation.initialGear<0?'Начать задним ходом':'Начать движением вперёд',
  turn:selectedRecommendation.firstTurn?
   (selectedRecommendation.firstTurn==='left'?'Руль влево':'Руль вправо')+' примерно через '+selectedRecommendation.firstTurnDistance.toFixed(1)+' м'+(selectedRecommendation.firstTurnGear<0?' задним ходом':' вперёд'):
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
 if(routeAssessment.collision||!routeAssessment.reached||routeAssessment.maxBeta>=ARTICULATION_STOP_DEMO_DEG||
    (Number.isFinite(clearance)&&clearance<0.35)){
  $('routeAdvice').textContent='Автопроезд не запущен: предварительная проверка маршрута выявила риск или не смогла подтвердить прохождение. Перестройте маршрут и проверьте препятствия, угол сочленения и зазоры.';
  return;
 }
 if(routePlayback.active&&routePlayback.paused){
  routePlayback.paused=false;
  $('routePause').textContent='Ⅱ Пауза';
  return;
 }
 routePlayback={active:true,paused:false,index:1,stopReason:'',currentGear:plannedRoute[1].gear||1};
 speed=0;drive=0;braking=false;
 targetSpeed=0.6;
 $('speedSet').value='0.6';
 $('routeAdvice').textContent='Виртуальный автопроезд начат на малой скорости. Наблюдайте за моделью; при предупреждении нажмите «Стоп».';
};
$('routePause').onclick=()=>{
 if(!routePlayback.active)return;
 routePlayback.paused=!routePlayback.paused;
 if(routePlayback.paused){drive=0;braking=false;speed=0;$('routePause').textContent='▶ Продолжить';}
 else $('routePause').textContent='Ⅱ Пауза';
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
