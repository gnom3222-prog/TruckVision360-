
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
 margin:0.2,
 measurementUncertainty:0.15
};

let state,steer=0,targetSpeed=2,speed=0,drive=0;
let braking=false,distance=0,trace=[],showTrace=true;
let camera={x:0,y:0},lastTime=0;
let showObstacles=true,helperEnabled=true;
let plannedRoute=[],routeGoal=null,routeAssessment=null;
let routePlayback={active:false,paused:false,index:1,stopReason:'',currentGear:1};
let plannerWorker=null,plannerRequestId=0,plannerReject=null,routeBuildToken=0;

const obstacles=[
 {x:8.0,y:2.9,w:1.2,h:1.2,name:'Паллет A'},
 {x:11.5,y:-2.9,w:1.0,h:1.0,name:'Паллет B'},
 {x:15.0,y:0.0,w:0.7,h:6.0,name:'Стена справа'},
 {x:6.5,y:-5.0,w:0.7,h:5.0,name:'Стена слева'}
];

const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const norm=a=>Math.atan2(Math.sin(a),Math.cos(a));
const physics=window.TruckVisionPhysics;
if(!physics)throw new Error('TruckVision physics module failed to load.');
const planner=window.TruckVisionPlanner;
if(!planner)throw new Error('TruckVision articulated planner module failed to load.');

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
  margin:[0,5],
  measurementUncertainty:[0,5]
 };
 for(const [key,range] of Object.entries(limits)){
  const value=candidate[key];
  if(!Number.isFinite(value)||value<range[0]||value>range[1]){
   errors.push(key+': допустимый диапазон '+range[0]+'–'+range[1]+'.');
  }
 }
 if(errors.length)return {ok:false,changed:false,errors};
 const changed=Object.keys(cfg).some(key=>!Object.is(cfg[key],candidate[key]));
 Object.assign(cfg,candidate);
 return {ok:true,changed,errors:[]};
}

function invalidateRouteForSettingsChange(){
 routeBuildToken++;
 cancelPlannerWorker('Параметры модели изменены; прежний расчёт отменён.');
 $('planRoute').disabled=false;
 if(routePlayback.active)stopRoutePlayback('Автопроезд остановлен: параметры модели изменились.');
 plannedRoute=[];
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
 trace=[];
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
  for(const o of obstacles){
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
function drawObstacles(){
 if(!showObstacles)return;

 for(const o of obstacles){
  const p=obstaclePolygon(o).map(toScreen);
  ctx.beginPath();
  p.forEach((z,i)=>i?ctx.lineTo(z.x,z.y):ctx.moveTo(z.x,z.y));
  ctx.closePath();
  ctx.fillStyle='#8b4c43';
  ctx.fill();
  ctx.strokeStyle='#ffc0a8';
  ctx.lineWidth=2;
  ctx.stroke();

  const c=toScreen({x:o.x,y:o.y});
  ctx.fillStyle='#fff';
  ctx.font='12px system-ui';
  ctx.fillText(o.name,c.x+5,c.y-5);
 }
}

function routeBlocked(ix,iy){
 if(!showObstacles)return false;
 const x=ix*0.5;
 const y=iy*0.5;
 const pad=2.2;

 return obstacles.some(o=>
  x>=o.x-o.w/2-pad&&
  x<=o.x+o.w/2+pad&&
  y>=o.y-o.h/2-pad&&
  y<=o.y+o.h/2+pad
 );
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
 const checkClearance=showObstacles?planner.createClearanceChecker(obstacles,cfg):null;
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
   worker=new Worker('./planner-worker.js?v=20261124-4');
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
   obstacles:obstacles.map(o=>({...o})),
   maxExpansions:mobile?4500:12000,
   alternativeMaxExpansions:mobile?2200:4500,
   costTolerance:0.25,
   minClearance:0.65,
   bounds:{minX:-20,maxX:42,minY:-20,maxY:20}
  },progress=>{
   if(token!==routeBuildToken)return;
   const candidateText=progress.candidateIndex?'Кандидат '+progress.candidateIndex+'/'+progress.candidateCount+'; суммарно проверено ':'Проверено ';
   $('routeAdvice').textContent='Рассчитываю варианты траектории… '+candidateText+progress.expanded+
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

  if(result.reason==='ambiguous-start'){
   if(routePlayback.active)stopRoutePlayback('Виртуальный автопроезд остановлен: начальные варианты манёвра неоднозначны.');
   plannedRoute=[];
   routeAssessment=null;
   const alternatives=(result.candidateSummaries||[]).map(candidate=>{
    const gear=candidate.initialGear<0?'задний ход':'вперёд';
    const turn=candidate.firstTurn==='left'?'руль влево':
     candidate.firstTurn==='right'?'руль вправо':'без значимого поворота';
    return candidate.label+': '+gear+', '+turn+', эвристический балл '+candidate.cost.toFixed(2);
   }).join('; ');
   $('routeAdvice').textContent='Найдено несколько близких по эвристической стоимости вариантов с разными начальными действиями. Однозначная сторона и передача не выбраны. '+alternatives+'. Балл объединяет длину пути, переключения, пиковый угол и запас зазора; это не вероятность безопасности и не доказательство глобального оптимума.';
   $('difficultyAdvice').textContent='Сложность не присвоена одному маршруту: варианты требуют разных начальных действий. Сравните геометрию площадки и повторите расчёт с уточнёнными измерениями. Виртуальный автопроезд не запускается.';
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

  const peakGearText=m.articulationPeak.gear<0?'задним ходом':'вперёд';
  const articulationPeakText='пик угла сочленения '+m.articulationPeak.maxAngleDegrees.toFixed(1)+
   '° на '+m.articulationPeak.distanceAlongRoute.toFixed(1)+' м пути (X '+
   m.articulationPeak.state.x.toFixed(1)+' м, Y '+m.articulationPeak.state.y.toFixed(1)+' м), '+peakGearText;
  $('routeAdvice').textContent=
   'Начало манёвра: '+gearText+'; первое рулевое действие: '+firstTurnText+
   '. Сторона определяется по первому значимому управляющему углу планировщика (порог 0,08 рад), а не по одному лишь направлению на цель.'+recommendationNote+' Маршрут до точки ('+
   gx.toFixed(1)+', '+gy.toFixed(1)+') м; длина '+m.length.toFixed(1)+
   ' м; переключений передачи: '+m.gearChanges+'; изменений руля: '+m.steeringChanges+
   '; задний ход: '+m.reverseDistance.toFixed(1)+' м. Заданный конечный курс '+headingDegrees.toFixed(0)+
   '°; ошибка курса '+m.finalHeadingErrorDeg.toFixed(1)+'°; угол сочленения у цели '+
   m.finalArticulationDeg.toFixed(1)+'° (допуск '+articulationLimitDegrees.toFixed(0)+'°); '+articulationPeakText+'.';

  const closestLocationText=m.clearanceDetail?
   m.clearanceDetail.vehiclePart+' рядом с препятствием «'+m.clearanceDetail.obstacle+
   '» около X '+m.clearanceDetail.state.x.toFixed(1)+' м, Y '+
   m.clearanceDetail.state.y.toFixed(1)+' м':
   'не определена (препятствия выключены или зазор бесконечен)';
  const sweptLocationText=m.sweptClearanceDetail&&m.sweptClearanceDetail.kind==='segment-lower-bound'?
   'сегмент X '+m.sweptClearanceDetail.from.x.toFixed(1)+', Y '+m.sweptClearanceDetail.from.y.toFixed(1)+
   ' → X '+m.sweptClearanceDetail.to.x.toFixed(1)+', Y '+m.sweptClearanceDetail.to.y.toFixed(1):
   'отдельный ограничивающий сегмент не выделен';
  $('difficultyAdvice').textContent=
   'Сложность: '+m.level+'. Причины: '+m.reasons.join('; ')+
   '. Максимальный угол сочленения '+m.maxBeta.toFixed(1)+'°; '+articulationPeakText+'; конечная ошибка курса '+m.finalHeadingErrorDeg.toFixed(1)+
   '°; конечный угол тягач–полуприцеп '+m.finalArticulationDeg.toFixed(1)+'°; геометрический индикатор боковой скорости крайних осей при расчётной скорости 0,7 м/с '+m.maxAxleScrubSpeed.toFixed(3)+' м/с; по осям (передняя/средняя/задняя): '+m.axleScrubSpeedByAxle.map(value=>value.toFixed(3)).join('/')+' м/с; накопленный proxy по осям: '+m.axleScrubTravelByAxle.map(value=>value.toFixed(2)).join('/')+' м; суммарный proxy '+m.axleScrubTravelProxy.toFixed(2)+' м (не модель шин); минимальный зазор до тестовых препятствий '+
   (Number.isFinite(m.clearance)?m.clearance.toFixed(2)+' м':'не определён')+
   '; ближайшая проверенная часть автопоезда: '+closestLocationText+
   '; нижняя оценка между узлами '+(Number.isFinite(m.clearanceLowerBound)?m.clearanceLowerBound.toFixed(2)+' м':'не ограничена')+
   ' на '+sweptLocationText+' (общая оценка движения контура; точная часть в середине сегмента не атрибутируется)'+
   '; после резерва погрешности: '+(Number.isFinite(m.effectiveClearance)?m.effectiveClearance.toFixed(2)+' м':'не определено')+
   '; столкновение: '+(m.collision?'да':'не выявлено')+
   '. Курс у цели проверен относительно заданного угла и допуска; учебный расчёт, не гарантия безопасности.';
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
 trace.push({x:state.x,y:state.y,ax:a.x,ay:a.y});
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
  ' м. Это только сравнение ширины, не анализ траектории и не гарантия проезда.';
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
 buildRoute();
};

$('clearRoute').onclick=()=>{
 routeBuildToken++;cancelPlannerWorker('Расчёт отменён: маршрут удалён.');$('planRoute').disabled=false;
 stopRoutePlayback('');
 plannedRoute=[];
 routeGoal=null;
 routeAssessment=null;
 $('routeAdvice').textContent='Маршрут удалён.';
 $('difficultyAdvice').textContent='Сложность: пока не рассчитана.';
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
