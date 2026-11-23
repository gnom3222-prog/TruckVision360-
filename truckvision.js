
(()=>{
'use strict';

const $=id=>document.getElementById(id);
const canvas=$('view');
const ctx=canvas.getContext('2d');
const G=9.81;
const SCALE=22;

const cfg={
 wheelbase:3.8,
 hitchOffset:0.575,
 tractorMass:8500,
 engineForce:24000,
 trailerLength:13.6,
 trailerWidth:2.55,
 kingpinAxles:8,
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
let routePlayback={active:false,paused:false,index:1,stopReason:''};

const obstacles=[
 {x:8.0,y:2.9,w:1.2,h:1.2,name:'Паллет A'},
 {x:11.5,y:-2.9,w:1.0,h:1.0,name:'Паллет B'},
 {x:15.0,y:0.0,w:0.7,h:6.0,name:'Стена справа'},
 {x:6.5,y:-5.0,w:0.7,h:5.0,name:'Стена слева'}
];

const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const norm=a=>Math.atan2(Math.sin(a),Math.cos(a));

function readSettings(){
 for(const key of Object.keys(cfg)){
  const input=$(key);
  if(input)cfg[key]=Number(input.value);
 }
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

function hitchPoint(q){
 return{
  x:q.x-cfg.hitchOffset*Math.cos(q.psi),
  y:q.y-cfg.hitchOffset*Math.sin(q.psi)
 };
}

function axlePoint(q){
 const h=hitchPoint(q);
 return{
  x:h.x-cfg.kingpinAxles*Math.cos(q.psiT),
  y:h.y-cfg.kingpinAxles*Math.sin(q.psiT)
 };
}

function derivative(q,v,delta){
 const yawRate=v/cfg.wheelbase*Math.tan(delta);
 const beta=norm(q.psi-q.psiT);
 const trailerYawRate=(
  v*Math.sin(beta)
  -cfg.hitchOffset*yawRate*Math.cos(beta)
 )/cfg.kingpinAxles;

 return{
  x:v*Math.cos(q.psi),
  y:v*Math.sin(q.psi),
  psi:yawRate,
  psiT:trailerYawRate
 };
}

function addState(q,k,f){
 return{
  x:q.x+k.x*f,
  y:q.y+k.y*f,
  psi:q.psi+k.psi*f,
  psiT:q.psiT+k.psiT*f
 };
}

function rk4(q,v,delta,dt){
 const k1=derivative(q,v,delta);
 const k2=derivative(addState(q,k1,dt/2),v,delta);
 const k3=derivative(addState(q,k2,dt/2),v,delta);
 const k4=derivative(addState(q,k3,dt),v,delta);

 return{
  x:q.x+dt*(k1.x+2*k2.x+2*k3.x+k4.x)/6,
  y:q.y+dt*(k1.y+2*k2.y+2*k3.y+k4.y)/6,
  psi:norm(q.psi+dt*(k1.psi+2*k2.psi+2*k3.psi+k4.psi)/6),
  psiT:norm(q.psiT+dt*(k1.psiT+2*k2.psiT+2*k3.psiT+k4.psiT)/6)
 };
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

function vehiclePolygons(q){
 const h=hitchPoint(q);
 return[
  bodyCorners(q,q.psi,5,-1.2,2.55),
  bodyCorners(localPoint(q,q.psi,2.1),q.psi,1.35,-0.9,2.25),
  bodyCorners(h,q.psiT,1.5,-(cfg.trailerLength-1.5),cfg.trailerWidth)
 ];
}

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
 let nearest={distance:Infinity,name:'нет препятствий'};
 let collision=false;

 if(showObstacles){
  for(const o of obstacles){
   const op=obstaclePolygon(o);
   for(const vp of vehicle){
    const d=polygonDistance(vp,op);
    if(d<nearest.distance)nearest={distance:d,name:o.name};
    if(d===0)collision=true;
   }
  }
 }
 return{
  nearest,
  collision,
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
 }else{
  const a=assessScene();
  const effectiveClearance=a.nearest.distance-2*Math.max(0,cfg.measurementUncertainty);

  if(a.collision){
   message='Стоп! Расчётные контуры автопоезда пересекаются с препятствием '+
    a.nearest.name+'. Остановитесь и измените траекторию.';
   color='#ff8585';
  }else if(a.beta>55){
   message='Осторожно! Большой угол сочленения '+
    a.beta.toFixed(0)+
    ' градусов. Уменьшите угол складывания. Не продолжайте резкий задний ход.';
   color='#ffd18b';
  }else if(effectiveClearance<0.45){
   message='Очень близко к объекту '+a.nearest.name+
    ': геометрический зазор '+a.nearest.distance.toFixed(2)+' м; после резерва неопределённости остаётся '+Math.max(0,effectiveClearance).toFixed(2)+' м. Остановитесь и проверьте пространство.';
   color='#ff8585';
  }else if(effectiveClearance<1.0){
   message='Внимание! Препятствие '+a.nearest.name+
    ' на расстоянии '+a.nearest.distance.toFixed(2)+' м; после резерва остаётся '+Math.max(0,effectiveClearance).toFixed(2)+' м. Маневрируйте медленно.';
   color='#ffd18b';
  }else{
   message='Путь по текущему положению свободен в пределах тестовой карты. Ближайший объект: '+
    a.nearest.name+' — '+
    (Number.isFinite(a.nearest.distance)?
     a.nearest.distance.toFixed(2)+' метра':'не обнаружен')+
    '. Продолжайте медленно и контролируйте зеркала.';
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
 const pad=1.7;

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

function routeMetrics(path){
 let length=0;
 let turns=0;
 for(let i=1;i<path.length;i++){
  length+=Math.hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
 }
 for(let i=1;i<path.length-1;i++){
  const a=Math.atan2(path[i].y-path[i-1].y,path[i].x-path[i-1].x);
  const b=Math.atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x);
  if(Math.abs(norm(b-a))>0.18)turns++;
 }

 // Follow the point route with a low-speed pure-pursuit controller while
 // checking the complete tractor + trailer footprint at every simulation step.
 const sim={...state};
 const obstaclesWereVisible=showObstacles;
 let minClearance=Infinity;
 let maxBeta=Math.abs(norm(sim.psi-sim.psiT))*180/Math.PI;
 let collision=false;
 let reached=false;
 let pathIndex=1;
 const dt=0.10;
 const lookAhead=2.0;
 const maxSteer=35*Math.PI/180;
 const maxSteps=Math.min(5000,Math.max(300,Math.ceil(length/(0.6*dt))*4));
 for(let step=0;step<maxSteps;step++){
  while(pathIndex<path.length-1 &&
   Math.hypot(path[pathIndex].x-sim.x,path[pathIndex].y-sim.y)<lookAhead*0.65)pathIndex++;
  const target=path[Math.min(pathIndex,path.length-1)];
  const dx=target.x-sim.x,dy=target.y-sim.y;
  const targetAngle=Math.atan2(dy,dx);
  const alpha=norm(targetAngle-sim.psi);
  const steerCmd=clamp(Math.atan2(2*cfg.wheelbase*Math.sin(alpha),Math.max(0.5,Math.hypot(dx,dy))),-maxSteer,maxSteer);
  const next=rk4(sim,0.6,steerCmd,dt);
  Object.assign(sim,next);
  const beta=Math.abs(norm(sim.psi-sim.psiT))*180/Math.PI;
  maxBeta=Math.max(maxBeta,beta);
  const vehicle=vehiclePolygons(sim);
  if(obstaclesWereVisible){
   for(const o of obstacles){
    const op=obstaclePolygon(o);
    for(const vp of vehicle){
     const d=polygonDistance(vp,op);
     minClearance=Math.min(minClearance,d);
     if(d===0)collision=true;
    }
   }
  }
  if(collision||beta>=75)break;
  if(Math.hypot(path[path.length-1].x-sim.x,path[path.length-1].y-sim.y)<1.0){
   reached=true;
   break;
  }
 }
 showObstacles=obstaclesWereVisible;
 const initial=assessScene(state);
 const effectiveClearance=minClearance-2*Math.max(0,cfg.measurementUncertainty);
 const score=turns*1.2+(maxBeta>45?3:maxBeta>30?1.5:0)+
  (effectiveClearance<0.35?4:effectiveClearance<0.75?2:effectiveClearance<1.5?1:0)+
  (!reached?3:0)+(collision?8:0);
 return{
  length,turns,
  level:collision||!reached||maxBeta>=75?'Не пройден':
   score>=7?'Высокая':score>=3?'Средняя':'Низкая',
  beta:initial.beta,
  maxBeta,
  clearance:minClearance,
  effectiveClearance,
  collision,reached
 };
}

function buildRoute(){
 if(!showObstacles){
  routeAssessment=null;
  $('routeAdvice').textContent='Нельзя рассчитать готовность к автопроезду: включите «Препятствия» и повторно постройте маршрут.';
  $('difficultyAdvice').textContent='Сложность не рассчитана: проверка препятствий отключена.';
  draw();
  return;
 }
 const gx=Number($('goalX').value);
 const gy=Number($('goalY').value);

 if(!Number.isFinite(gx)||!Number.isFinite(gy)){
  $('routeAdvice').textContent='Проверьте координаты конечной точки.';
  return;
 }

 routeGoal={x:gx,y:gy};
 plannedRoute=planAStar(
  {x:state.x,y:state.y},
  routeGoal
 )||[];

 if(plannedRoute.length<2){
  plannedRoute=[];
  $('routeAdvice').textContent=
   'Маршрут не найден. Попробуйте другую конечную точку.';
  $('difficultyAdvice').textContent='Сложность не рассчитана.';
  draw();
  return;
 }

 const m=routeMetrics(plannedRoute);
 routeAssessment=m;
 const next=plannedRoute[1];

 const relative=norm(
  Math.atan2(next.y-state.y,next.x-state.x)-state.psi
 );

 const first=Math.abs(relative)<0.18?
  'держать прямо':
  relative>0?'начать поворот влево':'начать поворот вправо';

 $('routeAdvice').textContent=
  'Маршрут до точки ('+gx.toFixed(1)+', '+
  gy.toFixed(1)+') м. Начать: '+first+
  '. Длина: '+m.length.toFixed(1)+' м; поворотов: '+m.turns+
  '. Симуляция всего автопоезда: '+(m.reached?'цель достигнута':'цель не достигнута')+'.';

 $('difficultyAdvice').textContent=
  'Сложность: '+m.level+'. При моделировании всего автопоезда: максимальный угол сочленения '+
  m.maxBeta.toFixed(1)+'°; минимальный зазор до тестовых препятствий '+
  (Number.isFinite(m.clearance)?m.clearance.toFixed(2)+' м':'не определён')+
  '; после резерва погрешности: '+(Number.isFinite(m.effectiveClearance)?m.effectiveClearance.toFixed(2)+' м':'не определено')+
  '; столкновение: '+(m.collision?'да':'не выявлено')+
  '. Оценка учебная, не гарантия безопасности.';

 draw();
}

function drawRoute(){
 if(plannedRoute.length<2)return;

 drawLine(plannedRoute,'#ffe082',4,[9,6]);

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
 const axle=axlePoint(state);

 drawBody(
  h,state.psiT,1.5,
  -(cfg.trailerLength-1.5),
  cfg.trailerWidth,'#315b78'
 );

 for(const offset of [-0.65,0,0.65]){
  const p=localPoint(axle,state.psiT,offset);
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

 $('mode').textContent=braking?'Торможение':
  drive>0?'Движение вперёд':
  drive<0?'Движение задним ходом':'Остановлен';

 $('speedLabel').textContent=targetSpeed.toFixed(1)+' м/с';
 $('steerLabel').textContent=(steer*180/Math.PI).toFixed(0)+'°';

 const angle=Math.abs(beta*180/Math.PI);

 if(angle>60){
  $('mode').textContent='ВНИМАНИЕ: большой угол сочленения';
  $('mode').style.color='#ff8585';
 }else if(angle>45){
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
 if(plannedRoute.length<2){stopRoutePlayback('Нет рассчитанного маршрута. Сначала постройте путь.');return;}

 // Find the closest point on the route polyline, then select a look-ahead
 // point by travelled route distance rather than jumping between A* corners.
 let best={distance:Infinity,segment:0,t:0,point:plannedRoute[0],along:0};
 let travelled=0;
 const lengths=[];
 for(let i=0;i<plannedRoute.length-1;i++){
  const a=plannedRoute[i],b=plannedRoute[i+1];
  const dx=b.x-a.x,dy=b.y-a.y,len=Math.hypot(dx,dy);
  lengths.push(len);
  const t=len?clamp(((state.x-a.x)*dx+(state.y-a.y)*dy)/(len*len),0,1):0;
  const p={x:a.x+t*dx,y:a.y+t*dy};
  const d=Math.hypot(state.x-p.x,state.y-p.y);
  if(d<best.distance)best={distance:d,segment:i,t,point:p,along:travelled+t*len};
  travelled+=len;
 }
 routePlayback.progress=best.along;
 const lookAheadDistance=2.2;
 let remaining=lookAheadDistance;
 let target=plannedRoute[plannedRoute.length-1];
 for(let i=best.segment;i<plannedRoute.length-1;i++){
  const a=i===best.segment?best.point:plannedRoute[i];
  const b=plannedRoute[i+1];
  const len=Math.hypot(b.x-a.x,b.y-a.y);
  if(len>=remaining){
   const f=len?remaining/len:0;
   target={x:a.x+(b.x-a.x)*f,y:a.y+(b.y-a.y)*f};
   break;
  }
  remaining-=len;
 }
 const goal=plannedRoute[plannedRoute.length-1];
 const goalDistance=Math.hypot(goal.x-state.x,goal.y-state.y);
 const goalHeading=Math.atan2(goal.y-state.y,goal.x-state.x);
 const headingError=norm(goalHeading-state.psi);
 if(goalDistance<0.75&&best.distance<0.85){
  stopRoutePlayback('Автопроезд завершён: виртуальная модель достигла конечной точки маршрута.');
  return;
 }
 const dx=target.x-state.x,dy=target.y-state.y;
 const targetDistance=Math.hypot(dx,dy);
 const alpha=norm(Math.atan2(dy,dx)-state.psi);
 if(Math.abs(alpha)>Math.PI*0.58 && best.along<1.0){
  stopRoutePlayback('Маршрут начинается позади тягача. Эта версия автопроезда выполняет только движение вперёд; перестройте маршрут с учётом направления тягача.');
  return;
 }
 if(best.distance>2.0){
  stopRoutePlayback('Автопроезд остановлен: модель слишком далеко отклонилась от рассчитанной линии маршрута.');
  return;
 }
 const lookAhead=Math.max(1.2,targetDistance);
 const command=Math.atan2(2*cfg.wheelbase*Math.sin(alpha),lookAhead);
 steer=clamp(command,-28*Math.PI/180,28*Math.PI/180);
 $('steer').value=String(Math.round(steer*180/Math.PI));
 drive=1;
 braking=false;
 targetSpeed=0.45;
 $('speedSet').value='0.45';
 const safety=assessScene(state);
 const conservativeClearance=safety.nearest.distance-2*Math.max(0,cfg.measurementUncertainty);
 if(safety.collision){
  stopRoutePlayback('Автопроезд остановлен: расчётные контуры пересекаются с препятствием '+safety.nearest.name+'.');
 }else if(safety.beta>55){
  stopRoutePlayback('Автопроезд остановлен: опасный расчётный угол сочленения '+safety.beta.toFixed(1)+'°. Перестройте манёвр.');
 }else if(Number.isFinite(conservativeClearance)&&conservativeClearance<0.35){
  stopRoutePlayback('Автопроезд остановлен: малый расчётный зазор после резерва неопределённости ('+conservativeClearance.toFixed(2)+' м).');
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

 if(drive!==0||braking||(routePlayback.active&&!routePlayback.paused))simulationStep(dt);

 draw();
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
 readSettings();
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
 stopRoutePlayback('');
 resetState();
 showObstacles=true;
 helperEnabled=true;
 $('obstaclesToggle').textContent='Препятствия: вкл.';
 $('helperToggle').textContent='Помощник: вкл.';
 draw();
};

$('assess').onclick=()=>{
 readSettings();

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
 if(!showObstacles){
  $('routeAdvice').textContent='Автопроезд заблокирован: сначала включите отображение и проверку препятствий.';
  return;
 }
 if(plannedRoute.length<2||!routeAssessment){
  $('routeAdvice').textContent='Сначала нажмите «Построить путь» и дождитесь оценки маршрута.';
  return;
 }
 const clearance=routeAssessment.effectiveClearance;
 if(routeAssessment.collision||!routeAssessment.reached||routeAssessment.maxBeta>=55||
    (Number.isFinite(clearance)&&clearance<0.35)){
  $('routeAdvice').textContent='Автопроезд не запущен: предварительная проверка маршрута выявила риск или не смогла подтвердить прохождение. Перестройте маршрут и проверьте препятствия, угол сочленения и зазоры.';
  return;
 }
 if(routePlayback.active&&routePlayback.paused){
  routePlayback.paused=false;
  $('routePause').textContent='Ⅱ Пауза';
  return;
 }
 routePlayback={active:true,paused:false,index:1,stopReason:''};
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

readSettings();
targetSpeed=Number($('speedSet').value);
resetState();
draw();
requestAnimationFrame(frame);

})();
