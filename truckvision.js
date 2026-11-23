
/* ==========================================
   TRUCKVISION 360 — VOICE MODULE
   Независимый голосовой помощник
========================================== */
(function () {
  'use strict';

  if (window.__truckVisionVoiceLoaded) return;
  window.__truckVisionVoiceLoaded = true;

  let enabled = false;
  let lastMessage = '';
  let lastTime = 0;

  function createPanel() {
    if (document.getElementById('truckVisionVoicePanel')) return;

    const panel = document.createElement('section');
    panel.id = 'truckVisionVoicePanel';

    panel.style.cssText = `
      box-sizing:border-box;
      width:calc(100% - 24px);
      max-width:900px;
      margin:16px auto;
      padding:18px;
      background:#142b40;
      border:1px solid #3983a8;
      border-radius:16px;
      color:white;
      font-family:system-ui,sans-serif;
      position:relative;
      z-index:1000;
    `;

    panel.innerHTML = `
      <div style="font-size:20px;font-weight:bold;margin-bottom:12px">
        🔊 Голосовой помощник TruckVision 360
      </div>

      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <button id="tvVoiceToggle"
          style="padding:13px;border:0;border-radius:10px;
          background:#0789ad;color:white;font-weight:bold">
          Включить голос
        </button>

        <button id="tvVoiceTest"
          style="padding:13px;border:0;border-radius:10px;
          background:#315675;color:white;font-weight:bold">
          Проверить голос
        </button>
      </div>

      <div id="tvVoiceStatus"
        style="margin-top:12px;color:#b8d9ec">
        Голос выключен.
      </div>

      <div id="tvVoiceAdvice"
        style="margin-top:10px;color:#83d6a8">
        Ожидание подсказок симулятора.
      </div>
    `;

    document.body.appendChild(panel);

    const toggle = panel.querySelector('#tvVoiceToggle');
    const test = panel.querySelector('#tvVoiceTest');
    const status = panel.querySelector('#tvVoiceStatus');

    function speak(message) {
      if (!enabled || !message) return;
      if (!('speechSynthesis' in window)) return;

      const now = Date.now();

      if (message === lastMessage) return;
      if (now - lastTime < 2500) return;

      lastMessage = message;
      lastTime = now;

      window.speechSynthesis.cancel();

      const speech = new SpeechSynthesisUtterance(message);
      speech.lang = 'ru-RU';
      speech.rate = 0.92;
      speech.pitch = 1;
      speech.volume = 1;

      window.speechSynthesis.speak(speech);
    }

    toggle.addEventListener('click', function () {
      if (!('speechSynthesis' in window)) {
        status.textContent = 'Голос не поддерживается браузером.';
        return;
      }

      enabled = !enabled;

      toggle.textContent = enabled
        ? 'Выключить голос'
        : 'Включить голос';

      status.textContent = enabled
        ? 'Голосовой помощник включён.'
        : 'Голосовой помощник выключен.';

      if (!enabled) {
        window.speechSynthesis.cancel();
        lastMessage = '';
      } else {
        window.speechSynthesis.cancel();

        const greeting = new SpeechSynthesisUtterance(
          'Голосовой помощник TruckVision 360 включён.'
        );

        greeting.lang = 'ru-RU';
        greeting.rate = 0.92;

        window.speechSynthesis.speak(greeting);
      }
    });

    test.addEventListener('click', function () {
      if (!enabled) {
        status.textContent = 'Сначала включи голос.';
        return;
      }

      lastMessage = '';
      lastTime = 0;

      speak(
        'Проверка системы. TruckVision 360 готов к работе.',
        true
      );
    });

    function findAdvice() {
      return document.getElementById('assistantAdvice');
    }

    function readAdvice() {
      const advice = findAdvice();

      if (!advice) return;

      const message = advice.innerText.trim();

      if (!message) return;

      panel.querySelector('#tvVoiceAdvice').textContent = message;
      speak(message);
    }

    const observer = new MutationObserver(readAdvice);

    observer.observe(document.body, {
      childList: true,
      subtree: true,
      characterData: true
    });

    readAdvice();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', createPanel);
  } else {
    createPanel();
  }

})();

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
 margin:0.2
};

let state,steer=0,targetSpeed=2,speed=0,drive=0;
let braking=false,distance=0,trace=[],showTrace=true;
let camera={x:0,y:0},lastTime=0;
let showObstacles=true,helperEnabled=true;
let plannedRoute=[],routeGoal=null;

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

/* ГОЛОСОВОЙ ПОМОЩНИК */

let voiceEnabled=false;
let lastVoiceMessage='';
let lastVoiceAt=0;

function speakTruckVision(message,force=false){
 if(!voiceEnabled||!('speechSynthesis' in window))return;

 const now=Date.now();
 if(!force&&message===lastVoiceMessage)return;
 if(!force&&now-lastVoiceAt<2200)return;

 lastVoiceMessage=message;
 lastVoiceAt=now;

 window.speechSynthesis.cancel();

 const utterance=new SpeechSynthesisUtterance(message);
 utterance.lang='ru-RU';
 utterance.rate=0.92;
 utterance.pitch=1;
 utterance.volume=1;

 window.speechSynthesis.speak(utterance);
}

function createVoiceControls(){
 if(document.getElementById('truckVisionVoicePanel'))return;

 const box=document.createElement('section');
 box.id='truckVisionVoicePanel';
 box.setAttribute('aria-label','Голосовой помощник TruckVision 360');

 box.style.cssText=
  'box-sizing:border-box;width:min(100% - 24px,900px);'+
  'margin:18px auto;padding:16px;border:1px solid #315675;'+
  'border-radius:16px;background:#142b40;color:#f3f7fb;'+
  'font:16px system-ui,sans-serif;position:relative;z-index:5;';

 box.innerHTML=`
  <div style="font-size:19px;font-weight:700;margin-bottom:8px">
   🔊 Голосовой помощник
  </div>
  <div style="display:flex;gap:10px;flex-wrap:wrap">
   <button id="voiceToggle" type="button"
    style="border:0;border-radius:10px;padding:12px 16px;
    background:#0789ad;color:white;font-weight:700;font-size:16px">
    Включить голос
   </button>
   <button id="voiceTest" type="button"
    style="border:1px solid #6b8ba5;border-radius:10px;
    padding:12px 16px;background:#203e58;color:white;
    font-weight:700;font-size:16px">
    Проверить голос
   </button>
  </div>
  <p id="voiceStatus" aria-live="polite"
   style="margin:10px 0 0;color:#b8c9d8">
   Голос выключен. Нажми «Включить голос», затем «Проверить голос».
  </p>
  <p id="voiceCurrentAdvice"
   style="margin:8px 0 0;color:#83d6a8">
   Ожидание подсказки симулятора.
  </p>
 `;

 // Панель создаётся даже без элемента assistantAdvice.
 const canvasElement=document.getElementById('view');
 const host=canvasElement&&(
  canvasElement.closest('main')||
  canvasElement.closest('.container')||
  canvasElement.parentElement
 );

 if(host&&host.parentNode){
  host.insertAdjacentElement('afterend',box);
 }else{
  document.body.appendChild(box);
 }

 const toggle=document.getElementById('voiceToggle');
 const test=document.getElementById('voiceTest');
 const status=document.getElementById('voiceStatus');

 toggle.addEventListener('click',()=>{
  if(!('speechSynthesis' in window)||
     typeof SpeechSynthesisUtterance==='undefined'){
   status.textContent='В этом браузере синтез речи недоступен.';
   return;
  }

  voiceEnabled=!voiceEnabled;
  toggle.textContent=voiceEnabled?
   'Выключить голос':'Включить голос';

  status.textContent=voiceEnabled?
   'Голос включён. Теперь можно проверить звучание.':
   'Голос выключен.';

  if(!voiceEnabled){
   window.speechSynthesis.cancel();
   lastVoiceMessage='';
  }else{
   window.speechSynthesis.cancel();
   const greeting=new SpeechSynthesisUtterance(
    'Голосовой помощник TruckVision 360 включён.'
   );
   greeting.lang='ru-RU';
   greeting.rate=0.92;
   greeting.volume=1;
   window.speechSynthesis.speak(greeting);
  }
 });

 test.addEventListener('click',()=>{
  if(!voiceEnabled){
   status.textContent='Сначала нажми «Включить голос».';
   return;
  }
  speakTruckVision(
   'Проверка голосовой системы. TruckVision 360 готов к работе.',
   true
  );
 });
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

  if(a.collision){
   message='Стоп! Расчётные контуры автопоезда пересекаются с препятствием '+
    a.nearest.name+'. Остановитесь и измените траекторию.';
   color='#ff8585';
  }else if(a.beta>55){
   message='Осторожно! Большой угол сочленения '+
    a.beta.toFixed(0)+
    ' градусов. Уменьшите угол складывания. Не продолжайте резкий задний ход.';
   color='#ffd18b';
  }else if(a.nearest.distance<0.45){
   message='Очень близко к объекту '+a.nearest.name+
    ': примерно '+a.nearest.distance.toFixed(2)+
    ' метра по упрощённой оценке. Остановитесь и проверьте пространство.';
   color='#ff8585';
  }else if(a.nearest.distance<1.0){
   message='Внимание! Препятствие '+a.nearest.name+
    ' на расстоянии около '+a.nearest.distance.toFixed(2)+
    ' метра. Маневрируйте медленно.';
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

 const voiceAdvice=$('voiceCurrentAdvice');
 if(voiceAdvice){
  voiceAdvice.textContent=message;
  voiceAdvice.style.color=color;
 }

 speakTruckVision(message);
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
  length+=Math.hypot(
   path[i].x-path[i-1].x,
   path[i].y-path[i-1].y
  );
 }

 for(let i=1;i<path.length-1;i++){
  const a=Math.atan2(
   path[i].y-path[i-1].y,
   path[i].x-path[i-1].x
  );
  const b=Math.atan2(
   path[i+1].y-path[i].y,
   path[i+1].x-path[i].x
  );
  if(Math.abs(norm(b-a))>0.18)turns++;
 }

 const scene=assessScene();
 const score=
  turns*2+
  (scene.beta>35?2:0)+
  (scene.nearest.distance<1?3:
   scene.nearest.distance<2?1:0);

 return{
  length,
  turns,
  level:score>=7?'Высокая':score>=3?'Средняя':'Низкая',
  beta:scene.beta,
  clearance:scene.nearest.distance
 };
}

function buildRoute(){
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
 const next=plannedRoute[1];

 const relative=norm(
  Math.atan2(next.y-state.y,next.x-state.x)-state.psi
 );

 const first=Math.abs(relative)<0.18?
  'держать прямо':
  relative>0?'начать поворот влево':'начать поворот вправо';

 $('routeAdvice').textContent=
  'Путь построен до точки ('+gx.toFixed(1)+', '+
  gy.toFixed(1)+') м. Первый ориентир: '+first+
  '. Длина: '+m.length.toFixed(1)+
  ' м; поворотов: '+m.turns+
  '. Двигайтесь медленно и контролируйте зеркала.';

 $('difficultyAdvice').textContent=
  'Сложность манёвра: '+m.level+
  '. Учтены повороты маршрута, текущий угол сочленения '+
  m.beta.toFixed(0)+'° и ближайшее препятствие '+
  (Number.isFinite(m.clearance)?
   m.clearance.toFixed(2)+' м':'не определено')+'.';

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

function simulationStep(dt){
 updateSpeed(dt);

 const oldX=state.x;
 const oldY=state.y;
 const steps=Math.max(1,Math.ceil(dt/0.01));
 const h=dt/steps;

 for(let i=0;i<steps;i++){
  state=rk4(state,speed,steer,h);
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

 if(drive!==0||braking)simulationStep(dt);

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
 resetState();
 plannedRoute=[];
 routeGoal=null;
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
 draw();
};

$('helperToggle').onclick=()=>{
 helperEnabled=!helperEnabled;
 $('helperToggle').textContent=
  'Помощник: '+(helperEnabled?'вкл.':'выкл.');
 updateAssistant();
};

$('scenarioReset').onclick=()=>{
 resetState();
 showObstacles=true;
 helperEnabled=true;
 $('obstaclesToggle').textContent='Препятствия: вкл.';
 $('helperToggle').textContent='Помощник: вкл.';
 draw();
};

$('assess').onclick=()=>{
 readSettings();

 const usable=cfg.gateWidth-2*cfg.margin;
 const difference=usable-cfg.trailerWidth;

 let result;

 if(difference<0)result='Проём уже ширины полуприцепа';
 else if(difference<0.3)result='Очень малый боковой запас';
 else if(difference<0.8)result='Ограниченный запас: нужен точный манёвр';
 else result='По ширине имеется предварительный запас';

 $('gateResult').textContent=
  result+'. Полезная ширина: '+usable.toFixed(2)+
  ' м; ширина полуприцепа: '+cfg.trailerWidth.toFixed(2)+
  ' м; суммарный запас: '+difference.toFixed(2)+
  ' м. Это только сравнение ширины, не анализ траектории и не гарантия проезда.';
};

$('planRoute').onclick=buildRoute;

$('clearRoute').onclick=()=>{
 plannedRoute=[];
 routeGoal=null;
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

createVoiceControls();
readSettings();
targetSpeed=Number($('speedSet').value);
resetState();
draw();
requestAnimationFrame(frame);

})();
