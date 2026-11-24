'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');

function bootSimulator(options={}){
 const root=path.join(__dirname,'..');
 const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
 const source=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 const values={};
 for(const match of html.matchAll(/<([a-z][a-z0-9-]*)\\b[^>]*\\bid="([^"]+)"[^>]*>/gi)){
  const value=match[1].toLowerCase()==='input'?match[0].match(/\\bvalue="([^"]*)"/):null;
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
 const makeElement=(id,value='')=>({
  id,value,textContent:'',style:{},onclick:null,oninput:null,onchange:null,
  children:[],appendChild(child){this.children.push(child);return child;}
 });
 const getElementById=id=>{
  if(!elements[id])elements[id]=makeElement(id,values[id]??'');
  return elements[id];
 };
 for(const id of Object.keys(values))getElementById(id);
 const canvas=getElementById('view');
 canvas.width=1000;
 canvas.height=650;
 canvas.getContext=()=>ctx;
 const windowObject={TruckVisionPhysics:physics,TruckVisionPlanner:planner,addEventListener:()=>{}};
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

test('simulator starts and exposes route controls',()=>{
 const {elements}=bootSimulator();
 assert.equal(typeof elements.planRoute.onclick,'function');
 assert.equal(elements.goalHeading.value,'0');
 assert.equal(elements.goalHeadingTolerance.value,'12');
 assert.equal(elements.goalArticulationLimit.value,'12');
 assert.equal(typeof elements.routeStart.onclick,'function');
 assert.equal(typeof elements.routePause.onclick,'function');
 assert.equal(typeof elements.routeStop.onclick,'function');
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

test('live assistant identifies the nearest vehicle part and named obstacle',()=>{
 const {elements}=bootSimulator();
 assert.match(elements.assistantAdvice.textContent,/Ближайший объект:/);
 assert.match(elements.assistantAdvice.textContent,/Часть автопоезда: (тягач|кабина|полуприцеп)\./);
 assert.match(elements.assistantAdvice.textContent,/Расчётный зазор/);
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


test('camera registration atomically applies measured gate pose and enables jamb obstacles',()=>{
 const {elements,windowObject}=bootSimulator();
 const applied=windowObject.TruckVisionApplyGateRegistration({
  centerX:14.5,centerY:-2.25,clearWidth:3.8,headingDeg:37
 });
 assert.equal(applied,true);
 assert.equal(elements.gateWidth.value,'3.80');
 assert.equal(elements.gateCenterX.value,'14.50');
 assert.equal(elements.gateCenterY.value,'-2.25');
 assert.equal(elements.gateHeadingDeg.value,'37.00');
 assert.equal(elements.gateEnabled.checked,true);
 assert.match(elements.routeAdvice.textContent,/Параметры модели изменены/);
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

test('ambiguous planner results preserve candidate states and simulator draws them without enabling playback',()=>{
 const root=path.join(__dirname,'..');
 const plannerSource=fs.readFileSync(path.join(root,'articulated-planner.js'),'utf8');
 const simulatorSource=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 assert.match(plannerSource,/candidateRoutes:candidates\.map\(\(candidate,index\)=>\(/);
 assert.match(plannerSource,/states:candidate\.states/);
 assert.match(simulatorSource,/alternativeRoutes=Array\.isArray\(result\.candidateRoutes\)/);
 assert.match(simulatorSource,/drawLine\(candidate\.states,colors\[index%colors\.length\],4,\[9,6\]\)/);
 assert.match(simulatorSource,/if\(plannedRoute\.length<2\|\|!routeAssessment\)/);
});


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
