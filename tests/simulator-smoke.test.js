'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');

function bootSimulator(){
 const root=path.join(__dirname,'..');
 const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
 const source=fs.readFileSync(path.join(root,'truckvision.js'),'utf8');
 const values={};
 for(const match of html.matchAll(/<input\b[^>]*\bid="([^"]+)"[^>]*>/g)){
  const value=match[0].match(/\bvalue="([^"]*)"/);
  values[match[1]]=value?value[1]:'';
 }
 values.model='hybrid';
 const elements={};
 const ctx={};
 for(const name of ['beginPath','moveTo','lineTo','setLineDash','stroke','closePath','fill','fillText','arc','clearRect','fillRect']){
  ctx[name]=()=>{};
 }
 const getElementById=id=>{
  if(!elements[id]){
   elements[id]={id,value:values[id]??'',textContent:'',style:{},onclick:null,oninput:null};
  }
  return elements[id];
 };
 const canvas=getElementById('view');
 canvas.width=1000;
 canvas.height=650;
 canvas.getContext=()=>ctx;
 const windowObject={TruckVisionPhysics:physics,TruckVisionPlanner:planner,addEventListener:()=>{}};
 const sandbox={
  window:windowObject,
  document:{getElementById},
  requestAnimationFrame:()=>{},
  console
 };
 vm.runInNewContext(source,sandbox,{filename:'truckvision.js'});
 return elements;
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
 const elements=bootSimulator();
 assert.equal(typeof elements.planRoute.onclick,'function');
 assert.equal(elements.goalHeading.value,'0');
 assert.equal(elements.goalHeadingTolerance.value,'12');
 assert.equal(elements.goalArticulationLimit.value,'12');
 assert.equal(typeof elements.routeStart.onclick,'function');
 assert.equal(typeof elements.routePause.onclick,'function');
 assert.equal(typeof elements.routeStop.onclick,'function');
});

test('automatic route remains blocked when obstacle checks are disabled',()=>{
 const elements=bootSimulator();
 elements.obstaclesToggle.onclick();
 elements.planRoute.onclick();
 assert.match(elements.difficultyAdvice.textContent,/проверка препятствий отключена/);
 elements.routeStart.onclick();
 assert.match(elements.routeAdvice.textContent,/Автопроезд заблокирован/);
});

test('route planning reports missing Web Worker support without blanking the simulator',async()=>{
 const elements=bootSimulator();
 elements.planRoute.onclick();
 await new Promise(resolve=>setImmediate(resolve));
 assert.match(elements.routeAdvice.textContent,/не поддерживает фоновый расчёт маршрута/);
 assert.equal(elements.planRoute.disabled,false);
 assert.equal(typeof elements.routeStart.onclick,'function');
});
