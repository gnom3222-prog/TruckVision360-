'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');
const params={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55};
const integrate=(q,v,steer,dt,p)=>physics.rk4(q,v,steer,dt,p);
test('articulated planner reaches a clear goal in open space',()=>{
 const result=planner.plan({start:{x:0,y:0,psi:0,psiT:0},goal:{x:7,y:0},params,obstacles:[],integrate,maxExpansions:5000});
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.states.length>10);
 assert.ok(Math.hypot(result.states.at(-1).x-7,result.states.at(-1).y)<0.9);
});
test('articulated planner routes the complete trailer around an obstacle',()=>{
 const obstacles=[{x:20,y:0,w:1.2,h:1.2,name:'test pallet'}];
 const result=planner.plan({start:{x:0,y:0,psi:0,psiT:0},goal:{x:40,y:0},params,obstacles,integrate,maxExpansions:24000,bounds:{minX:-18,maxX:50,minY:-16,maxY:16}});
 assert.equal(result.ok,true,JSON.stringify(result));
 for(const q of result.states.slice(1)){
  assert.ok(planner.clearance(q,obstacles,params).distance>=0.30-1e-6,'vehicle footprint must remain clear');
 }
});

test('articulated planner can select reverse gear and reports progress',()=>{
 const progress=[];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:-3,y:0},
  params,
  obstacles:[],
  integrate,
  maxExpansions:5000,
  onProgress:p=>progress.push(p.expanded)
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.states.some((q,i)=>i>0&&q.gear===-1),'reverse motion should be represented in the route');
 assert.ok(progress.length>0,'planner should report progress');
 assert.ok(progress.every((n,i)=>i===0||n>progress[i-1]),'progress counters should increase');
});
 
test('planner and simulator use exactly the same articulated collision geometry',()=>{
 const q={x:3.2,y:-1.7,psi:0.28,psiT:-0.19};
 assert.deepEqual(planner.polygons(q,params),physics.vehiclePolygons(q,params));
 const hitch=physics.hitchPoint(q,params);
 const axle=physics.axlePoint(q,params);
 assert.ok(Math.hypot(hitch.x-(q.x-params.hitchOffset*Math.cos(q.psi)),hitch.y-(q.y-params.hitchOffset*Math.sin(q.psi)))<1e-10);
 assert.ok(Math.hypot(axle.x-(hitch.x-params.kingpinAxles*Math.cos(q.psiT)),axle.y-(hitch.y-params.kingpinAxles*Math.sin(q.psiT)))<1e-10);
});
