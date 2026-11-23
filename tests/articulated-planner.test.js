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
test('articulated planner satisfies the requested terminal tractor heading and trailer alignment',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:2,y:0,heading:0},
  params,obstacles:[],integrate,
  maxExpansions:1000,
  positionTolerance:0.9,
  headingTolerance:5*Math.PI/180,
  maxTerminalArticulation:5*Math.PI/180
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 assert.ok(result.terminalErrors.position<=0.9);
 assert.ok(result.terminalErrors.headingError<=5*Math.PI/180);
 assert.ok(result.terminalErrors.articulation<=5*Math.PI/180);
});

test('articulated planner does not accept a position-only match with the wrong final heading',()=>{
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:0.5,y:0,heading:Math.PI},
  params,obstacles:[],integrate,
  maxExpansions:1,
  headingTolerance:5*Math.PI/180,
  maxTerminalArticulation:5*Math.PI/180
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'no-path');
 assert.ok(result.headingError>Math.PI-1e-9);
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

test('prepared clearance checker matches direct collision-distance queries',()=>{
 const q={x:2.3,y:1.1,psi:-0.17,psiT:0.09};
 const obstacles=[
  {x:7,y:3,w:1.2,h:1.2,name:'A'},
  {x:-4,y:-2,w:0.8,h:2.0,name:'B'}
 ];
 const check=planner.createClearanceChecker(obstacles,params);
 assert.deepEqual(check(q),planner.clearance(q,obstacles,params));
});

test('articulated planner passes through a two-pallet clearance gate',()=>{
 const obstacles=[
  {x:12,y:3,w:1.2,h:2,name:'upper pallet'},
  {x:12,y:-3,w:1.2,h:2,name:'lower pallet'}
 ];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:25,y:0},
  params,obstacles,integrate,
  maxExpansions:5000,
  minClearance:0.30,
  bounds:{minX:-18,maxX:32,minY:-12,maxY:12}
 });
 assert.equal(result.ok,true,JSON.stringify(result));
 for(const q of result.states.slice(1)){
  assert.ok(planner.clearance(q,obstacles,params).distance>=0.30-1e-6,'entire articulated footprint must clear both pallets');
 }
});

test('articulated planner rejects an initial tractor or trailer collision',()=>{
 const obstacles=[{x:0,y:0,w:1,h:1,name:'blocking pallet'}];
 const result=planner.plan({
  start:{x:0,y:0,psi:0,psiT:0},
  goal:{x:8,y:0},
  params,obstacles,integrate,
  maxExpansions:1000
 });
 assert.equal(result.ok,false);
 assert.equal(result.reason,'start-collision');
});


test('start recommendation identifies the first steering side after a straight lead-in',()=>{
 const states=[
  {x:0,y:0,gear:1,steer:0},
  {x:0.2,y:0,gear:1,steer:0},
  {x:0.4,y:0,gear:1,steer:0.12},
  {x:0.6,y:0.02,gear:1,steer:0.12}
 ];
 const result=planner.recommendStart(states);
 assert.equal(result.ok,true);
 assert.equal(result.initialGear,1);
 assert.equal(result.firstTurn,'left');
 assert.equal(result.firstTurnGear,1);
 assert.equal(result.firstTurnAfterGearChange,false);
 assert.ok(Math.abs(result.firstTurnDistance-0.4)<1e-9);
});

test('start recommendation keeps steering-wheel direction explicit during reverse',()=>{
 const result=planner.recommendStart([
  {x:0,y:0,gear:1,steer:0},
  {x:-0.1,y:0,gear:-1,steer:-0.16},
  {x:-0.2,y:-0.01,gear:-1,steer:-0.16}
 ]);
 assert.equal(result.initialGear,-1);
 assert.equal(result.firstTurn,'right');
 assert.equal(result.firstTurnGear,-1);
 assert.equal(result.firstTurnAfterGearChange,false);
});

test('start recommendation reports a turn that begins after a gear change',()=>{
 const result=planner.recommendStart([
  {x:0,y:0,gear:1,steer:0},
  {x:0.3,y:0,gear:1,steer:0},
  {x:0.3,y:0,gear:-1,steer:0},
  {x:0.2,y:0,gear:-1,steer:0.12}
 ]);
 assert.equal(result.initialGear,1);
 assert.equal(result.firstTurn,'left');
 assert.equal(result.firstTurnGear,-1);
 assert.equal(result.firstTurnAfterGearChange,true);
 assert.ok(Math.abs(result.firstTurnDistance-0.4)<1e-9);
});

test('start recommendation reports straight travel when no steering input is planned',()=>{
 const result=planner.recommendStart([
  {x:0,y:0,gear:1,steer:0},
  {x:0.5,y:0,gear:1,steer:0},
  {x:1,y:0,gear:1,steer:0}
 ]);
 assert.equal(result.initialGear,1);
 assert.equal(result.firstTurn,null);
 assert.equal(result.firstTurnDistance,null);
});

test('start recommendation rejects a route without movement samples',()=>{
 assert.equal(planner.recommendStart([{x:0,y:0}]).reason,'route-too-short');
});


test('start recommendation ignores zero and invalid steering thresholds',()=>{
 const states=[{x:0,y:0,gear:1,steer:0},{x:0.5,y:0,gear:1,steer:0}];
 assert.equal(planner.recommendStart(states,{steeringThreshold:0}).firstTurn,null);
 assert.equal(planner.recommendStart(states,{steeringThreshold:-1}).firstTurn,null);
});
