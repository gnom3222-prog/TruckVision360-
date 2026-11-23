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
 const obstacles=[{x:20,y:0,w:1.2,h:3.0,name:'test wall'}];
 const result=planner.plan({start:{x:0,y:0,psi:0,psiT:0},goal:{x:28,y:0},params,obstacles,integrate,maxExpansions:24000,bounds:{minX:-18,maxX:38,minY:-16,maxY:16}});
 assert.equal(result.ok,true,JSON.stringify(result));
 for(const q of result.states.slice(1)){
  assert.ok(planner.clearance(q,obstacles,params).distance>=0.30-1e-6,'vehicle footprint must remain clear');
 }
});
