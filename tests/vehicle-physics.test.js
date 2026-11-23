'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');

const params={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8};
const near=(actual,expected,tolerance=1e-10)=>
 assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} is not within ${tolerance} of ${expected}`);

test('trailer axle has zero lateral velocity for forward and reverse motion',()=>{
 for(const v of [4.2,-2.1]){
  const q={x:1,y:-2,psi:0.31,psiT:-0.22};
  const delta=0.14;
  const d=physics.derivative(q,v,delta,params);
  const hitchVx=v*Math.cos(q.psi)+params.hitchOffset*d.psi*Math.sin(q.psi);
  const hitchVy=v*Math.sin(q.psi)-params.hitchOffset*d.psi*Math.cos(q.psi);
  const lateralAtTrailerAxle=-Math.sin(q.psiT)*hitchVx+
   Math.cos(q.psiT)*hitchVy-params.kingpinAxles*d.psiT;
  near(lateralAtTrailerAxle,0,1e-10);
 }
});

test('straight forward and reverse motion preserve both headings',()=>{
 const q={x:2,y:3,psi:0.4,psiT:0.4};
 const forward=physics.derivative(q,2,0,params);
 const reverse=physics.derivative(q,-2,0,params);
 near(forward.x,2*Math.cos(0.4)); near(reverse.x,-2*Math.cos(0.4));
 near(forward.y,2*Math.sin(0.4)); near(reverse.y,-2*Math.sin(0.4));
 near(forward.psi,0); near(forward.psiT,0);
 near(reverse.psi,0); near(reverse.psiT,0);
});

test('RK4 integrates straight travel in both directions',()=>{
 const q={x:1,y:-1,psi:0,psiT:0};
 const f=physics.rk4(q,3,0,2,params);
 const r=physics.rk4(q,-3,0,2,params);
 near(f.x,7); near(f.y,-1); near(f.psi,0); near(f.psiT,0);
 near(r.x,-5); near(r.y,-1); near(r.psi,0); near(r.psiT,0);
});

test('angle normalization remains bounded across a full turn',()=>{
 const a=physics.norm(7*Math.PI);
 assert.ok(a>=-Math.PI&&a<=Math.PI);
 near(Math.abs(a),Math.PI);
});
