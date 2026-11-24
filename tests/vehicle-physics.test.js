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

test('tractor steering axle is one wheelbase ahead of rear-axle state reference',()=>{
 const q={x:4,y:-2,psi:Math.PI/2};
 const [front,rear]=physics.tractorAxlePoints(q,{wheelbase:3.8});
 near(front.x,4);
 near(front.y,1.8);
 near(rear.x,4);
 near(rear.y,-2);
 near(Math.hypot(front.x-rear.x,front.y-rear.y),3.8);
 assert.throws(()=>physics.tractorAxlePoints(q,{wheelbase:0}),/Параметры осей тягача/);
});

 
test('three trailer axle centers use the configured spacing around the equivalent group center',()=>{
 const p={...params,trailerAxleSpacing:1.3,trailerLength:13.6,trailerWidth:2.55};
 const q={x:4,y:-2,psi:0.28,psiT:-0.16};
 const points=physics.trailerAxlePoints(q,p);
 const center=physics.axlePoint(q,p);
 assert.equal(points.length,3);
 near(points[1].x,center.x);
 near(points[1].y,center.y);
 near(Math.hypot(points[0].x-points[1].x,points[0].y-points[1].y),1.3);
 near(Math.hypot(points[1].x-points[2].x,points[1].y-points[2].y),1.3);
});

test('three-axle lateral scrub proxy is zero straight and symmetric in forward and reverse turns',()=>{
 const q={x:0,y:0,psi:0.31,psiT:-0.12};
 const aligned={x:0,y:0,psi:0.31,psiT:0.31};
 for(const v of [3,-3]){
  const straight=physics.trailerAxleDiagnostics(aligned,v,0,params);
  near(straight.maxAbsLateralVelocity,0);
  const turning=physics.trailerAxleDiagnostics(q,v,0.18,{...params,trailerAxleSpacing:1.3});
  assert.equal(turning.axles.length,3);
  near(turning.axles[1].lateralVelocity,0);
  near(turning.axles[0].lateralVelocity,-turning.axles[2].lateralVelocity);
  assert.ok(turning.maxAbsLateralVelocity>0);
 }
 const forward=physics.trailerAxleDiagnostics(q,3,0.18,{...params,trailerAxleSpacing:1.3});
 const reverse=physics.trailerAxleDiagnostics(q,-3,0.18,{...params,trailerAxleSpacing:1.3});
 near(forward.axles[0].lateralVelocity,-reverse.axles[0].lateralVelocity);
});

test('vehicle geometry validation accepts the documented Volvo and trailer baseline',()=>{
 const errors=physics.validateVehicleParams({
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3
 });
 assert.deepEqual(errors,[]);
});

test('vehicle geometry validation rejects missing, non-finite and implausible dimensions',()=>{
 assert.ok(physics.validateVehicleParams(null).length>0);
 const errors=physics.validateVehicleParams({
  wheelbase:0,hitchOffset:-0.1,kingpinAxles:NaN,
  trailerLength:31,trailerWidth:0.5,trailerAxleSpacing:8
 });
 assert.equal(errors.length,6);
 assert.ok(errors.some(message=>message.startsWith('wheelbase:')));
 assert.ok(errors.some(message=>message.startsWith('kingpinAxles:')));
});

test('vehicle geometry rejects a three-axle group extending beyond the trailer body',()=>{
 const base={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55};
 const tooWide={...base,trailerAxleSpacing:5};
 const tooFarRear={...base,kingpinAxles:12,trailerAxleSpacing:1.3};
 assert.ok(physics.validateVehicleParams(tooWide).some(message=>message.includes('положение оси 3')));
 assert.ok(physics.validateVehicleParams(tooFarRear).some(message=>message.includes('положение оси 3')));
});

test('vehicle geometry accepts a long trailer with a correspondingly rearward axle group',()=>{
 const errors=physics.validateVehicleParams({
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:25,
  trailerLength:30,trailerWidth:2.55,trailerAxleSpacing:1.3
 });
 assert.deepEqual(errors,[]);
});

test('three-axle scrub exposure is zero on a straight aligned path',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 const path=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0},
  {x:1,y:0,psi:0,psiT:0,gear:1,steer:0},
  {x:2,y:0,psi:0,psiT:0,gear:1,steer:0}
 ];
 const result=physics.trailerAxleScrubExposure(path,p,0.7);
 near(result.maxAbsLateralVelocity,0);
 near(result.totalAbsoluteLateralTravel,0);
 assert.deepEqual(result.cumulativeAbsoluteLateralTravelByAxle,[0,0,0]);
});

test('three-axle scrub exposure accumulates symmetric outer-axle geometric travel',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 let q={x:0,y:0,psi:0,psiT:0};
 const path=[{...q,gear:1,steer:0.18}];
 for(let i=0;i<30;i++){
  q=physics.rk4(q,0.7,0.18,0.1,p);
  path.push({...q,gear:1,steer:0.18});
 }
 const result=physics.trailerAxleScrubExposure(path,p,0.7);
 assert.ok(result.maxAbsLateralVelocity>0);
 assert.ok(result.totalAbsoluteLateralTravel>0);
 near(result.cumulativeAbsoluteLateralTravelByAxle[0],result.cumulativeAbsoluteLateralTravelByAxle[2],1e-9);
 near(result.cumulativeAbsoluteLateralTravelByAxle[1],0,1e-9);
 near(result.totalAbsoluteLateralTravel,result.cumulativeAbsoluteLateralTravelByAxle[0]*2,1e-9);
 const scaled=physics.trailerAxleScrubExposure(path,p,1.4);
 near(scaled.maxAbsLateralVelocity,result.maxAbsLateralVelocity*2,1e-9);
 near(scaled.totalAbsoluteLateralTravel,result.totalAbsoluteLateralTravel,1e-9);
 let reverseState={x:0,y:0,psi:0,psiT:0};
 const reversePath=[{...reverseState,gear:-1,steer:0.18}];
 for(let i=0;i<30;i++){
  reverseState=physics.rk4(reverseState,-0.7,0.18,0.1,p);
  reversePath.push({...reverseState,gear:-1,steer:0.18});
 }
 const reverse=physics.trailerAxleScrubExposure(reversePath,p,0.7);
 assert.ok(reverse.maxAbsLateralVelocity>0);
 assert.ok(reverse.totalAbsoluteLateralTravel>0);
 near(reverse.cumulativeAbsoluteLateralTravelByAxle[0],reverse.cumulativeAbsoluteLateralTravelByAxle[2],1e-9);
 assert.equal(result.model,'rigid-axle-group-kinematic-proxy');
});


test('three-axle scrub exposure reports peak lateral speed separately for each axle',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 let q={x:0,y:0,psi:0,psiT:0};
 const path=[{...q,gear:1,steer:0.18}];
 for(let i=0;i<20;i++){
  q=physics.rk4(q,0.7,0.18,0.1,p);
  path.push({...q,gear:1,steer:0.18});
 }
 const result=physics.trailerAxleScrubExposure(path,p,0.7);
 assert.equal(result.maxAbsLateralVelocityByAxle.length,3);
 assert.ok(result.maxAbsLateralVelocityByAxle[0]>0);
 near(result.maxAbsLateralVelocityByAxle[0],result.maxAbsLateralVelocityByAxle[2],1e-9);
 near(result.maxAbsLateralVelocityByAxle[1],0,1e-9);
 near(result.maxAbsLateralVelocity,Math.max(...result.maxAbsLateralVelocityByAxle),1e-12);
 assert.equal(result.integration,'composite-simpson-4-subintervals');
});

test('Simpson axle exposure converges on a coarse curved path against a fine reference',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 const makePath=dt=>{
  let q={x:0,y:0,psi:0,psiT:0};
  const path=[{...q,gear:1,steer:0.18}];
  for(let i=0;i<Math.round(3/dt);i++){
   q=physics.rk4(q,0.7,0.18,dt,p);
   path.push({...q,gear:1,steer:0.18});
  }
  return path;
 };
 const coarse=physics.trailerAxleScrubExposure(makePath(0.5),p,0.7);
 const fine=physics.trailerAxleScrubExposure(makePath(0.01),p,0.7);
 assert.ok(Math.abs(coarse.totalAbsoluteLateralTravel-fine.totalAbsoluteLateralTravel)<1e-5,
  'coarse-segment Simpson integration should closely match the fine sampled reference');
});

test('three-axle scrub exposure rejects malformed route states and invalid speed',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 assert.throws(()=>physics.trailerAxleScrubExposure([{x:0,y:0,psi:0,psiT:0,gear:0}],p),/Передача/);
 assert.throws(()=>physics.trailerAxleScrubExposure([{x:0,y:0,psi:0,psiT:0}],p,0),/скорость/);
 assert.throws(()=>physics.trailerAxleScrubExposure([{x:0,y:0,psi:NaN,psiT:0}],p),/некорректные координаты/);
});


test('three-axle scrub exposure rejects an empty route instead of reporting zero exposure',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 assert.throws(()=>physics.trailerAxleScrubExposure([],p),/хотя бы одно состояние/);
});


test('three-axle scrub exposure rejects sparse route arrays',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 assert.throws(()=>physics.trailerAxleScrubExposure(new Array(1),p),/некорректные координаты/);
 assert.throws(()=>physics.trailerAxleScrubExposure([{x:0,y:0,psi:0,psiT:0},,],p),/некорректные координаты/);
});
 
test('RK4 trajectory converges under step refinement across steering reversals',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8};
 const integrate=dt=>{
  let q={x:0,y:0,psi:0.2,psiT:-0.15};
  const duration=10;
  const steps=Math.round(duration/dt);
  for(let i=0;i<steps;i++){
   const time=i*dt;
   const steer=time<3?0.55:(time<6?-0.5:0.4);
   q=physics.rk4(q,2,steer,dt,p);
  }
  return q;
 };
 const reference=integrate(0.002);
 const coarse=integrate(0.5);
 const refined=integrate(0.25);
 const stateError=q=>Math.hypot(
  q.x-reference.x,
  q.y-reference.y,
  physics.norm(q.psi-reference.psi),
  physics.norm(q.psiT-reference.psiT)
 );
 const coarseError=stateError(coarse);
 const refinedError=stateError(refined);
 assert.ok(coarseError>0);
 assert.ok(refinedError<coarseError/8,
  'halving the RK4 step should substantially reduce the trajectory error');
 assert.ok(refinedError<1e-6,
  'the refined trajectory should remain close to the fine-step reference');
});
 
test('longer kingpin-to-axle distance increases articulation for the same forward steering input',()=>{
 const base={wheelbase:3.8,hitchOffset:0.575};
 const articulationAfterTurn=kingpinAxles=>{
  let q={x:0,y:0,psi:0,psiT:0};
  const p={...base,kingpinAxles};
  const dt=0.02;
  for(let i=0;i<250;i++)q=physics.rk4(q,1,0.25,dt,p);
  return Math.abs(physics.norm(q.psi-q.psiT));
 };
 const short=articulationAfterTurn(6);
 const baseline=articulationAfterTurn(8);
 const long=articulationAfterTurn(10);
 assert.ok(short<baseline);
 assert.ok(baseline<long);
});
 
test('integrated equivalent axle center remains tangent to trailer heading across forward and reverse steering changes',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8};
 const cases=[
  {q:{x:1,y:-2,psi:0.31,psiT:-0.22},firstSteer:0.14,secondSteer:-0.22},
  {q:{x:-4,y:3,psi:2.8,psiT:-2.7},firstSteer:-0.3,secondSteer:0.35},
  {q:{x:0,y:0,psi:-1.1,psiT:0.7},firstSteer:0.28,secondSteer:-0.18}
 ];
 for(const speed of [0.7,-0.7]){
  for(const scenario of cases){
   const dt=0.01;
   const start=physics.axlePoint(scenario.q,p);
   const middle=physics.rk4(scenario.q,speed,scenario.firstSteer,dt/2,p);
   const end=physics.rk4(middle,speed,scenario.secondSteer,dt/2,p);
   const finish=physics.axlePoint(end,p);
   const dx=finish.x-start.x;
   const dy=finish.y-start.y;
   const midTrailerHeading=physics.norm(
    scenario.q.psiT+0.5*physics.norm(end.psiT-scenario.q.psiT)
   );
   const lateralDisplacement=-Math.sin(midTrailerHeading)*dx+
    Math.cos(midTrailerHeading)*dy;
   assert.ok(Math.hypot(dx,dy)>0);
   assert.ok(Math.abs(lateralDisplacement)<2e-7,
    'equivalent axle-center displacement should remain tangent to the trailer heading');
  }
 }
});

test('bounded footprint uncertainty expands tractor, cab, and trailer contours outward',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55};
 const base=physics.vehiclePolygons(q,p);
 const expanded=physics.vehiclePolygons(q,{...p,footprintUncertainty:0.1});
 const width=poly=>Math.max(...poly.map(point=>point.y))-Math.min(...poly.map(point=>point.y));
 near(width(base[0]),2.55);
 near(width(expanded[0]),2.75);
 near(width(base[1]),2.25);
 near(width(expanded[1]),2.45);
 near(width(base[2]),2.55);
 near(width(expanded[2]),2.75);
 const xBounds=poly=>({
  min:Math.min(...poly.map(point=>point.x)),
  max:Math.max(...poly.map(point=>point.x))
 });
 const trailerBase=xBounds(base[2]);
 const trailerExpanded=xBounds(expanded[2]);
 near(trailerExpanded.min,trailerBase.min-0.1);
 near(trailerExpanded.max,trailerBase.max+0.1);
});

test('vehicle geometry rejects footprint uncertainty beyond its documented bound',()=>{
 const errors=physics.validateVehicleParams({
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,footprintUncertainty:1.01
 });
 assert.ok(errors.some(message=>message.includes('footprintUncertainty')));
});
