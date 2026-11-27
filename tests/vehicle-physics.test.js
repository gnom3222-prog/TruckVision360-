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

test('vehicle geometry rejects a kingpin front offset that leaves insufficient rear body',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:5,
  trailerWidth:2.55,trailerAxleSpacing:1.3,trailerKingpinToFront:4.6};
 const errors=physics.validateVehicleParams(p);
 assert.ok(errors.some(message=>message.includes('trailerKingpinToFront')));
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

test('three-axle scrub exposure uses planner stepDistance instead of endpoint chord',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0.18};
 const end={x:0.5,y:0.2,psi:0.4,psiT:0.25,gear:1,steer:0.18};
 const chord=Math.hypot(end.x-start.x,end.y-start.y);
 const fallback=physics.trailerAxleScrubExposure([start,end],p,0.7);
 const plannerPath=physics.trailerAxleScrubExposure([start,{...end,stepDistance:1.0}],p,0.7);
 assert.equal(fallback.distanceMethod,'chord-fallback');
 assert.equal(fallback.segmentsUsingChordFallback,1);
 assert.equal(plannerPath.distanceMethod,'planner-step-distance');
 assert.equal(plannerPath.segmentsUsingStepDistance,1);
 assert.equal(plannerPath.segmentsUsingChordFallback,0);
 near(plannerPath.totalAbsoluteLateralTravel,fallback.totalAbsoluteLateralTravel/chord,1e-10);
 assert.ok(plannerPath.totalAbsoluteLateralTravel>fallback.totalAbsoluteLateralTravel);
});

test('three-axle scrub exposure rejects inconsistent planner step distances',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 const start={x:0,y:0,psi:0,psiT:0,gear:1,steer:0.18};
 const end={x:0.5,y:0.2,psi:0.4,psiT:0.25,gear:1,steer:0.18};
 assert.throws(()=>physics.trailerAxleScrubExposure([{...start,stepDistance:0.1}],p),/stepDistance/);
 assert.throws(()=>physics.trailerAxleScrubExposure([start,{...end,stepDistance:0.5}],p),/превышает stepDistance/);
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

test('trailer body contour uses the measured kingpin-to-front offset and derives rear overhang',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,
  trailerWidth:2.55,trailerKingpinToFront:2.1,footprintUncertainty:0};
 const hitch=physics.hitchPoint(q,p);
 const polygon=physics.vehiclePolygons(q,p)[2];
 const xs=polygon.map(point=>point.x);
 near(Math.max(...xs),hitch.x+2.1);
 near(Math.min(...xs),hitch.x-(13.6-2.1));
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

test('optional mirror envelopes are symmetric and grouped with cab collision geometry',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0.45,mirrorLongitudinal:3.0,mirrorLength:0.8
 };
 const parts=physics.vehicleCollisionParts(q,p);
 assert.equal(parts.length,3);
 assert.equal(parts[1].vehiclePart,'кабина');
 assert.deepEqual(parts[1].components.map(component=>component.name),[
  'кабина','левое зеркало','правое зеркало'
 ]);
 const bounds=poly=>({
  minX:Math.min(...poly.map(point=>point.x)),
  maxX:Math.max(...poly.map(point=>point.x)),
  minY:Math.min(...poly.map(point=>point.y)),
  maxY:Math.max(...poly.map(point=>point.y))
 });
 const left=bounds(parts[1].components[1].polygon);
 const right=bounds(parts[1].components[2].polygon);
 near(left.minX,2.6);
 near(left.maxX,3.4);
 near(left.minY,1.125);
 near(left.maxY,1.575);
 near(right.minX,left.minX);
 near(right.maxX,left.maxX);
 near(right.minY,-left.maxY);
 near(right.maxY,-left.minY);
});

test('mirror collision geometry is explicitly disabled when no outward projection is configured',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55};
 const parts=physics.vehicleCollisionParts(q,p);
 assert.equal(parts[1].components.length,1);
 assert.equal(parts[1].components[0].name,'кабина');
});

test('vehicle geometry rejects a cab outside the tractor envelope and bounds mirrors by the outer tractor envelope',()=>{
 const base={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55
 };
 const cabErrors=physics.validateVehicleParams({...base,cabOffset:4.8,cabFront:1.35,cabRear:0.9,cabWidth:2.25});
 assert.ok(cabErrors.some(message=>message.includes('Геометрия кабины выходит за пределы')));
 const mirrorBeyondCab=physics.validateVehicleParams({
  ...base,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0.45,mirrorLongitudinal:4.8,mirrorLength:0.2
 });
 assert.deepEqual(mirrorBeyondCab,[]);
 const mirrorBeyondTractor=physics.validateVehicleParams({
  ...base,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  mirrorOutboard:0.45,mirrorLongitudinal:5.0,mirrorLength:0.8
 });
 assert.ok(mirrorBeyondTractor.some(message=>message.includes('mirrorLongitudinal/mirrorLength')));
});


test('Ackermann steering gives the inner front wheel a larger angle for left and right turns',()=>{
 const p={wheelbase:3.8,tractorFrontTrack:2.05,tractorSteeringTrack:2.05};
 const straight=physics.frontWheelSteerAngles(0,p);
 near(straight.left,0);near(straight.right,0);
 const left=physics.frontWheelSteerAngles(0.3,p);
 assert.ok(left.left>left.right&&left.right>0);
 near(1/Math.tan(left.right)-1/Math.tan(left.left),2.05/3.8,1e-9);
 const right=physics.frontWheelSteerAngles(-0.3,p);
 assert.ok(right.left<0&&right.right<0);
 assert.ok(Math.abs(right.right)>Math.abs(right.left));
 near(1/Math.tan(Math.abs(right.left))-1/Math.tan(Math.abs(right.right)),2.05/3.8,1e-9);
});

test('Ackermann steering rejects a turn radius that cannot fit inside half the steering track',()=>{
 const p={wheelbase:3.8,tractorSteeringTrack:2.4};
 const limitingSteer=Math.atan(3.8/1.2);
 assert.throws(()=>physics.frontWheelSteerAngles(limitingSteer,p),/Радиус поворота слишком мал/);
});

test('calibrated wheel footprints cover two tractor axles and all three trailer axles',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.79,
  tireLength:0.72,tireWidth:0.44
 };
 const q={x:4,y:-2,psi:Math.PI/2,psiT:Math.PI/2};
 const wheels=physics.wheelFootprints(q,p,0.18);
 const wheelsFromState=physics.wheelFootprints({...q,steer:0.18},p);
 assert.deepEqual(wheelsFromState,wheels);
 const steeringAngles=physics.frontWheelSteerAngles(0.18,p);
 assert.equal(wheels.length,10);
 assert.equal(wheels.filter(wheel=>wheel.vehiclePart==='тягач').length,4);
 assert.equal(wheels.filter(wheel=>wheel.vehiclePart==='полуприцеп').length,6);
 const frontLeft=wheels.find(wheel=>wheel.name==='левое переднее колесо тягача');
 assert.ok(frontLeft);
 near(frontLeft.center.x,4-2.05/2);
 near(frontLeft.center.y,1.8);
 near(frontLeft.heading,Math.PI/2+steeringAngles.left);
 const frontRight=wheels.find(wheel=>wheel.name==='правое переднее колесо тягача');
 assert.ok(frontRight);
 near(frontRight.heading,Math.PI/2+steeringAngles.right);
 assert.ok(steeringAngles.left>steeringAngles.right);
 const trailerLeft=wheels.find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа');
 assert.ok(trailerLeft);
 near(trailerLeft.heading,Math.PI/2);
 assert.equal(trailerLeft.polygon.length,4);
});

test('trailer tire footprints are part of the collision envelope beyond the cargo body',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const q={x:0,y:0,psi:0,psiT:0};
 const parts=physics.vehicleCollisionParts(q,p);
 const trailer=parts.find(part=>part.vehiclePart==='полуприцеп');
 assert.equal(trailer.components.length,7);
 assert.ok(trailer.components.some(component=>component.name==='левое колесо 1-й оси полуприцепа'));
 const leftWheel=physics.wheelFootprints(q,p,0).find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа');
 const outerY=Math.max(...leftWheel.polygon.map(point=>point.y));
 near(outerY,2.79/2+0.44/2);
 assert.ok(outerY>p.trailerWidth/2);
});

test('unconfigured rear and trailer tire groups explicitly use the single-tire fallback',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorRearTrack:2.05,trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const wheels=physics.wheelFootprints({x:0,y:0,psi:0,psiT:0},p,0);
 const rear=wheels.find(wheel=>wheel.name==='левое заднее колесо тягача');
 const trailer=wheels.find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа');
 assert.equal(rear.footprintModel,'single-tire-fallback');
 assert.equal(trailer.footprintModel,'single-tire-fallback');
 assert.equal(rear.groupWidthConfigured,false);
 assert.equal(trailer.groupWidthConfigured,false);
 near(rear.footprintWidth,0.44);
 near(trailer.footprintWidth,0.44);
});

test('configured dual-tire group envelopes span the measured outer width on each axle side',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorRearTrack:2.05,trailerTrack:2.79,tireLength:0.72,tireWidth:0.44,
  tractorRearTireGroupWidth:0.72,trailerTireGroupWidth:0.68
 };
 const q={x:0,y:0,psi:0,psiT:0};
 const wheels=physics.wheelFootprints(q,p,0);
 const rear=wheels.find(wheel=>wheel.name==='левое заднее колесо тягача');
 const trailer=wheels.find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа');
 assert.equal(rear.footprintModel,'configured-axle-side-group-envelope');
 assert.equal(trailer.footprintModel,'configured-axle-side-group-envelope');
 assert.equal(rear.groupWidthConfigured,true);
 assert.equal(trailer.groupWidthConfigured,true);
 const trailerGroups=wheels.filter(wheel=>wheel.vehiclePart==='полуприцеп');
 assert.equal(trailerGroups.length,6);
 assert.ok(trailerGroups.every(wheel=>wheel.footprintWidth===0.68&&wheel.groupWidthConfigured));
 near(Math.max(...rear.polygon.map(point=>point.y))-Math.min(...rear.polygon.map(point=>point.y)),0.72);
 near(Math.max(...trailer.polygon.map(point=>point.y))-Math.min(...trailer.polygon.map(point=>point.y)),0.68);
 const parts=physics.vehicleCollisionParts(q,p);
 assert.ok(parts.find(part=>part.vehiclePart==='тягач').components.some(component=>component.name==='левое заднее колесо тягача'));
 assert.ok(parts.find(part=>part.vehiclePart==='полуприцеп').components.some(component=>component.name==='левое колесо 1-й оси полуприцепа'));
});

test('tire group calibration rejects widths smaller than one tire, overlapping pairs, and excessive trailer width',()=>{
 const base={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  tractorRearTrack:2.05,trailerTrack:2.79,tireWidth:0.44
 };
 assert.ok(physics.validateVehicleParams({...base,tractorRearTireGroupWidth:0.4})
  .some(message=>message.includes('не может быть меньше ширины')));
 assert.ok(physics.validateVehicleParams({...base,tractorRearTrack:1.4,tractorRearTireGroupWidth:1.4})
  .some(message=>message.includes('группы шин перекрываются')));
 assert.ok(physics.validateVehicleParams({...base,trailerTireGroupWidth:0.8})
  .some(message=>message.includes('колёсные группы выходят')));
});

test('wheel calibration rejects an implausible track and tire-width combination',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  tractorFrontTrack:3.0,tireWidth:0.8
 };
 const errors=physics.validateVehicleParams(p);
 assert.ok(errors.some(message=>message.includes('tractorTrack/tireWidth')));
 assert.throws(()=>physics.wheelFootprints({x:0,y:0,psi:0,psiT:0},p),/Некорректные параметры колёс/);
});

test('trailer wheel trajectory diagnostics returns sampled centers for six axle-side groups and three axle centers',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44
 };
 const radius=12,offset=p.kingpinAxles+p.hitchOffset;
 const path=[];
 for(let i=0;i<=5;i++){
  const theta=i*0.05;
  const center={x:radius*Math.sin(theta),y:radius*(1-Math.cos(theta))};
  path.push({
   x:center.x+offset*Math.cos(theta),
   y:center.y+offset*Math.sin(theta),
   psi:theta,psiT:theta,gear:1,steer:0
  });
 }
 const result=physics.trailerWheelTrajectoryDiagnostics(path,p);
 assert.equal(result.samples,6);
 assert.equal(result.axles.length,3);
 assert.equal(result.segments.length,5);
 for(const axle of result.axles){
  assert.equal(axle.center.points.length,6);
  assert.equal(axle.left.points.length,6);
  assert.equal(axle.right.points.length,6);
  assert.ok(axle.center.sampledTravel>0);
  assert.ok(axle.left.sampledTravel>0);
  assert.ok(axle.right.sampledTravel>0);
 }
 near(result.axles[0].left.points[0].y,p.trailerTrack/2);
 near(result.axles[0].right.points[0].y,-p.trailerTrack/2);
 assert.equal(result.model,'single-equivalent-trailer-axle-group');
});

test('trailer wheel trajectory diagnostics integrates straight wheel paths from planner RK4 steps',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79
 };
 const step=0.5;
 let q={x:0,y:0,psi:0,psiT:0};
 const path=[{...q,gear:1,steer:0,stepDistance:0}];
 for(let i=0;i<4;i++){
  q=physics.rk4(q,0.7,0,step/0.7,p);
  path.push({...q,gear:1,steer:0,stepDistance:step});
 }
 const result=physics.trailerWheelTrajectoryDiagnostics(path,p);
 assert.equal(result.wheelTravelMethod,'rk4-reconstructed-composite-simpson-4-subintervals');
 assert.equal(result.segmentsWithKinematicEstimate,4);
 assert.equal(result.segmentsWithoutKinematicEstimate,0);
 near(result.totalRouteDistance,2,1e-9);
 for(const axle of result.axles){
  for(const track of [axle.center,axle.left,axle.right]){
   near(track.sampledTravel,2,1e-8);
   near(track.integratedTravelEstimate,2,1e-8);
   near(track.integratedTravelCoveredDistance,2,1e-9);
  }
 }
});

test('trailer wheel trajectory diagnostics estimates curved wheel arcs beyond endpoint chords',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79
 };
 const step=0.5;
 let q={x:0,y:0,psi:0,psiT:0};
 const path=[{...q,gear:1,steer:0.18,stepDistance:0}];
 for(let i=0;i<6;i++){
  q=physics.rk4(q,0.7,0.18,step/0.7,p);
  path.push({...q,gear:1,steer:0.18,stepDistance:step});
 }
 const result=physics.trailerWheelTrajectoryDiagnostics(path,p);
 assert.equal(result.segmentsWithKinematicEstimate,6);
 assert.equal(result.segmentsWithoutKinematicEstimate,0);
 for(const axle of result.axles){
  for(const track of [axle.center,axle.left,axle.right]){
   assert.ok(track.integratedTravelEstimate>=track.sampledTravel-1e-7);
   near(track.integratedTravelCoveredDistance,3,1e-8);
  }
 }
 for(const segment of result.segments){
  assert.equal(segment.integrationStatus,'rk4-simpson-estimate');
  assert.ok(segment.endpointResidual.position<=result.kinematicEndpointTolerance.position);
  assert.ok(segment.axles.every(axle=>axle.integratedOuterMinusInner>0));
 }
});

test('trailer wheel trajectory diagnostics withholds integrated estimates when planner endpoints do not match RK4',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 const path=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0.18,stepDistance:0},
  {x:0.5,y:0,psi:0,psiT:0,gear:1,steer:0.18,stepDistance:0.5}
 ];
 const result=physics.trailerWheelTrajectoryDiagnostics(path,p);
 assert.equal(result.segments[0].integrationStatus,'kinematic-endpoint-mismatch');
 assert.equal(result.segmentsWithKinematicEstimate,0);
 assert.equal(result.segmentsWithoutKinematicEstimate,1);
 assert.equal(result.axles[0].center.integratedTravelEstimate,null);
 assert.ok(result.segments[0].endpointResidual.position>result.kinematicEndpointTolerance.position);
});

test('trailer wheel trajectory diagnostics keeps chord-only fallback explicit when stepDistance is absent',()=>{
 const p={wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3};
 const path=[
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0},
  {x:1,y:0,psi:0,psiT:0,gear:1,steer:0}
 ];
 const result=physics.trailerWheelTrajectoryDiagnostics(path,p);
 assert.equal(result.distanceMethod,'chord-fallback');
 assert.equal(result.wheelTravelMethod,'sampled-chords-only');
 assert.equal(result.segments[0].integrationStatus,'step-distance-missing');
 assert.equal(result.axles[0].center.integratedTravelEstimate,null);
 near(result.axles[0].center.sampledTravel,1,1e-9);
});

test('trailer wheel paths identify inner and outer sides for forward and reverse turns',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79
 };
 const makeArc=(angles,gear)=>{
  const radius=12,offset=p.kingpinAxles+p.hitchOffset;
  return angles.map(theta=>{
   const center={x:radius*Math.sin(theta),y:radius*(1-Math.cos(theta))};
   return {
    x:center.x+offset*Math.cos(theta),
    y:center.y+offset*Math.sin(theta),
    psi:theta,psiT:theta,gear,steer:0
   };
  });
 };
 const forward=physics.trailerWheelTrajectoryDiagnostics(
  makeArc([0,0.05,0.1,0.15,0.2],1),p
 );
 for(const segment of forward.segments){
  assert.equal(segment.axles[0].turnDirection,'left');
  assert.equal(segment.axles[0].innerSide,'left');
  assert.equal(segment.axles[0].outerSide,'right');
  assert.ok(segment.axles.every(axle=>axle.outerMinusInner>0));
 }
 const reverse=physics.trailerWheelTrajectoryDiagnostics(
  makeArc([0,-0.05,-0.1,-0.15,-0.2],-1),p
 );
 for(const segment of reverse.segments){
  assert.equal(segment.axles[0].turnDirection,'right');
  assert.equal(segment.axles[0].innerSide,'left');
  assert.equal(segment.axles[0].outerSide,'right');
  assert.ok(segment.axles.every(axle=>axle.outerMinusInner>0));
 }
});

test('trailer wheel trajectory diagnostics rejects malformed and sparse route data',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3
 };
 assert.throws(()=>physics.trailerWheelTrajectoryDiagnostics([],p),/хотя бы одно состояние/);
 assert.throws(()=>physics.trailerWheelTrajectoryDiagnostics(new Array(1),p),/некорректные координаты/);
 assert.throws(()=>physics.trailerWheelTrajectoryDiagnostics([
  {x:0,y:0,psi:0,psiT:0,gear:0}
 ],p),/Передача/);
 assert.doesNotThrow(()=>physics.trailerWheelTrajectoryDiagnostics([
  {x:0,y:0,psi:0,psiT:0,stepDistance:0}
 ],p));
 assert.throws(()=>physics.trailerWheelTrajectoryDiagnostics([
  {x:0,y:0,psi:0,psiT:0,stepDistance:0.01}
 ],p),/stepDistance/);
 assert.throws(()=>physics.trailerWheelTrajectoryDiagnostics([
  {x:0,y:0,psi:0,psiT:0},
  {x:2,y:0,psi:0,psiT:0,stepDistance:1}
 ],p),/превышает stepDistance/);
});

test('trailer wheel trajectory diagnostics marks gear changes without hiding sampled geometry',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3
 };
 const path=[
  {x:0,y:0,psi:0,psiT:0,gear:1},
  {x:0.5,y:0,psi:0,psiT:0,gear:-1,stepDistance:0.5}
 ];
 const result=physics.trailerWheelTrajectoryDiagnostics(path,p);
 assert.equal(result.segments.length,1);
 assert.equal(result.segments[0].gear,-1);
 assert.equal(result.segments[0].gearChanged,true);
 assert.equal(result.segments[0].axles[0].turnDirection,'straight');
});



test('collision components use the calibrated steering angle for protruding front tires',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  tractorFrontTrack:2.4,tractorRearTrack:2.05,trailerTrack:2.5,
  tireLength:1.2,tireWidth:0.8
 };
 assert.deepEqual(physics.validateVehicleParams(p),[]);
 const q={x:0,y:0,psi:0,psiT:0};
 const findLeftFront=groups=>groups[0].components.find(component=>
  component.name==='левое переднее колесо тягача'
 );
 const straight=findLeftFront(physics.vehicleCollisionParts(q,p,0));
 const steered=findLeftFront(physics.vehicleCollisionParts(q,p,0.6));
 const steeredFromState=findLeftFront(physics.vehicleCollisionParts({...q,steer:0.6},p));
 const expected=physics.wheelFootprints(q,p,0.6).find(wheel=>
  wheel.name==='левое переднее колесо тягача'
 );
 assert.ok(straight&&steered&&expected,'the outboard front tire must be a separate collision component');
 assert.notDeepEqual(steered.polygon,straight.polygon);
 assert.deepEqual(steered.polygon,expected.polygon);
 assert.deepEqual(steeredFromState.polygon,expected.polygon);
 assert.throws(()=>physics.vehicleCollisionParts(q,p,Math.PI/2),/Угол поворота колёс/);
});
 
test('trailer axle-specific tire group widths override the shared fallback independently',()=>{
 const p={
  ...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireLength:0.72,tireWidth:0.44,
  trailerTireGroupWidth:0.70,
  trailerAxle1TireGroupWidth:0.74,
  trailerAxle3TireGroupWidth:0.72
 };
 const q={x:0,y:0,psi:0,psiT:0};
 const groups=physics.wheelFootprints(q,p);
 const widths=[1,2,3].map(axle=>{
  const group=groups.find(item=>item.vehiclePart==='полуприцеп'&&item.axleIndex===axle&&item.side==='left');
  assert.ok(group);
  return group.footprintWidth;
 });
 assert.deepEqual(widths,[0.74,0.70,0.72]);
 assert.equal(groups.filter(item=>item.vehiclePart==='полуприцеп').length,6);
 assert.ok(groups.filter(item=>item.vehiclePart==='полуприцеп').every(item=>
  item.footprintModel==='configured-axle-side-group-envelope'
 ));
});

test('trailer axle-specific tire group widths are range-checked against single tire width and track',()=>{
 const base={
  ...params,trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
  trailerTrack:2.79,tireWidth:0.44
 };
 const tooNarrow=physics.validateVehicleParams({...base,trailerAxle2TireGroupWidth:0.4});
 assert.ok(tooNarrow.some(message=>message.includes('trailerAxle2TireGroupWidth')));
 const overlapping=physics.validateVehicleParams({...base,trailerTrack:1.2,trailerAxle3TireGroupWidth:1.2});
 assert.ok(overlapping.some(message=>message.includes('trailerAxle3TireGroupWidth')));
 const valid=physics.validateVehicleParams({...base,trailerTireGroupWidth:0.7,trailerAxle1TireGroupWidth:0.74});
 assert.deepEqual(valid,[]);
});
 
test('tire group uncertainty lower bounds stay above the maximum single tire width',()=>{
 const base={
  tireWidth:0.44,tireWidthUncertainty:0.05,
  tractorRearTireGroupWidth:0.72,tractorRearTireGroupWidthUncertainty:0.2,
  trailerTireGroupWidth:0.70,trailerTireGroupWidthUncertainty:0.2,
  trailerAxle1TireGroupWidth:0.72,trailerAxle1TireGroupWidthUncertainty:0.2
 };
 assert.deepEqual(physics.validateTireGroupWidthUncertainty(base),[]);
 const tooNarrow=physics.validateTireGroupWidthUncertainty({
  ...base,trailerAxle2TireGroupWidth:0.60,trailerAxle2TireGroupWidthUncertainty:0.15
 });
 assert.ok(tooNarrow.some(message=>message.includes('trailerAxle2TireGroupWidthUncertainty')));
 const missingNominal=physics.validateTireGroupWidthUncertainty({
  ...base,trailerAxle3TireGroupWidth:0,trailerAxle3TireGroupWidthUncertainty:0.1
 });
 assert.ok(missingNominal.some(message=>message.includes('trailerAxle3TireGroupWidthUncertainty')));
});

test('independent adjacent axle spacings preserve the middle axle reference',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  trailerAxleSpacing:1.3,trailerAxleSpacing12:1.1,trailerAxleSpacing23:1.6
 };
 assert.deepEqual(physics.trailerAxleSpacings(p),[1.1,1.6]);
 assert.deepEqual(physics.trailerAxleOffsets(p),[1.1,0,-1.6]);
 const q={x:2,y:-1,psi:0.2,psiT:-0.3};
 const center=physics.axlePoint(q,p);
 const points=physics.trailerAxlePoints(q,p);
 near(Math.hypot(points[0].x-center.x,points[0].y-center.y),1.1);
 near(Math.hypot(points[2].x-center.x,points[2].y-center.y),1.6);
 near(physics.norm(Math.atan2(points[0].y-center.y,points[0].x-center.x)-q.psiT),0);
 near(physics.norm(Math.atan2(points[2].y-center.y,points[2].x-center.x)-q.psiT),Math.PI);
});

test('per-axle track overrides are used by each left and right wheel-group footprint',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.7,
  trailerTrack:2.79,trailerAxle1Track:2.7,trailerAxle2Track:2.8,trailerAxle3Track:2.9,
  trailerAxle1TireGroupWidth:0.62,trailerAxle2TireGroupWidth:0.68,trailerAxle3TireGroupWidth:0.72
 };
 assert.deepEqual(physics.trailerAxleTracks(p),[2.7,2.8,2.9]);
 const q={x:0,y:0,psi:0,psiT:0};
 const wheels=physics.wheelFootprints(q,p,0);
 for(let index=1;index<=3;index++){
  const left=wheels.find(wheel=>wheel.name==='левое колесо '+index+'-й оси полуприцепа');
  const right=wheels.find(wheel=>wheel.name==='правое колесо '+index+'-й оси полуприцепа');
  assert.ok(left&&right);
  near(left.center.y-right.center.y,p['trailerAxle'+index+'Track']);
  near(Math.max(...left.polygon.map(point=>point.y))-Math.min(...left.polygon.map(point=>point.y)),
   p['trailerAxle'+index+'TireGroupWidth']);
 }
});

test('wheel trajectory diagnostics use the configured track of each axle independently',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  trailerTrack:2.79,trailerAxle1Track:2.7,trailerAxle2Track:2.8,trailerAxle3Track:2.9
 };
 const result=physics.trailerWheelTrajectoryDiagnostics([
  {x:0,y:0,psi:0,psiT:0,gear:1,steer:0}
 ],p);
 assert.equal(result.axles.length,3);
 for(let index=0;index<3;index++){
  const axle=result.axles[index];
  near(axle.left.points[0].y-axle.right.points[0].y,[2.7,2.8,2.9][index]);
 }
});

test('per-axle trailer spacing and track validation rejects invalid values and body-envelope violations',()=>{
 const base={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  trailerAxleSpacing:1.3,trailerTrack:2.79,tireWidth:0.44
 };
 const shortSpacing=physics.validateVehicleParams({...base,trailerAxleSpacing12:0.2});
 assert.ok(shortSpacing.some(message=>message.includes('trailerAxleSpacing12')));
 const axleOutside=physics.validateVehicleParams({...base,trailerAxleSpacing23:5});
 assert.ok(axleOutside.some(message=>message.includes('положение оси 3')));
 const narrowTrack=physics.validateVehicleParams({...base,trailerAxle2Track:0.8});
 assert.ok(narrowTrack.some(message=>message.includes('trailerAxle2Track')));
 const excessiveOuter=physics.validateVehicleParams({...base,trailerAxle3Track:3.2});
 assert.ok(excessiveOuter.some(message=>message.includes('trailerAxle3Track')));
});

test('zero per-axle values inherit common spacing and track settings',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  trailerAxleSpacing:1.4,trailerAxleSpacing12:0,trailerAxleSpacing23:0,
  trailerTrack:2.85,trailerAxle1Track:0,trailerAxle2Track:0,trailerAxle3Track:0
 };
 assert.deepEqual(physics.trailerAxleSpacings(p),[1.4,1.4]);
 assert.deepEqual(physics.trailerAxleOffsets(p),[1.4,0,-1.4]);
 assert.deepEqual(physics.trailerAxleTracks(p),[2.85,2.85,2.85]);
});


test('asymmetric axle spacings produce asymmetric rigid-group scrub diagnostics',()=>{
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  trailerAxleSpacing12:1.1,trailerAxleSpacing23:1.6
 };
 const diagnostic=physics.trailerAxleDiagnostics(
  {x:0,y:0,psi:0.3,psiT:0},1,0.18,p
 );
 assert.ok(Math.abs(diagnostic.axles[0].lateralVelocity)>0);
 near(
  Math.abs(diagnostic.axles[2].lateralVelocity)/Math.abs(diagnostic.axles[0].lateralVelocity),
  1.6/1.1,1e-9
 );
});


const tirePathParams={
 wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
 trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
 tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.11,
 tireLength:0.72,tireWidth:0.32,
 tractorRearTireCenterSpacing:0.36,
 trailerAxle1TireCenterSpacing:0.36,
 trailerAxle2TireCenterSpacing:0.36,
 trailerAxle3TireCenterSpacing:0.36
};
const makeTirePath=(gear,steer,steps=4,stepDistance=0.25)=>{
 let q={x:0,y:0,psi:0,psiT:0};
 const path=[{...q,gear,steer,stepDistance:0}];
 for(let i=0;i<steps;i++){
  q=physics.rk4(q,gear*0.7,steer,stepDistance/0.7,tirePathParams);
  path.push({...q,gear,steer,stepDistance});
 }
 return path;
};

test('per-tire trajectory diagnostics integrate all 18 calibrated tire centers on a straight route',()=>{
 const result=physics.wheelTireTrajectoryDiagnostics(makeTirePath(1,0),tirePathParams);
 assert.equal(result.tires.length,18);
 assert.equal(result.integratedTravelComplete,true);
 assert.equal(result.segmentsWithKinematicEstimate,4);
 assert.equal(result.segmentsWithoutKinematicEstimate,0);
 for(const tire of result.tires){
  near(tire.sampledTravel,1,1e-8);
  near(tire.integratedTravelEstimate,1,1e-8);
  assert.equal(tire.points.length,5);
 }
 near(result.totalIntegratedTireTravel,18,1e-8);
 assert.equal(result.tires.filter(tire=>tire.pairPosition==='outer').length,8);
 assert.equal(result.tires.filter(tire=>tire.pairPosition==='inner').length,8);
 assert.ok(result.longestIntegratedTire);
 assert.equal(result.integration,'rk4-reconstructed-composite-simpson-4-subintervals');
});

test('per-tire path estimates distinguish inner and outer tires during a forward turn',()=>{
 const result=physics.wheelTireTrajectoryDiagnostics(makeTirePath(1,0.28,30,0.1),tirePathParams);
 assert.equal(result.integratedTravelComplete,true);
 const outer=result.tires.find(tire=>
  tire.vehiclePart==='полуприцеп'&&tire.axleIndex===2&&tire.side==='right'&&tire.pairPosition==='outer'
 );
 const inner=result.tires.find(tire=>
  tire.vehiclePart==='полуприцеп'&&tire.axleIndex===2&&tire.side==='right'&&tire.pairPosition==='inner'
 );
 assert.ok(outer&&inner);
 assert.ok(Math.abs(outer.integratedTravelEstimate-inner.integratedTravelEstimate)>1e-5);
 assert.ok(result.maxSegmentRefinementDelta>=0);
 assert.ok(Number.isFinite(outer.refinementIndicatorTotal));
});

test('per-tire trajectory diagnostics support reverse motion and retain positive traveled lengths',()=>{
 const result=physics.wheelTireTrajectoryDiagnostics(makeTirePath(-1,-0.22,12,0.1),tirePathParams);
 assert.equal(result.integratedTravelComplete,true);
 assert.equal(result.segmentsWithKinematicEstimate,12);
 assert.ok(result.tires.every(tire=>tire.integratedTravelEstimate>0));
 assert.ok(result.tires.every(tire=>tire.points.every(point=>point.gear===-1)));
});

test('per-tire sampled chords remain available but integrated estimates are withheld without stepDistance',()=>{
 const q0={x:0,y:0,psi:0,psiT:0};
 const q1=physics.rk4(q0,0.7,0.18,1/0.7,tirePathParams);
 const result=physics.wheelTireTrajectoryDiagnostics([
  {...q0,gear:1,steer:0.18},
  {...q1,gear:1,steer:0.18}
 ],tirePathParams);
 assert.equal(result.distanceMethod,'chord-fallback');
 assert.equal(result.segmentsWithoutKinematicEstimate,1);
 assert.equal(result.integratedTravelComplete,false);
 assert.ok(result.tires.every(tire=>tire.sampledTravel>0));
 assert.ok(result.tires.every(tire=>tire.integratedTravelEstimate===null));
});

test('per-tire integrated estimates are withheld when RK4 endpoint reconstruction mismatches',()=>{
 const q0={x:0,y:0,psi:0,psiT:0};
 const q1=physics.rk4(q0,0.7,0.12,0.1/0.7,tirePathParams);
 q1.psiT+=0.02;
 const result=physics.wheelTireTrajectoryDiagnostics([
  {...q0,gear:1,steer:0.12,stepDistance:0},
  {...q1,gear:1,steer:0.12,stepDistance:0.1}
 ],tirePathParams);
 assert.equal(result.segmentsWithoutKinematicEstimate,1);
 assert.equal(result.integratedTravelComplete,false);
 assert.ok(result.tires.every(tire=>tire.integratedTravelEstimate===null));
 assert.equal(result.segmentDiagnostics.length,1);
 assert.ok(result.segmentDiagnostics[0].endpointResidual.trailerHeading>1e-6);
});


test('coarse per-tire Simpson path estimates converge toward a dense RK4 reference',()=>{
 const coarse=physics.wheelTireTrajectoryDiagnostics(makeTirePath(1,0.28,30,0.1),tirePathParams);
 const fine=physics.wheelTireTrajectoryDiagnostics(makeTirePath(1,0.28,300,0.01),tirePathParams);
 assert.equal(coarse.integratedTravelComplete,true);
 assert.equal(fine.integratedTravelComplete,true);
 const fineByName=new Map(fine.tires.map(tire=>[tire.name,tire.integratedTravelEstimate]));
 const errors=coarse.tires.map(tire=>Math.abs(
  tire.integratedTravelEstimate-fineByName.get(tire.name)
 ));
 assert.ok(errors.every(Number.isFinite));
 assert.ok(Math.max(...errors)<1e-6,'refined route sampling should closely match each tire-center path integral');
});


test('bounded planar road surface computes elevation extrema across a component footprint',()=>{
 const polygon=[
  {x:10,y:0},{x:12,y:0},{x:12,y:2},{x:10,y:2}
 ];
 const surface={
  originX:0,originY:0,originZ:0,gradeX:0.1,gradeY:0.05,
  originZUncertainty:0.01,gradeXUncertainty:0.001,gradeYUncertainty:0.002,
  surfaceResidualUncertainty:0.01
 };
 const elevation=physics.roadSurfaceElevationRange(polygon,surface);
 assert.equal(elevation.status,'configured-planar-surface-with-bounds');
 near(elevation.min,0.97,1e-12);
 near(elevation.max,1.336,1e-12);
 const shifted=physics.verticalRangeOnRoadSurface({min:2,max:3},polygon,surface);
 near(shifted.min,2.97,1e-12);
 near(shifted.max,4.336,1e-12);
 assert.equal(shifted.roadSurfaceStatus,'configured-planar-surface-with-bounds');
});

test('missing road-surface calibration provides no vertical datum',()=>{
 const polygon=[{x:0,y:0},{x:1,y:0},{x:1,y:1},{x:0,y:1}];
 assert.equal(physics.roadSurfaceElevationRange(polygon,null),null);
 assert.equal(physics.verticalRangeOnRoadSurface({min:0.2,max:0.8},polygon,null),null);
});


test('road-surface uncertainty bounds must be explicit and positive',()=>{
 const valid={originX:0,originY:0,originZ:0,gradeX:0,gradeY:0,
  originZUncertainty:0.001,gradeXUncertainty:0.0001,gradeYUncertainty:0.0001,surfaceResidualUncertainty:0.001};
 assert.deepEqual(physics.validateRoadSurface(valid),[]);
 for(const key of ['originZUncertainty','gradeXUncertainty','gradeYUncertainty','surfaceResidualUncertainty']){
  assert.ok(physics.validateRoadSurface({...valid,[key]:0}).some(error=>error.includes(key)),key+' must reject zero');
  const missing={...valid};delete missing[key];
  assert.ok(physics.validateRoadSurface(missing).some(error=>error.includes(key)),key+' must reject missing bounds');
 }
});

test('road-surface calibration rejects missing uncertainty bounds and excessive grades',()=>{
 const incomplete={originX:0,originY:0,originZ:0,gradeX:0,gradeY:0};
 assert.ok(physics.validateRoadSurface(incomplete).some(error=>error.includes('originZUncertainty')));
 const steep={
  originX:0,originY:0,originZ:0,gradeX:0.3,gradeY:0,
  originZUncertainty:0.001,gradeXUncertainty:0.0001,gradeYUncertainty:0.0001,surfaceResidualUncertainty:0.001
 };
 assert.ok(physics.validateRoadSurface(steep).some(error=>error.includes('gradeX')));
 assert.deepEqual(physics.validateRoadSurface(null),[]);
});


function makeSuspensionTravelPoint(compression=0.03,rebound=0.03,uncertainty=0.005){
 return {compression,rebound,uncertainty};
}
function makeSuspensionTravelConfig(point=makeSuspensionTravelPoint()){
 const axle=()=>({left:{...point},right:{...point}});
 return {
  tractor:{front:axle(),rear:axle()},
  trailer:{axle1:axle(),axle2:axle(),axle3:axle()}
 };
}
const suspensionGeometryParams={
 wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
 trailerLength:13.6,trailerWidth:2.55,trailerAxleSpacing:1.3,
 tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.11,
 wheelbaseUncertainty:0.02,tractorFrontTrackUncertainty:0.02,
 tractorRearTrackUncertainty:0.02,trailerAxleSpacingUncertainty:0.02,
 trailerTrackUncertainty:0.02
};

test('suspension travel requires every axle side and explicit compression, rebound, and uncertainty',()=>{
 const incomplete=makeSuspensionTravelConfig();
 delete incomplete.trailer.axle3.right;
 assert.ok(physics.validateSuspensionTravel(incomplete,suspensionGeometryParams).some(
  error=>error.includes('trailer.axle3.right')
 ));
 const missingBound=makeSuspensionTravelConfig();
 delete missingBound.tractor.front.left.uncertainty;
 assert.ok(physics.validateVehicleParams({
  ...suspensionGeometryParams,suspensionTravel:missingBound
 }).some(error=>error.includes('uncertainty')));
 assert.deepEqual(physics.validateSuspensionTravel(null,suspensionGeometryParams),[]);
 const zeroGeometry={...suspensionGeometryParams,wheelbaseUncertainty:0};
 assert.ok(physics.validateSuspensionTravel(makeSuspensionTravelConfig(),zeroGeometry)
  .some(error=>error.includes('wheelbaseUncertainty')));
});

test('bounded rigid-plane suspension envelope includes heave pitch and roll over tractor body corners',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const polygon=physics.vehiclePolygons(q,suspensionGeometryParams)[0];
 const config=makeSuspensionTravelConfig();
 config.tractor.front.left.compression=0.06;
 config.tractor.front.right.rebound=0.07;
 const range=physics.suspensionTravelDisplacementRange(
  polygon,q,{...suspensionGeometryParams,suspensionTravel:config},'tractor'
 );
 assert.equal(range.status,'bounded-rigid-plane-support-envelope');
 assert.equal(range.model,'independent-support-intervals-linearized-rigid-plane');
 assert.ok(range.min<0);
 assert.ok(range.max>0);
 assert.ok(range.max-range.min>0.1,'independent support travel should widen the body envelope');
});

test('independent trailer axle support intervals produce different vertical envelopes',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const polygon=physics.vehiclePolygons(q,suspensionGeometryParams)[2];
 const first=makeSuspensionTravelConfig(makeSuspensionTravelPoint(0,0,0.001));
 first.trailer.axle1.left.compression=0.06;
 const third=makeSuspensionTravelConfig(makeSuspensionTravelPoint(0,0,0.001));
 third.trailer.axle3.left.compression=0.06;
 const firstRange=physics.suspensionTravelDisplacementRange(
  polygon,q,{...suspensionGeometryParams,suspensionTravel:first},'trailer'
 );
 const thirdRange=physics.suspensionTravelDisplacementRange(
  polygon,q,{...suspensionGeometryParams,suspensionTravel:third},'trailer'
 );
 assert.ok(Math.abs(firstRange.min-thirdRange.min)>1e-5||
  Math.abs(firstRange.max-thirdRange.max)>1e-5,
 'moving a different axle support must change the trailer-body interval');
});

test('suspension travel expands measured body and cab profiles but leaves road-contact tire profiles unchanged',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const config=makeSuspensionTravelConfig(makeSuspensionTravelPoint(0.03,0.04,0.005));
 const p={
  ...suspensionGeometryParams,suspensionTravel:config,
  vehicleVerticalProfiles:{
   'tractor-body':{zMin:0.8,zMax:1.2,uncertaintyMargin:0.02},
   'tractor-cab':{zMin:1.8,zMax:2.2,uncertaintyMargin:0.02},
   'trailer-body':{zMin:1,zMax:4,uncertaintyMargin:0.02},
   'tractor-front-wheel':{zMin:0,zMax:0.9,uncertaintyMargin:0.02}
  }
 };
 const parts=physics.vehicleCollisionParts(q,p,0,true);
 const tractorBody=parts[0].components.find(component=>component.name==='основной контур тягача');
 const cab=parts[1].components.find(component=>component.name==='кабина');
 const trailer=parts[2].components.find(component=>component.name==='кузов полуприцепа');
 assert.equal(tractorBody.verticalRange.suspensionStatus,'bounded-rigid-plane-support-envelope');
 assert.ok(tractorBody.verticalRange.min<0.8&&tractorBody.verticalRange.max>1.2);
 assert.ok(cab.verticalRange.min<1.8&&cab.verticalRange.max>2.2);
 assert.ok(trailer.verticalRange.min<1&&trailer.verticalRange.max>4);
 const frontWheel=physics.wheelFootprints(q,p,0).find(wheel=>
  wheel.vehiclePart==='тягач'&&wheel.axleIndex===1
 );
 assert.deepEqual(frontWheel.verticalRange,{min:-0.02,max:0.92});
});

test('auxiliary envelopes move with suspension only when body attachment is explicit',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const config=makeSuspensionTravelConfig(makeSuspensionTravelPoint(0.02,0.05,0.005));
 const base={
  ...suspensionGeometryParams,suspensionTravel:config,
  auxiliaryEnvelopes:[{
   name:'Frame bracket',anchor:'tractor-front-axle',
   longitudinalOffset:0,lateralOffset:0,length:0.4,width:0.2,
   headingOffsetDeg:0,uncertaintyMargin:0,zMin:1,zMax:1.5,verticalUncertaintyMargin:0.02
  }]
 };
 const axleMounted=physics.auxiliaryEnvelopePolygons(q,base)[0];
 const bodyMounted=physics.auxiliaryEnvelopePolygons(q,{
  ...base,auxiliaryEnvelopes:[{...base.auxiliaryEnvelopes[0],suspensionAttachment:'tractor-body'}]
 })[0];
 assert.deepEqual(axleMounted.verticalRange,{min:0.98,max:1.52});
 assert.equal(bodyMounted.verticalRange.suspensionStatus,'bounded-rigid-plane-support-envelope');
 assert.ok(bodyMounted.verticalRange.max>axleMounted.verticalRange.max);
 const mismatched=[{...base.auxiliaryEnvelopes[0],suspensionAttachment:'tractor-body',anchor:'trailer-axle-1'}];
 assert.ok(physics.validateAuxiliaryEnvelopes(mismatched).some(error=>error.includes('tractor-body')));
});

test('suspension travel rejects derived pitch or roll slopes outside the linearized model bound',()=>{
 const config=makeSuspensionTravelConfig(makeSuspensionTravelPoint(0,0,0.001));
 config.trailer.axle1.left.compression=0.25;
 config.trailer.axle1.right.rebound=0.25;
 config.trailer.axle1.left.uncertainty=0.05;
 config.trailer.axle1.right.uncertainty=0.05;
 const errors=physics.validateSuspensionTravel(config,suspensionGeometryParams);
 assert.ok(errors.some(error=>error.includes('превышает 0,25')));
});

test('vehicle geometry calibration status treats zero uncertainty as unbounded, not exact',()=>{
 const nominal={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerKingpinToFront:1.5,trailerLength:13.6,trailerWidth:2.55,
  trailerAxleSpacing:1.3,trailerAxleSpacing12:0,trailerAxleSpacing23:0,
  tractorSteeringTrack:2.05,tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.79,
  trailerAxle1Track:0,trailerAxle2Track:0,trailerAxle3Track:0,
  tireLength:0.72,tireWidth:0.44,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  tractorFrontOverhang:5,tractorRearOverhang:1.2,tractorBodyWidth:2.55,
  mirrorOutboard:0,mirrorLongitudinal:3.0,mirrorLength:0.8,vehicleFootprintUncertainty:0.03,
  wheelbaseUncertainty:0,hitchOffsetUncertainty:0,kingpinAxlesUncertainty:0,
  trailerLengthUncertainty:0,trailerKingpinToFrontUncertainty:0,trailerWidthUncertainty:0,trailerAxleSpacingUncertainty:0,
  tractorSteeringTrackUncertainty:0,tractorFrontTrackUncertainty:0,tractorRearTrackUncertainty:0,
  trailerTrackUncertainty:0,tireLengthUncertainty:0,tireWidthUncertainty:0,
  cabOffsetUncertainty:0,cabFrontUncertainty:0,cabRearUncertainty:0,cabWidthUncertainty:0,
  tractorFrontOverhangUncertainty:0,tractorRearOverhangUncertainty:0,tractorBodyWidthUncertainty:0
 };
 const status=physics.vehicleGeometryCalibrationStatus(nominal);
 assert.equal(status.status,'preliminary');
 assert.equal(status.positiveBoundsPresent,false);
 assert.equal(status.measurementProvenanceVerified,false);
 assert.ok(status.missingBounds.some(item=>item.parameter==='wheelbase'));
 assert.ok(status.missingBounds.some(item=>item.parameter==='hitchOffset'));
 assert.ok(status.missingBounds.some(item=>item.parameter==='tractorFrontOverhang'));
 assert.ok(status.missingBounds.some(item=>item.parameter==='tractorRearOverhang'));
 assert.ok(status.missingBounds.some(item=>item.parameter==='tractorBodyWidth'));
 assert.ok(status.missingBounds.some(item=>item.uncertainty==='trailerAxleSpacingUncertainty'));
 const missingCommonSpacing={...nominal};delete missingCommonSpacing.trailerAxleSpacing;
 assert.ok(physics.vehicleGeometryCalibrationStatus(missingCommonSpacing).missingBounds.some(item=>item.parameter==='trailerAxleSpacing'));
 assert.ok(status.missingBounds.some(item=>item.parameter==='trailerTrack'));
});

test('vehicle geometry calibration status accepts explicit bounds and checks enabled mirror/individual axle geometry',()=>{
 const params={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,trailerKingpinToFront:1.5,trailerLength:13.6,trailerWidth:2.55,
  trailerAxleSpacing:1.3,trailerAxleSpacing12:0,trailerAxleSpacing23:0,
  tractorSteeringTrack:2.05,tractorFrontTrack:2.05,tractorRearTrack:2.05,trailerTrack:2.79,
  trailerAxle1Track:0,trailerAxle2Track:0,trailerAxle3Track:0,
  tireLength:0.72,tireWidth:0.44,cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25,
  tractorFrontOverhang:5,tractorRearOverhang:1.2,tractorBodyWidth:2.55,
  mirrorOutboard:0,mirrorLongitudinal:3.0,mirrorLength:0.8,vehicleFootprintUncertainty:0.03,
  wheelbaseUncertainty:0.02,hitchOffsetUncertainty:0.02,kingpinAxlesUncertainty:0.05,
  trailerLengthUncertainty:0.03,trailerKingpinToFrontUncertainty:0.02,trailerWidthUncertainty:0.02,trailerAxleSpacingUncertainty:0.01,
  tractorSteeringTrackUncertainty:0.01,tractorFrontTrackUncertainty:0.01,tractorRearTrackUncertainty:0.01,
  trailerTrackUncertainty:0.02,tireLengthUncertainty:0.01,tireWidthUncertainty:0.01,
  cabOffsetUncertainty:0.02,cabFrontUncertainty:0.02,cabRearUncertainty:0.02,cabWidthUncertainty:0.02,
  tractorFrontOverhangUncertainty:0.02,tractorRearOverhangUncertainty:0.02,tractorBodyWidthUncertainty:0.02
 };
 let status=physics.vehicleGeometryCalibrationStatus(params);
 assert.equal(status.status,'positive-bounds-present');
 assert.equal(status.missingBounds.length,0);
 for(const key of ['tractorFrontOverhang','tractorRearOverhang','tractorBodyWidth']){
  const missingNominal={...params};
  delete missingNominal[key];
  const uncalibrated=physics.vehicleGeometryCalibrationStatus(missingNominal);
  assert.equal(uncalibrated.status,'preliminary',key);
  assert.ok(uncalibrated.missingBounds.some(item=>item.parameter===key),key);
 }
 params.trailerAxleSpacing12=1.25;
 status=physics.vehicleGeometryCalibrationStatus(params);
 assert.ok(status.missingBounds.some(item=>item.uncertainty==='trailerAxleSpacing12Uncertainty'));
 params.trailerAxleSpacing12Uncertainty=0.01;
 params.trailerAxle1Track=2.8;
 status=physics.vehicleGeometryCalibrationStatus(params);
 assert.ok(status.missingBounds.some(item=>item.uncertainty==='trailerAxle1TrackUncertainty'));
 params.trailerAxle1TrackUncertainty=0.02;
 params.mirrorOutboard=0.4;
 status=physics.vehicleGeometryCalibrationStatus(params);
 assert.ok(status.missingBounds.some(item=>item.uncertainty==='mirrorOutboardUncertainty'));
 assert.ok(status.missingBounds.some(item=>item.uncertainty==='mirrorLongitudinalUncertainty'));
 assert.ok(status.missingBounds.some(item=>item.uncertainty==='mirrorLengthUncertainty'));
 params.mirrorOutboardUncertainty=0.02;
 params.mirrorLongitudinalUncertainty=0.02;
 params.mirrorLengthUncertainty=0.02;
 status=physics.vehicleGeometryCalibrationStatus(params);
 assert.equal(status.status,'positive-bounds-present');
});


test('tractor outer envelope uses configurable measured front, rear, and width dimensions',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  tractorFrontOverhang:6.1,tractorRearOverhang:1.6,tractorBodyWidth:2.8,
  footprintUncertainty:0.1
 };
 const polygon=physics.vehiclePolygons(q,p)[0];
 const xs=polygon.map(point=>point.x),ys=polygon.map(point=>point.y);
 near(Math.max(...xs),6.2);
 near(Math.min(...xs),-1.7);
 near(Math.max(...ys)-Math.min(...ys),3.0);
});

test('cab and mirror packaging validation follows the configured tractor outer envelope',()=>{
 const base={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  tractorFrontOverhang:6.0,tractorRearOverhang:1.5,tractorBodyWidth:2.8,
  cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25
 };
 const validMirror=physics.validateVehicleParams({
  ...base,mirrorOutboard:0.45,mirrorLongitudinal:5.5,mirrorLength:0.8
 });
 assert.deepEqual(validMirror,[]);
 const mirrorOutside=physics.validateVehicleParams({
  ...base,mirrorOutboard:0.45,mirrorLongitudinal:6.0,mirrorLength:0.8
 });
 assert.ok(mirrorOutside.some(message=>message.includes('mirrorLongitudinal/mirrorLength')));
 const cabOutside=physics.validateVehicleParams({...base,tractorFrontOverhang:3.0});
 assert.ok(cabOutside.some(message=>message.includes('Геометрия кабины выходит')));
 const cabTooWide=physics.validateVehicleParams({...base,tractorBodyWidth:2.2});
 assert.ok(cabTooWide.some(message=>message.includes('Геометрия кабины выходит')));
});

test('tractor outer envelope is used to classify wheels and auxiliary parts outside the chassis body',()=>{
 const q={x:0,y:0,psi:0,psiT:0};
 const p={
  wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
  trailerLength:13.6,trailerWidth:2.55,
  tractorFrontOverhang:4.0,tractorRearOverhang:1.0,tractorBodyWidth:2.3,
  cabOffset:2.1,cabFront:1.35,cabRear:0.9,cabWidth:2.25
 };
 const parts=physics.vehicleCollisionParts(q,p);
 const tractor=parts.find(part=>part.vehiclePart==='тягач');
 assert.ok(tractor.components.some(component=>component.name==='левое переднее колесо тягача'));
});
