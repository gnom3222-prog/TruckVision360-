'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');

const base={
 wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
 trailerLength:13.6,trailerWidth:2.65,trailerAxleSpacing:1.3,
 tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
 trailerTrack:2.74,tireLength:0.72,tireWidth:0.32
};
const pose={x:0,y:0,psi:0,psiT:0};
const pairParams={
 ...base,
 tractorRearTireCenterSpacing:0.36,
 trailerAxle1TireCenterSpacing:0.36,
 trailerAxle2TireCenterSpacing:0.36,
 trailerAxle3TireCenterSpacing:0.36
};

test('unknown dual-tire spacing retains the conservative group-envelope fallback',()=>{
 const p={...base,tractorRearTireGroupWidth:0.68,trailerTireGroupWidth:0.68};
 const wheels=physics.wheelFootprints(pose,p);
 assert.equal(wheels.length,10);
 const rear=wheels.find(wheel=>wheel.name==='левое заднее колесо тягача');
 const trailer=wheels.find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа');
 assert.equal(rear.footprintModel,'configured-axle-side-group-envelope');
 assert.equal(trailer.footprintModel,'configured-axle-side-group-envelope');
 assert.equal(rear.footprintWidth,0.68);
 assert.equal(trailer.footprintWidth,0.68);
});

test('measured center spacing expands all four dual groups into separate tire footprints',()=>{
 assert.deepEqual(physics.validateVehicleParams(pairParams),[]);
 const wheels=physics.wheelFootprints(pose,pairParams);
 assert.equal(wheels.length,18);
 assert.equal(wheels.filter(wheel=>wheel.vehiclePart==='тягач').length,6);
 assert.equal(wheels.filter(wheel=>wheel.vehiclePart==='полуприцеп').length,12);
 const pair=wheels.filter(wheel=>
  wheel.vehiclePart==='полуприцеп'&&wheel.axleIndex===1&&wheel.side==='left'
 );
 assert.equal(pair.length,2);
 const outer=pair.find(wheel=>wheel.pairPosition==='outer');
 const inner=pair.find(wheel=>wheel.pairPosition==='inner');
 assert.ok(outer&&inner);
 assert.equal(outer.footprintModel,'explicit-dual-tire-pair');
 assert.equal(inner.footprintModel,'explicit-dual-tire-pair');
 assert.equal(outer.footprintWidth,base.tireWidth);
 assert.equal(inner.footprintWidth,base.tireWidth);
 assert.ok(Math.abs(Math.hypot(outer.center.x-inner.center.x,outer.center.y-inner.center.y)-0.36)<1e-10);
 assert.ok(outer.center.y>inner.center.y);
 assert.ok(Math.abs(outer.pairLateralOffset-0.18)<1e-10);
 assert.ok(Math.abs(inner.pairLateralOffset+0.18)<1e-10);
 assert.equal(new Set(wheels.map(wheel=>wheel.name)).size,wheels.length);
});

test('one axle can use explicit dual tires while the other axles retain group fallback',()=>{
 const p={...base,trailerAxle1TireCenterSpacing:0.36,trailerAxle1TireGroupWidth:0};
 const wheels=physics.wheelFootprints(pose,p);
 assert.equal(wheels.length,12);
 assert.equal(wheels.filter(wheel=>wheel.footprintModel==='explicit-dual-tire-pair').length,4);
 assert.equal(wheels.filter(wheel=>wheel.footprintModel==='single-tire-fallback').length,8);
});

test('center spacing cannot overlap tires or exceed the calibrated outer group limit',()=>{
 const overlap=physics.validateVehicleParams({...base,trailerAxle1TireCenterSpacing:0.30});
 assert.ok(overlap.some(message=>message.includes('расстояние между центрами покрышек')));
 const tooWide=physics.validateVehicleParams({...base,trailerAxle1TireCenterSpacing:1.25,tireWidth:0.32});
 assert.ok(tooWide.some(message=>message.includes('наружная ширина пары')));
});

test('pair-spacing uncertainty requires a nominal value and prevents overlap at its lower bound',()=>{
 const missing=physics.validateTireGroupWidthUncertainty({
  ...base,trailerAxle1TireCenterSpacing:0,
  trailerAxle1TireCenterSpacingUncertainty:0.02
 });
 assert.ok(missing.some(message=>message.includes('сначала задайте измеренное расстояние')));
 const overlap=physics.validateTireGroupWidthUncertainty({
  ...base,trailerAxle1TireCenterSpacing:0.36,
  trailerAxle1TireCenterSpacingUncertainty:0.05,
  tireWidthUncertainty:0.02
 });
 assert.ok(overlap.some(message=>message.includes('перекрытие покрышек')));
});

test('per-wheel clearance reports the individual outer tire as the limiting obstacle contact',()=>{
 const q={...pose,gear:1,steer:0,stepDistance:0};
 const firstAxle=physics.trailerAxlePoints(q,pairParams)[0];
 const outerEdgeY=pairParams.trailerTrack/2+0.36/2+pairParams.tireWidth/2;
 const obstacle={x:firstAxle.x,y:outerEdgeY+0.05,w:0.1,h:0.1,name:'outer dual tire post'};
 const result=planner.analyzeWheelTrajectoryClearance([q],[obstacle],pairParams);
 assert.equal(result.wheels.length,18);
 assert.equal(result.limitingWheel.name,'левое колесо 1-й оси полуприцепа — наружная покрышка');
 assert.equal(result.limitingWheel.limitingDetail.obstacle,'outer dual tire post');
});

test('planner reconstructs and checks the same variable tire set at control and intermediate poses',()=>{
 const steer=0.28,stepDistance=0.5;
 const start={
  x:0,y:0,psi:0.18,psiT:0.12,gear:1,steer:0,
  gearKnown:true,steerKnown:false,stepDistance:0
 };
 const end=physics.rk4(start,0.7,steer,stepDistance/0.7,pairParams);
 const obstacle={x:100,y:100,w:1,h:1,name:'distant reference'};
 const result=planner.analyzeWheelTrajectoryClearance([
  start,{...end,gear:1,steer,gearKnown:true,steerKnown:true,stepDistance}
 ],[obstacle],pairParams);
 assert.equal(result.wheels.length,18);
 assert.equal(result.controlStartWheelChecks,18);
 assert.equal(result.segmentsWithIntermediateKinematicSamples,1);
 const outer=result.wheels.find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа — наружная покрышка');
 const inner=result.wheels.find(wheel=>wheel.name==='левое колесо 1-й оси полуприцепа — внутренняя покрышка');
 assert.ok(outer&&inner);
 assert.ok(outer.limitingDetail.displacementRate>inner.limitingDetail.displacementRate,
  'outer tire center must include its additional lateral offset in the swept-rate bound');
});



test('collision components retain each exposed tire rectangle separately',()=>{
 const parts=physics.vehicleCollisionParts(pose,pairParams);
 const trailer=parts.find(part=>part.vehiclePart==='полуприцеп');
 assert.ok(trailer);
 const tireComponents=trailer.components.filter(component=>component.name.includes('покрышка'));
 assert.equal(tireComponents.length,12);
 assert.equal(new Set(tireComponents.map(component=>component.name)).size,12);
});

test('swept displacement bounds use the derived outer width of explicit tire pairs',()=>{
 const single=planner.sweptDisplacementRatesPerMeter(0.24,base);
 const paired=planner.sweptDisplacementRatesPerMeter(0.24,pairParams);
 assert.ok(paired.tractor>single.tractor);
 assert.ok(paired.trailer>single.trailer);
 assert.deepEqual(physics.trailerAxleTireGroupWidths(pairParams),[0.68,0.68,0.68]);
});
