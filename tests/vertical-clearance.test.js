'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');

const params={
 wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
 trailerLength:13.6,trailerWidth:2.65,trailerAxleSpacing:1.3,
 tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
 trailerTrack:2.74,tireLength:0.72,tireWidth:0.32
};
const pose={x:0,y:0,psi:0,psiT:0};
const allProfiles={
 'tractor-body':{zMin:0.2,zMax:3.0,uncertaintyMargin:0.05},
 'tractor-cab':{zMin:0.3,zMax:3.5,uncertaintyMargin:0.05},
 'trailer-body':{zMin:1.0,zMax:4.0,uncertaintyMargin:0.05},
 mirror:{zMin:2.0,zMax:4.0,uncertaintyMargin:0.05},
 'tractor-wheel':{zMin:0,zMax:1.1,uncertaintyMargin:0.02},
 'trailer-wheel':{zMin:0,zMax:1.1,uncertaintyMargin:0.02}
};
const overheadAtCab={name:'Балка над кабиной',x:1.8,y:-0.1,w:0.4,h:0.2,zMin:4.5,zMax:4.8,zUncertainty:0.05};

test('vertical vehicle profiles are optional but strictly validated when supplied',()=>{
 assert.deepEqual(physics.validateVehicleVerticalProfiles(undefined),[]);
 assert.deepEqual(physics.validateVehicleVerticalProfiles({}),[]);
 assert.deepEqual(physics.validateVehicleVerticalProfiles(allProfiles),[]);
 assert.ok(physics.validateVehicleVerticalProfiles({'unknown-part':{zMin:0,zMax:1}}).length>0);
 assert.ok(physics.validateVehicleVerticalProfiles({'tractor-cab':{zMin:2,zMax:1}}).length>0);
 assert.ok(physics.validateVehicleParams({...params,vehicleVerticalProfiles:allProfiles}).length===0);
});

test('vertical obstacle validation requires a complete finite height interval',()=>{
 assert.equal(planner.validateObstacles([{x:0,y:0,w:1,h:1,zMin:2,zMax:3}]).length,0);
 assert.ok(planner.validateObstacles([{x:0,y:0,w:1,h:1,zMin:2}]).some(e=>e.includes('zMin и zMax')));
 assert.ok(planner.validateObstacles([{x:0,y:0,w:1,h:1,zMin:3,zMax:2}]).some(e=>e.includes('zMin/zMax')));
 assert.ok(planner.validateObstacles([{x:0,y:0,w:1,h:1,zMin:2,zMax:3,zUncertainty:3}]).some(e=>e.includes('zUncertainty')));
});

test('missing vehicle height profile keeps only the horizontal lower bound and reports it unverified',()=>{
 const result=planner.clearance(pose,[overheadAtCab],params);
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vehicle-height-unverified');
 assert.ok(result.verticalUnverifiedCount>0);
});

test('measured vertical profiles allow a separated overhead obstacle to use 3D prism distance',()=>{
 const result=planner.clearance(pose,[overheadAtCab],{...params,vehicleVerticalProfiles:allProfiles});
 assert.ok(result.distance>=0.9);
 assert.equal(result.verticalStatus,'vertical-separated');
 assert.equal(result.verticalUnverifiedCount,0);
 assert.ok(result.verticalEvaluatedCount>0);
});

test('overlapping vertical intervals remain a zero-clearance contact',()=>{
 const obstacle={...overheadAtCab,zMin:3.3,zMax:3.6,zUncertainty:0};
 const result=planner.clearance(pose,[obstacle],{...params,vehicleVerticalProfiles:allProfiles});
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vertical-overlap');
});

test('auxiliary axle hardware can carry its own measured vertical interval',()=>{
 const p={...params,auxiliaryEnvelopes:[{
  name:'Корпус ступицы',anchor:'trailer-axle-1',longitudinalOffset:0,lateralOffset:1.7,
  length:0.6,width:0.5,zMin:0.35,zMax:0.9,verticalUncertaintyMargin:0.04
 }]};
 assert.deepEqual(physics.validateVehicleParams(p),[]);
 const envelope=physics.auxiliaryEnvelopePolygons(pose,p)[0];
 assert.deepEqual(envelope.verticalRange,{min:0.31,max:0.94});
 assert.ok(physics.validateVehicleParams({...p,auxiliaryEnvelopes:[{...p.auxiliaryEnvelopes[0],zMin:0.5}]}).length>0);
});

test('per-wheel swept diagnostics apply measured tire height intervals',()=>{
 const p={...params,vehicleVerticalProfiles:allProfiles};
 const wheel=physics.wheelFootprints(pose,p,0)[0];
 const obstacle={name:'Балка над передним колесом',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:2.5,zMax:2.8};
 const result=planner.analyzeWheelTrajectoryClearance([pose],[obstacle],p);
 const record=result.wheels.find(item=>item.name===wheel.name);
 assert.ok(record);
 assert.ok(record.minimumSampledEnvelopeClearance>1.3);
 assert.equal(record.minimumSampledEnvelopeDetail.verticalStatus,'vertical-separated');
 assert.equal(result.verticalUnverifiedChecks,0);
});

test('per-wheel vertical clearance remains conservative when tire height is unknown',()=>{
 const wheel=physics.wheelFootprints(pose,params,0)[0];
 const obstacle={name:'Неизвестная балка',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:2.5,zMax:2.8};
 const result=planner.analyzeWheelTrajectoryClearance([pose],[obstacle],params);
 const record=result.wheels.find(item=>item.name===wheel.name);
 assert.equal(record.minimumSampledEnvelopeClearance,0);
 assert.equal(record.minimumSampledEnvelopeDetail.verticalStatus,'vehicle-height-unverified');
 assert.ok(result.verticalUnverifiedChecks>0);
});

test('route clearance reports unverified vertical component profiles',()=>{
 const result=planner.analyzeTrajectoryClearance([pose],[overheadAtCab],params);
 assert.ok(result.verticalUnverifiedChecks>0);
 assert.equal(result.verticalEvaluatedChecks,0);
});
