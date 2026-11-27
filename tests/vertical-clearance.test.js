'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const physics=require('../vehicle-physics.js');
const planner=require('../articulated-planner.js');

const params={
 wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
 trailerLength:13.6,trailerWidth:2.65,trailerAxleSpacing:1.3,
 tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
 trailerTrack:2.74,tireLength:0.72,tireWidth:0.32
};
const flatRoadSurface={
 originX:0,originY:0,originZ:0,gradeX:0,gradeY:0,
 originZUncertainty:0.001,gradeXUncertainty:0.0001,gradeYUncertainty:0.0001,
 surfaceResidualUncertainty:0.001
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
const overheadAtCab={name:'Балка над кабиной',x:3.0,y:-0.1,w:0.4,h:0.2,zMin:4.5,zMax:4.8,zUncertainty:0.05};

test('vertical vehicle profiles are optional but strictly validated when supplied',()=>{
 assert.deepEqual(physics.validateVehicleVerticalProfiles(undefined),[]);
 assert.deepEqual(physics.validateVehicleVerticalProfiles({}),[]);
 assert.deepEqual(physics.validateVehicleVerticalProfiles(allProfiles),[]);
 assert.ok(physics.validateVehicleVerticalProfiles({'unknown-part':{zMin:0,zMax:1}}).length>0);
 assert.ok(physics.validateVehicleVerticalProfiles({'tractor-cab':{zMin:2,zMax:1,uncertaintyMargin:0.02}}).length>0);
 assert.ok(physics.validateVehicleParams({...params,vehicleVerticalProfiles:allProfiles}).length===0);
});


test('vehicle vertical profiles require an explicit positive uncertainty margin',()=>{
 const missing={'tractor-cab':{zMin:0.3,zMax:3.5}};
 assert.ok(physics.validateVehicleVerticalProfiles(missing).some(error=>error.includes('uncertaintyMargin')));
 assert.ok(physics.validateVehicleVerticalProfiles({'tractor-cab':{zMin:0.3,zMax:3.5,uncertaintyMargin:0}}).length>0);
 assert.ok(physics.validateVehicleVerticalProfiles({'tractor-cab':{zMin:0.3,zMax:3.5,uncertaintyMargin:0.0005}}).length>0);
 assert.deepEqual(physics.validateVehicleVerticalProfiles({'tractor-cab':{zMin:0.3,zMax:3.5,uncertaintyMargin:0.001}}),[]);
 const p={...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:missing};
 const cab=physics.vehicleCollisionParts(pose,p,0,true).find(part=>part.vehiclePart==='кабина');
 assert.ok(cab);
 assert.equal(cab.components.find(component=>component.name==='кабина').verticalRange,null);
 assert.throws(()=>planner.clearance(pose,[overheadAtCab],p),/uncertaintyMargin/);
});

test('auxiliary vertical envelopes require an explicit positive vertical uncertainty',()=>{
 const base={name:'Measured bracket',anchor:'tractor-front-axle',longitudinalOffset:0,lateralOffset:0,length:0.4,width:0.2,headingOffsetDeg:0,uncertaintyMargin:0.02,zMin:1,zMax:1.5};
 assert.ok(physics.validateVehicleParams({...params,auxiliaryEnvelopes:[base]}).some(error=>error.includes('verticalUncertaintyMargin')));
 assert.ok(physics.validateVehicleParams({...params,auxiliaryEnvelopes:[{...base,verticalUncertaintyMargin:0}]}).some(error=>error.includes('verticalUncertaintyMargin')));
 assert.deepEqual(physics.validateVehicleParams({...params,auxiliaryEnvelopes:[{...base,verticalUncertaintyMargin:0.001}]}),[]);
});

test('vertical obstacle validation requires a complete height interval and explicit positive uncertainty',()=>{
 const base={x:0,y:0,w:1,h:1,zMin:2,zMax:3};
 assert.ok(planner.validateObstacles([base]).some(e=>e.includes('zUncertainty')));
 assert.ok(planner.validateObstacles([{...base,zUncertainty:0}]).some(e=>e.includes('zUncertainty')));
 assert.ok(planner.validateObstacles([{...base,zUncertainty:0.0005}]).some(e=>e.includes('zUncertainty')));
 assert.equal(planner.validateObstacles([{...base,zUncertainty:0.01}]).length,0);
 assert.ok(planner.validateObstacles([{...base,zUncertainty:3}]).some(e=>e.includes('zUncertainty')));
 assert.ok(planner.validateObstacles([{x:0,y:0,w:1,h:1,zMin:2}]).some(e=>e.includes('zMin и zMax')));
 assert.ok(planner.validateObstacles([{x:0,y:0,w:1,h:1,zMin:3,zMax:2,zUncertainty:0.01}]).some(e=>e.includes('zMin/zMax')));
});

test('missing vehicle height profile keeps only the horizontal lower bound and reports it unverified',()=>{
 const result=planner.clearance(pose,[overheadAtCab],{...params,roadSurface:flatRoadSurface});
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vehicle-height-unverified');
 assert.ok(result.verticalUnverifiedCount>0);
});


test('vertical profiles do not enable vertical separation while road surface remains uncalibrated',()=>{
 const result=planner.clearance(pose,[overheadAtCab],{...params,vehicleVerticalProfiles:allProfiles});
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'road-surface-unverified');
 assert.equal(result.roadSurfaceStatus,'road-surface-unverified-horizontal-only');
 assert.equal(result.verticalEvaluatedCount,0);
 assert.ok(result.verticalUnverifiedCount>0);
});
test('measured vertical profiles allow a separated overhead obstacle to use 3D prism distance',()=>{
 const result=planner.clearance(pose,[overheadAtCab],{...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:allProfiles});
 assert.ok(result.distance>=0.89,JSON.stringify(result));
 assert.equal(result.verticalStatus,'vertical-separated');
 assert.equal(result.verticalUnverifiedCount,0);
 assert.ok(result.verticalEvaluatedCount>0);
});

test('overlapping vertical intervals remain a zero-clearance contact',()=>{
 const obstacle={...overheadAtCab,zMin:3.3,zMax:3.6,zUncertainty:0.01};
 const result=planner.clearance(pose,[obstacle],{...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:allProfiles});
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vertical-overlap');
});

test('auxiliary axle hardware can carry its own measured vertical interval',()=>{
 const p={...params,auxiliaryEnvelopes:[{
  name:'Корпус ступицы',anchor:'trailer-axle-1',longitudinalOffset:0,lateralOffset:1.7,
  length:0.6,width:0.5,headingOffsetDeg:0,uncertaintyMargin:0.04,zMin:0.35,zMax:0.9,verticalUncertaintyMargin:0.04
 }]};
 assert.deepEqual(physics.validateVehicleParams(p),[]);
 const envelope=physics.auxiliaryEnvelopePolygons(pose,p)[0];
 assert.ok(Math.abs(envelope.verticalRange.min-0.31)<1e-12);
 assert.ok(Math.abs(envelope.verticalRange.max-0.94)<1e-12);
 const partial={...p.auxiliaryEnvelopes[0],zMin:0.5};delete partial.zMax;
 assert.ok(physics.validateVehicleParams({...p,auxiliaryEnvelopes:[partial]}).length>0);
});

test('per-wheel swept diagnostics apply measured tire height intervals',()=>{
 const p={...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:allProfiles};
 const wheel=physics.wheelFootprints(pose,p,0)[0];
 const obstacle={name:'Балка над передним колесом',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:2.5,zMax:2.8,zUncertainty:0.02};
 const result=planner.analyzeWheelTrajectoryClearance([pose],[obstacle],p);
 const record=result.wheels.find(item=>item.name===wheel.name);
 assert.ok(record);
 assert.ok(record.minimumSampledEnvelopeClearance>1.3);
 assert.equal(record.minimumSampledEnvelopeDetail.verticalStatus,'vertical-separated');
 assert.equal(result.verticalUnverifiedChecks,0);
});

test('per-wheel vertical clearance remains conservative when tire height is unknown',()=>{
 const wheel=physics.wheelFootprints(pose,params,0)[0];
 const obstacle={name:'Неизвестная балка',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:2.5,zMax:2.8,zUncertainty:0.02};
 const result=planner.analyzeWheelTrajectoryClearance([pose],[obstacle],{...params,roadSurface:flatRoadSurface});
 const record=result.wheels.find(item=>item.name===wheel.name);
 assert.equal(record.minimumSampledEnvelopeClearance,0);
 assert.equal(record.minimumSampledEnvelopeDetail.verticalStatus,'vehicle-height-unverified');
 assert.ok(result.verticalUnverifiedChecks>0);
});

test('route clearance reports unverified vertical component profiles',()=>{
 const result=planner.analyzeTrajectoryClearance([pose],[overheadAtCab],{...params,roadSurface:flatRoadSurface});
 assert.ok(result.verticalUnverifiedChecks>0);
 assert.equal(result.verticalEvaluatedChecks,0);
});

test('measured wheel height is checked even when the wheel footprint lies inside the 2D body outline',()=>{
 const profiles={...allProfiles,
  'tractor-body':{zMin:0.5,zMax:3.0,uncertaintyMargin:0.02},
  'tractor-cab':{zMin:0.5,zMax:3.5,uncertaintyMargin:0.02}
 };
 const p={...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:profiles};
 const wheel=physics.wheelFootprints(pose,p,0)[0];
 const obstacle={name:'Низкая балка у колеса',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:0.1,zMax:0.2,zUncertainty:0.01};
 const parts=physics.vehicleCollisionParts(pose,p);
 const tractor=parts.find(part=>part.vehiclePart==='тягач');
 assert.ok(tractor.components.some(component=>component.name===wheel.name));
 const result=planner.clearance(pose,[obstacle],p);
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vertical-overlap');
});

test('measured auxiliary height remains a separate component even inside the body footprint',()=>{
 const p={...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:{
  ...allProfiles,'trailer-body':{zMin:1.0,zMax:3.0,uncertaintyMargin:0.02}
 },auxiliaryEnvelopes:[{
  name:'Верхний кронштейн внутри контура',anchor:'trailer-axle-1',
  longitudinalOffset:0,lateralOffset:0,length:0.4,width:0.4,headingOffsetDeg:0,uncertaintyMargin:0.02,
  zMin:3.4,zMax:3.8,verticalUncertaintyMargin:0.02
 }]};
 const envelope=physics.auxiliaryEnvelopePolygons(pose,p)[0];
 const center=envelope.polygon.reduce((sum,point)=>({x:sum.x+point.x/4,y:sum.y+point.y/4}),{x:0,y:0});
 const parts=physics.vehicleCollisionParts(pose,p,0,true);
 const trailer=parts.find(part=>part.vehiclePart==='полуприцеп');
 assert.ok(trailer.components.some(component=>component.name==='Верхний кронштейн внутри контура'));
 const obstacle={name:'Низкая балка',x:center.x,y:center.y,w:0.1,h:0.1,zMin:3.5,zMax:3.6,zUncertainty:0.02};
 const result=planner.clearance(pose,[obstacle],p);
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vertical-overlap');
});

test('unknown internal wheel height cannot be hidden by a measured body height profile',()=>{
 const p={...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:{
  'tractor-body':{zMin:0.5,zMax:3.0,uncertaintyMargin:0.02},
  'tractor-cab':{zMin:0.5,zMax:3.5,uncertaintyMargin:0.02}
 }};
 const wheel=physics.wheelFootprints(pose,p,0)[0];
 const obstacle={name:'Низкая балка у колеса без профиля',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:0.1,zMax:0.2,zUncertainty:0.01};
 const result=planner.clearance(pose,[obstacle],p);
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vehicle-height-unverified');
 assert.ok(result.verticalUnverifiedCount>0);
});

test('route assessment preserves vertical intervals while retaining the explicit obstacle-check gate',()=>{
 const app=fs.readFileSync(path.resolve(__dirname,'../truckvision.js'),'utf8');
 assert.match(app,/const routeObstacles=showObstacles\?sceneObstacles\(\):\[\]/);
 assert.match(app,/Автопроезд заблокирован: сначала включите отображение и проверку препятствий/);
 assert.match(app,/zMin:obstacle\.zMin,zMax:obstacle\.zMax,zUncertainty:obstacle\.zUncertainty/);
 assert.match(app,/road-surface-unverified-horizontal-only/);
 assert.match(app,/вертикальное разделение отключено/);
 assert.doesNotMatch(app,/flat-reference-assumed/);
});


test('measured mirror height is propagated as its own cab collision component',()=>{
 const p={...params,mirrorOutboard:0.45,vehicleVerticalProfiles:{
  ...allProfiles,mirror:{zMin:1.9,zMax:3.8,uncertaintyMargin:0.1}
 }};
 const parts=physics.vehicleCollisionParts(pose,p,0,true);
 const cab=parts.find(part=>part.vehiclePart==='кабина');
 const mirror=cab.components.find(component=>component.name==='левое зеркало');
 assert.ok(mirror);
 assert.ok(Math.abs(mirror.verticalRange.min-1.8)<1e-12);
 assert.ok(Math.abs(mirror.verticalRange.max-3.9)<1e-12);
});

test('unknown mirror height is not inferred from the cab height profile',()=>{
 const p={...params,roadSurface:flatRoadSurface,mirrorOutboard:0.45,vehicleVerticalProfiles:{
  'tractor-body':{zMin:0.2,zMax:3.0,uncertaintyMargin:0.02},
  'tractor-cab':{zMin:0.3,zMax:3.5,uncertaintyMargin:0.02}
 }};
 const parts=physics.vehicleCollisionParts(pose,p,0,true);
 const cab=parts.find(part=>part.vehiclePart==='кабина');
 const mirror=cab.components.find(component=>component.name==='левое зеркало');
 assert.ok(mirror);
 assert.equal(mirror.verticalRange,null);
 const center=mirror.polygon.reduce((sum,point)=>({x:sum.x+point.x/4,y:sum.y+point.y/4}),{x:0,y:0});
 const obstacle={name:'Балка у зеркала без профиля',x:center.x,y:center.y,w:0.1,h:0.1,zMin:4.0,zMax:4.2,zUncertainty:0.02};
 const result=planner.clearance(pose,[obstacle],p);
 assert.equal(result.distance,0);
 assert.equal(result.verticalStatus,'vehicle-height-unverified');
 assert.ok(result.verticalUnverifiedCount>0);
});


test('individual trailer axle vertical profiles override the shared trailer-wheel fallback',()=>{
 const p={...params,vehicleVerticalProfiles:{
  'trailer-wheel':{zMin:0,zMax:1.1,uncertaintyMargin:0.02},
  'trailer-axle-1-wheel':{zMin:1.4,zMax:1.9,uncertaintyMargin:0.05},
  'trailer-axle-3-wheel':{zMin:0.2,zMax:1.2,uncertaintyMargin:0.01}
 }};
 assert.deepEqual(physics.validateVehicleParams(p),[]);
 const wheels=physics.wheelFootprints(pose,p,0).filter(w=>w.vehiclePart==='полуприцеп');
 const axle1=wheels.filter(w=>w.axleIndex===1);
 const axle2=wheels.filter(w=>w.axleIndex===2);
 const axle3=wheels.filter(w=>w.axleIndex===3);
 assert.ok(axle1.every(w=>Math.abs(w.verticalRange.min-1.35)<1e-12&&Math.abs(w.verticalRange.max-1.95)<1e-12));
 assert.ok(axle2.every(w=>Math.abs(w.verticalRange.min+0.02)<1e-12&&Math.abs(w.verticalRange.max-1.12)<1e-12));
 assert.ok(axle3.every(w=>Math.abs(w.verticalRange.min-0.19)<1e-12&&Math.abs(w.verticalRange.max-1.21)<1e-12));
});

test('tractor front and rear wheel height profiles can be calibrated independently',()=>{
 const p={...params,vehicleVerticalProfiles:{
  'tractor-wheel':{zMin:0,zMax:1.1,uncertaintyMargin:0.02},
  'tractor-front-wheel':{zMin:0.1,zMax:0.9,uncertaintyMargin:0.02},
  'tractor-rear-wheel':{zMin:0.3,zMax:1.2,uncertaintyMargin:0.04}
 }};
 const wheels=physics.wheelFootprints(pose,p,0).filter(w=>w.vehiclePart==='тягач');
 const front=wheels.filter(w=>w.axleIndex===1);
 const rear=wheels.filter(w=>w.axleIndex===2);
 assert.ok(front.every(w=>Math.abs(w.verticalRange.min-0.08)<1e-12&&Math.abs(w.verticalRange.max-0.92)<1e-12));
 assert.ok(rear.every(w=>Math.abs(w.verticalRange.min-0.26)<1e-12&&Math.abs(w.verticalRange.max-1.24)<1e-12));
});

test('per-wheel vertical clearance uses the selected axle-specific profile',()=>{
 const p={...params,roadSurface:flatRoadSurface,vehicleVerticalProfiles:{
  'trailer-wheel':{zMin:0,zMax:1.1,uncertaintyMargin:0.02},
  'trailer-axle-1-wheel':{zMin:1.4,zMax:1.9,uncertaintyMargin:0.02}
 }};
 const wheel=physics.wheelFootprints(pose,p,0).find(w=>w.vehiclePart==='полуприцеп'&&w.axleIndex===1&&w.side==='left');
 const obstacle={name:'Балка на высоте первой оси',x:wheel.center.x,y:wheel.center.y,w:0.1,h:0.1,zMin:1.5,zMax:1.6,zUncertainty:0.01};
 const result=planner.analyzeWheelTrajectoryClearance([pose],[obstacle],p);
 const record=result.wheels.find(item=>item.name===wheel.name);
 assert.ok(record);
 assert.equal(record.minimumSampledEnvelopeClearance,0);
 assert.equal(record.minimumSampledEnvelopeDetail.verticalStatus,'vertical-overlap');
});
