'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const physics=require('../vehicle-physics.js');

const params={
 wheelbase:3.8,hitchOffset:0.575,kingpinAxles:8,
 trailerLength:13.6,trailerWidth:2.65,trailerAxleSpacing:1.3,
 tractorFrontTrack:2.05,tractorSteeringTrack:2.05,tractorRearTrack:2.05,
 trailerTrack:2.74,tireLength:0.72,tireWidth:0.32
};
const pose={x:0,y:0,psi:0,psiT:0};

test('empty auxiliary envelope list preserves existing vehicle collision geometry',()=>{
 const parts=physics.vehicleCollisionParts(pose,params);
 assert.equal(parts.length,3);
 assert.ok(parts.every(part=>part.components.every(component=>component.componentType!=='measured-auxiliary-envelope')));
 assert.deepEqual(physics.validateVehicleParams(params),[]);
});

test('measured trailer axle hardware envelope is positioned in the axle-local frame',()=>{
 const p={...params,auxiliaryEnvelopes:[{
  name:'Кронштейн 1-й оси',anchor:'trailer-axle-1',
  longitudinalOffset:0.2,lateralOffset:0.4,
  length:0.8,width:0.5,headingOffsetDeg:0,uncertaintyMargin:0.05
 }]};
 assert.deepEqual(physics.validateVehicleParams(p),[]);
 const envelope=physics.auxiliaryEnvelopePolygons(pose,p)[0];
 const axle=physics.trailerAxlePoints(pose,p)[0];
 assert.equal(envelope.vehiclePart,'полуприцеп');
 assert.equal(envelope.componentType,'measured-auxiliary-envelope');
 assert.ok(Math.abs(envelope.polygon[0].x-(axle.x+0.2+0.45))<1e-10);
 assert.ok(Math.abs(envelope.polygon[0].y-(axle.y+0.4+0.30))<1e-10);
});

test('measured external hub or suspension envelope joins trailer collision components',()=>{
 const p={...params,auxiliaryEnvelopes:[{
  name:'Наружный корпус ступицы',anchor:'trailer-axle-1',
  longitudinalOffset:0,lateralOffset:1.0,
  length:0.9,width:0.5,uncertaintyMargin:0.05
 }]};
 const parts=physics.vehicleCollisionParts(pose,p);
 const trailer=parts.find(part=>part.vehiclePart==='полуприцеп');
 assert.ok(trailer.components.some(component=>
  component.name==='Наружный корпус ступицы'&&component.componentType==='measured-auxiliary-envelope'
 ));
});

test('measured tractor front and rear axle hardware envelopes map to tractor collision part',()=>{
 const p={...params,auxiliaryEnvelopes:[
  {name:'Передний поворотный кулак',anchor:'tractor-front-axle',length:0.8,width:3.0},
  {name:'Задний корпус моста',anchor:'tractor-rear-axle',length:0.9,width:3.2}
 ]};
 assert.deepEqual(physics.validateVehicleParams(p),[]);
 const envelopes=physics.auxiliaryEnvelopePolygons(pose,p);
 assert.equal(envelopes.length,2);
 assert.ok(envelopes.every(envelope=>envelope.vehiclePart==='тягач'));
});

test('auxiliary envelope validation rejects unknown anchors, invalid dimensions and duplicate names',()=>{
 const invalid=[
  {name:'same',anchor:'trailer-axle-4',length:0,width:0.5},
  {name:'same',anchor:'trailer-axle-1',length:0.8,width:0.5}
 ];
 const errors=physics.validateAuxiliaryEnvelopes(invalid);
 assert.ok(errors.some(message=>message.includes('неизвестная опорная ось')));
 assert.ok(errors.some(message=>message.includes('название должно быть уникальным')));
 assert.ok(errors.some(message=>message.includes('длина должна быть измеренным числом')));
});

test('auxiliary envelope uncertainty margin expands all sides of the measured rectangle',()=>{
 const nominal=physics.auxiliaryEnvelopePolygons(pose,{...params,auxiliaryEnvelopes:[
  {name:'Балка',anchor:'trailer-axle-2',length:1,width:0.6}
 ]})[0].polygon;
 const expanded=physics.auxiliaryEnvelopePolygons(pose,{...params,auxiliaryEnvelopes:[
  {name:'Балка',anchor:'trailer-axle-2',length:1,width:0.6,uncertaintyMargin:0.1}
 ]})[0].polygon;
 const nominalBounds={
  minX:Math.min(...nominal.map(point=>point.x)),maxX:Math.max(...nominal.map(point=>point.x)),
  minY:Math.min(...nominal.map(point=>point.y)),maxY:Math.max(...nominal.map(point=>point.y))
 };
 const expandedBounds={
  minX:Math.min(...expanded.map(point=>point.x)),maxX:Math.max(...expanded.map(point=>point.x)),
  minY:Math.min(...expanded.map(point=>point.y)),maxY:Math.max(...expanded.map(point=>point.y))
 };
 assert.ok(expandedBounds.minX<nominalBounds.minX);
 assert.ok(expandedBounds.maxX>nominalBounds.maxX);
 assert.ok(expandedBounds.minY<nominalBounds.minY);
 assert.ok(expandedBounds.maxY>nominalBounds.maxY);
});
