'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const perception=require('../sensor-perception.js');

function frame(verticalRange){
 return {
  frameId:'vertical-test',timestamp:Date.now(),coordinateFrame:'vehicle-local',
  vehiclePose:{x:10,y:20,heading:0},
  objects:[{
   id:'beam-1',type:'overhead-beam',pose:{x:3,y:0,heading:0},
   dimensions:{length:2,width:0.2},confidence:0.95,source:'lidar',
   ...(verticalRange===undefined?{}:{verticalRange})
  }]
 };
}

test('perception frame preserves a measured vertical interval and uncertainty',()=>{
 const result=perception.toObstacles(frame({zMin:3.4,zMax:3.8,uncertainty:0.08}));
 assert.equal(result.length,1);
 assert.equal(result[0].zMin,3.4);
 assert.equal(result[0].zMax,3.8);
 assert.equal(result[0].zUncertainty,0.08);
});

test('perception frame remains backward compatible without vertical data',()=>{
 const result=perception.toObstacles(frame());
 assert.equal(result[0].zMin,undefined);
 assert.equal(result[0].zMax,undefined);
});

test('perception rejects malformed or inverted vertical intervals',()=>{
 const invalid=perception.validateFrame(frame({zMin:4,zMax:3}),Date.now());
 assert.ok(invalid.some(message=>message.includes('verticalRange')));
 const invalidUncertainty=perception.validateFrame(frame({zMin:2,zMax:3,uncertainty:4}),Date.now());
 assert.ok(invalidUncertainty.some(message=>message.includes('uncertainty')));
});
