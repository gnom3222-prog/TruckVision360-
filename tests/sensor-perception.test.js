'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const perception=require('../sensor-perception.js');
function frame(overrides={}){
 return {frameId:'frame-1',timestamp:Date.now(),coordinateFrame:'vehicle-local',objects:[{
  id:'obj-1',type:'pallet',pose:{x:10,y:2,heading:0},dimensions:{length:1.2,width:0.8},confidence:0.98,source:'fused'
 }],...overrides};
}
test('accepts a fresh fused perception frame',()=>{
 assert.deepEqual(perception.validateFrame(frame()),[]);
});
test('rejects stale and future frames',()=>{
 const now=Date.now();
 assert.ok(perception.validateFrame(frame({timestamp:now-2000}),now).some(e=>e.includes('устарел')));
 assert.ok(perception.validateFrame(frame({timestamp:now+1000}),now).some(e=>e.includes('будущем')));
});
test('rejects duplicate IDs, invalid confidence and unsupported frame',()=>{
 const f=frame({coordinateFrame:'world',objects:[
  {id:'same',type:'wall',pose:{x:1,y:0,heading:0},dimensions:{length:2,width:0.2},confidence:1,source:'lidar'},
  {id:'same',type:'wall',pose:{x:2,y:0,heading:0},dimensions:{length:2,width:0.2},confidence:2,source:'camera'}
 ]});
 const errors=perception.validateFrame(f);
 assert.ok(errors.some(e=>e.includes('vehicle-local')));
 assert.ok(errors.some(e=>e.includes('повторяется')));
 assert.ok(errors.some(e=>e.includes('confidence')));
});
test('converts vehicle-local oriented boxes into map polygons',()=>{
 const result=perception.toObstacles(frame(),{x:5,y:7,psi:Math.PI/2});
 assert.equal(result.length,1);
 assert.equal(result[0].kind,'perception-obstacle');
 assert.equal(result[0].polygon.length,4);
 const center=result[0].polygon.reduce((a,p)=>({x:a.x+p.x/4,y:a.y+p.y/4}),{x:0,y:0});
 assert.ok(Math.abs(center.x-3)<1e-9);
 assert.ok(Math.abs(center.y-17)<1e-9);
});
test('rejects an object outside the supported 60 m sensor range',()=>{
 const f=frame({objects:[{id:'far',type:'truck',pose:{x:61,y:0,heading:0},dimensions:{length:12,width:2.5},confidence:0.9,source:'lidar'}]});
 assert.ok(perception.validateFrame(f).some(e=>e.includes('дальше 60 м')));
});
