'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const camera=require('../camera-geometry.js');

const reference=[
 {x:100,y:100},{x:500,y:100},
 {x:500,y:300},{x:100,y:300}
];
const near=(actual,expected,tolerance=1e-8)=>
 assert.ok(Math.abs(actual-expected)<=tolerance, `${actual} is not within ${tolerance} of ${expected}`);

test('camera calibration accepts a convex four-corner reference in clockwise order',()=>{
 assert.equal(camera.validQuad(reference),true);
});

test('camera calibration rejects crossed, degenerate and non-finite corner sets',()=>{
 assert.equal(camera.validQuad([{x:0,y:0},{x:100,y:100},{x:0,y:100},{x:100,y:0}],0,0),false);
 assert.equal(camera.validQuad([{x:0,y:0},{x:100,y:0},{x:200,y:0},{x:0,y:100}],0,0),false);
 assert.equal(camera.validQuad([{x:0,y:0},{x:100,y:0},{x:100,y:100},{x:NaN,y:100}],0,0),false);
});

test('perspective transform maps all four reference corners to the entered real dimensions',()=>{
 const H=camera.homography(reference,2.97,2.10);
 const expected=[{x:0,y:0},{x:2.97,y:0},{x:2.97,y:2.10},{x:0,y:2.10}];
 reference.forEach((point,i)=>{
  const mapped=camera.map(point,H);
  near(mapped.x,expected[i].x);
  near(mapped.y,expected[i].y);
 });
});

test('perspective transform preserves the center of a rectangular reference',()=>{
 const H=camera.homography(reference,2.97,2.10);
 const center=camera.map({x:300,y:200},H);
 near(center.x,1.485);
 near(center.y,1.05);
});

test('camera calibration rejects singular point arrangements and invalid dimensions',()=>{
 assert.throws(()=>camera.homography([{x:0,y:0},{x:10,y:0},{x:20,y:0},{x:30,y:0}],2,1));
 assert.throws(()=>camera.homography(reference,0,1));
});

test('perspective mapping rejects malformed input and non-finite results',()=>{
 assert.throws(()=>camera.map({x:0,y:0},[1,2]));
 assert.throws(()=>camera.map({x:0,y:0},[0,0,0,0,0,0,0,0]));
});
