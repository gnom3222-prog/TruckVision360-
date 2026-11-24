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
 assert.throws(()=>camera.map({x:1,y:0},[0,0,0,0,0,0,-1,0]));
});


const perspectiveReference=[
 {x:100,y:100},{x:500,y:130},
 {x:450,y:350},{x:80,y:320}
];
const measuredOpening=[
 {x:160,y:150},{x:430,y:165},
 {x:415,y:290},{x:145,y:280}
];

test('camera measurement reports finite point-selection and reference-size uncertainty',()=>{
 const result=camera.estimateUncertainty({
  referencePoints:perspectiveReference,
  referenceWidth:0.297,
  referenceHeight:0.210,
  measuredPoints:measuredOpening,
  sigmaPixels:2,
  sigmaReferenceWidth:0.001,
  sigmaReferenceHeight:0.001
 });
 assert.ok(Number.isFinite(result.sigmaWidth)&&result.sigmaWidth>0);
 assert.ok(Number.isFinite(result.sigmaHeight)&&result.sigmaHeight>0);
 assert.ok(result.sigmaWidth<result.width);
 assert.ok(result.sigmaHeight<result.height);
 assert.equal(result.uncertaintyModel,'linearized-independent-coordinate-errors');
});

test('camera uncertainty scales linearly when all assumed input uncertainties double',()=>{
 const base={
  referencePoints:perspectiveReference,
  referenceWidth:0.297,
  referenceHeight:0.210,
  measuredPoints:measuredOpening
 };
 const one=camera.estimateUncertainty({...base,sigmaPixels:1,sigmaReferenceWidth:0.001,sigmaReferenceHeight:0.001});
 const two=camera.estimateUncertainty({...base,sigmaPixels:2,sigmaReferenceWidth:0.002,sigmaReferenceHeight:0.002});
 near(two.sigmaWidth,one.sigmaWidth*2,1e-7);
 near(two.sigmaHeight,one.sigmaHeight*2,1e-7);
});

test('camera uncertainty rejects negative or non-finite assumptions',()=>{
 assert.throws(()=>camera.estimateUncertainty({
  referencePoints:perspectiveReference,referenceWidth:0.297,referenceHeight:0.210,
  measuredPoints:measuredOpening,sigmaPixels:-1
 }));
 assert.throws(()=>camera.estimateUncertainty({
  referencePoints:perspectiveReference,referenceWidth:0.297,referenceHeight:0.210,
  measuredPoints:measuredOpening,sigmaReferenceWidth:NaN
 }));
});

const horizonCrossingOpening=[
 {x:100,y:2500},{x:500,y:2500},
 {x:500,y:3500},{x:100,y:3500}
];

test('camera measurement rejects a convex opening that crosses the projective horizon',()=>{
 assert.equal(camera.validQuad(horizonCrossingOpening,0,0),true);
 assert.throws(
  ()=>camera.measureQuad(perspectiveReference,0.297,0.210,horizonCrossingOpening),
  /пересекает линию вырождения/
 );
});

test('camera uncertainty estimation also fails closed for a projective-horizon crossing',()=>{
 assert.throws(
  ()=>camera.estimateUncertainty({
   referencePoints:perspectiveReference,
   referenceWidth:0.297,
   referenceHeight:0.210,
   measuredPoints:horizonCrossingOpening,
   sigmaPixels:2
  }),
  /пересекает линию вырождения/
 );
});

test('normalized homography is invariant to high-resolution image coordinate scaling',()=>{
 const scale=8;
 const scaledReference=perspectiveReference.map(point=>({x:point.x*scale,y:point.y*scale}));
 const scaledOpening=measuredOpening.map(point=>({x:point.x*scale,y:point.y*scale}));
 const original=camera.measureQuad(perspectiveReference,0.297,0.210,measuredOpening);
 const scaled=camera.measureQuad(scaledReference,0.297,0.210,scaledOpening);
 near(scaled.width,original.width,1e-8);
 near(scaled.height,original.height,1e-8);
 assert.ok(Number.isFinite(scaled.conditionEstimate));
 assert.ok(scaled.conditionEstimate>0);
});

test('camera calibration rejects a nearly degenerate reference by condition estimate',()=>{
 const narrowReference=[
  {x:0,y:0},{x:1000,y:0},
  {x:1000,y:0.0001},{x:0,y:0.0001}
 ];
 assert.equal(camera.validQuad(narrowReference,0,0),true);
 assert.throws(
  ()=>camera.homography(narrowReference,0.297,0.210),
  /плохо обусловлена/
 );
});

test('camera measurement exposes a conditioning warning for a very thin calibration quadrilateral',()=>{
 const thinReference=[
  {x:0,y:0},{x:1000,y:0},
  {x:1000,y:0.02},{x:0,y:0.02}
 ];
 const opening=[
  {x:100,y:0.004},{x:900,y:0.004},
  {x:900,y:0.016},{x:100,y:0.016}
 ];
 const result=camera.measureQuad(thinReference,0.297,0.210,opening);
 assert.ok(result.conditionEstimate>camera.CONDITION_WARNING_ESTIMATE);
 assert.equal(result.conditioningWarning,true);
});
