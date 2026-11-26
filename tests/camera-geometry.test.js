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
const stableUncertainty={
 sigmaWidth:0.01,sigmaHeight:0.02,
 nonlinearSensitivityAvailable:true,nonlinearWarning:false,
 nonlinearSensitivityModel:'axiswise-one-sigma-finite-perturbations',
 nonlinearMismatchWidth:0,maxAxisAsymmetryRatio:0
};

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


test('camera-measured gate registers its clear opening on a manually anchored map axis',()=>{
 const points=[
  {x:0,y:0},{x:4,y:0},{x:4,y:2},{x:0,y:2}
 ];
 const result=camera.registerGateOnMap(
  {width:4,points,...stableUncertainty},
  {x:12,y:-3},
  Math.PI/2
 );
 near(result.centerX,12);
 near(result.centerY,-3);
 near(result.headingDeg,90);
 near(result.openingStart.x,12);
 near(result.openingStart.y,-5);
 near(result.openingEnd.x,12);
 near(result.openingEnd.y,-1);
 near(result.sourceAxisLength,4);
 assert.equal(result.mapRegistration,'manual-center-and-heading');
});

test('camera gate map registration rejects missing or invalid manual map pose',()=>{
 const measurement={width:4,points:[
  {x:0,y:0},{x:4,y:0},{x:4,y:2},{x:0,y:2},
 ],...stableUncertainty};
 assert.throws(()=>camera.registerGateOnMap(measurement,{x:NaN,y:0},0));
 assert.throws(()=>camera.registerGateOnMap(measurement,{x:0,y:0},Infinity));
 assert.throws(()=>camera.registerGateOnMap({...measurement,points:[]},{x:0,y:0},0));
});


test('camera registration carries a two-sigma width envelope into gate geometry',()=>{
 const points=[
  {x:0,y:0},{x:4,y:0},{x:4,y:2},{x:0,y:2}
 ];
 const registration=camera.registerGateOnMap(
  {width:4,sigmaWidth:0.03,points,...stableUncertainty},
  {x:12,y:0},
  Math.PI/2
 );
 near(registration.widthSigma,0.03);
 near(registration.widthUncertaintyBound,0.06);
 assert.equal(registration.widthUncertaintyMultiplier,2);
 assert.equal(registration.widthUncertaintyModel,'2-sigma-normal-approximation');
 assert.throws(
  ()=>camera.registerGateOnMap({width:4,sigmaWidth:-0.01,points,...stableUncertainty},{x:0,y:0},0),
  /1σ ширины/
 );
 assert.throws(
  ()=>camera.registerGateOnMap({width:4,sigmaWidth:2.1,points,...stableUncertainty},{x:0,y:0},0),
  /превышает поддерживаемый предел/
 );
});

const rectangularReference=[
 {x:0,y:0},{x:100,y:0},{x:100,y:50},{x:0,y:50}
];

test('camera measurement reports rectangularity diagnostics for a consistent gate opening',()=>{
 const result=camera.measureQuad(
  rectangularReference,4,2,
  [{x:20,y:10},{x:80,y:10},{x:80,y:40},{x:20,y:40}]
 );
 assert.equal(result.shapeWarning,false);
 assert.equal(result.shapeDiagnostics.warning,false);
 near(result.shapeDiagnostics.widthSpreadRatio,0);
 near(result.shapeDiagnostics.heightSpreadRatio,0);
 near(result.shapeDiagnostics.maxOrthogonalityErrorDeg,0);
 near(result.shapeDiagnostics.diagonalSpreadRatio,0);
});

test('camera measurement flags a trapezoidal or skewed opening instead of silently averaging its edges',()=>{
 const result=camera.measureQuad(
  rectangularReference,4,2,
  [{x:20,y:10},{x:90,y:12},{x:80,y:40},{x:20,y:40}]
 );
 assert.equal(result.shapeWarning,true);
 assert.ok(
  result.shapeDiagnostics.widthSpreadRatio>camera.MAX_QUAD_EDGE_SPREAD_RATIO||
  result.shapeDiagnostics.heightSpreadRatio>camera.MAX_QUAD_EDGE_SPREAD_RATIO||
  result.shapeDiagnostics.maxOrthogonalityErrorDeg>camera.MAX_QUAD_ANGLE_ERROR_DEG
 );
});

test('map registration rejects a non-rectangular measured gate and inconsistent reported width',()=>{
 const skewed=camera.measureQuad(
  rectangularReference,4,2,
  [{x:20,y:10},{x:90,y:12},{x:80,y:40},{x:20,y:40}]
 );
 assert.throws(
  ()=>camera.registerGateOnMap(skewed,{x:12,y:0},Math.PI/2),
  /Форма измеренного проёма/
 );
 const rectangular=camera.measureQuad(
  rectangularReference,4,2,
  [{x:20,y:10},{x:80,y:10},{x:80,y:40},{x:20,y:40}]
 );
 assert.throws(
  ()=>camera.registerGateOnMap({...rectangular,width:rectangular.width+0.2},{x:12,y:0},Math.PI/2),
  /не совпадает с шириной/
 );
});


test('camera uncertainty cross-check agrees with finite one-sigma perturbations for an affine calibration',()=>{
 const result=camera.estimateUncertainty({
  referencePoints:rectangularReference,
  referenceWidth:4,
  referenceHeight:2,
  measuredPoints:[{x:20,y:10},{x:80,y:10},{x:80,y:40},{x:20,y:40}],
  sigmaPixels:2,
  sigmaReferenceWidth:0.002,
  sigmaReferenceHeight:0.002
 });
 assert.equal(result.nonlinearSensitivityAvailable,true);
 assert.equal(result.nonlinearSensitivityChecks,18);
 assert.equal(result.nonlinearSensitivityModel,'axiswise-one-sigma-finite-perturbations');
 assert.ok(result.nonlinearSigmaWidth>0&&result.nonlinearSigmaHeight>0);
 assert.ok(result.nonlinearMismatchWidth<0.05);
 assert.ok(result.nonlinearMismatchHeight<0.05);
 assert.ok(result.maxAxisAsymmetryRatio<0.25);
 assert.equal(result.nonlinearWarning,false);
});

test('camera uncertainty flags a one-sigma perturbation that invalidates the reference dimensions',()=>{
 const result=camera.estimateUncertainty({
  referencePoints:rectangularReference,
  referenceWidth:4,
  referenceHeight:2,
  measuredPoints:[{x:20,y:10},{x:80,y:10},{x:80,y:40},{x:20,y:40}],
  sigmaPixels:2,
  sigmaReferenceWidth:4,
  sigmaReferenceHeight:0.002
 });
 assert.equal(result.nonlinearSensitivityAvailable,false);
 assert.equal(result.nonlinearWarning,true);
 assert.ok(result.nonlinearWarningReasons.includes('finite-perturbation-unavailable'));
 assert.match(result.nonlinearSensitivityFailure,/Размеры эталона|корректны/);
});

test('gate map registration refuses measurements without a successful nonlinear sensitivity cross-check',()=>{
 const measurement=camera.estimateUncertainty({
  referencePoints:rectangularReference,
  referenceWidth:4,
  referenceHeight:2,
  measuredPoints:[{x:20,y:10},{x:80,y:10},{x:80,y:40},{x:20,y:40}],
  sigmaPixels:2,
  sigmaReferenceWidth:0.002,
  sigmaReferenceHeight:0.002
 });
 assert.throws(
  ()=>camera.registerGateOnMap({...measurement,nonlinearWarning:true},{x:12,y:0},Math.PI/2),
  /Нелинейная проверка неопределённости/
 );
 assert.throws(
  ()=>camera.registerGateOnMap({width:measurement.width,points:measurement.points,sigmaWidth:measurement.sigmaWidth},{x:12,y:0},Math.PI/2),
  /Нелинейная проверка неопределённости/
 );
});
