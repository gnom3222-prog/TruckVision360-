'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');

function makeHarness(options={}){
 const elements={};
 const makeElement=id=>{
  const el={id,value:'',textContent:'',style:{},listeners:{},addEventListener(type,fn){this.listeners[type]=fn;},fire(type,event={}){return this.listeners[type]?.(event);}};
  elements[id]=el;return el;
 };
 const ids=['measureVideo','measureOverlay','cameraStatus','cameraStart','cameraStop','cameraReferenceWidth','cameraReferenceHeight','cameraPointSigma','cameraReferenceSigma','cameraCalibrate','cameraMeasureGate','cameraMeasureObstacle','cameraReset','cameraObstacleName','cameraObstacleHeading','cameraObstacleX','cameraObstacleY','cameraObstaclePositionUncertaintyX','cameraObstaclePositionUncertaintyY','cameraObstacleHeadingUncertaintyDeg','gateCenterX','gateCenterY','gateHeadingDeg','gatePositionUncertainty','gateHeadingUncertaintyDeg','gatePostWidthUncertainty','gateDepthUncertainty','gateResult','cameraRepeatabilityWidthTolerance','cameraRepeatabilityHeightTolerance','cameraRepeatabilityStatus','cameraClearGateRepeats'];
 ids.forEach(makeElement);
 const video=elements.measureVideo;
 Object.assign(video,{videoWidth:640,videoHeight:480,clientWidth:640,clientHeight:480,offsetLeft:0,offsetTop:0,readyState:4,currentTime:0.1,paused:true,srcObject:null,playCalls:0,pauseCalls:0,play(){this.playCalls++;this.currentTime+=0.1;this.paused=false;return Promise.resolve();},pause(){this.pauseCalls++;this.paused=true;}});
 const context={clearRect(){},beginPath(){},moveTo(){},lineTo(){},closePath(){},stroke(){},arc(){},fill(){},fillText(){}};
 const canvas=elements.measureOverlay;
 Object.assign(canvas,{width:0,height:0,getContext(){return context;},getBoundingClientRect(){return {left:0,top:0,width:this.width,height:this.height};}});
 Object.assign(elements.cameraReferenceWidth,{value:'0.297'});
 Object.assign(elements.cameraReferenceHeight,{value:'0.210'});
 Object.assign(elements.cameraPointSigma,{value:'2'});
 Object.assign(elements.cameraReferenceSigma,{value:'2'});
 Object.assign(elements.cameraRepeatabilityWidthTolerance,{value:'0.02'});
 Object.assign(elements.cameraRepeatabilityHeightTolerance,{value:'0.02'});
 Object.assign(elements.gateCenterX,{value:'12'});
 Object.assign(elements.gateCenterY,{value:'0'});
 Object.assign(elements.gateHeadingDeg,{value:'90'});
 Object.assign(elements.gatePositionUncertainty,{value:'0.05'});
 Object.assign(elements.gateHeadingUncertaintyDeg,{value:'1'});
 Object.assign(elements.gatePostWidthUncertainty,{value:'0.02'});
 Object.assign(elements.gateDepthUncertainty,{value:'0.03'});
 let permissionCalls=0,registrationCalls=0,obstacleCalls=0,trackStops=0;
 const tracks=[];
 function createStream(){
  const listeners={};
  const track={
   readyState:'live',
   addEventListener(type,handler){(listeners[type]||(listeners[type]=[])).push(handler);},
   fire(type){
    if(type==='ended')this.readyState='ended';
    for(const handler of listeners[type]||[])handler();
    if(type==='ended')delete listeners.ended;
   },
   stop(){
    if(this.readyState==='ended')return;
    trackStops++;
    this.fire('ended');
   }
  };
  tracks.push(track);
  return {getTracks(){return [track];}};
 }
 const geometry={
  validQuad(){return true;},
  homography(){return [1,0,0,0,1,0,0,0];},
  estimateUncertainty({measuredPoints}){return {width:4,height:2,points:measuredPoints.map(p=>({...p})),sigmaWidth:0.01,sigmaHeight:0.02,conditioningWarning:false,shapeWarning:false,nonlinearSensitivityAvailable:options.nonlinearSensitivityAvailable??true,nonlinearWarning:options.nonlinearWarning??false,nonlinearMismatchWidth:0.01,maxAxisAsymmetryRatio:0.02,shapeDiagnostics:{widthSpreadRatio:0,heightSpreadRatio:0,maxOrthogonalityErrorDeg:0,diagonalSpreadRatio:0}};},
  summarizeRepeatability(readings,options){
  const stats=key=>{const values=readings.map(r=>r[key]);const mean=values.reduce((a,b)=>a+b,0)/values.length;return{mean,range:Math.max(...values)-Math.min(...values),maxDeviation:Math.max(...values.map(v=>Math.abs(v-mean))),sampleStdDev:0};};
  const w=stats('width'),h=stats('height');
  return{count:readings.length,meanWidth:w.mean,meanHeight:h.mean,sampleStdDevWidth:w.sampleStdDev,sampleStdDevHeight:h.sampleStdDev,rangeWidth:w.range,rangeHeight:h.range,maxDeviationWidth:w.maxDeviation,maxDeviationHeight:h.maxDeviation,widthTolerance:options.widthTolerance,heightTolerance:options.heightTolerance,widthEnvelopeBound:Math.max(...readings.map(r=>Math.abs(r.width-w.mean)+2*r.sigmaWidth)),heightEnvelopeBound:Math.max(...readings.map(r=>Math.abs(r.height-h.mean)+2*r.sigmaHeight)),withinTolerance:w.range<=options.widthTolerance&&h.range<=options.heightTolerance,status:'repeatable-within-operator-tolerance',uncertaintyModel:'observed-range-plus-per-frame-2sigma-screening'};
 },
 registerGateOnMap(){return {centerX:12,centerY:0,headingDeg:90,clearWidth:4,clearHeight:2,widthUncertaintyBound:0.02,heightUncertaintyBound:0.04,openingStart:{x:12,y:-2},openingEnd:{x:12,y:2}};}
 };
 const window={TruckVisionCameraGeometry:geometry,TruckVisionApplyGateRegistration(){registrationCalls++;return true;},TruckVisionAddCameraObstacle(){obstacleCalls++;return true;},addEventListener(){}};
 const documentListeners={};
 const document={visibilityState:'visible',getElementById(id){return elements[id]||null;},addEventListener(type,fn){(documentListeners[type]||(documentListeners[type]=[])).push(fn);}};
 const navigator={mediaDevices:{async getUserMedia(){
  permissionCalls++;
  if(options.getUserMedia)return options.getUserMedia({createStream});
  return createStream();
 }}};
 const source=fs.readFileSync(require.resolve('../camera-measure.js'),'utf8');
 vm.runInNewContext(source,{document,window,navigator,isSecureContext:options.isSecureContext??true,console});
 const tapQuad=()=>{
  for(const [clientX,clientY] of [[10,10],[630,10],[630,470],[10,470]])canvas.fire('pointerdown',{clientX,clientY});
 };
 return {elements,video,tapQuad,tracks,setVisibility(state){document.visibilityState=state;for(const fn of documentListeners.visibilitychange||[])fn();},get permissionCalls(){return permissionCalls;},get registrationCalls(){return registrationCalls;},get obstacleCalls(){return obstacleCalls;},get trackStops(){return trackStops;}};
}

test('camera permission is requested only after an explicit user action',async()=>{
 const app=makeHarness();
 assert.equal(app.permissionCalls,0);
 await app.elements.cameraStart.fire('click');
 assert.equal(app.permissionCalls,1);
 assert.equal(app.video.paused,false);
});

test('calibration and gate points use three separately frozen frames before map registration',async()=>{
 const app=makeHarness();
 await app.elements.cameraStart.fire('click');
 assert.equal(app.video.paused,false);
 for(let index=0;index<3;index++){
  if(index)app.elements.cameraReset.fire('click');
  app.elements.cameraCalibrate.fire('click');
  assert.equal(app.video.paused,true,'calibration must freeze the live video frame');
  app.tapQuad();
  assert.match(app.elements.cameraStatus.textContent,/Плоскость откалибрована/i);
  app.elements.cameraMeasureGate.fire('click');
  assert.equal(app.video.paused,true,'gate measurement must keep the calibration frame frozen');
  app.tapQuad();
  assert.equal(app.registrationCalls,index===2?1:0);
 }
 assert.equal(app.video.paused,true,'successful registration must not silently resume a moving image');
 app.elements.cameraReset.fire('click');
 assert.equal(app.video.paused,false,'reset must resume the live camera before a new calibration');
 app.elements.cameraMeasureGate.fire('click');
 assert.match(app.elements.cameraStatus.textContent,/Сначала выполните калибровку/);
});

test('stopping the camera pauses video and releases every media track',async()=>{
 const app=makeHarness();
 await app.elements.cameraStart.fire('click');
 app.elements.cameraCalibrate.fire('click');
 app.elements.cameraStop.fire('click');
 assert.equal(app.video.paused,true);
 assert.equal(app.video.srcObject,null);
 assert.equal(app.trackStops,1);
 assert.match(app.elements.cameraStatus.textContent,/Камера выключена/);
 assert.doesNotMatch(app.elements.cameraStatus.textContent,/завершился или был прерван/);
});

test('a denied camera permission leaves no active stream and explains how to retry',async()=>{
 const denied=Object.assign(new Error('permission denied'),{name:'NotAllowedError'});
 const app=makeHarness({getUserMedia:async()=>{throw denied;}});
 await app.elements.cameraStart.fire('click');
 assert.equal(app.video.srcObject,null);
 assert.match(app.elements.cameraStatus.textContent,/Доступ к камере запрещён/);
});

test('an unexpectedly ended camera track clears stale state and allows a fresh permission request',async()=>{
 const app=makeHarness();
 await app.elements.cameraStart.fire('click');
 assert.equal(app.permissionCalls,1);
 app.tracks[0].fire('ended');
 assert.equal(app.video.srcObject,null);
 assert.equal(app.video.paused,true);
 assert.match(app.elements.cameraStatus.textContent,/Видеопоток камеры завершился или был прерван/);
 await app.elements.cameraStart.fire('click');
 assert.equal(app.permissionCalls,2);
 assert.match(app.elements.cameraStatus.textContent,/Камера включена/);
});


test('gate measurement is not transferred to the map when nonlinear uncertainty validation warns',async()=>{
 const app=makeHarness({nonlinearWarning:true});
 await app.elements.cameraStart.fire('click');
 for(let index=0;index<3;index++){
  if(index)app.elements.cameraReset.fire('click');
  app.elements.cameraCalibrate.fire('click');
  app.tapQuad();
  app.elements.cameraMeasureGate.fire('click');
  app.tapQuad();
 }
 assert.equal(app.registrationCalls,0);
 assert.match(app.elements.gateResult.textContent,/Нелинейная проверка неопределённости не прошла/);
 assert.match(app.elements.cameraStatus.textContent,/Ширина ворот не обновлена/);
});

test('camera obstacle is not added when nonlinear uncertainty validation is unavailable',async()=>{
 const app=makeHarness({nonlinearSensitivityAvailable:false});
 await app.elements.cameraStart.fire('click');
 app.elements.cameraCalibrate.fire('click');
 app.tapQuad();
 app.elements.cameraMeasureObstacle.fire('click');
 app.tapQuad();
 assert.equal(app.obstacleCalls,0);
 assert.match(app.elements.cameraStatus.textContent,/Нелинейная проверка неопределённости не пройдена/);
});
