'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');

function makeHarness(){
 const elements={};
 const makeElement=id=>{
  const el={id,value:'',textContent:'',style:{},listeners:{},addEventListener(type,fn){this.listeners[type]=fn;},fire(type,event={}){return this.listeners[type]?.(event);}};
  elements[id]=el;return el;
 };
 const ids=['measureVideo','measureOverlay','cameraStatus','cameraStart','cameraStop','cameraReferenceWidth','cameraReferenceHeight','cameraPointSigma','cameraReferenceSigma','cameraCalibrate','cameraMeasureGate','cameraReset','gateCenterX','gateCenterY','gateHeadingDeg','gateResult'];
 ids.forEach(makeElement);
 const video=elements.measureVideo;
 Object.assign(video,{videoWidth:640,videoHeight:480,clientWidth:640,clientHeight:480,offsetLeft:0,offsetTop:0,readyState:4,paused:true,srcObject:null,playCalls:0,pauseCalls:0,play(){this.playCalls++;this.paused=false;return Promise.resolve();},pause(){this.pauseCalls++;this.paused=true;}});
 const context={clearRect(){},beginPath(){},moveTo(){},lineTo(){},closePath(){},stroke(){},arc(){},fill(){},fillText(){}};
 const canvas=elements.measureOverlay;
 Object.assign(canvas,{width:0,height:0,getContext(){return context;},getBoundingClientRect(){return {left:0,top:0,width:this.width,height:this.height};}});
 Object.assign(elements.cameraReferenceWidth,{value:'0.297'});
 Object.assign(elements.cameraReferenceHeight,{value:'0.210'});
 Object.assign(elements.cameraPointSigma,{value:'2'});
 Object.assign(elements.cameraReferenceSigma,{value:'2'});
 Object.assign(elements.gateCenterX,{value:'12'});
 Object.assign(elements.gateCenterY,{value:'0'});
 Object.assign(elements.gateHeadingDeg,{value:'90'});
 let permissionCalls=0,registrationCalls=0,trackStops=0;
 const mediaStream={getTracks(){return [{stop(){trackStops++;}}];}};
 const geometry={
  validQuad(){return true;},
  homography(){return [1,0,0,0,1,0,0,0];},
  estimateUncertainty({measuredPoints}){return {width:4,height:2,points:measuredPoints.map(p=>({...p})),sigmaWidth:0.01,sigmaHeight:0.02,conditioningWarning:false,shapeWarning:false,shapeDiagnostics:{widthSpreadRatio:0,heightSpreadRatio:0,maxOrthogonalityErrorDeg:0,diagonalSpreadRatio:0}};},
  registerGateOnMap(){return {centerX:12,centerY:0,headingDeg:90,clearWidth:4,widthUncertaintyBound:0.02,openingStart:{x:12,y:-2},openingEnd:{x:12,y:2}};}
 };
 const window={TruckVisionCameraGeometry:geometry,TruckVisionApplyGateRegistration(){registrationCalls++;return true;},addEventListener(){}};
 const document={getElementById(id){return elements[id]||null;}};
 const navigator={mediaDevices:{async getUserMedia(){permissionCalls++;return mediaStream;}}};
 const source=fs.readFileSync(require.resolve('../camera-measure.js'),'utf8');
 vm.runInNewContext(source,{document,window,navigator,isSecureContext:true,console});
 const tapQuad=()=>{
  for(const [clientX,clientY] of [[10,10],[630,10],[630,470],[10,470]])canvas.fire('pointerdown',{clientX,clientY});
 };
 return {elements,video,tapQuad,get permissionCalls(){return permissionCalls;},get registrationCalls(){return registrationCalls;},get trackStops(){return trackStops;}};
}

test('camera permission is requested only after an explicit user action',async()=>{
 const app=makeHarness();
 assert.equal(app.permissionCalls,0);
 await app.elements.cameraStart.fire('click');
 assert.equal(app.permissionCalls,1);
 assert.equal(app.video.paused,false);
});

test('calibration and gate points use one frozen frame until the user resets calibration',async()=>{
 const app=makeHarness();
 await app.elements.cameraStart.fire('click');
 assert.equal(app.video.paused,false);
 app.elements.cameraCalibrate.fire('click');
 assert.equal(app.video.paused,true,'calibration must freeze the live video frame');
 app.tapQuad();
 assert.match(app.elements.cameraStatus.textContent,/Кадр зафиксирован/i);
 app.elements.cameraMeasureGate.fire('click');
 assert.equal(app.video.paused,true,'gate measurement must keep the calibration frame frozen');
 app.tapQuad();
 assert.equal(app.registrationCalls,1);
 assert.equal(app.video.paused,true,'successful measurement must not silently resume a moving image');
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
});
