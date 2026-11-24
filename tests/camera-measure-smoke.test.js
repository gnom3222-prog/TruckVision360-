'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const geometry=require('../camera-geometry.js');

function bootCamera(options={}){
 const elements={},windowListeners={};
 const context={clearRect(){},beginPath(){},moveTo(){},lineTo(){},closePath(){},stroke(){},arc(){},fill(){},fillText(){}};
 const ids=['measureVideo','measureOverlay','cameraStatus','cameraStart','cameraStop','cameraCalibrate','cameraMeasureGate','cameraReset','cameraReferenceWidth','cameraReferenceHeight','cameraPointSigma','cameraReferenceSigma','gateWidth','gateResult'];
 for(const id of ids){
  elements[id]={
   id,value:({
    cameraReferenceWidth:'0.297',cameraReferenceHeight:'0.210',
    cameraPointSigma:'2',cameraReferenceSigma:'2',gateWidth:'3.50'
   })[id]||'',
   textContent:'',style:{},listeners:{},width:0,height:0,
   videoWidth:id==='measureVideo'?640:0,
   videoHeight:id==='measureVideo'?480:0,
   clientWidth:640,clientHeight:480,offsetLeft:0,offsetTop:0,
   addEventListener(type,fn){(this.listeners[type]??=[]).push(fn);},
   getBoundingClientRect(){return{left:0,top:0,width:640,height:480};},
   getContext(){return context;},
   play(){return options.playError?Promise.reject(new Error('video playback failed')):Promise.resolve();}
  };
 }
 let permissionRequests=0,trackStops=0;
 const win={
  TruckVisionCameraGeometry:geometry,
  addEventListener(type,fn){(windowListeners[type]??=[]).push(fn);}
 };
 const sandbox={
  window:win,
  document:{getElementById:id=>elements[id]},
  navigator:{mediaDevices:{getUserMedia:async()=>{permissionRequests++;if(options.permissionError){const error=new Error('camera permission error');error.name=options.permissionError;throw error;}return{getTracks:()=>[{stop(){trackStops++;}}]};}}},
  isSecureContext:options.secureContext!==false,
  console
 };
 const source=fs.readFileSync(path.join(__dirname,'..','camera-measure.js'),'utf8');
 vm.runInNewContext(source,sandbox,{filename:'camera-measure.js'});
 const fire=(id,type,event={})=>elements[id].listeners[type][0](event);
 const tap=async points=>{
  for(const p of points)fire('measureOverlay','pointerdown',{clientX:p.x,clientY:p.y});
 };
 return {elements,windowListeners,fire,tap,get permissionRequests(){return permissionRequests;},get trackStops(){return trackStops;}};
}

test('camera permission is requested only on tap; measurement reports uncertainty and orientation invalidates calibration',async()=>{
 const camera=bootCamera();
 assert.equal(camera.permissionRequests,0,'loading the camera module must not request permission');
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1,'the explicit camera button should request permission');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 assert.match(camera.elements.cameraStatus.textContent,/Плоскость откалибрована/);

 camera.windowListeners.orientationchange[0]();
 assert.match(camera.elements.cameraStatus.textContent,/Повторите калибровку/);
 camera.fire('cameraMeasureGate','click');
 assert.match(camera.elements.cameraStatus.textContent,/Сначала выполните калибровку/);

 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.match(camera.elements.gateResult.textContent,/оценка чувствительности/);
 assert.match(camera.elements.cameraStatus.textContent,/1σ/);
 assert.ok(Number(camera.elements.gateWidth.value)>0);
 camera.fire('cameraStop','click');
 assert.equal(camera.trackStops,1,'stopping the camera must stop its media track');
});

test('camera permission denial is explained and leaves no stream attached',async()=>{
 const camera=bootCamera({permissionError:'NotAllowedError'});
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1);
 assert.match(camera.elements.cameraStatus.textContent,/Доступ к камере запрещён/);
 assert.equal(camera.elements.measureVideo.srcObject,null);
 assert.equal(camera.trackStops,0);
});

test('insecure context is rejected before requesting camera permission',async()=>{
 const camera=bootCamera({secureContext:false});
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,0);
 assert.match(camera.elements.cameraStatus.textContent,/HTTPS/);
 assert.equal(camera.elements.measureVideo.srcObject,null);
});

test('camera stream is stopped if inline video playback fails after permission was granted',async()=>{
 const camera=bootCamera({playError:true});
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1);
 assert.equal(camera.trackStops,1);
 assert.equal(camera.elements.measureVideo.srcObject,null);
 assert.match(camera.elements.cameraStatus.textContent,/Не удалось включить камеру/);
});
