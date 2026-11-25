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
 const ids=['measureVideo','measureOverlay','cameraStatus','cameraStart','cameraStop','cameraCalibrate','cameraMeasureGate','cameraReset','cameraReferenceWidth','cameraReferenceHeight','cameraPointSigma','cameraReferenceSigma','gateWidth','gateWidthUncertainty','gateCenterX','gateCenterY','gateHeadingDeg','gateResult','cameraMeasureObstacle','cameraClearObstacles','cameraObstacleName','cameraObstacleX','cameraObstacleY','cameraObstacleHeading'];
 for(const id of ids){
  elements[id]={
   id,value:({
    cameraReferenceWidth:'2.970',cameraReferenceHeight:'2.100',
    cameraPointSigma:'2',cameraReferenceSigma:'2',gateWidth:'3.50',gateWidthUncertainty:'0',gateCenterX:'12',gateCenterY:'0',gateHeadingDeg:'90',cameraObstacleName:'Опорный столб',cameraObstacleX:'3',cameraObstacleY:'4',cameraObstacleHeading:'90'
   })[id]||'',
   textContent:'',style:{},listeners:{},width:0,height:0,
   videoWidth:id==='measureVideo'?640:0,
   videoHeight:id==='measureVideo'?480:0,
   clientWidth:640,clientHeight:480,offsetLeft:0,offsetTop:0,
   addEventListener(type,fn){(this.listeners[type]??=[]).push(fn);},
   getBoundingClientRect(){return{left:0,top:0,width:640,height:480};},
   getContext(){return context;},
   paused:true,
   play(){this.paused=false;return options.playError?Promise.reject(new Error('video playback failed')):Promise.resolve();},
   pause(){this.paused=true;}
  };
 }
 let permissionRequests=0,trackStops=0,clearCalls=0;
 const appliedRegistrations=[],obstacleRegistrations=[];
 const win={
  TruckVisionCameraGeometry:options.geometry||geometry,
  TruckVisionApplyGateRegistration:options.applyGateRegistration||((registration)=>{appliedRegistrations.push(registration);elements.gateWidth.value=registration.clearWidth.toFixed(2);elements.gateWidthUncertainty.value=registration.widthUncertaintyBound.toFixed(3);elements.gateCenterX.value=registration.centerX.toFixed(2);elements.gateCenterY.value=registration.centerY.toFixed(2);elements.gateHeadingDeg.value=registration.headingDeg.toFixed(2);return true;}),
  TruckVisionAddCameraObstacle:options.addCameraObstacle||((measurement,anchor,heading,name)=>{obstacleRegistrations.push({measurement,anchor,heading,name});return true;}),
  TruckVisionClearCameraObstacles:options.clearCameraObstacles||(()=>{clearCalls++;return true;}),
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
 return {elements,windowListeners,fire,tap,appliedRegistrations,obstacleRegistrations,get clearCalls(){return clearCalls;},get permissionRequests(){return permissionRequests;},get trackStops(){return trackStops;}};
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
 assert.ok(Number(camera.elements.gateWidth.value)>1);
 assert.notEqual(camera.elements.gateWidth.value,'3.50','successful registration must replace the previous gate width');
 assert.equal(camera.elements.gateCenterX.value,'12.00');
 assert.equal(camera.elements.gateCenterY.value,'0.00');
 assert.equal(camera.elements.gateHeadingDeg.value,'90.00');
 assert.equal(camera.appliedRegistrations.length,1,'successful measurement must be registered on the simulator map');
 assert.equal(camera.appliedRegistrations[0].centerX,12);
 assert.equal(camera.appliedRegistrations[0].centerY,0);
 assert.equal(camera.appliedRegistrations[0].headingDeg,90);
 assert.ok(camera.appliedRegistrations[0].widthSigma>0);
 assert.ok(camera.appliedRegistrations[0].widthUncertaintyBound>0);
 assert.ok(Math.abs(camera.appliedRegistrations[0].widthUncertaintyBound-2*camera.appliedRegistrations[0].widthSigma)<1e-10);
 assert.equal(camera.appliedRegistrations[0].widthUncertaintyModel,'2-sigma-normal-approximation');
 assert.equal(camera.elements.gateWidthUncertainty.value,camera.appliedRegistrations[0].widthUncertaintyBound.toFixed(3));
 assert.match(camera.elements.gateResult.textContent,/Ширина оболочки/);
 camera.fire('cameraStop','click');
 assert.equal(camera.trackStops,1,'stopping the camera must stop its media track');
});

test('camera permission denial is explained and leaves no stream attached',async()=>{
 const camera=bootCamera({permissionError:'NotAllowedError'});
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1);
 assert.match(camera.elements.cameraStatus.textContent,/Доступ к камере запрещён/);
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
 assert.equal(camera.trackStops,0);
});

test('insecure context is rejected before requesting camera permission',async()=>{
 const camera=bootCamera({secureContext:false});
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,0);
 assert.match(camera.elements.cameraStatus.textContent,/HTTPS/);
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
});

test('camera stream is stopped if inline video playback fails after permission was granted',async()=>{
 const camera=bootCamera({playError:true});
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1);
 assert.equal(camera.trackStops,1);
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
 assert.match(camera.elements.cameraStatus.textContent,/Не удалось включить камеру/);
});

test('failed projective measurement keeps the previous gate width and explains that it was not updated',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:150},{x:400,y:300},{x:200,y:250}]);
 assert.match(camera.elements.cameraStatus.textContent,/Плоскость откалибрована/);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:180,y:390},{x:460,y:390},{x:460,y:460},{x:180,y:460}]);
 assert.equal(camera.elements.gateWidth.value,'3.50');
 assert.match(camera.elements.cameraStatus.textContent,/пересекает линию вырождения/);
 assert.match(camera.elements.cameraStatus.textContent,/Ширина ворот не обновлена/);
});

test('poorly conditioned camera calibration never overwrites the previous gate width',async()=>{
 const warningGeometry={
  ...geometry,
  estimateUncertainty:options=>({
   ...geometry.estimateUncertainty(options),
   conditioningWarning:true,
   conditionEstimate:2e5
  })
 };
 const camera=bootCamera({geometry:warningGeometry});
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.elements.gateWidth.value,'3.50');
 assert.match(camera.elements.cameraStatus.textContent,/Калибровка плохо обусловлена/);
 assert.match(camera.elements.gateResult.textContent,/не передан в настройки ворот/);
 assert.equal(camera.appliedRegistrations.length,0,'poor conditioning must not register gate geometry');
 camera.fire('cameraStop','click');
});


test('camera measurement fails closed when simulator map registration rejects the result',async()=>{
 const camera=bootCamera({applyGateRegistration:()=>false});
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.elements.gateWidth.value,'3.50');
 assert.match(camera.elements.cameraStatus.textContent,/Привязка ворот не прошла проверку/);
 assert.equal(camera.appliedRegistrations.length,0);
 camera.fire('cameraStop','click');
});


test('camera rejects a skewed gate quadrilateral and keeps the previous width and map pose',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:180},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.elements.gateWidth.value,'3.50');
 assert.match(camera.elements.gateResult.textContent,/Форма измеренного проёма не соответствует прямоугольнику/);
 assert.match(camera.elements.cameraStatus.textContent,/Ширина ворот не обновлена/);
 assert.equal(camera.appliedRegistrations.length,0);
 camera.fire('cameraStop','click');
});


test('camera-measured obstacle is registered with manual map pose and can be cleared',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,1);
 const registration=camera.obstacleRegistrations[0];
 assert.equal(registration.anchor.x,3);
 assert.equal(registration.anchor.y,4);
 assert.equal(registration.heading,Math.PI/2);
 assert.equal(registration.name,'Опорный столб');
 assert.ok(registration.measurement.width>0);
 assert.ok(registration.measurement.height>0);
 assert.match(camera.elements.cameraStatus.textContent,/Препятствие добавлено на карту/);
 await camera.fire('cameraClearObstacles','click');
 assert.equal(camera.clearCalls,1);
 assert.match(camera.elements.cameraStatus.textContent,/препятствия удалены с карты/);
 camera.fire('cameraStop','click');
});

test('camera obstacle registration requires explicit map coordinates and heading',async()=>{
 const camera=bootCamera();
 camera.elements.cameraObstacleX.value='';
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Введите координаты центра и направление препятствия/);
 camera.fire('cameraStop','click');
});
