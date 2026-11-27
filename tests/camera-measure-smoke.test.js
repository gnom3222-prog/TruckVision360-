'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const geometry=require('../camera-geometry.js');

function bootCamera(options={}){
 const elements={},windowListeners={},documentListeners={};
 const context={clearRect(){},beginPath(){},moveTo(){},lineTo(){},closePath(){},stroke(){},arc(){},fill(){},fillText(){}};
 const ids=['measureVideo','measureOverlay','cameraStatus','cameraStart','cameraStop','cameraCalibrate','cameraMeasureGate','cameraReset','cameraReferenceWidth','cameraReferenceHeight','cameraPointSigma','cameraReferenceSigma','gateWidth','gateWidthUncertainty','gateCenterX','gateCenterY','gateHeadingDeg','gatePostWidth','gateDepth','gatePositionUncertainty','gateHeadingUncertaintyDeg','gatePostWidthUncertainty','gateDepthUncertainty','gateResult','cameraMeasureObstacle','cameraClearObstacles','cameraObstacleName','cameraObstacleX','cameraObstacleY','cameraObstacleHeading','cameraObstaclePositionUncertaintyX','cameraObstaclePositionUncertaintyY','cameraObstacleHeadingUncertaintyDeg','cameraObstacleRepeatabilityWidthTolerance','cameraObstacleRepeatabilityDepthTolerance','cameraObstacleRepeatabilityStatus','cameraClearObstacleRepeats','cameraObstacleHeight','cameraObstacleHeightUncertainty','cameraObstacleBaseDatumUncertainty','cameraObstacleVerticalDatumConfirmed','cameraRepeatabilityWidthTolerance','cameraRepeatabilityHeightTolerance','cameraRepeatabilityStatus','cameraClearGateRepeats'];
 for(const id of ids){
  elements[id]={
   id,value:({
    cameraReferenceWidth:'2.970',cameraReferenceHeight:'2.100',
    cameraPointSigma:'2',cameraReferenceSigma:'2',cameraRepeatabilityWidthTolerance:'0.02',cameraRepeatabilityHeightTolerance:'0.02',gateWidth:'3.50',gateWidthUncertainty:'0',gateCenterX:'12',gateCenterY:'0',gateHeadingDeg:'90',gatePostWidth:'0.25',gateDepth:'0.6',gatePositionUncertainty:'0.05',gateHeadingUncertaintyDeg:'1',gatePostWidthUncertainty:'0.02',gateDepthUncertainty:'0.03',cameraObstacleName:'Опорный столб',cameraObstacleX:'3',cameraObstacleY:'4',cameraObstacleHeading:'90',cameraObstaclePositionUncertaintyX:'0.10',cameraObstaclePositionUncertaintyY:'0.15',cameraObstacleHeadingUncertaintyDeg:'2',cameraObstacleRepeatabilityWidthTolerance:'0.02',cameraObstacleRepeatabilityDepthTolerance:'0.02',cameraObstacleHeight:'2.5',cameraObstacleHeightUncertainty:'0.03',cameraObstacleBaseDatumUncertainty:'0.02'
   })[id]||'',
   checked:id==='cameraObstacleVerticalDatumConfirmed'?(options.verticalConfirmed!==false):false,
   textContent:'',style:{},listeners:{},width:0,height:0,
   videoWidth:id==='measureVideo'?640:0,
   videoHeight:id==='measureVideo'?480:0,
   currentTime:id==='measureVideo'?0.1:undefined,
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
 const mediaTracks=[];
 const createStream=()=>{
  const listeners={};
  const track={
   stop(){trackStops++;},
   addEventListener(type,fn){(listeners[type]??=[]).push(fn);},
   emit(type){for(const fn of listeners[type]||[])fn();}
  };
  mediaTracks.push(track);
  return{getTracks:()=>[track]};
 };
 const appliedRegistrations=[],obstacleRegistrations=[];
 const win={
  TruckVisionCameraGeometry:options.geometry||geometry,
  TruckVisionApplyGateRegistration:options.applyGateRegistration||((registration)=>{appliedRegistrations.push(registration);elements.gateWidth.value=registration.clearWidth.toFixed(2);elements.gateWidthUncertainty.value=registration.widthUncertaintyBound.toFixed(3);elements.gateCenterX.value=registration.centerX.toFixed(2);elements.gateCenterY.value=registration.centerY.toFixed(2);elements.gateHeadingDeg.value=registration.headingDeg.toFixed(2);return true;}),
  TruckVisionAddCameraObstacle:options.addCameraObstacle||((measurement,anchor,heading,name,poseUncertainty,repeatability,verticalMeasurement)=>{obstacleRegistrations.push({measurement,anchor,heading,name,poseUncertainty,repeatability,verticalMeasurement});return true;}),
  TruckVisionClearCameraObstacles:options.clearCameraObstacles||(()=>{clearCalls++;return true;}),
  addEventListener(type,fn){(windowListeners[type]??=[]).push(fn);}
 };
 const sandbox={
  window:win,
  document:{visibilityState:'visible',getElementById:id=>elements[id],addEventListener(type,fn){(documentListeners[type]??=[]).push(fn);}},
  navigator:{mediaDevices:{getUserMedia:async()=>{
   permissionRequests++;
   if(options.permissionError){const error=new Error('camera permission error');error.name=options.permissionError;throw error;}
   if(options.getUserMedia)return options.getUserMedia({createStream});
   return createStream();
  }}},
  isSecureContext:options.secureContext!==false,
  console
 };
 const source=fs.readFileSync(path.join(__dirname,'..','camera-measure.js'),'utf8');
 vm.runInNewContext(source,sandbox,{filename:'camera-measure.js'});
 const fire=(id,type,event={})=>elements[id].listeners[type][0](event);
 const tap=async points=>{
  for(const p of points)fire('measureOverlay','pointerdown',{clientX:p.x,clientY:p.y});
 };
 return {
  elements,windowListeners,fire,tap,appliedRegistrations,obstacleRegistrations,
  setVisibility(state){sandbox.document.visibilityState=state;for(const fn of documentListeners.visibilitychange||[])fn();},
  setVideoTime(time){elements.measureVideo.currentTime=time;},
  endTrack(index=mediaTracks.length-1){mediaTracks[index]?.emit('ended');},
  get clearCalls(){return clearCalls;},
  get permissionRequests(){return permissionRequests;},
  get trackStops(){return trackStops;}
 };
}

const referenceFramePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
const defaultObstaclePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
async function measureObstacleFrame(camera,time,points=defaultObstaclePoints){
 if(time!==null){
  camera.fire('cameraReset','click');
  camera.setVideoTime(time);
 }
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referenceFramePoints);
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap(points);
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

 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.equal(camera.appliedRegistrations.length,0,'one frame must not update the gate map');
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 for(const time of [0.2,0.3]){
  camera.fire('cameraReset','click');
  camera.setVideoTime(time);
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gatePoints);
 }
 assert.match(camera.elements.gateResult.textContent,/оценка чувствительности/);
 assert.match(camera.elements.gateResult.textContent,/Повторяемость по 3 отдельным кадрам/);
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
 assert.ok(camera.appliedRegistrations[0].widthUncertaintyBound>=2*camera.appliedRegistrations[0].widthSigma-1e-10);
 assert.equal(camera.appliedRegistrations[0].widthUncertaintyModel,'observed-range-plus-per-frame-2sigma-screening');
 assert.equal(camera.appliedRegistrations[0].repeatability.count,3);
 assert.ok(camera.appliedRegistrations[0].clearHeight>0);
 assert.ok(camera.appliedRegistrations[0].heightUncertaintyBound>0);
 assert.equal(camera.appliedRegistrations[0].heightUncertaintyBound,camera.appliedRegistrations[0].repeatability.heightEnvelopeBound);
 assert.equal(camera.elements.gateWidthUncertainty.value,camera.appliedRegistrations[0].widthUncertaintyBound.toFixed(3));
 assert.match(camera.elements.gateResult.textContent,/Ширина оболочки/);
 camera.fire('cameraStop','click');
 assert.equal(camera.trackStops,1,'stopping the camera must stop its media track');
});

test('camera measurement refuses a blank point-selection uncertainty and preserves the previous gate width',async()=>{
 const camera=bootCamera();
 camera.elements.cameraPointSigma.value='';
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.elements.gateWidth.value,'3.50');
 assert.equal(camera.appliedRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Введите положительную неопределённость/);
 camera.fire('cameraStop','click');
});

test('camera obstacle measurement refuses a blank reference-size uncertainty',async()=>{
 const camera=bootCamera();
 camera.elements.cameraReferenceSigma.value='';
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Введите положительную неопределённость/);
 camera.fire('cameraStop','click');
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
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 for(let index=0;index<3;index++){
  if(index){camera.fire('cameraReset','click');camera.setVideoTime(0.1+index*0.1);}
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gatePoints);
 }
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


test('camera-measured obstacle is registered only after three distinct frames and can be cleared',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 assert.equal(camera.obstacleRegistrations.length,0,'one frame must not update the obstacle map');
 assert.match(camera.elements.cameraObstacleRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 await measureObstacleFrame(camera,0.2);
 assert.equal(camera.obstacleRegistrations.length,0,'two frames must not update the obstacle map');
 await measureObstacleFrame(camera,0.3);
 assert.equal(camera.obstacleRegistrations.length,1);
 const registration=camera.obstacleRegistrations[0];
 assert.equal(registration.anchor.x,3);
 assert.equal(registration.anchor.y,4);
 assert.equal(registration.heading,Math.PI/2);
 assert.equal(registration.name,'Опорный столб');
 assert.equal(registration.poseUncertainty.positionX,0.10);
 assert.equal(registration.poseUncertainty.positionY,0.15);
 assert.equal(registration.poseUncertainty.headingDeg,2);
 assert.ok(registration.measurement.width>0);
 assert.ok(registration.measurement.height>0);
 assert.equal(registration.repeatability.count,3);
 assert.ok(registration.repeatability.widthEnvelopeBound>0);
 assert.ok(registration.repeatability.heightEnvelopeBound>0);
 assert.equal(registration.verticalMeasurement.height,2.5);
 assert.equal(registration.verticalMeasurement.heightUncertainty,0.03);
 assert.equal(registration.verticalMeasurement.baseDatumUncertainty,0.02);
 assert.equal(registration.verticalMeasurement.totalUncertainty,0.05);
 assert.equal(registration.verticalMeasurement.datum,'local-road-surface');
 assert.equal(registration.verticalMeasurement.source,'manual-independent-measurement');
 assert.equal(registration.verticalMeasurement.confirmed,true);
 assert.match(camera.elements.cameraStatus.textContent,/Высота 2.50 м введена вручную/);
 await camera.fire('cameraClearObstacles','click');
 assert.equal(camera.clearCalls,1);
 assert.match(camera.elements.cameraStatus.textContent,/препятствия удалены с карты/);
 camera.fire('cameraStop','click');
});

test('stopping the camera clears a partial obstacle repeatability series',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 assert.match(camera.elements.cameraObstacleRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 camera.fire('cameraStop','click');
 assert.match(camera.elements.cameraObstacleRepeatabilityStatus.textContent,/Повторяемость препятствия не оценена/);
 assert.equal(camera.obstacleRegistrations.length,0);
 await camera.fire('cameraStart','click');
 camera.setVideoTime(0.2);
 await measureObstacleFrame(camera,null);
 assert.match(camera.elements.cameraObstacleRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 assert.equal(camera.obstacleRegistrations.length,0);
 camera.fire('cameraStop','click');
});

test('an ended camera track clears a partial obstacle repeatability series before restart',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 assert.ok(camera.elements.cameraObstacleRepeatabilityStatus.textContent.includes('1/3 отдельных кадров'));
 camera.endTrack();
 assert.match(camera.elements.cameraObstacleRepeatabilityStatus.textContent,/Повторяемость препятствия не оценена/);
 assert.equal(camera.obstacleRegistrations.length,0);
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,2,'a restarted camera must request a new stream');
 camera.setVideoTime(0.2);
 await measureObstacleFrame(camera,null);
 assert.ok(camera.elements.cameraObstacleRepeatabilityStatus.textContent.includes('1/3 отдельных кадров'));
 assert.equal(camera.elements.cameraObstacleRepeatabilityStatus.textContent.includes('2/3 отдельных кадров'),false);
 assert.equal(camera.obstacleRegistrations.length,0);
 camera.fire('cameraStop','click');
});

test('camera obstacle measurement cannot reuse one frozen frame',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 await camera.fire('cameraMeasureObstacle','click');
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Этот кадр уже использован/);
 camera.fire('cameraStop','click');
});

test('camera obstacle repeatability rejects dimensions outside operator tolerances',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 const narrower=[{x:160,y:150},{x:440,y:150},{x:440,y:250},{x:160,y:250}];
 await measureObstacleFrame(camera,0.2,narrower);
 await measureObstacleFrame(camera,0.3,narrower);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraObstacleRepeatabilityStatus.textContent,/Разброс превышает допуск/);
 assert.match(camera.elements.cameraStatus.textContent,/не подтверждена/);
 camera.fire('cameraStop','click');
});

test('camera obstacle repeatability locks the manual map pose across frames',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 camera.elements.cameraObstacleX.value='3.5';
 await measureObstacleFrame(camera,0.2);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/изменились внутри серии/);
 camera.fire('cameraStop','click');
});

test('camera obstacle height requires explicit independent measurement and locks its datum through all frames',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await measureObstacleFrame(camera,null);
 camera.elements.cameraObstacleHeight.value='2.8';
 await measureObstacleFrame(camera,0.2);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Высота препятствия или допуски вертикального datum изменились внутри серии/);
 camera.fire('cameraStop','click');

 const unconfirmed=bootCamera({verticalConfirmed:false});
 await unconfirmed.fire('cameraStart','click');
 await measureObstacleFrame(unconfirmed,null);
 assert.equal(unconfirmed.obstacleRegistrations.length,0);
 assert.match(unconfirmed.elements.cameraStatus.textContent,/Подтвердите независимое измерение высоты/);
 unconfirmed.fire('cameraStop','click');
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
 assert.ok(camera.elements.cameraStatus.textContent.includes('Введите название, центр X/Y и направление препятствия'));
 camera.fire('cameraStop','click');
});


test('measured gate height is exposed only as an optional overhead lintel input',()=>{
 const html=fs.readFileSync(path.join(__dirname,'..','index.html'),'utf8');
 const app=fs.readFileSync(path.join(__dirname,'..','truckvision.js'),'utf8');
 for(const id of [
  'gateLintelEnabled','gateLintelAlignmentConfirmed','gateLintelClearHeight','gateLintelCameraHeightUncertainty',
  'gateLintelFloorUncertainty','gateLintelThickness','gateLintelThicknessUncertainty',
  'gateLintelSpan','gateLintelSpanUncertainty','gateLintelDepth','gateLintelDepthUncertainty'
 ]){
  assert.match(html,new RegExp('id="'+id+'"'),'missing lintel control '+id);
 }
 assert.match(app,/registration\.clearHeight/);
 assert.match(app,/registration\.heightUncertaintyBound/);
 assert.match(app,/gateLintelEnabled/);
 assert.match(app,/lintelEnabled\)gate\.lintel=lintel/);
 assert.match(app,/geometry\.lintel=\{/);
 assert.match(app,/planner\.createGateObstacles\(gateGeometry\)/);
 assert.match(html,/вертикальный datum пола/);
});

test('measurement page exposes the complete camera obstacle workflow',()=>{
 const html=fs.readFileSync(path.join(__dirname,'..','index.html'),'utf8');
 for(const id of ['cameraMeasureObstacle','cameraClearObstacles','cameraClearObstacleRepeats','cameraObstacleRepeatabilityWidthTolerance','cameraObstacleRepeatabilityDepthTolerance','cameraObstacleRepeatabilityStatus','cameraObstacleName','cameraObstacleX','cameraObstacleY','cameraObstacleHeading','cameraObstaclePositionUncertaintyX','cameraObstaclePositionUncertaintyY','cameraObstacleHeadingUncertaintyDeg','cameraObstacleHeight','cameraObstacleHeightUncertainty','cameraObstacleBaseDatumUncertainty','cameraObstacleVerticalDatumConfirmed']){
  assert.match(html,new RegExp('id="'+id+'"'),'missing camera obstacle control '+id);
 }
 assert.match(html,/в той же плоскости пола/);
 assert.match(html,/Центр основания X на карте/);
 assert.match(html,/placeholder="Введите направление"/);
 assert.match(html,/трёх отдельных кадров/);
 assert.match(html,/Допустимый разброс ширины основания/);
 assert.match(html,/Камера измеряет только основание/);
 assert.match(html,/независимое ручное измерение/);
 assert.match(html,/откалиброванной плоскости дороги/);
});

test('camera-registered obstacles remain in planner input while live perception is active',()=>{
 const app=fs.readFileSync(path.join(__dirname,'..','truckvision.js'),'utf8');
 const match=app.match(/function sceneObstacles\(\)\s*\{([\s\S]*?)\n\}/);
 assert.ok(match,'sceneObstacles function must exist');
 const body=match[1];
 const cameraPush=body.indexOf('result.push(...cameraObstacles');
 const perceptionPush=body.indexOf('result.push(...perceptionObstacles');
 assert.ok(cameraPush>=0,'camera obstacles must always be appended');
 assert.ok(perceptionPush>cameraPush,'live perception objects must be appended after camera obstacles');
 assert.doesNotMatch(body,/if\s*\(!perceptionFrameTimestamp\)\s*result\.push\(\.\.\.cameraObstacles/);
 assert.match(app,/inflateObstaclePoseUncertainty/);
 assert.match(app,/mapPoseUncertainty/);
 assert.match(app,/Нижние оценки по каждому из 10 колёс/);
});


test('camera obstacle registration requires explicit bounded map-pose uncertainty',async()=>{
 const camera=bootCamera();
 camera.elements.cameraObstaclePositionUncertaintyX.value='';
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/отдельные допуски положения X\/Y и направления/);
 camera.elements.cameraObstaclePositionUncertaintyX.value='0';
 camera.elements.cameraObstaclePositionUncertaintyY.value='0.1';
 camera.elements.cameraObstacleHeadingUncertaintyDeg.value='2';
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Допуски привязки препятствия должны быть положительными/);
 camera.elements.cameraObstaclePositionUncertaintyX.value='0.1';
 camera.elements.cameraObstaclePositionUncertaintyY.value='0';
 camera.elements.cameraObstacleHeadingUncertaintyDeg.value='2';
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Допуски привязки препятствия должны быть положительными/);
 camera.elements.cameraObstaclePositionUncertaintyY.value='0.1';
 camera.elements.cameraObstacleHeadingUncertaintyDeg.value='0';
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Допуски привязки препятствия должны быть положительными/);
 camera.elements.cameraObstacleHeadingUncertaintyDeg.value='46';
 await camera.fire('cameraMeasureObstacle','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.obstacleRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Допуски привязки препятствия должны быть положительными/);
 camera.fire('cameraStop','click');
});


test('ended iPhone camera track clears calibration and requires a new explicit permission tap',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1);
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 assert.match(camera.elements.cameraStatus.textContent,/Плоскость откалибрована/);
 camera.endTrack();
 assert.match(camera.elements.cameraStatus.textContent,/Видеопоток камеры завершился или был прерван/);
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,2,'after an ended track the next start must request permission again');
 camera.fire('cameraStop','click');
 assert.equal(camera.trackStops,1,'the replacement stream must be stopped normally');
});

test('pagehide stops the active camera stream and invalidates the frozen calibration',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 assert.match(camera.elements.cameraStatus.textContent,/Плоскость откалибрована/);
 camera.windowListeners.pagehide[0]();
 assert.equal(camera.trackStops,1,'pagehide must release the active media track');
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
 assert.match(camera.elements.cameraStatus.textContent,/Камера выключена/);
 camera.fire('cameraMeasureGate','click');
 assert.match(camera.elements.cameraStatus.textContent,/Сначала включите камеру/);
});


test('backgrounding the page stops camera, clears calibration and requires an explicit restart',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 assert.match(camera.elements.cameraStatus.textContent,/Плоскость откалибрована/);
 camera.setVisibility('hidden');
 assert.equal(camera.trackStops,1,'hidden page must release the active media track');
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
 assert.match(camera.elements.cameraStatus.textContent,/Страница перешла в фон/);
 camera.setVisibility('visible');
 assert.equal(camera.permissionRequests,1,'returning to the foreground must not restart camera automatically');
 camera.fire('cameraMeasureGate','click');
 assert.match(camera.elements.cameraStatus.textContent,/Сначала включите камеру/);
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,2,'a fresh explicit tap must acquire a new stream');
 assert.match(camera.elements.cameraStatus.textContent,/Камера включена/);
 camera.fire('cameraStop','click');
});


test('backgrounding while camera permission is pending stops a late stream and requires a fresh tap',async()=>{
 let resolvePermission;
 let deferFirstRequest=true;
 const camera=bootCamera({getUserMedia:({createStream})=>{
  if(deferFirstRequest){deferFirstRequest=false;return new Promise(resolve=>{resolvePermission=()=>resolve(createStream());});}
  return Promise.resolve(createStream());
 }});
 const pendingStart=camera.fire('cameraStart','click');
 await Promise.resolve();
 assert.equal(camera.permissionRequests,1);
 camera.setVisibility('hidden');
 assert.match(camera.elements.cameraStatus.textContent,/Страница перешла в фон/);
 camera.setVisibility('visible');
 resolvePermission();
 await pendingStart;
 assert.equal(camera.trackStops,1,'a stream granted after backgrounding must be stopped immediately');
 assert.equal(camera.elements.measureVideo.srcObject??null,null);
 assert.equal(camera.permissionRequests,1,'background return must not reuse the pending request');
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,2,'a new explicit tap must start a fresh permission request');
 assert.match(camera.elements.cameraStatus.textContent,/Камера включена/);
 camera.fire('cameraStop','click');
});

test('rapid repeated camera-start taps while permission is pending issue only one request',async()=>{
 let resolvePermission;
 const camera=bootCamera({getUserMedia:({createStream})=>new Promise(resolve=>{resolvePermission=()=>resolve(createStream());})});
 const firstStart=camera.fire('cameraStart','click');
 await Promise.resolve();
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,1,'startPending must prevent duplicate permission prompts');
 resolvePermission();
 await firstStart;
 assert.match(camera.elements.cameraStatus.textContent,/Камера включена/);
 camera.fire('cameraStop','click');
 assert.equal(camera.trackStops,1);
});


test('manual stop invalidates a pending request without allowing it to replace a newer stream',async()=>{
 let resolveFirstPermission;
 let requestCount=0;
 const camera=bootCamera({getUserMedia:({createStream})=>{
  requestCount++;
  if(requestCount===1)return new Promise(resolve=>{resolveFirstPermission=()=>resolve(createStream());});
  return Promise.resolve(createStream());
 }});
 const firstStart=camera.fire('cameraStart','click');
 await Promise.resolve();
 assert.equal(camera.permissionRequests,1);
 camera.fire('cameraStop','click');
 await camera.fire('cameraStart','click');
 assert.equal(camera.permissionRequests,2);
 const currentStream=camera.elements.measureVideo.srcObject;
 assert.ok(currentStream,'the newer explicit request should own the video element');
 resolveFirstPermission();
 await firstStart;
 assert.equal(camera.elements.measureVideo.srcObject,currentStream,'the stale late stream must not replace the newer stream');
 assert.equal(camera.trackStops,1,'the stale stream must be stopped');
 camera.fire('cameraStop','click');
 assert.equal(camera.trackStops,2,'the current stream must also be released on stop');
});

test('camera refuses a second gate measurement from the same frozen frame',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 await camera.fire('cameraMeasureGate','click');
 assert.match(camera.elements.cameraStatus.textContent,/Этот кадр уже использован/);
 assert.equal(camera.elements.cameraRepeatabilityStatus.textContent.includes('1/3'),true);
 assert.equal(camera.appliedRegistrations.length,0);
 camera.fire('cameraStop','click');
});

test('camera gate is registered only after three independent frames within operator tolerances',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 for(const [index,time] of [0.1,0.2,0.3].entries()){
  if(index)camera.fire('cameraReset','click');
  camera.setVideoTime(time);
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gatePoints);
  assert.equal(camera.appliedRegistrations.length,index===2?1:0);
 }
 assert.equal(camera.appliedRegistrations[0].repeatability.count,3);
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/Допуск повторяемости выполнен/);
 camera.fire('cameraStop','click');
});

test('camera gate repeatability failure never overwrites the previous gate registration',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gates=[
  [{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}],
  [{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}],
  [{x:150,y:150},{x:400,y:150},{x:400,y:250},{x:150,y:250}]
 ];
 for(let i=0;i<3;i++){
  if(i)camera.fire('cameraReset','click');
  camera.setVideoTime(0.1+i*0.1);
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gates[i]);
 }
 assert.equal(camera.appliedRegistrations.length,0);
 assert.equal(camera.elements.gateWidth.value,'3.50');
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/Разброс превышает операторский допуск/);
 assert.match(camera.elements.gateResult.textContent,/Повторяемость не подтверждена/);
 camera.fire('cameraStop','click');
});

test('camera repeatability series cannot mix operator tolerances',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 camera.fire('cameraReset','click');
 camera.setVideoTime(0.2);
 camera.elements.cameraRepeatabilityWidthTolerance.value='0.03';
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.equal(camera.appliedRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Допуски повторяемости изменились/);
 camera.fire('cameraStop','click');
});



test('changing the reference dimensions after a reset clears the repeatability series',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 camera.fire('cameraReset','click');
 camera.elements.cameraReferenceWidth.value='3.000';
 camera.fire('cameraReferenceWidth','input');
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/Повторяемость не оценена/);
 assert.match(camera.elements.cameraStatus.textContent,/Размер эталона изменён/);
 camera.setVideoTime(0.2);
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 camera.fire('cameraStop','click');
});

test('repeat series rejects silently changed reference-size assumptions even without an input event',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 camera.fire('cameraReset','click');
 camera.setVideoTime(0.2);
 camera.elements.cameraReferenceWidth.value='3.000';
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.match(camera.elements.cameraStatus.textContent,/Размер эталона или допущения неопределённости изменились/);
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/1\/3 отдельных кадров/);
 assert.equal(camera.appliedRegistrations.length,0);
 camera.fire('cameraStop','click');
});

test('changing camera uncertainty assumptions clears the repeatability series',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 camera.fire('cameraReset','click');
 camera.elements.cameraPointSigma.value='3';
 camera.fire('cameraPointSigma','input');
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/Повторяемость не оценена/);
 assert.match(camera.elements.cameraStatus.textContent,/Допущения неопределённости изменены/);
 camera.fire('cameraStop','click');
});

test('camera refuses a repeat capture until the video time advances',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 camera.fire('cameraReset','click');
 await camera.fire('cameraCalibrate','click');
 assert.match(camera.elements.cameraStatus.textContent,/Дождитесь следующего видеокадра/);
 assert.equal(camera.elements.measureVideo.paused,false);
 camera.fire('cameraStop','click');
});

test('clearing repeat readings does not allow reuse of the same frozen frame',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 camera.fire('cameraClearGateRepeats','click');
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/Повторяемость не оценена/);
 await camera.fire('cameraMeasureGate','click');
 assert.match(camera.elements.cameraStatus.textContent,/Этот кадр уже использован/);
 camera.fire('cameraStop','click');
});

test('camera gate repeatability requires explicit operator tolerances before collecting a reading',async()=>{
 const camera=bootCamera();
 camera.elements.cameraRepeatabilityWidthTolerance.value='';
 await camera.fire('cameraStart','click');
 await camera.fire('cameraCalibrate','click');
 await camera.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(camera.appliedRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/операторские допуски разброса/);
 assert.match(camera.elements.cameraRepeatabilityStatus.textContent,/Повторяемость не оценена/);
 camera.fire('cameraStop','click');
});

test('camera gate registration requires positive manual position and heading uncertainty',async()=>{
 const missing=bootCamera();
 missing.elements.gatePositionUncertainty.value='';
 await missing.fire('cameraStart','click');
 await missing.fire('cameraCalibrate','click');
 await missing.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await missing.fire('cameraMeasureGate','click');
 await missing.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(missing.appliedRegistrations.length,0);
 assert.match(missing.elements.cameraStatus.textContent,/Введите положительные допуски ручного положения и направления ворот/);
 missing.fire('cameraStop','click');

 const zeroPosition=bootCamera();
 zeroPosition.elements.gatePositionUncertainty.value='0';
 await zeroPosition.fire('cameraStart','click');
 await zeroPosition.fire('cameraCalibrate','click');
 await zeroPosition.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await zeroPosition.fire('cameraMeasureGate','click');
 await zeroPosition.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(zeroPosition.appliedRegistrations.length,0);
 assert.match(zeroPosition.elements.cameraStatus.textContent,/Допуски привязки ворот должны быть положительными/);
 zeroPosition.fire('cameraStop','click');

 const zero=bootCamera();
 zero.elements.gateHeadingUncertaintyDeg.value='0';
 await zero.fire('cameraStart','click');
 await zero.fire('cameraCalibrate','click');
 await zero.tap([{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}]);
 await zero.fire('cameraMeasureGate','click');
 await zero.tap([{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}]);
 assert.equal(zero.appliedRegistrations.length,0);
 assert.match(zero.elements.cameraStatus.textContent,/Допуски привязки ворот должны быть положительными/);
 zero.fire('cameraStop','click');
});

test('camera gate repeatability series cannot mix manually entered pose uncertainty bounds',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 camera.fire('cameraReset','click');
 camera.setVideoTime(0.2);
 camera.elements.gatePositionUncertainty.value='0.08';
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.equal(camera.appliedRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/допуски привязки ворот изменились внутри серии/);
 camera.fire('cameraStop','click');
});

test('camera measurement UI declares positive minimum map-pose uncertainty',()=>{
 const html=fs.readFileSync(path.join(__dirname,'..','index.html'),'utf8');
 for(const id of ['gateWidthUncertainty','gatePositionUncertainty','gateHeadingUncertaintyDeg','gatePostWidthUncertainty','gateDepthUncertainty','gateLintelCameraHeightUncertainty','gateLintelFloorUncertainty','gateLintelThicknessUncertainty','gateLintelSpanUncertainty','gateLintelDepthUncertainty','cameraObstaclePositionUncertaintyX','cameraObstaclePositionUncertaintyY','cameraObstacleHeadingUncertaintyDeg']){
  const input=html.match(new RegExp('<input[^>]*id="'+id+'"[^>]*>'));
  assert.ok(input,'missing input '+id);
  assert.match(input[0],/min="0\.001"/,'input '+id+' must reject zero uncertainty');
 }
});

test('camera gate measurement requires positive jamb thickness and depth bounds',async()=>{
 for(const field of ['gatePostWidthUncertainty','gateDepthUncertainty']){
  const camera=bootCamera();
  camera.elements[field].value='0';
  await camera.fire('cameraStart','click');
  const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
  const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gatePoints);
  assert.equal(camera.appliedRegistrations.length,0,field+' must block camera gate registration');
  assert.match(camera.elements.cameraStatus.textContent,/Толщина косяка должна быть/);
  camera.fire('cameraStop','click');
 }
});

test('camera gate repeatability series cannot mix manually entered jamb geometry or bounds',async()=>{
 const changes=[
  ['gatePostWidth','0.3'],['gateDepth','0.7'],
  ['gatePostWidthUncertainty','0.04'],['gateDepthUncertainty','0.04']
 ];
 for(const [field,value] of changes){
  const camera=bootCamera();
  await camera.fire('cameraStart','click');
  const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
  const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gatePoints);
  camera.fire('cameraReset','click');
  camera.setVideoTime(0.2);
  camera.elements[field].value=value;
  await camera.fire('cameraCalibrate','click');
  await camera.tap(referencePoints);
  await camera.fire('cameraMeasureGate','click');
  await camera.tap(gatePoints);
  assert.equal(camera.appliedRegistrations.length,0,field+' must invalidate the repeatability series');
  assert.match(camera.elements.cameraStatus.textContent,/геометрия\/допуски косяков ворот изменились внутри серии/);
  camera.fire('cameraStop','click');
 }
});

test('camera gate repeatability series cannot mix manually entered map poses',async()=>{
 const camera=bootCamera();
 await camera.fire('cameraStart','click');
 const referencePoints=[{x:100,y:100},{x:500,y:100},{x:500,y:300},{x:100,y:300}];
 const gatePoints=[{x:150,y:150},{x:450,y:150},{x:450,y:250},{x:150,y:250}];
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 camera.fire('cameraReset','click');
 camera.setVideoTime(0.2);
 camera.elements.gateCenterX.value='13';
 await camera.fire('cameraCalibrate','click');
 await camera.tap(referencePoints);
 await camera.fire('cameraMeasureGate','click');
 await camera.tap(gatePoints);
 assert.equal(camera.appliedRegistrations.length,0);
 assert.match(camera.elements.cameraStatus.textContent,/Координаты, направление или допуски привязки ворот изменились/);
 camera.fire('cameraStop','click');
});
