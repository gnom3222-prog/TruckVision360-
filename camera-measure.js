(()=>{'use strict';
const $=id=>document.getElementById(id),v=$('measureVideo'),c=$('measureOverlay'),ctx=c.getContext('2d'),status=$('cameraStatus'),geometry=window.TruckVisionCameraGeometry;
let stream=null,streamRequestId=0,startPending=false,mode=null,pts=[],Hcal=null,referencePoints=null,referenceWidth=0,referenceHeight=0,fw=0,fh=0,frameLocked=false,frameCaptureId=0,frameCaptureTime=null,lastGateMeasurementFrameId=-1,lastGateFrameTime=null,gateRepeatabilityTolerances=null,gateRepeatabilityComplete=false;
const gateReadings=[];
const say=s=>status.textContent=s,dist=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
function readMeasurementUncertainty(){
 const pointText=$('cameraPointSigma').value.trim();
 const referenceText=$('cameraReferenceSigma').value.trim();
 if(!pointText||!referenceText)throw new Error('Введите положительную неопределённость отметки угла и погрешность размера эталона. Пустые поля не считаются нулевой погрешностью.');
 const sigmaPixels=Number(pointText),referenceSigmaMm=Number(referenceText);
 if(!Number.isFinite(sigmaPixels)||sigmaPixels<0.5||sigmaPixels>20||
    !Number.isFinite(referenceSigmaMm)||referenceSigmaMm<0.1||referenceSigmaMm>50){
  throw new Error('Допуски измерения должны быть в пределах 0,5–20 px для отметки угла и 0,1–50 мм для размера эталона.');
 }
 return {sigmaPixels,sigmaReference:referenceSigmaMm/1000};
}
function readGateRepeatabilityTolerance(){
 const widthText=$('cameraRepeatabilityWidthTolerance').value.trim();
 const heightText=$('cameraRepeatabilityHeightTolerance').value.trim();
 if(!widthText||!heightText)throw new Error('Введите операторские допуски разброса ширины и высоты для серии повторных измерений.');
 const widthTolerance=Number(widthText),heightTolerance=Number(heightText);
 if(!Number.isFinite(widthTolerance)||widthTolerance<0.001||widthTolerance>0.5||
    !Number.isFinite(heightTolerance)||heightTolerance<0.001||heightTolerance>0.5){
  throw new Error('Допуски повторяемости должны быть в диапазоне 0,001–0,5 м.');
 }
 return {widthTolerance,heightTolerance};
}
function clearGateRepeatability(resetFrameTracking=false){
 gateReadings.length=0;
 gateRepeatabilityTolerances=null;
 gateRepeatabilityComplete=false;
 if(resetFrameTracking){
  frameCaptureId=0;frameCaptureTime=null;lastGateMeasurementFrameId=-1;lastGateFrameTime=null;
 }
 const output=$('cameraRepeatabilityStatus');
 if(output)output.textContent='Повторяемость не оценена. Нужны минимум три отдельно зафиксированных кадра.';
}
function lockFrame(){
 if(frameLocked)return true;
 if(!stream||!v.videoWidth||!v.videoHeight)return false;
 const captureTime=Number(v.currentTime);
 if(!Number.isFinite(captureTime)){
  say('Браузер не сообщил время видеокадра. Дождитесь стабильного изображения и повторите калибровку.');
  return false;
 }
 if(lastGateFrameTime!==null&&captureTime<=lastGateFrameTime+0.005){
  say('Дождитесь следующего видеокадра после сброса калибровки.');
  return false;
 }
 v.pause();frameLocked=true;frameCaptureId++;frameCaptureTime=captureTime;return true;
}
function resumeFrame(){if(!frameLocked)return;frameLocked=false;if(stream)v.play().catch(()=>say('Не удалось возобновить видеопоток. Выключите и снова включите камеру.'));}
function draw(){if(!v.videoWidth)return;const w=v.videoWidth,h=v.videoHeight;if(c.width!==w||c.height!==h){if(fw&&(w!==fw||h!==fh)){Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];clearGateRepeatability(true);resumeFrame();say('Изменился размер видеокадра. Калибровка и серия повторов сброшены; повторите её.');}c.width=w;c.height=h;fw=w;fh=h;}c.style.left=v.offsetLeft+'px';c.style.top=v.offsetTop+'px';c.style.width=v.clientWidth+'px';c.style.height=v.clientHeight+'px';ctx.clearRect(0,0,w,h);if(pts.length){ctx.strokeStyle='#ffe082';ctx.fillStyle='#ffe082';ctx.lineWidth=Math.max(3,w/500);ctx.beginPath();pts.forEach((p,i)=>i?ctx.lineTo(p.x,p.y):ctx.moveTo(p.x,p.y));if(pts.length===4)ctx.closePath();ctx.stroke();pts.forEach((p,i)=>{ctx.beginPath();ctx.arc(p.x,p.y,Math.max(8,w/160),0,7);ctx.fill();ctx.fillStyle='#081321';ctx.font='bold '+Math.max(16,w/40)+'px system-ui';ctx.fillText((mode==='reference'?['1','2','3','4']:['A','B','C','D'])[i],p.x+8,p.y-8);ctx.fillStyle='#ffe082';});}}
async function start(){
 if(stream||startPending)return;
 if(document.visibilityState==='hidden'){say('Страница находится в фоне. Вернитесь в приложение и нажмите «Включить камеру».');return;}
 if(!geometry){say('Модуль геометрии камеры не загрузился. Обновите страницу и повторите попытку.');return;}
 if(!isSecureContext||!navigator.mediaDevices?.getUserMedia){say('Камера доступна только на защищённой HTTPS-странице в Safari.');return;}
 const requestId=++streamRequestId;
 startPending=true;
 let acquiredStream=null;
 try{
  acquiredStream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'},width:{ideal:1920},height:{ideal:1080}}});
  if(requestId!==streamRequestId||document.visibilityState==='hidden'){
   acquiredStream.getTracks().forEach(t=>t.stop());
   if(document.visibilityState==='hidden')say('Страница перешла в фон. Запрошенный поток остановлен; вернитесь и нажмите «Включить камеру».');
   return;
  }
  stream=acquiredStream;
  const activeStream=stream;
  const handleEnded=()=>{
   if(stream!==activeStream)return;
   stream=null;
   streamRequestId++;
   startPending=false;
   v.pause();
   v.srcObject=null;
   Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];frameLocked=false;
   ctx.clearRect(0,0,c.width,c.height);
   clearGateRepeatability(true);
   say('Видеопоток камеры завершился или был прерван. Калибровка сброшена; нажмите «Включить камеру» и повторите измерение.');
  };
  for(const track of activeStream.getTracks()){
   if(typeof track.addEventListener==='function')track.addEventListener('ended',handleEnded,{once:true});
   else track.onended=handleEnded;
  }
  v.srcObject=activeStream;
  await v.play();
  if(stream!==activeStream||requestId!==streamRequestId)throw new Error('Видеопоток камеры завершился до запуска.');
  Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];frameLocked=false;clearGateRepeatability(true);
  say('Камера включена. Установите прямоугольный эталон в ту же плоскость, что и объект. При нажатии «Калибровать» текущий кадр будет зафиксирован: отметьте эталон и объект на одном неподвижном изображении.');
  draw();
 }catch(e){
  if(requestId!==streamRequestId){
   acquiredStream?.getTracks().forEach(t=>t.stop());
   return;
  }
  const failedStream=stream;stream=null;failedStream?.getTracks().forEach(t=>t.stop());v.srcObject=null;
  const messages={
   NotAllowedError:'Доступ к камере запрещён. Разрешите камеру для этого сайта в настройках Safari и нажмите «Включить камеру» ещё раз.',
   NotFoundError:'Камера не найдена. Проверьте, что устройство имеет доступную камеру.',
   NotReadableError:'Камера занята другим приложением или недоступна. Закройте другие приложения, использующие камеру.',
   OverconstrainedError:'Камера не поддерживает запрошенное разрешение. Повторите запуск.',
   SecurityError:'Браузер заблокировал камеру. Откройте приложение по HTTPS и проверьте разрешения сайта.'
  };
  say(messages[e.name]||('Не удалось включить камеру: '+(e.message||e.name)));
 }finally{
  if(requestId===streamRequestId)startPending=false;
 }
}
function stop(){streamRequestId++;startPending=false;const activeStream=stream;stream=null;v.pause();activeStream?.getTracks().forEach(t=>t.stop());v.srcObject=null;Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];frameLocked=false;clearGateRepeatability(true);ctx.clearRect(0,0,c.width,c.height);say('Камера выключена. При следующем запуске нужна новая калибровка.');}
function choose(m){
 if(!stream||!v.videoWidth){say('Сначала включите камеру.');return;}
 if((m==='gate'||m==='obstacle')&&!Hcal){say('Сначала выполните калибровку.');return;}
 if(m==='gate'&&gateRepeatabilityComplete){say('Серия ворот уже завершена. Очистите повторные измерения для новой серии.');return;}
 if(m==='gate'&&lastGateMeasurementFrameId===frameCaptureId){say('Этот кадр уже использован. Сбросьте калибровку, дождитесь нового кадра и повторите измерение.');return;}
 if(m==='reference'){
  Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;
  if(!lockFrame())return;
 }else if(!frameLocked){
  say('Кадр измерения не зафиксирован. Сбросьте калибровку и выполните её заново.');return;
 }
 mode=m;pts=[];draw();
 say(m==='reference'
  ?'Кадр зафиксирован. Коснитесь четырёх углов эталона по часовой стрелке: верхний левый, верхний правый, нижний правый, нижний левый. Не перемещайте телефон.'
  :m==='obstacle'
   ?'Отметьте четыре угла основания препятствия на том же зафиксированном кадре. Эталон и основание должны находиться в одной плоскости пола.'
   :'Отметьте четыре угла проёма на том же зафиксированном кадре. Все точки эталона и ворот будут взяты из одного изображения.');
}
c.addEventListener('pointerdown',e=>{if(!stream||!mode)return;let r=c.getBoundingClientRect();pts.push({x:(e.clientX-r.left)*c.width/r.width,y:(e.clientY-r.top)*c.height/r.height});draw();if(pts.length!==4)return;try{if(mode==='reference'){let W=+$('cameraReferenceWidth').value,H=+$('cameraReferenceHeight').value;if(!(W>.05&&H>.05&&W<=20&&H<=20))throw Error('Введите ширину и высоту эталона от 0,05 до 20 м.');if(!geometry.validQuad(pts))throw Error('Углы эталона должны образовывать чёткий прямоугольник по часовой стрелке.');Hcal=geometry.homography(pts,W,H);referencePoints=pts.map(p=>({...p}));referenceWidth=W;referenceHeight=H;mode=null;pts=[];say('Плоскость откалибрована. Не двигайте телефон и эталон. Теперь выберите измерение ворот или основания препятствия в этой же плоскости.');}else if(mode==='obstacle'){
if(c.width!==fw||c.height!==fh)throw Error('Размер кадра изменился. Повторите калибровку.');
if(!geometry.validQuad(pts,1000,20))throw Error('Четыре угла основания препятствия должны образовывать чёткий четырёхугольник по часовой стрелке.');
const {sigmaPixels,sigmaReference}=readMeasurementUncertainty();
const measurement=geometry.estimateUncertainty({referencePoints,referenceWidth,referenceHeight,measuredPoints:pts,sigmaPixels,sigmaReferenceWidth:sigmaReference,sigmaReferenceHeight:sigmaReference});
if(measurement.conditioningWarning)throw Error('Калибровка численно неустойчива. Увеличьте эталон в кадре и повторите.');
if(measurement.shapeWarning)throw Error('Основание объекта не похоже на прямоугольник. Проверьте точки, плоскость пола и эталон.');
if(measurement.nonlinearSensitivityAvailable!==true||measurement.nonlinearWarning===true)throw Error('Нелинейная проверка неопределённости не пройдена. Основание не добавлено на карту; повторите калибровку с более крупным эталоном и устойчивым ракурсом.');
const xText=$('cameraObstacleX').value.trim(),yText=$('cameraObstacleY').value.trim(),headingText=$('cameraObstacleHeading').value.trim();
const xUncertaintyText=$('cameraObstaclePositionUncertaintyX').value.trim(),yUncertaintyText=$('cameraObstaclePositionUncertaintyY').value.trim(),headingUncertaintyText=$('cameraObstacleHeadingUncertaintyDeg').value.trim();
if(!xText||!yText||!headingText)throw Error('Введите координаты центра и направление препятствия на карте.');
if(!xUncertaintyText||!yUncertaintyText||!headingUncertaintyText)throw Error('Введите отдельные допуски положения X/Y и направления препятствия на карте. Нулевой допуск задавайте только при наличии независимого подтверждения.');
const anchor={x:Number(xText),y:Number(yText)};
const headingDeg=Number(headingText);
const poseUncertainty={positionX:Number(xUncertaintyText),positionY:Number(yUncertaintyText),headingDeg:Number(headingUncertaintyText)};
if(![anchor.x,anchor.y,headingDeg].every(Number.isFinite)||Math.abs(anchor.x)>100||Math.abs(anchor.y)>100||headingDeg< -180||headingDeg>180)throw Error('Проверьте координаты центра и ориентацию препятствия на карте.');
if(![poseUncertainty.positionX,poseUncertainty.positionY,poseUncertainty.headingDeg].every(Number.isFinite)||poseUncertainty.positionX<0||poseUncertainty.positionX>2||poseUncertainty.positionY<0||poseUncertainty.positionY>2||poseUncertainty.headingDeg<0||poseUncertainty.headingDeg>45)throw Error('Допуски привязки должны быть 0–2 м по X/Y и 0–45° по направлению.');
const applyObstacle=window.TruckVisionAddCameraObstacle;
if(typeof applyObstacle!=='function'||applyObstacle(measurement,anchor,headingDeg*Math.PI/180,$('cameraObstacleName').value,poseUncertainty)!==true)throw Error('Препятствие не прошло геометрическую проверку и не добавлено на карту.');
say('Препятствие добавлено на карту: размеры '+measurement.width.toFixed(2)+' × '+measurement.height.toFixed(2)+' м; чувствительность размеров ±'+measurement.sigmaWidth.toFixed(3)+' / ±'+measurement.sigmaHeight.toFixed(3)+' м (1σ). Нелинейная осевая проверка ±1σ выполнена; расхождение ширины '+(100*Number(measurement.nonlinearMismatchWidth||0)).toFixed(1)+'%, асимметрия '+(100*Number(measurement.maxAxisAsymmetryRatio||0)).toFixed(1)+'%. Отдельные границы ручной привязки: X ±'+poseUncertainty.positionX.toFixed(2)+' м, Y ±'+poseUncertainty.positionY.toFixed(2)+' м, курс ±'+poseUncertainty.headingDeg.toFixed(1)+'°. Это заданные абсолютные границы, не статистика камеры. Перестройте маршрут.');
}else{if(c.width!==fw||c.height!==fh)throw Error('Размер кадра изменился. Повторите калибровку.');if(!geometry.validQuad(pts,1000,20))throw Error('Углы ворот должны образовывать чёткий четырёхугольник по часовой стрелке.');const {sigmaPixels,sigmaReference}=readMeasurementUncertainty();const measurement=geometry.estimateUncertainty({referencePoints,referenceWidth,referenceHeight,measuredPoints:pts,sigmaPixels,sigmaReferenceWidth:sigmaReference,sigmaReferenceHeight:sigmaReference});const w=measurement.width,h=measurement.height;const conditioningWarning=measurement.conditioningWarning===true;const nonlinearWarning=measurement.nonlinearSensitivityAvailable!==true||measurement.nonlinearWarning===true;const shape=measurement.shapeDiagnostics||{};const shapeWarning=measurement.shapeWarning===true;const nonlinearText=' Нелинейная проверка ±1σ: '+(measurement.nonlinearSensitivityAvailable?'выполнена; расхождение линейной и конечной оценки ширины '+(100*Number(measurement.nonlinearMismatchWidth||0)).toFixed(1)+'%, нормированный чётный сдвиг '+(100*Number(measurement.maxAxisAsymmetryRatio||0)).toFixed(1)+'%, порог 25%.':'недоступна; результат не переносится на карту.');const shapeText=' Геометрия проёма: разброс ширины '+(100*Number(shape.widthSpreadRatio||0)).toFixed(1)+'%, разброс высоты '+(100*Number(shape.heightSpreadRatio||0)).toFixed(1)+'%, отклонение угла '+Number(shape.maxOrthogonalityErrorDeg||0).toFixed(1)+'°, расхождение диагоналей '+(100*Number(shape.diagonalSpreadRatio||0)).toFixed(1)+'%.';const dimensions='Предварительный результат: ширина ≈ '+w.toFixed(2)+' м (локальная оценка чувствительности ±'+measurement.sigmaWidth.toFixed(3)+' м, 1σ); высота ≈ '+h.toFixed(2)+' м (±'+measurement.sigmaHeight.toFixed(3)+' м, 1σ).'+shapeText+nonlinearText;if(conditioningWarning){$('gateResult').textContent='Калибровка численно неустойчива. '+dimensions+' Результат не передан в настройки ворот. Увеличьте эталон в кадре, измените угол съёмки и выполните калибровку заново.';say('Калибровка плохо обусловлена. Предварительные размеры показаны только для диагностики; ширина ворот не обновлена. Увеличьте площадь эталона в кадре и повторите калибровку.');}else if(nonlinearWarning){$('gateResult').textContent='Нелинейная проверка неопределённости не прошла. '+dimensions+' Результат не передан в настройки ворот. Повторите калибровку с более крупным эталоном, меньшей перспективной деформацией или обоснованно меньшей погрешностью отметки.';say('Нелинейная проверка погрешности не пройдена. Ширина ворот не обновлена; повторите калибровку и измерение.');}else if(shapeWarning){$('gateResult').textContent='Форма измеренного проёма не соответствует прямоугольнику в заданных пределах. '+dimensions+' Результат не передан в настройки ворот. Проверьте порядок четырёх углов, плоскость эталона и проёма, затем повторите измерение.';say('Форма измеренного проёма заметно отличается от прямоугольника. Ширина ворот не обновлена; проверьте четыре угла и положение эталона в той же плоскости.');}else{
 if(gateReadings.length>=10)throw Error('Достигнут предел 10 повторных кадров. Очистите серию и начните заново.');
 const tolerances=readGateRepeatabilityTolerance();
 if(gateRepeatabilityTolerances&&
    (Math.abs(tolerances.widthTolerance-gateRepeatabilityTolerances.widthTolerance)>1e-12||
     Math.abs(tolerances.heightTolerance-gateRepeatabilityTolerances.heightTolerance)>1e-12)){
  throw Error('Допуски повторяемости изменились внутри серии. Очистите серию и начните заново.');
 }
 if(!gateRepeatabilityTolerances)gateRepeatabilityTolerances=tolerances;
 gateReadings.push({width:w,height:h,sigmaWidth:measurement.sigmaWidth,sigmaHeight:measurement.sigmaHeight});
 lastGateMeasurementFrameId=frameCaptureId;
 lastGateFrameTime=frameCaptureTime;
 if(gateReadings.length<3){
  $('cameraRepeatabilityStatus').textContent='Собрано '+gateReadings.length+'/3 отдельных кадров. Для следующего повтора нажмите «Сбросить калибровку», дождитесь нового кадра и заново отметьте эталон и ворота.';
  $('gateResult').textContent=dimensions+' Повторяемость пока не оценена: собрано '+gateReadings.length+' из 3 отдельных кадров. На карту ничего не перенесено.';
  say('Повтор '+gateReadings.length+' из 3 сохранён только в текущей серии. Сбросьте калибровку, дождитесь нового кадра и повторите калибровку эталона и ворот.');
  mode=null;pts=[];draw();return;
 }
 const repeatability=geometry.summarizeRepeatability(gateReadings,gateRepeatabilityTolerances);
 const repeatabilityText=' Повторяемость по '+repeatability.count+' отдельным кадрам: средняя ширина '+repeatability.meanWidth.toFixed(3)+' м, диапазон '+repeatability.rangeWidth.toFixed(3)+' м, выборочное s '+repeatability.sampleStdDevWidth.toFixed(3)+' м (допуск '+repeatability.widthTolerance.toFixed(3)+' м); средняя высота '+repeatability.meanHeight.toFixed(3)+' м, диапазон '+repeatability.rangeHeight.toFixed(3)+' м, выборочное s '+repeatability.sampleStdDevHeight.toFixed(3)+' м (допуск '+repeatability.heightTolerance.toFixed(3)+' м). Screening-оболочки чувствительности: ширина ±'+repeatability.widthEnvelopeBound.toFixed(3)+' м, высота ±'+repeatability.heightEnvelopeBound.toFixed(3)+' м.';
 $('cameraRepeatabilityStatus').textContent=repeatabilityText+(repeatability.withinTolerance?' Допуск повторяемости выполнен; карта будет обновлена с консервативной оболочкой.':' Разброс превышает операторский допуск. Карта не обновлена; очистите серию и повторите измерения.');
 if(!repeatability.withinTolerance){
  gateRepeatabilityComplete=true;
  $('gateResult').textContent=dimensions+repeatabilityText+' Повторяемость не подтверждена; результат не передан в настройки ворот.';
  say('Повторяемость не подтверждена. Разброс ширины или высоты превышает введённый допуск. Ворота не обновлены; очистите серию и повторите измерения.');
  mode=null;pts=[];draw();return;
 }
 const anchor={x:Number($('gateCenterX').value),y:Number($('gateCenterY').value)};
 const headingDeg=Number($('gateHeadingDeg').value);
 const registration=geometry.registerGateOnMap(measurement,anchor,headingDeg*Math.PI/180,repeatability);
 const applyRegistration=window.TruckVisionApplyGateRegistration;
 if(typeof applyRegistration!=='function')throw Error('Привязка ворот к карте недоступна. Обновите приложение и повторите измерение.');
 if(applyRegistration(registration)!==true)throw Error('Привязка ворот не прошла проверку. Координаты и ширина не применены.');
 gateRepeatabilityComplete=true;
 $('gateResult').textContent=dimensions+repeatabilityText+' Измеренная средняя ширина, вручную заданные координаты центра ('+registration.centerX.toFixed(2)+', '+registration.centerY.toFixed(2)+' м) и курс линии ворот ('+registration.headingDeg.toFixed(1)+'°) перенесены в карту. Косяки ворот включены как препятствия. Ширина оболочки ±'+registration.widthUncertaintyBound.toFixed(3)+' м (наблюдаемый разброс плюс максимальная оценка 2σ на кадр, screening-оценка, не жёсткая граница); расчётный чистый проём сужен до '+(registration.clearWidth-registration.widthUncertaintyBound).toFixed(3)+' м. Центр и курс не определяются камерой — их нужно задать в разделе «Ворота» и указать их допуски. Перепроверьте рулеткой.';
 say('Измерение применено к карте: средняя ширина '+registration.clearWidth.toFixed(2)+' м; центр ('+registration.centerX.toFixed(2)+', '+registration.centerY.toFixed(2)+' м); направление линии '+registration.headingDeg.toFixed(1)+'°. Косяки включены. Оценка чувствительности (1σ): ширина ±'+measurement.sigmaWidth.toFixed(3)+' м; высота ±'+measurement.sigmaHeight.toFixed(3)+' м. Нелинейная осевая проверка ±1σ выполнена; расхождение ширины '+(100*Number(measurement.nonlinearMismatchWidth||0)).toFixed(1)+'%, асимметрия '+(100*Number(measurement.maxAxisAsymmetryRatio||0)).toFixed(1)+'%. Для геометрии ворот применена отдельная оболочка ширины ±'+registration.widthUncertaintyBound.toFixed(3)+' м (наблюдаемый разброс плюс максимальная оценка 2σ на кадр; screening-оценка, не жёсткая граница); расчётный чистый проём сужен до '+(registration.clearWidth-registration.widthUncertaintyBound).toFixed(3)+' м. Положение и курс введены вручную, их погрешность не определена камерой. Это не полная точность камеры и не гарантия безопасного проезда.');
}mode=null;pts=[];}}catch(err){const failedMode=mode;mode=null;pts=[];const unchanged=failedMode==='obstacle'?'Препятствие не добавлено. Проверьте координаты и повторите измерение.':'Ширина ворот не обновлена; проверьте актуальное значение рулеткой.';say((err.message||'Ошибка измерения. Повторите калибровку.')+' '+unchanged);}draw();});
v.addEventListener('resize',draw);window.addEventListener('resize',draw);window.addEventListener('orientationchange',()=>{Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];clearGateRepeatability(true);resumeFrame();draw();say('Ориентация телефона изменилась. Калибровка и серия повторов сброшены. Повторите калибровку.');});['cameraReferenceWidth','cameraReferenceHeight'].forEach(id=>$(id).addEventListener('input',()=>{if(Hcal){Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];clearGateRepeatability(false);draw();say('Размер эталона изменён. Серия повторов и калибровка сброшены; повторите калибровку.');}}));$('cameraStart').addEventListener('click',start);$('cameraStop').addEventListener('click',stop);$('cameraCalibrate').addEventListener('click',()=>choose('reference'));$('cameraMeasureGate').addEventListener('click',()=>choose('gate'));$('cameraClearGateRepeats')?.addEventListener('click',()=>{clearGateRepeatability(false);say('Серия повторных измерений очищена. Для нового кадра сбросьте калибровку и дождитесь обновления изображения.');});$('cameraMeasureObstacle')?.addEventListener('click',()=>choose('obstacle'));$('cameraClearObstacles')?.addEventListener('click',()=>{const clear=window.TruckVisionClearCameraObstacles;if(typeof clear==='function'&&clear()===true)say('Добавленные камерой препятствия удалены с карты.');});$('cameraReset').addEventListener('click',()=>{Hcal=null;referencePoints=null;referenceWidth=0;referenceHeight=0;mode=null;pts=[];draw();resumeFrame();say(stream?'Калибровка сброшена; видеопоток возобновлён. Серия повторов сохранена. Дождитесь нового кадра и выполните калибровку заново.':'Калибровка сброшена.');});v.addEventListener('loadedmetadata',draw);document.addEventListener('visibilitychange',()=>{if(document.visibilityState==='hidden'&&(stream||startPending)){stop();say('Страница перешла в фон. Камера остановлена, калибровка сброшена; вернитесь и нажмите «Включить камеру».');}});window.addEventListener('pagehide',stop);
})();