(()=>{
'use strict';
const $=id=>document.getElementById(id);
const video=$('measureVideo'),canvas=$('measureOverlay'),ctx=canvas.getContext('2d'),status=$('cameraStatus');
let stream=null,mode=null,points=[],calibration=null,lastWidth=0,lastHeight=0;
const say=message=>{status.textContent=message;};
const distance=(a,b)=>Math.hypot(b.x-a.x,b.y-a.y);
function draw(){
 if(!video.videoWidth||!video.videoHeight)return;
 const w=video.videoWidth,h=video.videoHeight;
 if(canvas.width!==w||canvas.height!==h){
  if(lastWidth&&calibration&&(w!==lastWidth||h!==lastHeight)){calibration=null;mode=null;points=[];say('Разрешение или ориентация видео изменились. Выполните калибровку заново.');}
  canvas.width=w;canvas.height=h;lastWidth=w;lastHeight=h;
 }
 ctx.clearRect(0,0,w,h);
 if(points.length){
  ctx.lineWidth=Math.max(3,w/320);ctx.strokeStyle='#ffe082';ctx.fillStyle='#ffe082';
  if(points.length===2){ctx.beginPath();ctx.moveTo(points[0].x,points[0].y);ctx.lineTo(points[1].x,points[1].y);ctx.stroke();}
  points.forEach((p,i)=>{ctx.beginPath();ctx.arc(p.x,p.y,Math.max(7,w/180),0,Math.PI*2);ctx.fill();ctx.fillStyle='#081321';ctx.font='bold '+Math.max(14,w/45)+'px system-ui';ctx.fillText(String(i+1),p.x+8,p.y-8);ctx.fillStyle='#ffe082';});
 }
}
function clearCalibration(){calibration=null;mode=null;points=[];draw();say('Калибровка сброшена. Для измерения сначала задайте эталон известной длины.');}
async function startCamera(){
 if(stream)return;
 if(!window.isSecureContext||!navigator.mediaDevices||!navigator.mediaDevices.getUserMedia){say('Камера недоступна: откройте сайт по HTTPS в Safari и проверьте поддержку доступа к камере.');return;}
 try{
  stream=await navigator.mediaDevices.getUserMedia({audio:false,video:{facingMode:{ideal:'environment'}}});
  video.srcObject=stream;await video.play();calibration=null;mode=null;points=[];
  say('Камера включена. Держите телефон неподвижно. Нажмите «Задать эталон» и коснитесь двух концов объекта с известной длиной.');draw();
 }catch(error){
  stream=null;
  const reason=error&&error.name==='NotAllowedError'?'Доступ запрещён. Разрешите камеру для этого сайта в настройках Safari.':error&&error.name==='NotFoundError'?'Камера не найдена на устройстве.':'Не удалось включить камеру: '+(error&&error.message?error.message:'неизвестная ошибка');
  say(reason);
 }
}
function stopCamera(){
 if(stream)stream.getTracks().forEach(track=>track.stop());
 stream=null;video.srcObject=null;calibration=null;mode=null;points=[];ctx.clearRect(0,0,canvas.width,canvas.height);
 say('Камера выключена. Калибровка удалена; при следующем запуске выполните её заново.');
}
function chooseMode(next){
 if(!stream||!video.videoWidth){say('Сначала включите камеру.');return;}
 if(next==='gate'&&!calibration){say('Сначала выполните калибровку по эталону известной длины.');return;}
 mode=next;points=[];draw();
 say(next==='reference'?'Коснитесь двух концов эталона известной длины. Эталон должен быть в той же плоскости, что и ворота.':'Коснитесь двух краёв ворот на одной линии измерения.');
}
canvas.addEventListener('pointerdown',event=>{
 if(!stream||!mode)return;
 const rect=canvas.getBoundingClientRect();
 const p={x:(event.clientX-rect.left)*canvas.width/rect.width,y:(event.clientY-rect.top)*canvas.height/rect.height};
 points.push(p);draw();if(points.length<2)return;
 const pixels=distance(points[0],points[1]);
 if(pixels<8){points=[];draw();say('Точки слишком близко. Повторите измерение.');return;}
 if(mode==='reference'){
  const known=Number($('cameraReferenceLength').value);
  if(!Number.isFinite(known)||known<0.05||known>100){points=[];draw();say('Введите длину эталона от 0,05 до 100 метров.');return;}
  calibration={metersPerPixel:known/pixels,videoWidth:canvas.width,videoHeight:canvas.height};mode=null;points=[];draw();
  say('Калибровка выполнена. Масштаб: '+calibration.metersPerPixel.toFixed(5)+' м/пиксель. Не перемещайте камеру и измеряйте объекты в той же плоскости.');
 }else if(mode==='gate'){
  const width=pixels*calibration.metersPerPixel;mode=null;points=[];draw();
  say('Предварительная ширина между отмеченными точками: '+width.toFixed(2)+' м. Погрешность не определена; проверьте ширину рулеткой и учтите запас по краям.');
 }
});
$('cameraStart').addEventListener('click',startCamera);
$('cameraStop').addEventListener('click',stopCamera);
$('cameraCalibrate').addEventListener('click',()=>chooseMode('reference'));
$('cameraMeasureGate').addEventListener('click',()=>chooseMode('gate'));
$('cameraReset').addEventListener('click',clearCalibration);
video.addEventListener('loadedmetadata',draw);
window.addEventListener('pagehide',stopCamera);
})();
