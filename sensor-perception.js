(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPerception=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';
 const MAX_OBJECTS=300,MAX_AGE_MS=1500,MAX_FUTURE_MS=250,MAX_RANGE_M=60;
 const finite=n=>typeof n==='number'&&Number.isFinite(n);
 function validateFrame(frame,now=Date.now()){
  const errors=[];
  if(!frame||typeof frame!=='object'||Array.isArray(frame))return ['Кадр датчиков должен быть объектом.'];
  if(typeof frame.frameId!=='string'||!frame.frameId.trim()||frame.frameId.length>128)errors.push('frameId должен быть непустой строкой до 128 символов.');
  if(!finite(frame.timestamp))errors.push('timestamp должен быть временем Unix в миллисекундах.');
  else if(frame.timestamp>now+MAX_FUTURE_MS)errors.push('Временная метка находится в будущем.');
  else if(now-frame.timestamp>MAX_AGE_MS)errors.push('Кадр устарел: допустимо не более 1500 мс.');
  if(frame.coordinateFrame!=='vehicle-local')errors.push('Поддерживается только coordinateFrame="vehicle-local".');
  const vp=frame.vehiclePose;
  if(!vp||![vp.x,vp.y,vp.heading].every(finite))errors.push('vehiclePose должен содержать синхронизированные x, y и heading в системе карты (метры, радианы).');
  if(!Array.isArray(frame.objects))errors.push('objects должен быть массивом.');
  else{
   if(frame.objects.length>MAX_OBJECTS)errors.push('В одном кадре допускается не более 300 объектов.');
   const ids=new Set();
   frame.objects.forEach((o,i)=>{
    const p='objects['+i+']';
    if(!o||typeof o!=='object'||Array.isArray(o)){errors.push(p+': ожидается объект.');return;}
    if(typeof o.id!=='string'||!o.id.trim()||o.id.length>128)errors.push(p+'.id некорректен.');
    else if(ids.has(o.id))errors.push(p+'.id повторяется.'); else ids.add(o.id);
    if(typeof o.type!=='string'||!o.type.trim()||o.type.length>64)errors.push(p+'.type некорректен.');
    const pose=o.pose,dim=o.dimensions;
    if(!pose||![pose.x,pose.y,pose.heading].every(finite))errors.push(p+'.pose должен содержать конечные x, y и heading (радианы).');
    else if(Math.hypot(pose.x,pose.y)>MAX_RANGE_M)errors.push(p+' находится дальше 60 м от тягача.');
    if(!dim||![dim.length,dim.width].every(finite)||dim.length<0.05||dim.width<0.05||dim.length>30||dim.width>15)errors.push(p+'.dimensions содержит недопустимые length/width.');
    if(!finite(o.confidence)||o.confidence<0||o.confidence>1)errors.push(p+'.confidence должен быть в диапазоне 0–1.');
    if(!['lidar','camera','fused'].includes(o.source))errors.push(p+'.source должен быть lidar, camera или fused.');
   });
  }
  return errors;
 }
 function toObstacles(frame){
  const errors=validateFrame(frame);
  if(errors.length)throw new Error(errors.join(' '));
  const vehiclePose=frame.vehiclePose;
  if(!vehiclePose||![vehiclePose.x,vehiclePose.y,vehiclePose.heading].every(finite))throw new Error('Синхронизированная поза тягача в кадре недоступна.');
  const c=Math.cos(vehiclePose.heading),s=Math.sin(vehiclePose.heading);
  return frame.objects.map(o=>{
   const x=vehiclePose.x+o.pose.x*c-o.pose.y*s;
   const y=vehiclePose.y+o.pose.x*s+o.pose.y*c;
   const heading=o.pose.heading+vehiclePose.heading;
   const ux=Math.cos(heading),uy=Math.sin(heading),nx=-uy,ny=ux;
   const hl=o.dimensions.length/2,hw=o.dimensions.width/2;
   const polygon=[
    {x:x-hl*ux-hw*nx,y:y-hl*uy-hw*ny},
    {x:x+hl*ux-hw*nx,y:y+hl*uy-hw*ny},
    {x:x+hl*ux+hw*nx,y:y+hl*uy+hw*ny},
    {x:x-hl*ux+hw*nx,y:y-hl*uy+hw*ny}
   ];
   return {kind:'perception-obstacle',id:o.id,name:o.type+' · '+o.id,source:o.source,confidence:o.confidence,
    detectedAt:frame.timestamp,polygon};
  });
 }
 return Object.freeze({validateFrame,toObstacles,MAX_AGE_MS,MAX_OBJECTS});
});
