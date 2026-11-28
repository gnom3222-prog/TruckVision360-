(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPhysics=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';

 const norm=angle=>Math.atan2(Math.sin(angle),Math.cos(angle));

 const VEHICLE_PARAMETER_LIMITS=Object.freeze({
  wheelbase:{min:1,max:15},
  hitchOffset:{min:0,max:5},
  kingpinAxles:{min:0.5,max:30},
  trailerKingpinToFront:{min:0.2,max:5,optional:true},
  trailerLength:{min:4,max:30},
  trailerWidth:{min:1.5,max:5},
  trailerAxleSpacing:{min:0.3,max:5,optional:true},
  trailerAxleSpacing12:{min:0,max:5,optional:true},
  trailerAxleSpacing23:{min:0,max:5,optional:true},
  footprintUncertainty:{min:0,max:1,optional:true},
  tractorFrontOverhang:{min:1,max:10,optional:true},
  tractorRearOverhang:{min:0.2,max:5,optional:true},
  tractorBodyWidth:{min:1.8,max:3.2,optional:true},
  cabOffset:{min:0,max:6,optional:true},
  cabFront:{min:0.5,max:3.5,optional:true},
  cabRear:{min:0.3,max:2.5,optional:true},
  cabWidth:{min:1.8,max:2.55,optional:true},
  mirrorOutboard:{min:0,max:1,optional:true},
  mirrorLongitudinal:{min:-2,max:8,optional:true},
  mirrorLength:{min:0.2,max:2,optional:true},
  tractorFrontTrack:{min:1,max:3.5,optional:true},
  tractorSteeringTrack:{min:1,max:3.5,optional:true},
  tractorRearTrack:{min:1,max:3.5,optional:true},
  trailerTrack:{min:1,max:4.5,optional:true},
  trailerAxle1Track:{min:0,max:4.5,optional:true},
  trailerAxle2Track:{min:0,max:4.5,optional:true},
  trailerAxle3Track:{min:0,max:4.5,optional:true},
  tractorRearTireGroupWidth:{min:0,max:1.5,optional:true},
  trailerTireGroupWidth:{min:0,max:1.5,optional:true},
  trailerAxle1TireGroupWidth:{min:0,max:1.5,optional:true},
  trailerAxle2TireGroupWidth:{min:0,max:1.5,optional:true},
  trailerAxle3TireGroupWidth:{min:0,max:1.5,optional:true},
  tractorRearTireCenterSpacing:{min:0,max:1.5,optional:true},
  trailerAxle1TireCenterSpacing:{min:0,max:1.5,optional:true},
  trailerAxle2TireCenterSpacing:{min:0,max:1.5,optional:true},
  trailerAxle3TireCenterSpacing:{min:0,max:1.5,optional:true},
  tireLength:{min:0.2,max:1.5,optional:true},
  tireWidth:{min:0.1,max:0.8,optional:true}
 });

 function trailerAxleTireCenterSpacings(params){
  return [1,2,3].map(index=>{
   const value=params&&params['trailerAxle'+index+'TireCenterSpacing'];
   return Number.isFinite(value)&&value>0?value:0;
  });
 }

 function trailerAxleTireGroupWidths(params){
  const tireWidth=Number.isFinite(params&&params.tireWidth)?params.tireWidth:0.44;
  const common=Number.isFinite(params&&params.trailerTireGroupWidth)&&params.trailerTireGroupWidth>0?
   params.trailerTireGroupWidth:tireWidth;
  const pairSpacings=trailerAxleTireCenterSpacings(params);
  return [1,2,3].map(index=>{
   const pairSpacing=pairSpacings[index-1];
   if(pairSpacing>0)return pairSpacing+tireWidth;
   const specific=params&&params['trailerAxle'+index+'TireGroupWidth'];
   return Number.isFinite(specific)&&specific>0?specific:common;
  });
 }

 function resolveTrailerAxleSpacings(params){
  const source=params&&typeof params==='object'?params:{};
  const present=key=>source[key]!==undefined&&source[key]!==null;
  const inRange=value=>Number.isFinite(value)&&value>=0.3&&value<=5;
  const commonPresent=present('trailerAxleSpacing');
  const commonValid=inRange(source.trailerAxleSpacing);
  if(commonPresent&&!commonValid){
   return {valid:false,spacings:null,reason:'Общий резервный шаг задан, но находится вне диапазона 0,30–5 м.'};
  }
  const readIndividual=key=>{
   if(!present(key)||source[key]===0)return {kind:'missing'};
   if(!inRange(source[key]))return {kind:'invalid'};
   return {kind:'valid',value:source[key]};
  };
  const first=readIndividual('trailerAxleSpacing12');
  const second=readIndividual('trailerAxleSpacing23');
  if(first.kind==='invalid'||second.kind==='invalid'){
   return {valid:false,spacings:null,reason:'Индивидуальные расстояния между соседними осями должны быть в диапазоне 0,30–5 м; ноль означает отсутствие измерения.'};
  }
  if((first.kind==='missing'||second.kind==='missing')&&!commonValid){
   return {valid:false,spacings:null,reason:'Задайте положительный общий шаг как резервное значение, если не заданы оба положительных индивидуальных шага осей 1–2 и 2–3.'};
  }
  return {
   valid:true,
   spacings:[
    first.kind==='valid'?first.value:source.trailerAxleSpacing,
    second.kind==='valid'?second.value:source.trailerAxleSpacing
   ],
   usedFallback:first.kind==='missing'||second.kind==='missing'
  };
 }

 function trailerAxleSpacings(params){
  const resolved=resolveTrailerAxleSpacings(params);
  if(!resolved.valid)throw new Error('Невозможно определить шаги трёхосной тележки: '+resolved.reason);
  return resolved.spacings;
 }

 function trailerAxleTracks(params){
  const common=Number.isFinite(params&&params.trailerTrack)&&params.trailerTrack>0?
   params.trailerTrack:
   (Number.isFinite(params&&params.trailerWidth)?params.trailerWidth+0.24:NaN);
  return [1,2,3].map(index=>{
   const specific=params&&params['trailerAxle'+index+'Track'];
   return Number.isFinite(specific)&&specific>0?specific:common;
  });
 }

 function validateTireGroupWidthUncertainty(params){
  const errors=[];
  if(!params||typeof params!=='object')return ['Параметры неопределённости групп шин отсутствуют.'];
  const tireWidth=Number.isFinite(params.tireWidth)?params.tireWidth:0.44;
  const tireWidthUncertainty=Number.isFinite(params.tireWidthUncertainty)?Math.max(0,params.tireWidthUncertainty):0;
  const minimumGroupWidth=tireWidth+tireWidthUncertainty;
  const groups=[
   ['tractorRearTireGroupWidth','tractorRearTireGroupWidthUncertainty','группы задних шин тягача','tractorRearTireCenterSpacing','tractorRearTireCenterSpacingUncertainty'],
   ['trailerTireGroupWidth','trailerTireGroupWidthUncertainty','группы шин полуприцепа',null,null],
   ['trailerAxle1TireGroupWidth','trailerAxle1TireGroupWidthUncertainty','группы шин 1-й оси полуприцепа','trailerAxle1TireCenterSpacing','trailerAxle1TireCenterSpacingUncertainty'],
   ['trailerAxle2TireGroupWidth','trailerAxle2TireGroupWidthUncertainty','группы шин 2-й оси полуприцепа','trailerAxle2TireCenterSpacing','trailerAxle2TireCenterSpacingUncertainty'],
   ['trailerAxle3TireGroupWidth','trailerAxle3TireGroupWidthUncertainty','группы шин 3-й оси полуприцепа','trailerAxle3TireCenterSpacing','trailerAxle3TireCenterSpacingUncertainty']
  ];
  groups.forEach(([widthKey,uncertaintyKey,label,pairSpacingKey,pairUncertaintyKey])=>{
   const width=Number.isFinite(params[widthKey])?params[widthKey]:0;
   const uncertainty=Number.isFinite(params[uncertaintyKey])?params[uncertaintyKey]:0;
   if(pairSpacingKey&&Number.isFinite(params[pairSpacingKey])&&params[pairSpacingKey]>0){
    if(uncertainty>0)errors.push(uncertaintyKey+': при раздельной модели покрышек используйте '+pairUncertaintyKey+' и tireWidthUncertainty.');
    return;
   }
   if(uncertainty<=0)return;
   if(width<=0){
    errors.push(uncertaintyKey+': сначала задайте измеренную номинальную ширину '+label+'.');
    return;
   }
   if(width-uncertainty<minimumGroupWidth-1e-9){
    errors.push(uncertaintyKey+': нижняя граница ширины группы должна быть не меньше максимальной ширины одной шины ('+minimumGroupWidth.toFixed(3)+' м с учётом её допуска).');
   }
  });
  const pairSpacings=[
   ['tractorRearTireCenterSpacing','tractorRearTireCenterSpacingUncertainty','сдвоенной группы задней оси тягача'],
   ['trailerAxle1TireCenterSpacing','trailerAxle1TireCenterSpacingUncertainty','сдвоенной группы 1-й оси полуприцепа'],
   ['trailerAxle2TireCenterSpacing','trailerAxle2TireCenterSpacingUncertainty','сдвоенной группы 2-й оси полуприцепа'],
   ['trailerAxle3TireCenterSpacing','trailerAxle3TireCenterSpacingUncertainty','сдвоенной группы 3-й оси полуприцепа']
  ];
  pairSpacings.forEach(([spacingKey,uncertaintyKey,label])=>{
   const spacing=Number.isFinite(params[spacingKey])?params[spacingKey]:0;
   const uncertainty=Number.isFinite(params[uncertaintyKey])?params[uncertaintyKey]:0;
   if(uncertainty<=0)return;
   if(spacing<=0){
    errors.push(uncertaintyKey+': сначала задайте измеренное расстояние между центрами покрышек '+label+'.');
    return;
   }
   if(spacing-uncertainty<tireWidth+tireWidthUncertainty-1e-9){
    errors.push(uncertaintyKey+': нижняя граница расстояния между центрами не должна допускать перекрытие покрышек.');
   }
   if(spacing+uncertainty+tireWidth+tireWidthUncertainty>1.5+1e-9){
    errors.push(uncertaintyKey+': верхняя граница наружной ширины пары превышает 1,50 м.');
   }
  });
  return errors;
 }


 const AUXILIARY_ENVELOPE_ANCHORS=Object.freeze([
  'tractor-front-axle','tractor-rear-axle',
  'trailer-axle-1','trailer-axle-2','trailer-axle-3'
 ]);

 function validateAuxiliaryEnvelopes(envelopes){
  const errors=[];
  if(envelopes===undefined)return errors;
  if(!Array.isArray(envelopes)){
   return ['Дополнительные оболочки осевых узлов должны быть массивом JSON.'];
  }
  if(envelopes.length>32){
   errors.push('Допускается не более 32 дополнительных оболочек осевых узлов.');
  }
  const names=new Set();
  envelopes.forEach((entry,index)=>{
   const prefix='Дополнительная оболочка '+(index+1)+': ';
   if(!entry||typeof entry!=='object'||Array.isArray(entry)){
    errors.push(prefix+'ожидается объект с измеренными размерами.');
    return;
   }
   if(typeof entry.name!=='string'||!entry.name.trim()||entry.name.trim().length>80){
    errors.push(prefix+'задайте название длиной от 1 до 80 символов.');
   }else{
    const normalized=entry.name.trim().toLocaleLowerCase();
    if(names.has(normalized))errors.push(prefix+'название должно быть уникальным.');
    names.add(normalized);
   }
   if(!AUXILIARY_ENVELOPE_ANCHORS.includes(entry.anchor)){
    errors.push(prefix+'неизвестная опорная ось.');
   }
   const suspensionAttachment=entry.suspensionAttachment??'axle';
   if(!['axle','tractor-body','trailer-body'].includes(suspensionAttachment)){
    errors.push(prefix+'suspensionAttachment должен быть axle, tractor-body или trailer-body.');
   }else if(suspensionAttachment==='tractor-body'&&!String(entry.anchor||'').startsWith('tractor-')){
    errors.push(prefix+'tractor-body допускается только для оболочки, привязанной к оси тягача.');
   }else if(suspensionAttachment==='trailer-body'&&!String(entry.anchor||'').startsWith('trailer-')){
    errors.push(prefix+'trailer-body допускается только для оболочки, привязанной к оси полуприцепа.');
   }
   const longitudinalOffset=entry.longitudinalOffset;
   const lateralOffset=entry.lateralOffset;
   const headingOffsetDeg=entry.headingOffsetDeg;
   const uncertaintyMargin=entry.uncertaintyMargin;
   if(!Number.isFinite(longitudinalOffset)||Math.abs(longitudinalOffset)>5){
    errors.push(prefix+'продольное смещение должно находиться в диапазоне ±5 м.');
   }
   if(!Number.isFinite(lateralOffset)||Math.abs(lateralOffset)>3){
    errors.push(prefix+'поперечное смещение должно находиться в диапазоне ±3 м.');
   }
   if(!Number.isFinite(headingOffsetDeg)||Math.abs(headingOffsetDeg)>180){
    errors.push(prefix+'угловое смещение должно находиться в диапазоне ±180°.');
   }
   if(!Number.isFinite(entry.length)||entry.length<0.05||entry.length>5){
    errors.push(prefix+'длина должна быть измеренным числом от 0,05 до 5 м.');
   }
   if(!Number.isFinite(entry.width)||entry.width<0.05||entry.width>5){
    errors.push(prefix+'ширина должна быть измеренным числом от 0,05 до 5 м.');
   }
   if(!Number.isFinite(uncertaintyMargin)||uncertaintyMargin<0.001||uncertaintyMargin>1){
    errors.push(prefix+'запас неопределённости должен быть явно задан положительным числом от 0,001 до 1 м.');
   }
   const hasZMin=entry.zMin!==undefined,hasZMax=entry.zMax!==undefined;
   const verticalUncertaintyMargin=entry.verticalUncertaintyMargin;
   if(hasZMin!==hasZMax){
    errors.push(prefix+'вертикальный диапазон должен содержать одновременно zMin и zMax.');
   }else if(hasZMin&&(!Number.isFinite(entry.zMin)||!Number.isFinite(entry.zMax)||
      entry.zMin< -2||entry.zMax>15||entry.zMax<=entry.zMin)){
    errors.push(prefix+'zMin/zMax должны задавать измеренный интервал высоты от −2 до 15 м.');
   }
   if(hasZMin&&(!Number.isFinite(verticalUncertaintyMargin)||verticalUncertaintyMargin<0.001||verticalUncertaintyMargin>1)){
    errors.push(prefix+'для вертикального интервала требуется явно заданная положительная verticalUncertaintyMargin от 0,001 до 1 м.');
   }else if(!hasZMin&&verticalUncertaintyMargin!==undefined&&
      (!Number.isFinite(verticalUncertaintyMargin)||verticalUncertaintyMargin<0||verticalUncertaintyMargin>1)){
    errors.push(prefix+'вертикальная неопределённость должна находиться в диапазоне 0–1 м.');
   }
   if(!hasZMin&&verticalUncertaintyMargin>0){
    errors.push(prefix+'вертикальную неопределённость можно задавать только вместе с zMin/zMax.');
   }
  });
  return errors;
 }

 const VEHICLE_VERTICAL_PROFILE_KEYS=Object.freeze([
  'tractor-body','tractor-cab','trailer-body','mirror','tractor-wheel','trailer-wheel',
  'tractor-front-wheel','tractor-rear-wheel',
  'trailer-axle-1-wheel','trailer-axle-2-wheel','trailer-axle-3-wheel'
 ]);

 function validateVehicleVerticalProfiles(profiles){
  const errors=[];
  if(profiles===undefined)return errors;
  if(!profiles||typeof profiles!=='object'||Array.isArray(profiles)){
   return ['Вертикальные профили автопоезда должны быть JSON-объектом.'];
  }
  for(const [key,profile] of Object.entries(profiles)){
   const prefix='Вертикальный профиль '+key+': ';
   if(!VEHICLE_VERTICAL_PROFILE_KEYS.includes(key)){
    errors.push(prefix+'неизвестный класс компонента.');
    continue;
   }
   if(!profile||typeof profile!=='object'||Array.isArray(profile)){
    errors.push(prefix+'ожидается объект с zMin и zMax.');
    continue;
   }
   if(!Number.isFinite(profile.zMin)||!Number.isFinite(profile.zMax)||
      profile.zMin< -2||profile.zMax>15||profile.zMax<=profile.zMin){
    errors.push(prefix+'zMin/zMax должны задавать измеренный интервал высоты от −2 до 15 м.');
   }
   const margin=profile.uncertaintyMargin;
   if(!Number.isFinite(margin)||margin<0.001||margin>1){
    errors.push(prefix+'uncertaintyMargin должен быть явно задан положительным числом от 0,001 до 1 м.');
   }
  }
  return errors;
 }

 function verticalRangeFromProfile(profile,extraMargin=0){
  if(!profile||!Number.isFinite(profile.zMin)||!Number.isFinite(profile.zMax)||profile.zMax<=profile.zMin)return null;
  const margin=profile.uncertaintyMargin;
  if(!Number.isFinite(margin)||margin<0.001||margin>1||!Number.isFinite(extraMargin)||extraMargin<0)return null;
  return {min:profile.zMin-margin-extraMargin,max:profile.zMax+margin+extraMargin};
 }

 function validateRoadSurface(surface){
  const errors=[];
  if(surface===undefined||surface===null)return errors;
  if(!surface||typeof surface!=='object'||Array.isArray(surface)){
   return ['roadSurface должен быть JSON-объектом или null.'];
  }
  const required=['originX','originY','originZ','gradeX','gradeY',
   'originZUncertainty','gradeXUncertainty','gradeYUncertainty','surfaceResidualUncertainty'];
  for(const key of required){
   if(!Number.isFinite(surface[key])){
    errors.push('roadSurface.'+key+' должен быть задан конечным измеренным числом.');
   }
  }
  const limits={
   originX:[-100000,100000],originY:[-100000,100000],originZ:[-100,100],
   gradeX:[-0.25,0.25],gradeY:[-0.25,0.25],
   originZUncertainty:[0.0001,1],gradeXUncertainty:[0.000001,0.05],
   gradeYUncertainty:[0.000001,0.05],surfaceResidualUncertainty:[0.0001,1]
  };
  for(const [key,[min,max]] of Object.entries(limits)){
   const value=surface[key];
   if(Number.isFinite(value)&&(value<min||value>max)){
    errors.push('roadSurface.'+key+' должен находиться в диапазоне '+min+'–'+max+'.');
   }
  }
  return errors;
 }

 function roadSurfaceElevationRange(polygon,surface){
  if(!Array.isArray(polygon)||polygon.length<3||
     !polygon.every(point=>point&&Number.isFinite(point.x)&&Number.isFinite(point.y)))return null;
  if(surface===undefined||surface===null)return null;
  const errors=validateRoadSurface(surface);
  if(errors.length)return null;
  const lower=[],upper=[];
  for(const point of polygon){
   const dx=point.x-surface.originX,dy=point.y-surface.originY;
   const elevation=surface.originZ+surface.gradeX*dx+surface.gradeY*dy;
   const uncertainty=surface.originZUncertainty+
    Math.abs(dx)*surface.gradeXUncertainty+
    Math.abs(dy)*surface.gradeYUncertainty+
    surface.surfaceResidualUncertainty;
   lower.push(elevation-uncertainty);
   upper.push(elevation+uncertainty);
  }
  return {
   min:Math.min(...lower),
   max:Math.max(...upper),
   status:'configured-planar-surface-with-bounds'
  };
 }

 function verticalRangeOnRoadSurface(range,polygon,surface){
  if(!range||!Number.isFinite(range.min)||!Number.isFinite(range.max)||range.max<range.min)return null;
  const elevation=roadSurfaceElevationRange(polygon,surface);
  if(!elevation)return null;
  return {
   min:range.min+elevation.min,
   max:range.max+elevation.max,
   roadSurfaceStatus:elevation.status,
   ...(range.suspensionStatus?{suspensionStatus:range.suspensionStatus}:{}),
   ...(range.suspensionDisplacement?{suspensionDisplacement:range.suspensionDisplacement}:{})
  };
 }

 function vehicleVerticalRange(params,key){
  const profiles=params&&params.vehicleVerticalProfiles;
  return profiles&&profiles[key]?verticalRangeFromProfile(profiles[key]):null;
 }

 function wheelVerticalRange(params,vehiclePart,axleIndex){
  const specificKey=vehiclePart==='тягач'?
   (axleIndex===1?'tractor-front-wheel':'tractor-rear-wheel'):
   'trailer-axle-'+axleIndex+'-wheel';
  const sharedKey=vehiclePart==='тягач'?'tractor-wheel':'trailer-wheel';
  return vehicleVerticalRange(params,specificKey)||vehicleVerticalRange(params,sharedKey);
 }
 
 const SUSPENSION_TRAVEL_MAX_SLOPE=0.25;
 const SUSPENSION_AXLE_KEYS=Object.freeze({
  tractor:['front','rear'],
  trailer:['axle1','axle2','axle3']
 });
 function suspensionSupportDefinitions(params,vehiclePart){
  if(vehiclePart==='tractor'){
   const frontTrack=params.tractorFrontTrack??2.05,rearTrack=params.tractorRearTrack??2.05;
   const frontUncertainty=params.tractorFrontTrackUncertainty??0,rearUncertainty=params.tractorRearTrackUncertainty??0;
   return [
    {key:'front',x:params.wheelbase,track:frontTrack,trackLowerBound:frontTrack-frontUncertainty},
    {key:'rear',x:0,track:rearTrack,trackLowerBound:rearTrack-rearUncertainty}
   ];
  }
  const offsets=trailerAxleOffsets(params),tracks=trailerAxleTracks(params);
  return offsets.map((x,index)=>{
   const axleIndex=index+1;
   const individualTrack=(params['trailerAxle'+axleIndex+'Track']??0)>0;
   const uncertainty=individualTrack?
    (params['trailerAxle'+axleIndex+'TrackUncertainty']??0):
    (params.trailerTrackUncertainty??0);
   return {key:'axle'+axleIndex,x,track:tracks[index],trackLowerBound:tracks[index]-uncertainty};
  });
 }
 function suspensionSupportIntervals(config,params,vehiclePart){
  const group=config[vehiclePart];
  return suspensionSupportDefinitions(params,vehiclePart).flatMap(axle=>
   ['left','right'].map(side=>{
    const point=group[axle.key][side];
    const uncertainty=point.uncertainty;
    return {
     axle:axle.key,side,x:axle.x,y:(side==='left'?1:-1)*axle.track/2,
     track:axle.track,
     min:-(point.compression+uncertainty),
     max:point.rebound+uncertainty
    };
   })
  );
 }
 function suspensionPoseCoefficientBounds(config,params,vehiclePart){
  const definitions=suspensionSupportDefinitions(params,vehiclePart);
  const supports=suspensionSupportIntervals(config,params,vehiclePart);
  const hMin=supports.reduce((sum,point)=>sum+point.min,0)/supports.length;
  const hMax=supports.reduce((sum,point)=>sum+point.max,0)/supports.length;
  const xCenter=definitions.reduce((sum,axle)=>sum+axle.x,0)/definitions.length;
  let pitchDistanceLowerBound=0,xCenterUncertainty=0;
  if(vehiclePart==='tractor'){
   const wheelbaseUncertainty=params.wheelbaseUncertainty??0;
   pitchDistanceLowerBound=params.wheelbase-wheelbaseUncertainty;
   xCenterUncertainty=wheelbaseUncertainty/2;
  }else{
   const spacings=trailerAxleSpacings(params);
   const spacing12Uncertainty=(params.trailerAxleSpacing12??0)>0?
    (params.trailerAxleSpacing12Uncertainty??0):(params.trailerAxleSpacingUncertainty??0);
   const spacing23Uncertainty=(params.trailerAxleSpacing23??0)>0?
    (params.trailerAxleSpacing23Uncertainty??0):(params.trailerAxleSpacingUncertainty??0);
   pitchDistanceLowerBound=spacings[0]+spacings[1]-spacing12Uncertainty-spacing23Uncertainty;
   xCenterUncertainty=(spacing12Uncertainty+spacing23Uncertainty)/3;
  }
  const pitchPair=vehiclePart==='tractor'?
   {first:'front',last:'rear',distance:pitchDistanceLowerBound}:
   {first:'axle1',last:'axle3',distance:pitchDistanceLowerBound};
  let pitchMin=Infinity,pitchMax=-Infinity;
  for(const side of ['left','right']){
   const first=supports.find(point=>point.axle===pitchPair.first&&point.side===side);
   const last=supports.find(point=>point.axle===pitchPair.last&&point.side===side);
   if(!first||!last||!Number.isFinite(pitchPair.distance)||pitchPair.distance<=0)return null;
   pitchMin=Math.min(pitchMin,(first.min-last.max)/pitchPair.distance);
   pitchMax=Math.max(pitchMax,(first.max-last.min)/pitchPair.distance);
  }
  let rollMin=Infinity,rollMax=-Infinity;
  for(const axle of definitions){
   const left=supports.find(point=>point.axle===axle.key&&point.side==='left');
   const right=supports.find(point=>point.axle===axle.key&&point.side==='right');
   if(!left||!right||!Number.isFinite(axle.trackLowerBound)||axle.trackLowerBound<=0)return null;
   rollMin=Math.min(rollMin,(left.min-right.max)/axle.trackLowerBound);
   rollMax=Math.max(rollMax,(left.max-right.min)/axle.trackLowerBound);
  }
  if(!Number.isFinite(pitchDistanceLowerBound)||pitchDistanceLowerBound<=0)return null;
  return {hMin,hMax,pitchMin,pitchMax,rollMin,rollMax,xCenter,xCenterUncertainty,supports};
 }
 function validateSuspensionTravel(config,params){
  const errors=[];
  if(config===undefined||config===null)return errors;
  if(!config||typeof config!=='object'||Array.isArray(config)){
   return ['suspensionTravel должен быть JSON-объектом или null.'];
  }
  for(const group of ['tractor','trailer']){
   if(!config[group]||typeof config[group]!=='object'||Array.isArray(config[group])){
    errors.push('suspensionTravel.'+group+' должен содержать измеренные опорные точки.');
    continue;
   }
   for(const axleKey of SUSPENSION_AXLE_KEYS[group]){
    const axle=config[group][axleKey];
    if(!axle||typeof axle!=='object'||Array.isArray(axle)){
     errors.push('suspensionTravel.'+group+'.'+axleKey+' должен содержать левую и правую стороны.');
     continue;
    }
    for(const side of ['left','right']){
     const point=axle[side],prefix='suspensionTravel.'+group+'.'+axleKey+'.'+side+'.';
     if(!point||typeof point!=='object'||Array.isArray(point)){
      errors.push(prefix+'ожидается объект compression/rebound/uncertainty.');
      continue;
     }
     for(const key of ['compression','rebound','uncertainty']){
      if(!Number.isFinite(point[key])){
       errors.push(prefix+key+' должен быть явно измеренным конечным числом.');
      }
     }
     const limits={compression:[0,0.25],rebound:[0,0.25],uncertainty:[0.0001,0.05]};
     for(const [key,[min,max]] of Object.entries(limits)){
      const value=point[key];
      if(Number.isFinite(value)&&(value<min||value>max)){
       errors.push(prefix+key+' должен находиться в диапазоне '+min+'–'+max+' м.');
      }
     }
    }
   }
  }
  if(errors.length===0&&params&&Number.isFinite(params.wheelbase)&&Number.isFinite(params.trailerLength)){
   const geometryBounds=[
    ['wheelbaseUncertainty',params.wheelbaseUncertainty],
    ['tractorFrontTrackUncertainty',params.tractorFrontTrackUncertainty],
    ['tractorRearTrackUncertainty',params.tractorRearTrackUncertainty]
   ];
   const axleSpacingResolution=resolveTrailerAxleSpacings(params);
   if(!axleSpacingResolution.valid){
    errors.push('suspensionTravel.trailer: '+axleSpacingResolution.reason);
   }
   geometryBounds.push(
    [(params.trailerAxleSpacing12??0)>0?'trailerAxleSpacing12Uncertainty':'trailerAxleSpacingUncertainty',
     (params.trailerAxleSpacing12??0)>0?params.trailerAxleSpacing12Uncertainty:params.trailerAxleSpacingUncertainty],
    [(params.trailerAxleSpacing23??0)>0?'trailerAxleSpacing23Uncertainty':'trailerAxleSpacingUncertainty',
     (params.trailerAxleSpacing23??0)>0?params.trailerAxleSpacing23Uncertainty:params.trailerAxleSpacingUncertainty]
   );
   for(let index=1;index<=3;index++){
    const individual=(params['trailerAxle'+index+'Track']??0)>0;
    const key=individual?'trailerAxle'+index+'TrackUncertainty':'trailerTrackUncertainty';
    geometryBounds.push([key,params[key]]);
   }
   for(const [key,value] of geometryBounds){
    if(!Number.isFinite(value)||value<=0){
     errors.push('suspensionTravel требует положительную измеренную границу '+key+'; ноль не считается подтверждённой точностью.');
    }
   }
   if(axleSpacingResolution.valid&&geometryBounds.every(([,value])=>Number.isFinite(value)&&value>0)){
    for(const group of ['tractor','trailer']){
    const bounds=suspensionPoseCoefficientBounds(config,params,group);
    if(!bounds){
     errors.push('suspensionTravel.'+group+': невозможно построить опорную плоскость по измеренной геометрии осей.');
     continue;
    }
    const maxSlope=Math.max(
     Math.abs(bounds.pitchMin),Math.abs(bounds.pitchMax),
     Math.abs(bounds.rollMin),Math.abs(bounds.rollMax)
    );
    if(maxSlope>SUSPENSION_TRAVEL_MAX_SLOPE+1e-9){
     errors.push('suspensionTravel.'+group+': расчётный наклон опорной плоскости превышает 0,25; линейная модель малых перемещений неприменима.');
    }
   }
   }
  }
  return errors;
 }
 function suspensionTravelStatus(params){
  const config=params&&params.suspensionTravel;
  if(config===undefined||config===null)return 'suspension-travel-unconfigured';
  return validateSuspensionTravel(config,params).length?
   'invalid-suspension-travel-config':'bounded-rigid-plane-support-envelope';
 }
 function suspensionTravelDisplacementRange(polygon,q,params,vehiclePart){
  if(!params||params.suspensionTravel===undefined||params.suspensionTravel===null)return null;
  if(vehiclePart!=='tractor'&&vehiclePart!=='trailer')throw new Error('Неизвестная часть автопоезда для проверки подвески.');
  const errors=validateSuspensionTravel(params.suspensionTravel,params);
  if(errors.length)throw new Error('Некорректные диапазоны хода подвески: '+errors.join(' '));
  if(!Array.isArray(polygon)||polygon.length<3||!q||
     ![q.x,q.y,q.psi,q.psiT].every(Number.isFinite))return null;
  const bounds=suspensionPoseCoefficientBounds(params.suspensionTravel,params,vehiclePart);
  if(!bounds)return null;
  const origin=vehiclePart==='tractor'?{x:q.x,y:q.y}:axlePoint(q,params);
  const heading=vehiclePart==='tractor'?q.psi:q.psiT;
  const c=Math.cos(heading),s=Math.sin(heading),lower=[],upper=[];
  for(const point of polygon){
   const dx=point.x-origin.x,dy=point.y-origin.y;
   const localX=dx*c+dy*s;
   const localY=-dx*s+dy*c;
   const xRelMin=localX-(bounds.xCenter+bounds.xCenterUncertainty);
   const xRelMax=localX-(bounds.xCenter-bounds.xCenterUncertainty);
   const pitchProducts=[
    bounds.pitchMin*xRelMin,bounds.pitchMin*xRelMax,
    bounds.pitchMax*xRelMin,bounds.pitchMax*xRelMax
   ];
   const pitchLow=Math.min(...pitchProducts);
   const pitchHigh=Math.max(...pitchProducts);
   const rollLow=Math.min(bounds.rollMin*localY,bounds.rollMax*localY);
   const rollHigh=Math.max(bounds.rollMin*localY,bounds.rollMax*localY);
   lower.push(bounds.hMin+pitchLow+rollLow);
   upper.push(bounds.hMax+pitchHigh+rollHigh);
  }
  return {
   min:Math.min(...lower),max:Math.max(...upper),
   status:'bounded-rigid-plane-support-envelope',
   model:'independent-support-intervals-linearized-rigid-plane'
  };
 }
 function applySuspensionTravelToVerticalRange(range,polygon,q,params,vehiclePart){
  if(!range)return null;
  const displacement=suspensionTravelDisplacementRange(polygon,q,params,vehiclePart);
  if(!displacement)return range;
  return {
   min:range.min+displacement.min,
   max:range.max+displacement.max,
   suspensionStatus:displacement.status,
   suspensionDisplacement:{min:displacement.min,max:displacement.max}
  };
 }

 function vehicleGeometryCalibrationStatus(params){
  const missingBounds=[];
  const axleSpacingResolution=params&&typeof params==='object'?
   resolveTrailerAxleSpacings(params):null;
  const check=(parameterKey,uncertaintyKey,label,active=true)=>{
   if(!active)return;
   const nominal=params&&params[parameterKey];
   const bound=params&&params[uncertaintyKey];
   if(!Number.isFinite(nominal)||!Number.isFinite(bound)||bound<=0){
    missingBounds.push({parameter:parameterKey,uncertainty:uncertaintyKey,label});
   }
  };
  check('wheelbase','wheelbaseUncertainty','колёсная база тягача');
  check('hitchOffset','hitchOffsetUncertainty','положение седла относительно задней оси');
  check('kingpinAxles','kingpinAxlesUncertainty','шкворень — эквивалентный центр тележки');
  check('trailerKingpinToFront','trailerKingpinToFrontUncertainty','расстояние от шкворня до переднего торца полуприцепа');
  check('trailerLength','trailerLengthUncertainty','длина полуприцепа');
  check('trailerWidth','trailerWidthUncertainty','ширина полуприцепа');
  const spacing12=(params&&params.trailerAxleSpacing12)>0;
  const spacing23=(params&&params.trailerAxleSpacing23)>0;
  check(spacing12?'trailerAxleSpacing12':'trailerAxleSpacing',
   spacing12?'trailerAxleSpacing12Uncertainty':'trailerAxleSpacingUncertainty','шаг осей 1–2');
  check(spacing23?'trailerAxleSpacing23':'trailerAxleSpacing',
   spacing23?'trailerAxleSpacing23Uncertainty':'trailerAxleSpacingUncertainty','шаг осей 2–3');
  if(axleSpacingResolution&&!axleSpacingResolution.valid&&
     !missingBounds.some(item=>item.parameter==='trailerAxleSpacing')){
   missingBounds.push({
    parameter:'trailerAxleSpacing12/23',
    uncertainty:'trailerAxleSpacing12Uncertainty/trailerAxleSpacing23Uncertainty',
    label:axleSpacingResolution.reason
   });
  }
  check('tractorSteeringTrack','tractorSteeringTrackUncertainty','колея управляемой оси тягача');
  check('tractorFrontTrack','tractorFrontTrackUncertainty','колея передней оси тягача');
  check('tractorRearTrack','tractorRearTrackUncertainty','колея задней оси тягача');
  for(let index=1;index<=3;index++){
   const individualTrack=(params&&params['trailerAxle'+index+'Track'])>0;
   check(individualTrack?'trailerAxle'+index+'Track':'trailerTrack',
    individualTrack?'trailerAxle'+index+'TrackUncertainty':'trailerTrackUncertainty',
    'колея '+index+'-й оси полуприцепа');
  }
  check('tireLength','tireLengthUncertainty','длина оболочки шины');
  check('tireWidth','tireWidthUncertainty','ширина одной шины');
  check('cabOffset','cabOffsetUncertainty','продольное положение кабины');
  check('cabFront','cabFrontUncertainty','передний вылет кабины');
  check('cabRear','cabRearUncertainty','задний вылет кабины');
  check('cabWidth','cabWidthUncertainty','ширина кабины');
  check('tractorFrontOverhang','tractorFrontOverhangUncertainty','передний вылет общего контура тягача от задней оси');
  check('tractorRearOverhang','tractorRearOverhangUncertainty','задний вылет общего контура тягача за заднюю ось');
  check('tractorBodyWidth','tractorBodyWidthUncertainty','ширина общего контура тягача');
  const footprint=params&&(Number.isFinite(params.vehicleFootprintUncertainty)?
   params.vehicleFootprintUncertainty:params.footprintUncertainty);
  if(!Number.isFinite(footprint)||footprint<=0){
   missingBounds.push({parameter:'vehicleFootprintUncertainty',uncertainty:'vehicleFootprintUncertainty',label:'общая оболочка габаритов автопоезда'});
  }
  if(params&&params.mirrorOutboard>0){
   check('mirrorOutboard','mirrorOutboardUncertainty','вынос зеркал');
   check('mirrorLongitudinal','mirrorLongitudinalUncertainty','продольное положение зеркал');
   check('mirrorLength','mirrorLengthUncertainty','длина оболочки зеркал');
  }
  if(params&&params.tractorRearTireCenterSpacing>0){
   check('tractorRearTireCenterSpacing','tractorRearTireCenterSpacingUncertainty','расстояние между центрами сдвоенных шин тягача');
  }else if(params&&params.tractorRearTireGroupWidth>0){
   check('tractorRearTireGroupWidth','tractorRearTireGroupWidthUncertainty','ширина группы задних шин тягача');
  }
  if(params&&params.trailerTireGroupWidth>0){
   check('trailerTireGroupWidth','trailerTireGroupWidthUncertainty','ширина группы шин полуприцепа');
  }
  for(let index=1;index<=3;index++){
   const spacingKey='trailerAxle'+index+'TireCenterSpacing';
   const widthKey='trailerAxle'+index+'TireGroupWidth';
   if(params&&params[spacingKey]>0){
    check(spacingKey,spacingKey+'Uncertainty','расстояние между центрами шин '+index+'-й оси');
   }else if(params&&params[widthKey]>0){
    check(widthKey,widthKey+'Uncertainty','ширина группы шин '+index+'-й оси');
   }
  }
  const auxiliaryEnvelopes=params&&params.auxiliaryEnvelopes;
  if(auxiliaryEnvelopes!==undefined){
   if(!Array.isArray(auxiliaryEnvelopes)){
    missingBounds.push({parameter:'auxiliaryEnvelopes',uncertainty:'auxiliaryEnvelopes',label:'дополнительные оболочки измеренных компонентов'});
   }else{
    auxiliaryEnvelopes.forEach((entry,index)=>{
     const fallbackName='оболочка '+(index+1);
     if(!entry||typeof entry!=='object'||Array.isArray(entry)){
      missingBounds.push({parameter:'auxiliaryEnvelopes['+index+']',uncertainty:'auxiliaryEnvelopes['+index+']',label:fallbackName});
      return;
     }
     const name=typeof entry.name==='string'&&entry.name.trim()?entry.name.trim():fallbackName;
     for(const key of ['longitudinalOffset','lateralOffset','headingOffsetDeg']){
      if(!Number.isFinite(entry[key])){
       missingBounds.push({parameter:'auxiliaryEnvelopes['+index+'].'+key,uncertainty:'auxiliaryEnvelopes['+index+'].uncertaintyMargin',label:name+': положение и ориентация должны быть заданы явно'});
      }
     }
     const margin=entry.uncertaintyMargin;
     if(!Number.isFinite(margin)||margin<0.001||margin>1){
      missingBounds.push({parameter:'auxiliaryEnvelopes['+index+'].uncertaintyMargin',uncertainty:'auxiliaryEnvelopes['+index+'].uncertaintyMargin',label:name+': положительная суммарная погрешность оболочки'});
     }
     const hasZMin=entry.zMin!==undefined,hasZMax=entry.zMax!==undefined;
     if(hasZMin||hasZMax){
      const verticalMargin=entry.verticalUncertaintyMargin;
      if(!hasZMin||!hasZMax||!Number.isFinite(verticalMargin)||verticalMargin<0.001||verticalMargin>1){
       missingBounds.push({parameter:'auxiliaryEnvelopes['+index+'].verticalUncertaintyMargin',uncertainty:'auxiliaryEnvelopes['+index+'].verticalUncertaintyMargin',label:name+': положительная вертикальная погрешность'});
      }
     }
    });
   }
  }
  return {
   status:missingBounds.length?'preliminary':'positive-bounds-present',
   missingBounds,
   positiveBoundsPresent:missingBounds.length===0,
   measurementProvenanceVerified:false,
   note:'Положительные границы — ввод оператора; функция не подтверждает источник, методику или фактическую точность измерений.'
  };
 }

 function validateVehicleParams(params){
  const errors=[];
  if(!params||typeof params!=='object'){
   return ['Параметры автопоезда отсутствуют.'];
  }
  // Resolve spacing without inventing a nominal default. Validation reports
  // the missing/invalid fallback; geometry helpers fail closed when called directly.
  const axleSpacingResolution=resolveTrailerAxleSpacings(params);
  if(!axleSpacingResolution.valid){
   errors.push('trailerAxleSpacing: '+axleSpacingResolution.reason);
  }
  for(const [key,limits] of Object.entries(VEHICLE_PARAMETER_LIMITS)){
   const value=params[key];
   if(limits.optional&&(value===undefined||value===null))continue;
   if(!Number.isFinite(value)){
    errors.push(key+': значение должно быть конечным числом.');
    continue;
   }
   if(value<limits.min||value>limits.max){
    errors.push(key+': значение вне допустимого диапазона '+limits.min+'–'+limits.max+'.');
   }
  }
  // The middle axle remains the equivalent kinematic reference. The two
  // measured adjacent spacings place the outer axle centers independently.
  const axleSpacings=axleSpacingResolution.spacings||[NaN,NaN];
  [
   ['trailerAxleSpacing12',params.trailerAxleSpacing12??0],
   ['trailerAxleSpacing23',params.trailerAxleSpacing23??0]
  ].forEach(([key,value])=>{
   if(Number.isFinite(value)&&value>0&&value<0.3){
    errors.push(key+': расстояние между соседними осями должно быть не меньше 0,30 м.');
   }
  });
  const trailerKingpinToFront=params.trailerKingpinToFront??1.5;
  if(Number.isFinite(params.trailerLength)&&Number.isFinite(trailerKingpinToFront)&&
     trailerKingpinToFront>params.trailerLength-0.5){
   errors.push('trailerKingpinToFront: задний вылет кузова за шкворнем должен оставаться не меньше 0,50 м.');
  }
  const validAxleEnvelope=Number.isFinite(params.kingpinAxles)&&
   params.kingpinAxles>=VEHICLE_PARAMETER_LIMITS.kingpinAxles.min&&
   params.kingpinAxles<=VEHICLE_PARAMETER_LIMITS.kingpinAxles.max&&
   Number.isFinite(params.trailerLength)&&
   params.trailerLength>=VEHICLE_PARAMETER_LIMITS.trailerLength.min&&
   params.trailerLength<=VEHICLE_PARAMETER_LIMITS.trailerLength.max&&
   Number.isFinite(trailerKingpinToFront)&&trailerKingpinToFront>=0.2&&
   trailerKingpinToFront<=params.trailerLength-0.5&&
   axleSpacings.every(spacing=>Number.isFinite(spacing)&&spacing>=0.3&&spacing<=5);
  if(validAxleEnvelope){
   const bodyFront=trailerKingpinToFront;
   const bodyRear=-(params.trailerLength-trailerKingpinToFront);
   trailerAxleOffsets(params).forEach((offset,index)=>{
    const position=-params.kingpinAxles+offset;
    if(position>bodyFront+1e-9||position<bodyRear-1e-9){
     errors.push('trailerAxleSpacing: продольное положение оси '+(index+1)+' выходит за границы кузова полуприцепа.');
    }
   });
  }

  // The outer tractor envelope is independently configurable; defaults preserve
  // the legacy illustration only and are not measured Volvo FH dimensions.
  const tractorFrontOverhang=params.tractorFrontOverhang??5.0;
  const tractorRearOverhang=params.tractorRearOverhang??1.2;
  const tractorBodyWidth=params.tractorBodyWidth??2.55;
  const cabOffset=params.cabOffset??2.1;
  const cabFront=params.cabFront??1.35;
  const cabRear=params.cabRear??0.9;
  const cabWidth=params.cabWidth??2.25;
  const validCabEnvelope=[tractorFrontOverhang,tractorRearOverhang,tractorBodyWidth,
   cabOffset,cabFront,cabRear,cabWidth].every(Number.isFinite)&&
   cabOffset>=0&&cabOffset<=6&&cabFront>=0.5&&cabFront<=3.5&&
   cabRear>=0.3&&cabRear<=2.5&&cabWidth>=1.8&&cabWidth<=2.55;
  if(validCabEnvelope&&(
   cabOffset+cabFront>tractorFrontOverhang+1e-9||
   cabOffset-cabRear< -tractorRearOverhang-1e-9||
   cabWidth>tractorBodyWidth+1e-9
  )){
   errors.push('Геометрия кабины выходит за пределы измеряемого общего контура тягача; проверьте передний/задний вылет и ширину.');
  }

  const mirrorOutboard=params.mirrorOutboard??0;
  const mirrorLongitudinal=params.mirrorLongitudinal??3.0;
  const mirrorLength=params.mirrorLength??0.8;
  if(validCabEnvelope&&Number.isFinite(mirrorOutboard)&&mirrorOutboard>0&&
     Number.isFinite(mirrorLongitudinal)&&Number.isFinite(mirrorLength)&&
     mirrorOutboard<=1&&mirrorLongitudinal>=-2&&mirrorLongitudinal<=8&&
     mirrorLength>=0.2&&mirrorLength<=2){
   // Mirror housings may extend beyond the cab's front/rear edges, but
   // they must remain within the tractor's outer longitudinal envelope.
   // This is a packaging bound, not a measured Volvo mirror-bracket model.
   const tractorRearEdge=-tractorRearOverhang;
   const tractorFrontEdge=tractorFrontOverhang;
   if(mirrorLongitudinal-mirrorLength/2<tractorRearEdge-1e-9||
      mirrorLongitudinal+mirrorLength/2>tractorFrontEdge+1e-9){
    errors.push('mirrorLongitudinal/mirrorLength: оболочка зеркал должна оставаться внутри текущего продольного контура тягача (-'+tractorRearOverhang.toFixed(2)+'…'+tractorFrontOverhang.toFixed(2)+' м).');
   }
  }

  // Track and tire-group widths are calibration inputs, not factory specifications.
  // A zero axle-specific value inherits the common track; a zero group width
  // falls back to the single-tire envelope.
  const frontTrack=params.tractorFrontTrack??2.05;
  const rearTrack=params.tractorRearTrack??2.05;
  const tireWidth=params.tireWidth??0.44;
  const rearGroupInput=params.tractorRearTireGroupWidth??0;
  const rearPairSpacing=params.tractorRearTireCenterSpacing??0;
  const rearGroupWidth=rearPairSpacing>0?rearPairSpacing+tireWidth:(rearGroupInput>0?rearGroupInput:tireWidth);
  const axleGroupInputs=[1,2,3].map(index=>params['trailerAxle'+index+'TireGroupWidth']??0);
  const axlePairSpacings=trailerAxleTireCenterSpacings(params);
  const pairSpacingSpecs=[
   ['tractorRearTireCenterSpacing',rearPairSpacing],
   ...axlePairSpacings.map((value,index)=>['trailerAxle'+(index+1)+'TireCenterSpacing',value])
  ];
  pairSpacingSpecs.forEach(([key,value])=>{
   if(value>0&&value<tireWidth-1e-9){
    errors.push(key+': расстояние между центрами покрышек не может быть меньше ширины одной шины.');
   }
   if(value>0&&value+tireWidth>1.5+1e-9){
    errors.push(key+': наружная ширина пары покрышек превышает 1,50 м.');
   }
  });
  const axleGroupWidths=trailerAxleTireGroupWidths(params);
  const axleTracks=trailerAxleTracks(params);
  const hasIndividualTrailerTrack=[1,2,3].some(index=>(params['trailerAxle'+index+'Track']??0)>0);
  const hasIndividualTrailerGroupWidth=axleGroupInputs.some(value=>value>0)||axlePairSpacings.some(value=>value>0);
  let commonTrailerEnvelopeErrorAdded=false;
  const commonTrailerTrack=params.trailerTrack??(Number.isFinite(params.trailerWidth)?params.trailerWidth+0.24:NaN);
  [1,2,3].forEach(index=>{
   const key='trailerAxle'+index+'Track';
   const value=params[key]??0;
   if(Number.isFinite(value)&&value>0&&value<1){
    errors.push(key+': колея оси должна быть не меньше 1,00 м.');
   }
  });
  if(rearPairSpacing<=0&&rearGroupInput>0&&rearGroupInput<tireWidth-1e-9){
   errors.push('tractorRearTireGroupWidth: ширина группы не может быть меньше ширины одной шины.');
  }
  if((params.trailerTireGroupWidth??0)>0&&params.trailerTireGroupWidth<tireWidth-1e-9){
   errors.push('trailerTireGroupWidth: ширина группы не может быть меньше ширины одной шины.');
  }
  axleGroupInputs.forEach((value,index)=>{
   if(axlePairSpacings[index]<=0&&value>0&&value<tireWidth-1e-9){
    errors.push('trailerAxle'+(index+1)+'TireGroupWidth: ширина группы не может быть меньше ширины одной шины.');
   }
  });
  if([frontTrack,rearTrack,commonTrailerTrack,tireWidth,rearGroupWidth,...axleGroupWidths,...axleTracks].every(Number.isFinite)){
   if(rearGroupWidth>=rearTrack){
    errors.push((rearPairSpacing>0?'tractorRearTireCenterSpacing':'tractorRearTireGroupWidth')+': левая и правая группы шин перекрываются при заданной колее.');
   }
   axleTracks.forEach((track,index)=>{
    const groupWidth=axleGroupWidths[index];
    const widthInput=axleGroupInputs[index];
    const widthKey=axlePairSpacings[index]>0?'trailerAxle'+(index+1)+'TireCenterSpacing':
     (widthInput>0?'trailerAxle'+(index+1)+'TireGroupWidth':
      ((params.trailerTireGroupWidth??0)>0?'trailerTireGroupWidth':'tireWidth'));
    if(groupWidth>=track){
     errors.push(widthKey+': левая и правая группы шин '+(index+1)+'-й оси перекрываются при заданной колее.');
    }
    if(Number.isFinite(params.trailerWidth)&&track+groupWidth>params.trailerWidth+1.0+1e-9){
     if(!hasIndividualTrailerTrack&&!hasIndividualTrailerGroupWidth){
      if(!commonTrailerEnvelopeErrorAdded){
       errors.push('trailerTrack/trailerTireGroupWidth: колёсные группы выходят за допустимую калибровочную оболочку относительно ширины полуприцепа.');
       commonTrailerEnvelopeErrorAdded=true;
      }
     }else{
      errors.push('trailerAxle'+(index+1)+'Track/trailerAxle'+(index+1)+'TireGroupWidth: наружная ширина колёсной группы выходит за допустимую калибровочную оболочку относительно ширины полуприцепа.');
     }
    }
   });
   if(frontTrack+tireWidth>3.2+1e-9||rearTrack+rearGroupWidth>3.2+1e-9){
    errors.push('tractorTrack/tireWidth/groupWidth: наружная ширина колёсных групп тягача выходит за калибровочную оболочку 3,20 м.');
   }
  }
  errors.push(...validateAuxiliaryEnvelopes(params.auxiliaryEnvelopes));
  errors.push(...validateVehicleVerticalProfiles(params.vehicleVerticalProfiles));
  errors.push(...validateRoadSurface(params.roadSurface));
  errors.push(...validateSuspensionTravel(params.suspensionTravel,params));
  return errors;
 }


 function derivative(q,v,delta,params){
  const yawRate=v/params.wheelbase*Math.tan(delta);
  const beta=norm(q.psi-q.psiT);
  // No-slip constraint at the trailer axle: lateral axle velocity is zero.
  const trailerYawRate=(
   v*Math.sin(beta)
   -params.hitchOffset*yawRate*Math.cos(beta)
  )/params.kingpinAxles;

  return{
   x:v*Math.cos(q.psi),
   y:v*Math.sin(q.psi),
   psi:yawRate,
   psiT:trailerYawRate
  };
 }

 function addState(q,k,f){
  return{
   x:q.x+k.x*f,
   y:q.y+k.y*f,
   psi:q.psi+k.psi*f,
   psiT:q.psiT+k.psiT*f
  };
 }

 function rk4(q,v,delta,dt,params){
  const k1=derivative(q,v,delta,params);
  const k2=derivative(addState(q,k1,dt/2),v,delta,params);
  const k3=derivative(addState(q,k2,dt/2),v,delta,params);
  const k4=derivative(addState(q,k3,dt),v,delta,params);

  return{
   x:q.x+dt*(k1.x+2*k2.x+2*k3.x+k4.x)/6,
   y:q.y+dt*(k1.y+2*k2.y+2*k3.y+k4.y)/6,
   psi:norm(q.psi+dt*(k1.psi+2*k2.psi+2*k3.psi+k4.psi)/6),
   psiT:norm(q.psiT+dt*(k1.psiT+2*k2.psiT+2*k3.psiT+k4.psiT)/6)
  };
 }

 
 function localPoint(origin,angle,longitudinal,lateral=0){
  return{
   x:origin.x+longitudinal*Math.cos(angle)-lateral*Math.sin(angle),
   y:origin.y+longitudinal*Math.sin(angle)+lateral*Math.cos(angle)
  };
 }

 function hitchPoint(q,params){
  return localPoint(q,q.psi,-params.hitchOffset);
 }

 function axlePoint(q,params){
  const h=hitchPoint(q,params);
  return localPoint(h,q.psiT,-params.kingpinAxles);
 }

 function tractorAxlePoints(q,params){
  if(!q||![q.x,q.y,q.psi].every(Number.isFinite)||!params||
   !Number.isFinite(params.wheelbase)||params.wheelbase<=0){
   throw new Error('Параметры осей тягача некорректны.');
  }
  // q is the rear-axle reference: the steer axle is one wheelbase forward.
  return [
   localPoint(q,q.psi,params.wheelbase),
   {x:q.x,y:q.y}
  ];
 }

 function bodyCorners(origin,angle,front,rear,width){
  return[
   localPoint(origin,angle,front,width/2),
   localPoint(origin,angle,rear,width/2),
   localPoint(origin,angle,rear,-width/2),
   localPoint(origin,angle,front,-width/2)
  ];
 }

 function polygonOutsideEnvelope(poly,origin,angle,front,rear,width){
  const c=Math.cos(angle),s=Math.sin(angle),halfWidth=width/2;
  return poly.some(point=>{
   const dx=point.x-origin.x,dy=point.y-origin.y;
   const longitudinal=dx*c+dy*s;
   const lateral=-dx*s+dy*c;
   return longitudinal>front+1e-9||longitudinal<rear-1e-9||Math.abs(lateral)>halfWidth+1e-9;
  });
 }

 function vehiclePolygons(q,params){
  const h=hitchPoint(q,params);
  const cabOffset=params.cabOffset??2.1;
  const cabFront=params.cabFront??1.35;
  const cabRear=params.cabRear??0.9;
  const cabWidth=params.cabWidth??2.25;
  const cab=localPoint(q,q.psi,cabOffset);
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  const trailerKingpinToFront=params.trailerKingpinToFront??1.5;
  const tractorFrontOverhang=params.tractorFrontOverhang??5.0;
  const tractorRearOverhang=params.tractorRearOverhang??1.2;
  const tractorBodyWidth=params.tractorBodyWidth??2.55;
  // The tractor rectangle is a measured/configurable outer envelope. Fallbacks
  // retain the legacy display only; calibration status still reports missing bounds.
  return[
   bodyCorners(q,q.psi,tractorFrontOverhang+uncertainty,-tractorRearOverhang-uncertainty,tractorBodyWidth+2*uncertainty),
   bodyCorners(cab,q.psi,cabFront+uncertainty,-cabRear-uncertainty,cabWidth+2*uncertainty),
   bodyCorners(h,q.psiT,trailerKingpinToFront+uncertainty,-(params.trailerLength-trailerKingpinToFront+uncertainty),params.trailerWidth+2*uncertainty)
  ];
 }

 // Optional side-mirror envelopes. mirrorOutboard is the measured projection
 // beyond each cab side; zero means the mirrors are not included in collision geometry.
 function mirrorPolygons(q,params){
  const outboard=params.mirrorOutboard??0;
  if(!Number.isFinite(outboard)||outboard<=0)return [];
  const cabWidth=params.cabWidth??2.25;
  const longitudinal=params.mirrorLongitudinal??3.0;
  const length=params.mirrorLength??0.8;
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  return [1,-1].map((side,index)=>{
   const center=localPoint(q,q.psi,longitudinal,side*(cabWidth/2+outboard/2));
   const polygon=bodyCorners(center,q.psi,length/2+uncertainty,-length/2-uncertainty,outboard+2*uncertainty);
   return {
    name:index===0?'левое зеркало':'правое зеркало',
    polygon,
    verticalRange:applySuspensionTravelToVerticalRange(vehicleVerticalRange(params,'mirror'),polygon,q,params,'tractor')
   };
  });
 }


 // User-measured non-tire hardware boxes (hub, brake chamber, suspension arm, etc.).
 // Each box is anchored to an axle reference and expanded by its stated margin.
 function auxiliaryEnvelopePolygons(q,params){
  const envelopes=params&&params.auxiliaryEnvelopes;
  const errors=validateAuxiliaryEnvelopes(envelopes);
  if(errors.length)throw new Error('Некорректные дополнительные оболочки: '+errors.join(' '));
  if(!Array.isArray(envelopes)||envelopes.length===0)return [];
  const tractorAxles=tractorAxlePoints(q,params);
  const trailerAxles=trailerAxlePoints(q,params);
  return envelopes.map(entry=>{
   const isTractor=entry.anchor.startsWith('tractor-');
   const anchor=entry.anchor==='tractor-front-axle'?tractorAxles[0]:
    (entry.anchor==='tractor-rear-axle'?tractorAxles[1]:
     trailerAxles[Number(entry.anchor.slice(-1))-1]);
   const baseHeading=isTractor?q.psi:q.psiT;
   const heading=norm(baseHeading+(entry.headingOffsetDeg??0)*Math.PI/180);
   const center=localPoint(anchor,heading,entry.longitudinalOffset??0,entry.lateralOffset??0);
   const margin=(entry.uncertaintyMargin??0)+(Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0);
   const polygon=bodyCorners(center,heading,entry.length/2+margin,-entry.length/2-margin,entry.width+2*margin);
   const suspensionAttachment=entry.suspensionAttachment??'axle';
   const verticalRange=entry.zMin!==undefined?verticalRangeFromProfile({
    zMin:entry.zMin,zMax:entry.zMax,uncertaintyMargin:entry.verticalUncertaintyMargin
   }):null;
   const attachedVerticalRange=suspensionAttachment==='tractor-body'?
    applySuspensionTravelToVerticalRange(verticalRange,polygon,q,params,'tractor'):
    (suspensionAttachment==='trailer-body'?
     applySuspensionTravelToVerticalRange(verticalRange,polygon,q,params,'trailer'):verticalRange);
   return {
    name:entry.name.trim(),
    anchor:entry.anchor,
    vehiclePart:isTractor?'тягач':'полуприцеп',
    componentType:'measured-auxiliary-envelope',
    uncertaintyMargin:margin,
    suspensionAttachment,
    verticalRange:attachedVerticalRange,
    polygon
   };
  });
 }

 // Collision components are grouped into the same three driver-facing parts.
 // Mirrors are separate cab components, so their nearest obstacle can be named.
 function vehicleCollisionParts(q,params,steer=Number.isFinite(q&&q.steer)?q.steer:0,includeInternalComponentsForVertical=false){
  if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
   throw new Error('Угол поворота колёс для проверки столкновений некорректен.');
  }
  const bodies=vehiclePolygons(q,params);
  const mirrors=mirrorPolygons(q,params);
  const wheels=buildWheelFootprints(q,params,steer);
  const auxiliary=auxiliaryEnvelopePolygons(q,params);
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  const tractorFrontOverhang=params.tractorFrontOverhang??5.0;
  const tractorRearOverhang=params.tractorRearOverhang??1.2;
  const tractorBodyWidth=params.tractorBodyWidth??2.55;
  const tractorWheelComponents=wheels.filter(wheel=>
   wheel.vehiclePart==='тягач'&&(includeInternalComponentsForVertical||!!wheel.verticalRange||polygonOutsideEnvelope(
    wheel.polygon,q,q.psi,tractorFrontOverhang+uncertainty,-tractorRearOverhang-uncertainty,tractorBodyWidth+2*uncertainty
   ))
  );
  const tractorAuxiliaryComponents=auxiliary.filter(component=>
   component.vehiclePart==='тягач'&&(includeInternalComponentsForVertical||!!component.verticalRange||polygonOutsideEnvelope(
    component.polygon,q,q.psi,tractorFrontOverhang+uncertainty,-tractorRearOverhang-uncertainty,tractorBodyWidth+2*uncertainty
   ))
  );
  const hitch=hitchPoint(q,params);
  const trailerKingpinToFront=params.trailerKingpinToFront??1.5;
  const trailerWheelComponents=wheels.filter(wheel=>
   wheel.vehiclePart==='полуприцеп'&&(includeInternalComponentsForVertical||!!wheel.verticalRange||polygonOutsideEnvelope(
    wheel.polygon,hitch,q.psiT,trailerKingpinToFront+uncertainty,
    -(params.trailerLength-trailerKingpinToFront+uncertainty),params.trailerWidth+2*uncertainty
   ))
  );
  const trailerAuxiliaryComponents=auxiliary.filter(component=>
   component.vehiclePart==='полуприцеп'&&(includeInternalComponentsForVertical||!!component.verticalRange||polygonOutsideEnvelope(
    component.polygon,hitch,q.psiT,trailerKingpinToFront+uncertainty,
    -(params.trailerLength-trailerKingpinToFront+uncertainty),params.trailerWidth+2*uncertainty
   ))
  );
  const tractorBodyRange=applySuspensionTravelToVerticalRange(vehicleVerticalRange(params,'tractor-body'),bodies[0],q,params,'tractor');
  const tractorCabRange=applySuspensionTravelToVerticalRange(vehicleVerticalRange(params,'tractor-cab'),bodies[1],q,params,'tractor');
  const trailerBodyRange=applySuspensionTravelToVerticalRange(vehicleVerticalRange(params,'trailer-body'),bodies[2],q,params,'trailer');
  return [
   {vehiclePart:'тягач',bodyIndex:0,components:[
    {name:'основной контур тягача',polygon:bodies[0],verticalRange:tractorBodyRange},
    ...tractorWheelComponents.map(wheel=>({name:wheel.name,polygon:wheel.polygon,verticalRange:wheel.verticalRange})),
    ...tractorAuxiliaryComponents.map(component=>({name:component.name,polygon:component.polygon,componentType:component.componentType,verticalRange:component.verticalRange}))
   ]},
   {vehiclePart:'кабина',bodyIndex:1,components:[
    {name:'кабина',polygon:bodies[1],verticalRange:tractorCabRange},
    ...mirrors
   ]},
   {vehiclePart:'полуприцеп',bodyIndex:2,components:[
    {name:'кузов полуприцепа',polygon:bodies[2],verticalRange:trailerBodyRange},
    ...trailerWheelComponents.map(wheel=>({name:wheel.name,polygon:wheel.polygon,verticalRange:wheel.verticalRange})),
    ...trailerAuxiliaryComponents.map(component=>({name:component.name,polygon:component.polygon,componentType:component.componentType,verticalRange:component.verticalRange}))
   ]}
  ];
 }

 function trailerAxleOffsets(params){
  const [spacing12,spacing23]=trailerAxleSpacings(params);
  return [spacing12,0,-spacing23];
 }

 function trailerAxlePoints(q,params){
  const center=axlePoint(q,params);
  return trailerAxleOffsets(params).map(offset=>localPoint(center,q.psiT,offset));
 }

 // Convert the planner's equivalent bicycle steering angle into left/right
 // Ackermann wheel headings. The steering-track input approximates the distance
 // between the front steering pivot axes; it is not a measured linkage model.
 function frontWheelSteerAngles(steer,params){
  if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
   throw new Error('Угол эквивалентного рулевого колеса некорректен.');
  }
  const wheelbase=params&&params.wheelbase;
  const steeringTrack=params?
   (params.tractorSteeringTrack??params.tractorFrontTrack??2.05):NaN;
  if(!Number.isFinite(wheelbase)||wheelbase<=0||
     !Number.isFinite(steeringTrack)||steeringTrack<=0){
   throw new Error('Колёсная база и расстояние между осями поворота передних колёс должны быть положительными.');
  }
  if(Math.abs(steer)<1e-12)return {left:0,right:0};
  const turnRadius=wheelbase/Math.abs(Math.tan(steer));
  const halfTrack=steeringTrack/2;
  if(!Number.isFinite(turnRadius)||turnRadius<=halfTrack+1e-9){
   throw new Error('Радиус поворота слишком мал для геометрии Аккермана при заданной колее рулевой оси.');
  }
  const inner=Math.atan2(wheelbase,turnRadius-halfTrack);
  const outer=Math.atan2(wheelbase,turnRadius+halfTrack);
  return steer>0?{left:inner,right:outer}:{left:-outer,right:-inner};
 }

 // The baseline has ten axle-side footprints. A measured tire-center spacing
 // expands one dual group into two individual tire rectangles; without it, the
 // configured group envelope (or explicit single-tire fallback) is retained.
 function buildWheelFootprints(q,params,steer=0){
  const frontTrack=params.tractorFrontTrack??2.05;
  const rearTrack=params.tractorRearTrack??2.05;
  const trailerTracks=trailerAxleTracks(params);
  const tireLength=params.tireLength??0.72;
  const tireWidth=params.tireWidth??0.44;
  const rearPairSpacing=Number.isFinite(params.tractorRearTireCenterSpacing)?params.tractorRearTireCenterSpacing:0;
  const rearGroupConfigured=Number.isFinite(params.tractorRearTireGroupWidth)&&params.tractorRearTireGroupWidth>0;
  const trailerGroupConfigured=Number.isFinite(params.trailerTireGroupWidth)&&params.trailerTireGroupWidth>0;
  const rearGroupWidth=rearPairSpacing>0?rearPairSpacing+tireWidth:(rearGroupConfigured?params.tractorRearTireGroupWidth:tireWidth);
  const trailerGroupWidths=trailerAxleTireGroupWidths(params);
  const trailerPairSpacings=trailerAxleTireCenterSpacings(params);
  const uncertainty=Number.isFinite(params.footprintUncertainty)?params.footprintUncertainty:0;
  const steeringAngles=frontWheelSteerAngles(steer,params);
  const wheels=[];
  const add=(vehiclePart,axleIndex,side,center,heading,name,footprintWidth=tireWidth,footprintModel='single-tire-assumption',metadata={})=>{
   wheels.push({
    vehiclePart,axleIndex,side,center,heading,name,
    footprintWidth,footprintModel,...metadata,
    verticalRange:wheelVerticalRange(params,vehiclePart,axleIndex),
    groupWidthConfigured:footprintModel==='configured-axle-side-group-envelope'||footprintModel==='explicit-dual-tire-pair',
    polygon:bodyCorners(center,heading,
     tireLength/2+uncertainty,-tireLength/2-uncertainty,footprintWidth+2*uncertainty)
   });
  };
  const addDualPair=(vehiclePart,axleIndex,side,groupCenter,heading,baseName,centerSpacing)=>{
   const sideSign=side==='left'?1:-1;
   const outerOffset=sideSign*centerSpacing/2;
   const innerOffset=-outerOffset;
   add(vehiclePart,axleIndex,side,localPoint(groupCenter,heading,0,outerOffset),heading,
    baseName+' — наружная покрышка',tireWidth,'explicit-dual-tire-pair',
    {pairPosition:'outer',pairLateralOffset:outerOffset,groupCenter,pairCenterSpacing:centerSpacing});
   add(vehiclePart,axleIndex,side,localPoint(groupCenter,heading,0,innerOffset),heading,
    baseName+' — внутренняя покрышка',tireWidth,'explicit-dual-tire-pair',
    {pairPosition:'inner',pairLateralOffset:innerOffset,groupCenter,pairCenterSpacing:centerSpacing});
  };
  const [frontAxle,rearAxle]=tractorAxlePoints(q,params);
  for(const side of [1,-1]){
   const sideName=side>0?'левое':'правое';
   add('тягач',1,side>0?'left':'right',
    localPoint(frontAxle,q.psi,0,side*frontTrack/2),
    q.psi+(side>0?steeringAngles.left:steeringAngles.right),
    sideName+' переднее колесо тягача');
   const rearGroupCenter=localPoint(rearAxle,q.psi,0,side*rearTrack/2);
   if(rearPairSpacing>0){
    addDualPair('тягач',2,side>0?'left':'right',rearGroupCenter,q.psi,
     sideName+' заднее колесо тягача',rearPairSpacing);
   }else{
    add('тягач',2,side>0?'left':'right',rearGroupCenter,q.psi,
     sideName+' заднее колесо тягача',rearGroupWidth,
     rearGroupConfigured?'configured-axle-side-group-envelope':'single-tire-fallback');
   }
  }
  trailerAxlePoints(q,params).forEach((axle,index)=>{
   const trailerTrack=trailerTracks[index];
   const specific=params['trailerAxle'+(index+1)+'TireGroupWidth'];
   const specificConfigured=Number.isFinite(specific)&&specific>0;
   const groupConfigured=specificConfigured||trailerGroupConfigured;
   const groupWidth=trailerGroupWidths[index];
   for(const side of [1,-1]){
    const sideName=side>0?'левое':'правое';
    const groupCenter=localPoint(axle,q.psiT,0,side*trailerTrack/2);
    const pairSpacing=trailerPairSpacings[index];
    if(pairSpacing>0){
     addDualPair('полуприцеп',index+1,side>0?'left':'right',groupCenter,q.psiT,
      sideName+' колесо '+(index+1)+'-й оси полуприцепа',pairSpacing);
    }else{
     add('полуприцеп',index+1,side>0?'left':'right',groupCenter,q.psiT,
      sideName+' колесо '+(index+1)+'-й оси полуприцепа',groupWidth,
      groupConfigured?'configured-axle-side-group-envelope':'single-tire-fallback');
    }
   }
  });
  return wheels;
 }

 function wheelFootprints(q,params,steer=Number.isFinite(q&&q.steer)?q.steer:0){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректные параметры колёс: '+errors.join(' '));
  if(!q||![q.x,q.y,q.psi,q.psiT].every(Number.isFinite)){
   throw new Error('Положение автопоезда для геометрии колёс некорректно.');
  }
  if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
   throw new Error('Угол поворота передних колёс некорректен.');
  }
  return buildWheelFootprints(q,params,steer);
 }

 // Rigid-body lateral velocity relative to the equivalent axle-group center.
 // This is a scrub indicator, not a tire-force or multi-body dynamics model.
 function trailerAxleDiagnostics(q,v,delta,params){
  const motion=derivative(q,v,delta,params);
  const offsets=trailerAxleOffsets(params);
  const axles=offsets.map((offset,index)=>({
   index:index+1,
   offsetFromGroupCenter:offset,
   lateralVelocity:motion.psiT*offset
  }));
  return {
   trailerYawRate:motion.psiT,
   axles,
   maxAbsLateralVelocity:Math.max(...axles.map(axle=>Math.abs(axle.lateralVelocity)))
  };
 }

 // Integrates a rigid-axle-group geometric incompatibility proxy along a sampled path.
 // It is not a tire-slip, tire-force, wear, or thermal model.
 function trailerAxleScrubExposure(path,params,nominalSpeed=0.7){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректная геометрия осевой группы: '+errors.join(' '));
  if(!Array.isArray(path))throw new Error('Траектория должна быть массивом состояний.');
  if(path.length===0)throw new Error('Траектория должна содержать хотя бы одно состояние.');
  if(!Number.isFinite(nominalSpeed)||nominalSpeed<=0||nominalSpeed>20){
   throw new Error('Расчётная скорость должна быть больше нуля и не превышать 20 м/с.');
  }
  const readState=(q,index)=>{
   if(!q||![q.x,q.y,q.psi,q.psiT].every(Number.isFinite)){
    throw new Error('Состояние траектории '+(index+1)+' содержит некорректные координаты или углы.');
   }
   const gear=q.gear===undefined?1:q.gear;
   const steer=q.steer===undefined?0:q.steer;
   if(gear!==1&&gear!==-1)throw new Error('Передача в состоянии '+(index+1)+' должна быть +1 или -1.');
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Угол руля в состоянии '+(index+1)+' некорректен.');
   }
   if(q.stepDistance!==undefined&&(
    !Number.isFinite(q.stepDistance)||
    (index===0?q.stepDistance!==0:q.stepDistance<=0)
   )){
    throw new Error('stepDistance в состоянии '+(index+1)+' должен быть конечным; для начального состояния — 0, для следующих — положительным.');
   }
   return {q,gear,steer};
  };
  const offsets=trailerAxleOffsets(params);
  const axleTravel=offsets.map(()=>0);
  const maxAbsLateralVelocityByAxle=offsets.map(()=>0);
  // Array.from visits sparse slots so a hole cannot masquerade as an empty, zero-exposure route.
  const states=Array.from(path,readState);
  const evaluate=(q,steer)=>{
   // Absolute lateral speed is identical in forward and reverse for the same pose and steering angle.
   const diagnostic=trailerAxleDiagnostics(q,nominalSpeed,steer,params);
   return diagnostic.axles.map((axle,index)=>{
    const speed=Math.abs(axle.lateralVelocity);
    maxAbsLateralVelocityByAxle[index]=Math.max(maxAbsLateralVelocityByAxle[index],speed);
    return speed;
   });
  };
  states.forEach(state=>evaluate(state.q,state.steer));
  const interpolateState=(a,b,t)=>({
   x:a.q.x+(b.q.x-a.q.x)*t,
   y:a.q.y+(b.q.y-a.q.y)*t,
   psi:norm(a.q.psi+norm(b.q.psi-a.q.psi)*t),
   psiT:norm(a.q.psiT+norm(b.q.psiT-a.q.psiT)*t)
  });
  // Composite Simpson quadrature (4 equal subintervals) reduces midpoint bias.
  // It remains a sampled-path diagnostic, not a certified integration-error bound.
  const sampleTimes=[0,0.25,0.5,0.75,1];
  let segmentsUsingStepDistance=0,segmentsUsingChordFallback=0;
  for(let i=1;i<states.length;i++){
   const previous=states[i-1],current=states[i];
   const chord=Math.hypot(current.q.x-previous.q.x,current.q.y-previous.q.y);
   let distance=chord;
   if(current.q.stepDistance!==undefined){
    if(current.q.stepDistance<=0||chord>current.q.stepDistance+1e-6){
     throw new Error('Хорда траектории превышает stepDistance на сегменте '+i+'.');
    }
    distance=current.q.stepDistance;
    segmentsUsingStepDistance++;
   }else{
    segmentsUsingChordFallback++;
   }
   if(distance<=0)continue;
   const samples=sampleTimes.map(t=>{
    const q=interpolateState(previous,current,t);
    const steer=previous.steer+(current.steer-previous.steer)*t;
    return evaluate(q,steer);
   });
   // Planner stepDistance is the traveled rear-axle arc length. The endpoint
   // chord is only a fallback for externally supplied paths without that field.
   const elapsed=distance/nominalSpeed;
   for(let axle=0;axle<offsets.length;axle++){
    const f0=samples[0][axle],f1=samples[1][axle],f2=samples[2][axle];
    const f3=samples[3][axle],f4=samples[4][axle];
    axleTravel[axle]+=elapsed*(f0+4*f1+2*f2+4*f3+f4)/12;
   }
  }
  const totalAbsoluteLateralTravel=axleTravel.reduce((sum,value)=>sum+value,0);
  const maxAbsLateralVelocity=Math.max(0,...maxAbsLateralVelocityByAxle);
  return {
   nominalSpeed,
   maxAbsLateralVelocity,
   maxAbsLateralVelocityByAxle,
   cumulativeAbsoluteLateralTravelByAxle:axleTravel,
   maxAbsoluteLateralTravelByAxle:axleTravel.length?Math.max(...axleTravel):0,
   totalAbsoluteLateralTravel,
   integration:'composite-simpson-4-subintervals',
   distanceMethod:segmentsUsingStepDistance&&segmentsUsingChordFallback?
    'step-distance-when-available-otherwise-chord':
    (segmentsUsingStepDistance?'planner-step-distance':'chord-fallback'),
   segmentsUsingStepDistance,segmentsUsingChordFallback,
   model:'rigid-axle-group-kinematic-proxy'
  };
 }


 // Returns sampled center tracks for six trailer axle-side tire groups and all three axle centers.
 // Left/right inner-side classification is relative to the direction of travel:
 // reverse gear swaps the vehicle-side label. Distances are chords between route samples.
 function trailerWheelTrajectoryDiagnostics(path,params){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректная геометрия траектории колёс: '+errors.join(' '));
  if(!Array.isArray(path))throw new Error('Траектория колёс должна быть массивом состояний.');
  if(path.length===0)throw new Error('Траектория колёс должна содержать хотя бы одно состояние.');
  const readState=(state,index)=>{
   if(!state||![state.x,state.y,state.psi,state.psiT].every(Number.isFinite)){
    throw new Error('Состояние траектории колёс '+(index+1)+' содержит некорректные координаты или углы.');
   }
   const gear=state.gear===undefined?1:state.gear;
   const steer=state.steer===undefined?0:state.steer;
   if(gear!==1&&gear!==-1)throw new Error('Передача в состоянии траектории колёс '+(index+1)+' должна быть +1 или -1.');
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Угол руля в состоянии траектории колёс '+(index+1)+' некорректен.');
   }
   return {q:state,gear,steer};
  };
  const states=Array.from(path,readState);
  const tracks=trailerAxleTracks(params);
  if(!tracks.every(track=>Number.isFinite(track)&&track>0))throw new Error('Колея одной из осей полуприцепа для траектории колёс некорректна.');
  const offsets=trailerAxleOffsets(params);
  const makeTrack=side=>({
   side,
   name:side==='center'?'центр оси':(side==='left'?'левая группа шин':'правая группа шин'),
   points:[],
   sampledTravel:0,
   integratedTravelAccumulator:0,
   integratedTravelCoveredDistance:0
  });
  const axles=offsets.map((offset,index)=>({
   axleIndex:index+1,offsetFromGroupCenter:offset,
   center:makeTrack('center'),
   left:makeTrack('left'),
   right:makeTrack('right')
  }));
  const segments=[];
  const sampleFractions=[0,0.25,0.5,0.75,1];
  const endpointTolerance={position:1e-5,heading:1e-6};
  let routeDistance=0;
  let previousTracks=null;
  let previousState=null;
  let segmentsUsingStepDistance=0,segmentsUsingChordFallback=0;
  let segmentsWithKinematicEstimate=0,segmentsWithoutKinematicEstimate=0;

  // Speed of a trailer wheel center per metre travelled by the tractor rear axle.
  // The state derivative is evaluated at unit signed reference speed (+/-1 m/s).
  const wheelSpeedPerReferenceMetre=(q,gear,steer,axleOffset,lateralOffset)=>{
   const motion=derivative(q,gear,steer,params);
   const hitchX=-params.hitchOffset*Math.cos(q.psi);
   const hitchY=-params.hitchOffset*Math.sin(q.psi);
   const hitchVx=motion.x-motion.psi*hitchY;
   const hitchVy=motion.y+motion.psi*hitchX;
   const longitudinal=-params.kingpinAxles+axleOffset;
   const wheelX=longitudinal*Math.cos(q.psiT)-lateralOffset*Math.sin(q.psiT);
   const wheelY=longitudinal*Math.sin(q.psiT)+lateralOffset*Math.cos(q.psiT);
   const vx=hitchVx-motion.psiT*wheelY;
   const vy=hitchVy+motion.psiT*wheelX;
   return Math.hypot(vx,vy);
  };
  const simpsonDistance=(distance,values)=>
   distance*(values[0]+4*values[1]+2*values[2]+4*values[3]+values[4])/12;

  for(let index=0;index<states.length;index++){
   const state=states[index];
   if(index===0&&state.q.stepDistance!==undefined&&
      (!Number.isFinite(state.q.stepDistance)||state.q.stepDistance!==0)){
    throw new Error('Начальное состояние траектории колёс должно иметь stepDistance равный нулю.');
   }
   let segmentDistance=0;
   let usesStepDistance=false;
   if(index>0){
    const chord=Math.hypot(state.q.x-previousState.q.x,state.q.y-previousState.q.y);
    segmentDistance=chord;
    if(state.q.stepDistance!==undefined){
     if(!Number.isFinite(state.q.stepDistance)||state.q.stepDistance<=0){
      throw new Error('Состояние траектории колёс '+(index+1)+' содержит некорректный stepDistance.');
     }
     if(chord>state.q.stepDistance+1e-6){
      throw new Error('Хорда траектории колёс превышает stepDistance на сегменте '+index+'.');
     }
     segmentDistance=state.q.stepDistance;
     usesStepDistance=true;
     segmentsUsingStepDistance++;
    }else{
     segmentsUsingChordFallback++;
    }
    routeDistance+=segmentDistance;
   }
   const axleCenters=trailerAxlePoints(state.q,params);
   const currentTracks=axleCenters.map((center,axleIndex)=>({
    center,
    left:localPoint(center,state.q.psiT,0,tracks[axleIndex]/2),
    right:localPoint(center,state.q.psiT,0,-tracks[axleIndex]/2)
   }));
   currentTracks.forEach((sample,axleIndex)=>{
    const axle=axles[axleIndex];
    const append=(trackData,point)=>{
     if(trackData.points.length){
      const previous=trackData.points[trackData.points.length-1];
      trackData.sampledTravel+=Math.hypot(point.x-previous.x,point.y-previous.y);
     }
     trackData.points.push({
      x:point.x,y:point.y,routeDistance,heading:state.q.psiT,gear:state.gear,sampleIndex:index
     });
    };
    append(axle.center,sample.center);
    append(axle.left,sample.left);
    append(axle.right,sample.right);
   });

   if(previousTracks){
    const deltaHeading=norm(state.q.psiT-previousState.q.psiT);
    let integratedByAxle=null;
    let integrationStatus=usesStepDistance?'kinematic-endpoint-mismatch':'step-distance-missing';
    let endpointResidual=null;
    if(usesStepDistance){
     // Parameterize the same RK4 bicycle-model segment by travelled distance.
     // With signed reference speed +/-1 m/s, dt numerically equals metres travelled.
     const reconstructed=sampleFractions.map(fraction=>
      rk4(previousState.q,state.gear,state.steer,segmentDistance*fraction,params)
     );
     const predictedEnd=reconstructed[reconstructed.length-1];
     endpointResidual={
      position:Math.hypot(predictedEnd.x-state.q.x,predictedEnd.y-state.q.y),
      tractorHeading:Math.abs(norm(predictedEnd.psi-state.q.psi)),
      trailerHeading:Math.abs(norm(predictedEnd.psiT-state.q.psiT))
     };
     const matchesPlannerState=
      endpointResidual.position<=endpointTolerance.position&&
      endpointResidual.tractorHeading<=endpointTolerance.heading&&
      endpointResidual.trailerHeading<=endpointTolerance.heading;
     if(matchesPlannerState){
      integratedByAxle=offsets.map((offset,axleIndex)=>{
       const track=tracks[axleIndex];
       const speeds=reconstructed.map(q=>({
        center:wheelSpeedPerReferenceMetre(q,state.gear,state.steer,offset,0),
        left:wheelSpeedPerReferenceMetre(q,state.gear,state.steer,offset,track/2),
        right:wheelSpeedPerReferenceMetre(q,state.gear,state.steer,offset,-track/2)
       }));
       return {
        center:simpsonDistance(segmentDistance,speeds.map(sample=>sample.center)),
        left:simpsonDistance(segmentDistance,speeds.map(sample=>sample.left)),
        right:simpsonDistance(segmentDistance,speeds.map(sample=>sample.right))
       };
      });
      integrationStatus='rk4-simpson-estimate';
      segmentsWithKinematicEstimate++;
      integratedByAxle.forEach((integrated,axleIndex)=>{
       for(const key of ['center','left','right']){
        axles[axleIndex][key].integratedTravelAccumulator+=integrated[key];
        axles[axleIndex][key].integratedTravelCoveredDistance+=segmentDistance;
       }
      });
     }else{
      segmentsWithoutKinematicEstimate++;
     }
    }else{
     segmentsWithoutKinematicEstimate++;
    }

    const axleSegments=currentTracks.map((sample,axleIndex)=>{
     const previous=previousTracks[axleIndex];
     const leftWheelDistance=Math.hypot(sample.left.x-previous.left.x,sample.left.y-previous.left.y);
     const rightWheelDistance=Math.hypot(sample.right.x-previous.right.x,sample.right.y-previous.right.y);
     const centerDistance=Math.hypot(sample.center.x-previous.center.x,sample.center.y-previous.center.y);
     const turnDirection=centerDistance<1e-9||Math.abs(deltaHeading)<1e-8?
      'straight':(deltaHeading>0?'left':'right');
     const innerSide=turnDirection==='straight'?null:
      (turnDirection==='left'?
       (state.gear===1?'left':'right'):
       (state.gear===1?'right':'left'));
     const outerSide=innerSide===null?null:(innerSide==='left'?'right':'left');
     const innerWheelDistance=innerSide==='left'?leftWheelDistance:
      (innerSide==='right'?rightWheelDistance:null);
     const outerWheelDistance=outerSide==='left'?leftWheelDistance:
      (outerSide==='right'?rightWheelDistance:null);
     const integrated=integratedByAxle?.[axleIndex]||null;
     const integratedInner=innerSide?integrated?.[innerSide]:null;
     const integratedOuter=outerSide?integrated?.[outerSide]:null;
     return {
      axleIndex:axleIndex+1,centerDistance,leftWheelDistance,rightWheelDistance,
      turnDirection,innerSide,outerSide,innerWheelDistance,outerWheelDistance,
      outerMinusInner:innerWheelDistance===null?null:outerWheelDistance-innerWheelDistance,
      integratedCenterDistance:integrated?.center??null,
      integratedLeftWheelDistance:integrated?.left??null,
      integratedRightWheelDistance:integrated?.right??null,
      integratedOuterMinusInner:integratedInner===null||integratedOuter===null?
       null:integratedOuter-integratedInner
     };
    });
    segments.push({
     fromSample:index-1,toSample:index,gear:state.gear,gearChanged:state.gear!==previousState.gear,
     tractorReferenceDistance:routeDistance,
     stepDistance:usesStepDistance?segmentDistance:null,
     trailerHeadingChange:deltaHeading,
     integrationStatus,endpointResidual,
     axles:axleSegments
    });
   }
   previousTracks=currentTracks;
   previousState=state;
  }

  const finalizeTrack=trackData=>{
   const complete=segmentsWithoutKinematicEstimate===0;
   const result={
    side:trackData.side,name:trackData.name,points:trackData.points,
    sampledTravel:trackData.sampledTravel,
    integratedTravelEstimate:complete?trackData.integratedTravelAccumulator:null,
    integratedTravelCoveredDistance:trackData.integratedTravelCoveredDistance
   };
   return result;
  };
  const finalizedAxles=axles.map(axle=>({
   axleIndex:axle.axleIndex,offsetFromGroupCenter:axle.offsetFromGroupCenter,
   center:finalizeTrack(axle.center),
   left:finalizeTrack(axle.left),
   right:finalizeTrack(axle.right)
  }));
  return {
   samples:states.length,segments,segmentCount:segments.length,totalRouteDistance:routeDistance,
   axles:finalizedAxles,
   segments,
   distanceMethod:segmentsUsingStepDistance&&segmentsUsingChordFallback?
    'step-distance-when-available-otherwise-chord':
    (segmentsUsingStepDistance?'planner-step-distance':'chord-fallback'),
   segmentsUsingStepDistance,segmentsUsingChordFallback,
   segmentsWithKinematicEstimate,segmentsWithoutKinematicEstimate,
   wheelTravelMethod:segmentsWithKinematicEstimate===0?'sampled-chords-only':
    (segmentsWithoutKinematicEstimate===0?
     'rk4-reconstructed-composite-simpson-4-subintervals':
     'mixed-rk4-estimates-and-sampled-chords'),
   kinematicEndpointTolerance:endpointTolerance,
   model:'single-equivalent-trailer-axle-group',
   limitations:[
    'sampledTravel — сумма хорд между узлами маршрута; это выборочная оценка и она может быть ниже длины кривой.',
    'integratedTravelEstimate доступна только для сегментов с stepDistance, чьи конечные состояния совпадают с реконструкцией той же кинематической RK4-модели в заданном допуске.',
    'Длина пути колёс оценивается производной жёсткого тела и составной формулой Симпсона с четырьмя подынтервалами; это численная оценка, не сертифицированная граница ошибки интегрирования.',
    'Все три оси жёстко привязаны к одному эквивалентному курсу полуприцепа; независимое руление, подвеска и деформация шин не моделируются.',
    'Внутренняя и внешняя стороны классифицируются по знаку изменения курса между соседними узлами и передаче; это геометрический индикатор среза, не измерение бокового скольжения.'
   ]
  };
 }


 // Returns separate sampled center paths and RK4/Simpson path-length estimates for every tire footprint.
 // stepDistance is the tractor rear-axle reference distance; missing stepDistance permits sampled
 // chords only and never produces an integrated estimate.
 function wheelTireTrajectoryDiagnostics(path,params){
  const errors=validateVehicleParams(params);
  if(errors.length)throw new Error('Некорректная геометрия траекторий покрышек: '+errors.join(' '));
  if(!Array.isArray(path))throw new Error('Траектория покрышек должна быть массивом состояний.');
  if(path.length===0)throw new Error('Траектория покрышек должна содержать хотя бы одно состояние.');
  const readState=(state,index)=>{
   if(!state||![state.x,state.y,state.psi,state.psiT].every(Number.isFinite)){
    throw new Error('Состояние траектории покрышек '+(index+1)+' содержит некорректные координаты или углы.');
   }
   const gear=state.gear===undefined?1:state.gear;
   const steer=state.steer===undefined?0:state.steer;
   if(gear!==1&&gear!==-1)throw new Error('Передача в состоянии траектории покрышек '+(index+1)+' должна быть +1 или -1.');
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Угол руля в состоянии траектории покрышек '+(index+1)+' некорректен.');
   }
   if(state.stepDistance!==undefined&&(
    !Number.isFinite(state.stepDistance)||
    (index===0?state.stepDistance!==0:state.stepDistance<=0)
   )){
    throw new Error('stepDistance в состоянии траектории покрышек '+(index+1)+' некорректен.');
   }
   return {q:state,gear,steer};
  };
  const states=Array.from(path,readState);
  const axleOffsets=trailerAxleOffsets(params);
  const axleTracks=trailerAxleTracks(params);
  const sampleFractions=[0,0.25,0.5,0.75,1];
  const endpointTolerance={position:1e-5,heading:1e-6};
  const records=new Map();
  let expectedNames=null,routeDistance=0,previousState=null,previousCenters=null;
  let segmentsUsingStepDistance=0,segmentsUsingChordFallback=0;
  let segmentsWithKinematicEstimate=0,segmentsWithoutKinematicEstimate=0;
  let maxSegmentRefinementDelta=0;

  const centerSpeedPerReferenceMetre=(q,wheel,gear,steer)=>{
   const motion=derivative(q,gear,steer,params);
   const sideSign=wheel.side==='left'?1:-1;
   const pairOffset=Number.isFinite(wheel.pairLateralOffset)?wheel.pairLateralOffset:0;
   let longitudinal,lateral,relativeX,relativeY,vx,vy;
   if(wheel.vehiclePart==='тягач'){
    const front=wheel.axleIndex===1;
    const track=front?(params.tractorFrontTrack??2.05):(params.tractorRearTrack??2.05);
    longitudinal=front?params.wheelbase:0;
    lateral=sideSign*track/2+pairOffset;
    relativeX=longitudinal*Math.cos(q.psi)-lateral*Math.sin(q.psi);
    relativeY=longitudinal*Math.sin(q.psi)+lateral*Math.cos(q.psi);
    vx=motion.x-motion.psi*relativeY;
    vy=motion.y+motion.psi*relativeX;
   }else if(wheel.vehiclePart==='полуприцеп'){
    const axleIndex=wheel.axleIndex-1;
    if(!Number.isInteger(axleIndex)||axleIndex<0||axleIndex>=3){
     throw new Error('Индекс оси покрышки полуприцепа некорректен.');
    }
    longitudinal=-params.kingpinAxles+axleOffsets[axleIndex];
    lateral=sideSign*axleTracks[axleIndex]/2+pairOffset;
    relativeX=longitudinal*Math.cos(q.psiT)-lateral*Math.sin(q.psiT);
    relativeY=longitudinal*Math.sin(q.psiT)+lateral*Math.cos(q.psiT);
    const hitchX=-params.hitchOffset*Math.cos(q.psi);
    const hitchY=-params.hitchOffset*Math.sin(q.psi);
    const hitchVx=motion.x-motion.psi*hitchY;
    const hitchVy=motion.y+motion.psi*hitchX;
    vx=hitchVx-motion.psiT*relativeY;
    vy=hitchVy+motion.psiT*relativeX;
   }else{
    throw new Error('Неизвестная часть автопоезда в траектории покрышки.');
   }
   return Math.hypot(vx,vy);
  };
  const simpson=(distance,values)=>
   distance*(values[0]+4*values[1]+2*values[2]+4*values[3]+values[4])/12;
  const coarseSimpson=(distance,values)=>
   distance*(values[0]+4*values[2]+values[4])/6;

  const segments=[];
  for(let index=0;index<states.length;index++){
   const state=states[index];
   let segmentDistance=0,usesStepDistance=false;
   if(index===0&&state.q.stepDistance!==undefined&&state.q.stepDistance!==0){
    throw new Error('Начальное состояние траектории покрышек должно иметь stepDistance равный нулю.');
   }
   if(index>0){
    const chord=Math.hypot(state.q.x-previousState.q.x,state.q.y-previousState.q.y);
    if(state.q.stepDistance!==undefined){
     if(!Number.isFinite(state.q.stepDistance)||state.q.stepDistance<=0){
      throw new Error('Состояние траектории покрышек '+(index+1)+' содержит некорректный stepDistance.');
     }
     if(chord>state.q.stepDistance+1e-6){
      throw new Error('Хорда траектории покрышек превышает stepDistance на сегменте '+index+'.');
     }
     segmentDistance=state.q.stepDistance;
     usesStepDistance=true;
     segmentsUsingStepDistance++;
    }else{
     segmentDistance=chord;
     segmentsUsingChordFallback++;
    }
    routeDistance+=segmentDistance;
   }

   const footprints=wheelFootprints(state.q,params,state.steer);
   const names=footprints.map(wheel=>wheel.name);
   if(new Set(names).size!==names.length)throw new Error('Имена расчётных покрышек должны быть уникальными.');
   if(expectedNames===null)expectedNames=names.slice().sort();
   else if(names.length!==expectedNames.length||names.slice().sort().some((name,i)=>name!==expectedNames[i])){
    throw new Error('Набор покрышек изменился между состояниями маршрута.');
   }

   const currentCenters=new Map();
   for(const wheel of footprints){
    if(!wheel.center||![wheel.center.x,wheel.center.y].every(Number.isFinite)){
     throw new Error('Центр покрышки содержит некорректные координаты.');
    }
    let record=records.get(wheel.name);
    if(!record){
     record={
      name:wheel.name,vehiclePart:wheel.vehiclePart,axleIndex:wheel.axleIndex,side:wheel.side,
      pairPosition:wheel.pairPosition??null,pairCenterSpacing:wheel.pairCenterSpacing??null,
      pairLateralOffset:wheel.pairLateralOffset??0,
      footprintModel:wheel.footprintModel||'legacy-single-tire-fallback',
      points:[],sampledTravel:0,integratedTravelAccumulator:0,
      integratedTravelCoveredDistance:0,refinementIndicatorTotal:0,maxSegmentRefinementDelta:0
     };
     records.set(wheel.name,record);
    }
    const previous=previousCenters?.get(wheel.name);
    const sampledSegmentDistance=previous?
     Math.hypot(wheel.center.x-previous.center.x,wheel.center.y-previous.center.y):0;
    record.sampledTravel+=sampledSegmentDistance;
    record.points.push({
     x:wheel.center.x,y:wheel.center.y,routeDistance,heading:wheel.heading,
     gear:state.gear,steer:state.steer,sampleIndex:index
    });
    currentCenters.set(wheel.name,{wheel,center:wheel.center,sampledSegmentDistance});
   }

   if(index>0){
    let integrationStatus=usesStepDistance?'kinematic-endpoint-mismatch':'step-distance-missing';
    let endpointResidual=null,segmentRefinementDelta=0;
    if(usesStepDistance){
     // Unit signed reference speed makes elapsed time numerically equal to metres travelled.
     // The kinematic path is invariant to the planner's constant 0.7 m/s scaling.
     const reconstructed=sampleFractions.map(fraction=>
      rk4(previousState.q,state.gear,state.steer,segmentDistance*fraction,params)
     );
     const predictedEnd=reconstructed[reconstructed.length-1];
     endpointResidual={
      position:Math.hypot(predictedEnd.x-state.q.x,predictedEnd.y-state.q.y),
      tractorHeading:Math.abs(norm(predictedEnd.psi-state.q.psi)),
      trailerHeading:Math.abs(norm(predictedEnd.psiT-state.q.psiT))
     };
     const endpointMatches=
      endpointResidual.position<=endpointTolerance.position&&
      endpointResidual.tractorHeading<=endpointTolerance.heading&&
      endpointResidual.trailerHeading<=endpointTolerance.heading;
     if(endpointMatches){
      for(const [name,current] of currentCenters){
       const record=records.get(name);
       const values=reconstructed.map(q=>
        centerSpeedPerReferenceMetre(q,current.wheel,state.gear,state.steer)
       );
       const integratedDistance=simpson(segmentDistance,values);
       const coarseDistance=coarseSimpson(segmentDistance,values);
       const refinementDelta=Math.abs(integratedDistance-coarseDistance);
       if(!Number.isFinite(integratedDistance)||integratedDistance<0||!Number.isFinite(refinementDelta)){
        throw new Error('Интегральная оценка длины пути покрышки некорректна.');
       }
       record.integratedTravelAccumulator+=integratedDistance;
       record.integratedTravelCoveredDistance+=segmentDistance;
       record.refinementIndicatorTotal+=refinementDelta;
       record.maxSegmentRefinementDelta=Math.max(record.maxSegmentRefinementDelta,refinementDelta);
       segmentRefinementDelta=Math.max(segmentRefinementDelta,refinementDelta);
      }
      integrationStatus='rk4-reconstructed-composite-simpson-4-subintervals';
      segmentsWithKinematicEstimate++;
      maxSegmentRefinementDelta=Math.max(maxSegmentRefinementDelta,segmentRefinementDelta);
     }else{
      segmentsWithoutKinematicEstimate++;
     }
    }else{
     segmentsWithoutKinematicEstimate++;
    }
    segments.push({
     fromSample:index-1,toSample:index,gear:state.gear,steer:state.steer,
     routeDistance,stepDistance:usesStepDistance?segmentDistance:null,
     integrationStatus,endpointResidual,refinementIndicator:segmentRefinementDelta
    });
   }
   previousState=state;
   previousCenters=currentCenters;
  }

  const complete=segmentsWithoutKinematicEstimate===0;
  const tires=[...records.values()].map(record=>({
   ...record,
   integratedTravelEstimate:complete?record.integratedTravelAccumulator:null
  }));
  const totalSampledTireTravel=tires.reduce((sum,tire)=>sum+tire.sampledTravel,0);
  const totalIntegratedTireTravel=complete?
   tires.reduce((sum,tire)=>sum+tire.integratedTravelEstimate,0):null;
  const longestSampledTire=tires.reduce((best,tire)=>!best||tire.sampledTravel>best.sampledTravel?tire:best,null);
  const longestIntegratedTire=complete?
   tires.reduce((best,tire)=>!best||tire.integratedTravelEstimate>best.integratedTravelEstimate?tire:best,null):null;
  return {
   samples:states.length,segments:segments.length,segmentDiagnostics:segments,totalRouteDistance:routeDistance,
   tires,totalSampledTireTravel,totalIntegratedTireTravel,longestSampledTire,longestIntegratedTire,
   segmentsUsingStepDistance,segmentsUsingChordFallback,
   segmentsWithKinematicEstimate,segmentsWithoutKinematicEstimate,
   integratedTravelComplete:complete,
   maxSegmentRefinementDelta,
   integration:complete?'rk4-reconstructed-composite-simpson-4-subintervals':'incomplete',
   distanceMethod:segmentsUsingStepDistance&&segmentsUsingChordFallback?
    'step-distance-when-available-otherwise-chord':
    (segmentsUsingStepDistance?'planner-step-distance':'chord-fallback'),
   endpointTolerance,
   refinementIndicator:'absolute difference between composite 4-subinterval and single-panel Simpson estimates; not an error bound',
   model:'rigid-body tire-center kinematic proxy',
   limitations:[
    'sampledTravel — сумма хорд между узлами маршрута и может быть ниже фактической длины кривой.',
    'integratedTravelEstimate доступна только если каждый сегмент имеет stepDistance и проходит проверку конечной позы RK4.',
    'Интегрирование скорости центра покрышки использует жёсткую кинематику и составную формулу Симпсона; refinementIndicator — индикатор уточнения, не сертифицированная граница ошибки.',
    'Смена передачи и рулевого воздействия трактуется как пространственная граница сегмента; остановка, время КПП и переходные процессы не моделируются.',
    'Независимое руление осей, подвеска, деформация шин, скольжение, нагрузка и контактные силы не моделируются.'
   ]
  };
 }

 return Object.freeze({norm,VEHICLE_PARAMETER_LIMITS,VEHICLE_VERTICAL_PROFILE_KEYS,vehicleGeometryCalibrationStatus,validateRoadSurface,roadSurfaceElevationRange,verticalRangeOnRoadSurface,validateSuspensionTravel,suspensionTravelStatus,suspensionTravelDisplacementRange,validateVehicleParams,validateTireGroupWidthUncertainty,validateAuxiliaryEnvelopes,validateVehicleVerticalProfiles,verticalRangeFromProfile,vehicleVerticalRange,auxiliaryEnvelopePolygons,trailerAxleTireGroupWidths,trailerAxleTireCenterSpacings,trailerAxleSpacings,trailerAxleTracks,trailerAxleOffsets,derivative,rk4,hitchPoint,axlePoint,tractorAxlePoints,trailerAxlePoints,frontWheelSteerAngles,wheelFootprints,trailerAxleDiagnostics,trailerAxleScrubExposure,trailerWheelTrajectoryDiagnostics,wheelTireTrajectoryDiagnostics,vehiclePolygons,mirrorPolygons,vehicleCollisionParts});
});
