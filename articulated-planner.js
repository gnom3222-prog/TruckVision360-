(function(root,factory){
 'use strict';
 const physics=root&&root.TruckVisionPhysics?root.TruckVisionPhysics:
  (typeof require==='function'?require('./vehicle-physics.js'):null);
 const api=factory(physics);
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionPlanner=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(rootPhysics){
 'use strict';
 const norm=a=>Math.atan2(Math.sin(a),Math.cos(a));
 const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
 function goalErrors(q,goal){
  return {
   position:Math.hypot(goal.x-q.x,goal.y-q.y),
   headingError:Number.isFinite(goal.heading)?Math.abs(norm(q.psi-goal.heading)):null,
   articulation:Math.abs(norm(q.psi-q.psiT))
  };
 }
 function goalReached(q,goal,options={}){
  const errors=goalErrors(q,goal);
  const positionTolerance=options.positionTolerance??0.9;
  const headingTolerance=options.headingTolerance??12*Math.PI/180;
  const maxTerminalArticulation=options.maxTerminalArticulation??12*Math.PI/180;
  return errors.position<=positionTolerance&&
   (errors.headingError===null||errors.headingError<=headingTolerance)&&
   errors.articulation<=maxTerminalArticulation;
 }
 function polygons(q,p){
  if(!rootPhysics||typeof rootPhysics.vehiclePolygons!=='function')throw new Error('Shared vehicle geometry is unavailable.');
  return rootPhysics.vehiclePolygons(q,p);
 }
 function polygonSignedArea(poly){
  let area=0;
  for(let i=0;i<poly.length;i++){
   const a=poly[i],b=poly[(i+1)%poly.length];
   area+=a.x*b.y-b.x*a.y;
  }
  return area/2;
 }
 function isConvexPolygon(poly){
  if(!Array.isArray(poly)||poly.length<3||poly.length>32)return false;
  let sign=0;
  for(let i=0;i<poly.length;i++){
   const a=poly[i],b=poly[(i+1)%poly.length],c=poly[(i+2)%poly.length];
   if(!a||!b||!c||![a.x,a.y,b.x,b.y,c.x,c.y].every(Number.isFinite))return false;
   const abx=b.x-a.x,aby=b.y-a.y,bcx=c.x-b.x,bcy=c.y-b.y;
   if(Math.hypot(abx,aby)<1e-9)return false;
   const cross=abx*bcy-aby*bcx;
   if(Math.abs(cross)<1e-10)return false;
   const current=Math.sign(cross);
   if(sign&&current!==sign)return false;
   sign=current;
  }
  const area=polygonSignedArea(poly);
  if(Math.abs(area)<=1e-8)return false;
  const orientation=Math.sign(area);
  // Every non-adjacent vertex must lie strictly on the same side of each
  // edge. This rejects self-intersecting star polygons as well as concave ones.
  for(let i=0;i<poly.length;i++){
   const a=poly[i],b=poly[(i+1)%poly.length];
   const dx=b.x-a.x,dy=b.y-a.y;
   for(let j=0;j<poly.length;j++){
    if(j===i||j===(i+1)%poly.length)continue;
    const point=poly[j];
    const side=(dx*(point.y-a.y)-dy*(point.x-a.x))*orientation;
    if(side<=1e-10)return false;
   }
  }
  return true;
 }
 function obstaclePoly(o){
  if(Array.isArray(o.polygon))return o.polygon.map(point=>({x:point.x,y:point.y}));
  return[
   {x:o.x-o.w/2,y:o.y-o.h/2},{x:o.x+o.w/2,y:o.y-o.h/2},
   {x:o.x+o.w/2,y:o.y+o.h/2},{x:o.x-o.w/2,y:o.y+o.h/2}
  ];
 }
 function inflateConvexPolygon(poly,distance){
  if(!isConvexPolygon(poly))throw new Error('Для расширения требуется конечный выпуклый многоугольник без вырожденных рёбер.');
  if(!Number.isFinite(distance)||distance<0)throw new Error('Расстояние расширения должно быть конечным неотрицательным числом.');
  let points=poly.map(point=>({x:point.x,y:point.y}));
  if(polygonSignedArea(points)<0)points.reverse();
  if(distance===0)return points;
  const edges=points.map((a,index)=>{
   const b=points[(index+1)%points.length];
   const dx=b.x-a.x,dy=b.y-a.y,length=Math.hypot(dx,dy);
   return {
    a:{x:a.x+dy/length*distance,y:a.y-dx/length*distance},
    b:{x:b.x+dy/length*distance,y:b.y-dx/length*distance},
    direction:{x:dx,y:dy}
   };
  });
  return points.map((_,index)=>{
   const previous=edges[(index+edges.length-1)%edges.length];
   const current=edges[index];
   const rx=previous.direction.x,ry=previous.direction.y;
   const sx=current.direction.x,sy=current.direction.y;
   const denominator=rx*sy-ry*sx;
   if(Math.abs(denominator)<1e-10)throw new Error('Невозможно устойчиво расширить почти вырожденный многоугольник.');
   const qx=current.a.x-previous.a.x,qy=current.a.y-previous.a.y;
   const t=(qx*sy-qy*sx)/denominator;
   return {x:previous.a.x+t*rx,y:previous.a.y+t*ry};
  });
 }
 function inflateObstaclePoseUncertainty(poly,anchor,uncertainty={}) {
  if(!isConvexPolygon(poly))throw new Error('Для оценки неопределённости положения требуется конечный выпуклый многоугольник.');
  if(!anchor||![anchor.x,anchor.y].every(Number.isFinite)||Math.abs(anchor.x)>10000||Math.abs(anchor.y)>10000){
   throw new Error('Опорная точка препятствия должна иметь конечные координаты.');
  }
  const positionX=uncertainty.positionX??0;
  const positionY=uncertainty.positionY??0;
  const headingDeg=uncertainty.headingDeg??0;
  if(![positionX,positionY,headingDeg].every(Number.isFinite)||
     positionX<0||positionX>2||positionY<0||positionY>2||
     headingDeg<0||headingDeg>45){
   throw new Error('Допуски положения препятствия должны быть в диапазоне 0–2 м по X/Y и 0–45° по курсу.');
  }
  const maxRadius=Math.max(...poly.map(point=>Math.hypot(point.x-anchor.x,point.y-anchor.y)));
  const positionInflation=Math.hypot(positionX,positionY);
  const headingInflation=2*maxRadius*Math.sin(headingDeg*Math.PI/360);
  const totalInflation=positionInflation+headingInflation;
  return {
   polygon:inflateConvexPolygon(poly,totalInflation),
   positionInflation,headingInflation,totalInflation,maxRadius,
   positionUncertaintyX:positionX,positionUncertaintyY:positionY,
   headingUncertaintyDeg:headingDeg
  };
 }

 function createGateObstacles(gate){
  if(!gate||typeof gate!=='object')throw new Error('Параметры ворот отсутствуют.');
  const {centerX,centerY,clearWidth,postWidth,depth,heading}=gate;
  const widthUncertainty=gate.widthUncertainty??0;
  const positionUncertainty=gate.positionUncertainty??0;
  const headingUncertaintyDeg=gate.headingUncertaintyDeg??0;
  if(![centerX,centerY,clearWidth,postWidth,depth,heading,
       widthUncertainty,positionUncertainty,headingUncertaintyDeg].every(Number.isFinite)||
     Math.abs(centerX)>10000||Math.abs(centerY)>10000||
     clearWidth<=0||clearWidth>50||postWidth<=0||postWidth>5||
     depth<=0||depth>10||
     widthUncertainty<0||widthUncertainty>2||widthUncertainty>=clearWidth||
     positionUncertainty<0||positionUncertainty>2||
     headingUncertaintyDeg<0||headingUncertaintyDeg>45){
   throw new Error('Ворота и их границы неопределённости должны быть конечными и находиться в допустимых диапазонах.');
  }
  // widthUncertainty is an absolute bound on the full clear width. Moving
  // both jambs inward by half this amount models the narrowest opening.
  const conservativeClearWidth=clearWidth-widthUncertainty;
  const ux=Math.cos(heading),uy=Math.sin(heading);
  const nx=-uy,ny=ux;
  const halfOffset=conservativeClearWidth/2+postWidth/2;
  const headingDelta=headingUncertaintyDeg*Math.PI/180;
  const positionInflation=Math.SQRT2*positionUncertainty;
  const makePost=(side,name)=>{
   const cx=centerX+side*halfOffset*ux;
   const cy=centerY+side*halfOffset*uy;
   const point=(along,across)=>({
    x:cx+along*ux+across*nx,
    y:cy+along*uy+across*ny
   });
   const nominalPolygon=[
    point(-postWidth/2,-depth/2),
    point(postWidth/2,-depth/2),
    point(postWidth/2,depth/2),
    point(-postWidth/2,depth/2)
   ];
   // A heading error rotates every jamb vertex around the manually supplied
   // gate center. 2*R*sin(delta/2) bounds that displacement for |delta|.
   const radius=Math.max(...nominalPolygon.map(p=>Math.hypot(p.x-centerX,p.y-centerY)));
   const headingInflation=2*radius*Math.sin(headingDelta/2);
   const uncertaintyInflation=positionInflation+headingInflation;
   const polygon=inflateConvexPolygon(nominalPolygon,uncertaintyInflation);
   return {
    name,kind:'gate-jamb',polygon,
    gateUncertainty:{
     nominalClearWidth:clearWidth,
     conservativeClearWidth,
     widthUncertainty,
     positionUncertaintyPerAxis:positionUncertainty,
     headingUncertaintyDeg,
     positionInflation,
     headingInflation,
     totalInflation:uncertaintyInflation
    }
   };
  };
  return [
   makePost(-1,'Левый косяк ворот'),
   makePost(1,'Правый косяк ворот')
  ];
 }
 function intersects(a,b){
  for(const poly of [a,b])for(let i=0;i<poly.length;i++){
   const n=poly[(i+1)%poly.length],c=poly[i],dx=n.x-c.x,dy=n.y-c.y,l=Math.hypot(dx,dy)||1;
   const ax=-dy/l,ay=dx/l,pa=a.map(v=>v.x*ax+v.y*ay),pb=b.map(v=>v.x*ax+v.y*ay);
   if(Math.max(...pa)<Math.min(...pb)||Math.max(...pb)<Math.min(...pa))return false;
  }
  return true;
 }
 function segDist(p,a,b){
  const dx=b.x-a.x,dy=b.y-a.y,l=dx*dx+dy*dy,t=l?clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l,0,1):0;
  return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
 }
 function polyDist(a,b){
  if(intersects(a,b))return 0;
  let d=Infinity;
  for(const p of a)for(let i=0;i<b.length;i++)d=Math.min(d,segDist(p,b[i],b[(i+1)%b.length]));
  for(const p of b)for(let i=0;i<a.length;i++)d=Math.min(d,segDist(p,a[i],a[(i+1)%a.length]));
  return d;
 }
 function polygonBounds(poly){
  let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
  for(const p of poly){minX=Math.min(minX,p.x);minY=Math.min(minY,p.y);maxX=Math.max(maxX,p.x);maxY=Math.max(maxY,p.y);}
  return {minX,minY,maxX,maxY};
 }
 function aabbDistance(a,b){
  const dx=Math.max(0,a.minX-b.maxX,b.minX-a.maxX);
  const dy=Math.max(0,a.minY-b.maxY,b.minY-a.maxY);
  return Math.hypot(dx,dy);
 }
 function validateObstacles(obstacles){
  const errors=[];
  if(!Array.isArray(obstacles))return ['Список препятствий должен быть массивом.'];
  obstacles.forEach((o,index)=>{
   if(!o||typeof o!=='object'){
    errors.push('Препятствие '+(index+1)+' имеет неверный формат.');
    return;
   }
   if(Array.isArray(o.polygon)){
    if(!isConvexPolygon(o.polygon)){
     errors.push('Препятствие '+(index+1)+' должно быть конечным невырожденным выпуклым многоугольником.');
    }
   }else if(!Number.isFinite(o.x)||!Number.isFinite(o.y)||
      !Number.isFinite(o.w)||!Number.isFinite(o.h)||o.w<=0||o.h<=0){
    errors.push('Препятствие '+(index+1)+' должно иметь конечные координаты и положительные ширину/высоту либо выпуклый polygon.');
   }
   if(o.nominalPolygon!==undefined&&!isConvexPolygon(o.nominalPolygon)){
    errors.push('Номинальный контур препятствия '+(index+1)+' должен быть конечным невырожденным выпуклым многоугольником.');
   }
  });
  return errors;
 }
 function validatePlanOptions(options){
  const errors=[];
  if(!options||typeof options!=='object')return ['Параметры планировщика отсутствуют.'];
  const {start,goal,params,obstacles,integrate}=options;
  if(!start||![start.x,start.y,start.psi,start.psiT].every(Number.isFinite)){
   errors.push('Начальное состояние должно содержать конечные x, y, psi и psiT.');
  }
  if(!goal||!Number.isFinite(goal.x)||!Number.isFinite(goal.y)||
     (goal.heading!==undefined&&goal.heading!==null&&!Number.isFinite(goal.heading))){
   errors.push('Цель должна содержать конечные координаты и корректный курс, если он задан.');
  }
  if(typeof integrate!=='function')errors.push('Функция интегрирования движения отсутствует.');
  const geometryErrors=rootPhysics&&typeof rootPhysics.validateVehicleParams==='function'?
   rootPhysics.validateVehicleParams(params):['Модуль проверки геометрии недоступен.'];
  errors.push(...geometryErrors);
  errors.push(...validateObstacles(obstacles));
  if(options.maxExpansions!==undefined&&(!Number.isInteger(options.maxExpansions)||options.maxExpansions<1||options.maxExpansions>500000)){
   errors.push('Лимит расширений должен быть целым числом от 1 до 500000.');
  }
  if(options.firstActionConstraint!==undefined){
   const constraint=options.firstActionConstraint;
   if(!constraint||![-1,1].includes(constraint.initialGear)||!['left','right'].includes(constraint.firstTurn)){
    errors.push('Ограничение первого действия должно задавать initialGear -1/1 и firstTurn left/right.');
   }
  }
  if(options.maxSteer!==undefined&&(!Number.isFinite(options.maxSteer)||options.maxSteer<=0||options.maxSteer>=Math.PI/2)){
   errors.push('Максимальный угол поворота должен быть больше нуля и меньше 90°.');
  }
  if(options.minClearance!==undefined&&(!Number.isFinite(options.minClearance)||options.minClearance<0||options.minClearance>10)){
   errors.push('Требуемый зазор должен быть числом от 0 до 10 м.');
  }
  const positionTolerance=options.positionTolerance??0.9;
  const headingTolerance=options.headingTolerance??12*Math.PI/180;
  const maxTerminalArticulation=options.maxTerminalArticulation??12*Math.PI/180;
  if(!Number.isFinite(positionTolerance)||positionTolerance<=0||positionTolerance>10){
   errors.push('Допуск положения должен быть больше нуля и не превышать 10 м.');
  }
  if(!Number.isFinite(headingTolerance)||headingTolerance<0||headingTolerance>Math.PI){
   errors.push('Допуск курса должен находиться в диапазоне 0–180°.');
  }
  if(!Number.isFinite(maxTerminalArticulation)||maxTerminalArticulation<0||maxTerminalArticulation>Math.PI){
   errors.push('Допуск угла сочленения должен находиться в диапазоне 0–180°.');
  }
  if(options.bounds!==undefined){
   const b=options.bounds;
   if(!b||![b.minX,b.maxX,b.minY,b.maxY].every(Number.isFinite)||
      b.minX>=b.maxX||b.minY>=b.maxY){
    errors.push('Границы поиска должны иметь конечные минимумы меньше максимумов.');
   }
  }
  return errors;
 }
 function createClearanceChecker(obstacles,p){
  const geometryErrors=rootPhysics&&typeof rootPhysics.validateVehicleParams==='function'?
   rootPhysics.validateVehicleParams(p):['Модуль проверки геометрии недоступен.'];
  if(geometryErrors.length)throw new Error('Некорректная геометрия автопоезда: '+geometryErrors.join(' '));
  if(!rootPhysics||typeof rootPhysics.vehicleCollisionParts!=='function'){
   throw new Error('Модуль отдельных collision-компонентов автопоезда недоступен.');
  }
  const obstacleErrors=validateObstacles(obstacles);
  if(obstacleErrors.length)throw new Error('Некорректные препятствия: '+obstacleErrors.join(' '));
  const prepared=obstacles.map(o=>{
   const poly=obstaclePoly(o);
   return {poly,bounds:polygonBounds(poly),name:o.name||'препятствие'};
  });
  return function(q,steerOverride){
   const steer=steerOverride!==undefined?steerOverride:(q&&q.steer!==undefined?q.steer:0);
   const groups=rootPhysics.vehicleCollisionParts(q,p,steer);
   const perPart=groups.map(group=>({
    distance:Infinity,obstacle:null,vehiclePart:group.vehiclePart,bodyIndex:group.bodyIndex,component:null
   }));
   for(const obstacle of prepared){
    for(let i=0;i<groups.length;i++){
     for(const component of groups[i].components){
      const bounds=polygonBounds(component.polygon);
      if(aabbDistance(bounds,obstacle.bounds)>=perPart[i].distance)continue;
      const d=polyDist(component.polygon,obstacle.poly);
      if(d<perPart[i].distance){
       perPart[i]={
        distance:d,obstacle:obstacle.name,vehiclePart:groups[i].vehiclePart,
        bodyIndex:groups[i].bodyIndex,component:component.name
       };
      }
     }
    }
   }
   let nearest=perPart[0];
   for(let i=1;i<perPart.length;i++)if(perPart[i].distance<nearest.distance)nearest=perPart[i];
   return {distance:nearest.distance,hit:nearest.obstacle,obstacle:nearest.obstacle,
    vehiclePart:nearest.vehiclePart,bodyIndex:nearest.bodyIndex,component:nearest.component,perPart};
  };
 }
 function sweptDisplacementRatesPerMeter(steer,p,q=null,maxDistance=0.05){
  if(!p||typeof p!=='object')throw new Error('Vehicle geometry is missing for swept-clearance checking.');
  if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2)throw new Error('Steering angle is outside the valid range.');
  const geometryErrors=rootPhysics&&typeof rootPhysics.validateVehicleParams==='function'?
   rootPhysics.validateVehicleParams(p):['Vehicle geometry validation is unavailable.'];
  if(geometryErrors.length)throw new Error('Vehicle geometry is incomplete or invalid for swept-clearance checking: '+geometryErrors.join(' '));
  const wheelbase=Number(p.wheelbase),hitchOffset=Number(p.hitchOffset);
  const kingpinAxles=Number(p.kingpinAxles),trailerLength=Number(p.trailerLength),trailerWidth=Number(p.trailerWidth);
  if(!Number.isFinite(maxDistance)||maxDistance<=0)throw new Error('Swept-check distance must be positive.');
  const tractorYawPerMeter=Math.abs(Math.tan(steer)/wheelbase);
  const footprintUncertainty=Number.isFinite(p.footprintUncertainty)?p.footprintUncertainty:0;
  const tractorFrontTrack=p.tractorFrontTrack??2.05;
  const tractorRearTrack=p.tractorRearTrack??2.05;
  const tireLength=p.tireLength??0.72;
  const tireWidth=p.tireWidth??0.44;
  const tractorWheelCornerRadius=Math.hypot(
   tireLength/2+footprintUncertainty,tireWidth/2+footprintUncertainty
  );
  // Bound each tire corner by distance to its axle center plus the tire's
  // circumscribed radius; this remains conservative for any steer orientation.
  const tractorRadius=Math.max(
   Math.hypot(5+footprintUncertainty,(2.55+2*footprintUncertainty)/2),
   Math.hypot(wheelbase,tractorFrontTrack/2)+tractorWheelCornerRadius,
   Math.hypot(0,tractorRearTrack/2)+tractorWheelCornerRadius
  );
  const cabOffset=p.cabOffset??2.1;
  const cabFront=p.cabFront??1.35;
  const cabWidth=p.cabWidth??2.25;
  let cabRadiusFromRearAxle=cabOffset+Math.hypot(
   cabFront+footprintUncertainty,(cabWidth+2*footprintUncertainty)/2
  );
  const mirrorOutboard=p.mirrorOutboard??0;
  if(mirrorOutboard>0){
   const mirrorLongitudinal=p.mirrorLongitudinal??3.0;
   const mirrorHalfLength=(p.mirrorLength??0.8)/2+footprintUncertainty;
   const mirrorOuterLateral=cabWidth/2+mirrorOutboard+footprintUncertainty;
   const mirrorRadius=Math.max(
    Math.hypot(mirrorLongitudinal-mirrorHalfLength,mirrorOuterLateral),
    Math.hypot(mirrorLongitudinal+mirrorHalfLength,mirrorOuterLateral)
   );
   cabRadiusFromRearAxle=Math.max(cabRadiusFromRearAxle,mirrorRadius);
  }
  const trailerFrontFromAxle=kingpinAxles+1.5+footprintUncertainty;
  const trailerRearFromAxle=kingpinAxles-(trailerLength-1.5+footprintUncertainty);
  const trailerAxleSpacing=p.trailerAxleSpacing??1.3;
  const trailerWheelHalfLength=tireLength/2+footprintUncertainty;
  const trailerWheelLongitudinals=[trailerAxleSpacing,0,-trailerAxleSpacing]
   .flatMap(offset=>[offset-trailerWheelHalfLength,offset+trailerWheelHalfLength]);
  if(q&&(!Number.isFinite(q.psi)||!Number.isFinite(q.psiT))){
   throw new Error('Articulation state is invalid for swept-clearance checking.');
  }
  const hitchYawCoupling=hitchOffset*tractorYawPerMeter;
  const trailerTrack=p.trailerTrack??(trailerWidth+0.24);
  // The tire outer track can extend beyond the cargo-body width and therefore
  // must be included in the trailer point-motion bound.
  const trailerWidthWithUncertainty=Math.max(
   trailerWidth+2*footprintUncertainty,
   trailerTrack+tireWidth+2*footprintUncertainty
  );
  const trailerLongitudinalOffsets=[trailerFrontFromAxle,trailerRearFromAxle,...trailerWheelLongitudinals];
  let trailerPointRate=0;
  for(const coupling of [-hitchYawCoupling,hitchYawCoupling]){
   for(const longitudinal of trailerLongitudinalOffsets){
    for(const lateral of [-trailerWidthWithUncertainty/2,trailerWidthWithUncertainty/2]){
     const m11=1+coupling*lateral/kingpinAxles;
     const m12=coupling-lateral/kingpinAxles;
     const m21=-coupling*longitudinal/kingpinAxles;
     const m22=longitudinal/kingpinAxles;
     const column1=m11*m11+m21*m21;
     const column2=m12*m12+m22*m22;
     const cross=m11*m12+m21*m22;
     const largestEigenvalue=0.5*(column1+column2+Math.hypot(column1-column2,2*cross));
     trailerPointRate=Math.max(trailerPointRate,Math.sqrt(Math.max(0,largestEigenvalue)));
    }
   }
  }
  return {
   tractor:1+tractorRadius*tractorYawPerMeter,
   cab:1+cabRadiusFromRearAxle*tractorYawPerMeter,
   trailer:trailerPointRate
  };
 }
 function sweptDisplacementPerMeter(steer,p,q=null,maxDistance=0.05){
  const rates=sweptDisplacementRatesPerMeter(steer,p,q,maxDistance);
  return Math.max(rates.tractor,rates.cab,rates.trailer);
 }
 function clearance(q,obstacles,p,steerOverride){return createClearanceChecker(obstacles,p)(q,steerOverride);}
 function analyzeTrajectoryClearance(states,obstacles,p){
  if(!Array.isArray(states)||states.length===0)throw new Error('Route states are required for per-part clearance analysis.');
  const check=createClearanceChecker(obstacles,p);
  const partNames=['тягач','кабина','полуприцеп'];
  const parts=partNames.map((vehiclePart,bodyIndex)=>({
   vehiclePart,bodyIndex,minimumSampledClearance:Infinity,minimumSampledDetail:null,
   minimumClearanceLowerBound:Infinity,limitingDetail:null
  }));
  let previousState=null,previousCheck=null,distanceAlongRoute=0;
  const snapshot=q=>({x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear:q.gear===-1?-1:1,
   steer:Number.isFinite(q.steer)?q.steer:0});
  for(let index=0;index<states.length;index++){
   const state=states[index];
   if(!state||![state.x,state.y,state.psi,state.psiT].every(Number.isFinite)){
    throw new Error('Route state '+index+' is invalid for per-part clearance analysis.');
   }
   const gear=state.gear===undefined?1:state.gear;
   const steer=state.steer===undefined?0:state.steer;
   if(gear!==1&&gear!==-1){
    throw new Error('Route state '+index+' has an invalid gear for per-part clearance analysis.');
   }
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Route state '+index+' has an invalid steering angle for per-part clearance analysis.');
   }
   const currentCheck=check(state);
   for(let partIndex=0;partIndex<parts.length;partIndex++){
    const sample=currentCheck.perPart[partIndex];
    if(sample.distance<parts[partIndex].minimumSampledClearance){
     parts[partIndex].minimumSampledClearance=sample.distance;
     parts[partIndex].minimumSampledDetail=Number.isFinite(sample.distance)?
      {...sample,state:snapshot(state),distanceAlongRoute}:null;
    }
    if(sample.distance<parts[partIndex].minimumClearanceLowerBound){
     parts[partIndex].minimumClearanceLowerBound=sample.distance;
     parts[partIndex].limitingDetail=Number.isFinite(sample.distance)?
      {kind:'sampled-pose',lowerBound:sample.distance,state:snapshot(state),distanceAlongRoute,
       obstacle:sample.obstacle,vehiclePart:sample.vehiclePart}:null;
    }
   }
   if(index>0){
    const previousGear=previousState.gear===-1?-1:1;
    const previousSteer=Number.isFinite(previousState.steer)?previousState.steer:0;
    const previousGearKnown=previousState.gearKnown!==false;
    const previousSteerKnown=previousState.steerKnown!==false;
    const initialGearSelectionAtSegmentStart=!previousGearKnown;
    const gearChangedAtSegmentStart=previousGearKnown&&previousGear!==gear;
    const initialSteeringSelectionAtSegmentStart=!previousSteerKnown;
    const steeringChangedAtSegmentStart=previousSteerKnown&&
     Math.abs(norm(steer-previousSteer))>1e-9;
    if(initialGearSelectionAtSegmentStart)segmentsWithInitialGearSelection++;
    if(gearChangedAtSegmentStart)segmentsWithGearChangeAtStart++;
    if(initialSteeringSelectionAtSegmentStart)segmentsWithInitialSteeringSelection++;
    if(steeringChangedAtSegmentStart)segmentsWithSteeringChangeAtStart++;
    const stepDistance=state.stepDistance;
    if(!Number.isFinite(stepDistance)||stepDistance<=0){
     throw new Error('Route state '+index+' is missing its positive integration stepDistance; continuous per-part bound cannot be established.');
    }
    const chordDistance=Math.hypot(state.x-previousState.x,state.y-previousState.y);
    if(chordDistance>stepDistance+1e-6){
     throw new Error('Route state '+index+' moves farther than its integration stepDistance; per-part swept bound is invalid.');
    }
    const rates=sweptDisplacementRatesPerMeter(
     Number.isFinite(state.steer)?state.steer:0,p,previousState,stepDistance
    );
    const rateByPart=[rates.tractor,rates.cab,rates.trailer];
    for(let partIndex=0;partIndex<parts.length;partIndex++){
     const reference=previousCheck.perPart[partIndex];
     if(!Number.isFinite(reference.distance))continue;
     const lowerBound=reference.distance-rateByPart[partIndex]*stepDistance;
     if(lowerBound<parts[partIndex].minimumClearanceLowerBound){
      parts[partIndex].minimumClearanceLowerBound=lowerBound;
      parts[partIndex].limitingDetail={
       kind:'segment-lower-bound',lowerBound,vehiclePart:partNames[partIndex],bodyIndex:partIndex,
       obstacleAtReference:reference.obstacle,componentAtReference:reference.component,
       referenceDistance:reference.distance,
       displacementRate:rateByPart[partIndex],stepDistance,distanceAlongRoute,
       from:snapshot(previousState),to:snapshot(state),
       gear:state.gear===-1?-1:1,steer:Number.isFinite(state.steer)?state.steer:0
      };
     }
    }
    distanceAlongRoute+=stepDistance;
   }
   previousState=state;
   previousCheck=currentCheck;
  }
  const limitingPart=parts.reduce((best,item)=>
   item.minimumClearanceLowerBound<best.minimumClearanceLowerBound?item:best,parts[0]);
  return {
   statesAnalyzed:states.length,segmentsAnalyzed:Math.max(0,states.length-1),distanceAlongRoute,
   parts,minimumClearanceLowerBound:limitingPart.minimumClearanceLowerBound,
   limitingPart:limitingPart.vehiclePart,limitingDetail:limitingPart.limitingDetail,
   method:'per-part Hausdorff displacement bound from each segment start pose',
   note:'Для сегмента атрибутируется часть автопоезда; ближайшее препятствие в середине сегмента может отличаться от препятствия в начальной позе.'
  };
 }

 // A wheel's circumscribed tire envelope is orientation-independent. This lets
 // the swept bound remain valid even when steering changes between sampled poses.
 function wheelEnvelopePolygon(center,radius,segments=16){
  if(!center||![center.x,center.y,radius].every(Number.isFinite)||radius<=0){
   throw new Error('Wheel envelope requires a finite center and positive radius.');
  }
  const vertexRadius=radius/Math.cos(Math.PI/segments);
  return Array.from({length:segments},(_,index)=>{
   const angle=2*Math.PI*index/segments;
   return {x:center.x+vertexRadius*Math.cos(angle),y:center.y+vertexRadius*Math.sin(angle)};
  });
 }
 function matrixSpectralNorm2x2(a,b,c,d){
  const column1=a*a+c*c,column2=b*b+d*d,cross=a*b+c*d;
  const largestEigenvalue=0.5*(column1+column2+Math.hypot(column1-column2,2*cross));
  return Math.sqrt(Math.max(0,largestEigenvalue));
 }
 function wheelCenterDisplacementRatePerMeter(wheel,p,previousState,currentState){
  const previousSteer=previousState.steer===undefined?0:previousState.steer;
  const currentSteer=currentState.steer===undefined?0:currentState.steer;
  const maxAbsSteer=Math.max(Math.abs(previousSteer),Math.abs(currentSteer));
  const curvature=Math.abs(Math.tan(maxAbsSteer)/p.wheelbase);
  if(wheel.vehiclePart==='тягач'){
   const longitudinal=wheel.axleIndex===1?p.wheelbase:0;
   const track=wheel.axleIndex===1?(p.tractorFrontTrack??2.05):(p.tractorRearTrack??2.05);
   const lateral=Math.abs(track/2);
   return Math.hypot(1+lateral*curvature,longitudinal*curvature);
  }
  if(wheel.vehiclePart==='полуприцеп'){
   const spacing=p.trailerAxleSpacing??1.3;
   const longitudinal=[spacing,0,-spacing][wheel.axleIndex-1];
   if(!Number.isFinite(longitudinal))throw new Error('Trailer wheel axle index is invalid for swept-clearance analysis.');
   const track=p.trailerTrack??(p.trailerWidth+0.24);
   const lateral=(wheel.side==='left'?1:-1)*track/2;
   const couplingMagnitude=p.hitchOffset*curvature;
   let maximum=0;
   for(const sign of [-1,1]){
    const coupling=sign*couplingMagnitude;
    const a=1+coupling*lateral/p.kingpinAxles;
    const b=coupling-lateral/p.kingpinAxles;
    const c=-coupling*longitudinal/p.kingpinAxles;
    const d=longitudinal/p.kingpinAxles;
    maximum=Math.max(maximum,matrixSpectralNorm2x2(a,b,c,d));
   }
   return maximum;
  }
  throw new Error('Unknown vehicle part in wheel swept-clearance analysis.');
 }
 function analyzeWheelTrajectoryClearance(states,obstacles,p){
  if(!Array.isArray(states)||states.length===0){
   throw new Error('Route states are required for per-wheel clearance analysis.');
  }
  const geometryErrors=rootPhysics&&typeof rootPhysics.validateVehicleParams==='function'?
   rootPhysics.validateVehicleParams(p):['Vehicle geometry validation is unavailable.'];
  if(geometryErrors.length)throw new Error('Invalid vehicle geometry for per-wheel clearance analysis: '+geometryErrors.join(' '));
  if(!rootPhysics||typeof rootPhysics.wheelFootprints!=='function'){
   throw new Error('Shared wheel geometry is unavailable for per-wheel clearance analysis.');
  }
  const obstacleErrors=validateObstacles(obstacles);
  if(obstacleErrors.length)throw new Error('Invalid obstacles for per-wheel clearance analysis: '+obstacleErrors.join(' '));
  const prepared=obstacles.map(obstacle=>({poly:obstaclePoly(obstacle),name:obstacle.name||'препятствие'}));
  const uncertainty=Number.isFinite(p.footprintUncertainty)?p.footprintUncertainty:0;
  const tireLength=p.tireLength??0.72,tireWidth=p.tireWidth??0.44;
  const tireRadius=Math.hypot(tireLength/2+uncertainty,tireWidth/2+uncertainty);
  const records=new Map();
  let previousState=null,previousChecks=null,distanceAlongRoute=0;
  let segmentsWithIntermediateKinematicSamples=0,segmentsWithoutIntermediateKinematicSamples=0;
  let segmentsSkippedForNoObstacles=0,intermediateSamplesAnalyzed=0;
  let segmentsAtAdaptiveSamplingCap=0,maxAdaptiveIntervalsUsed=0;
  let segmentsWithGearChangeAtStart=0,segmentsWithInitialGearSelection=0;
  let segmentsWithSteeringChangeAtStart=0,segmentsWithInitialSteeringSelection=0;
  let controlStartPosesAnalyzed=0,controlStartWheelChecks=0;
  let maximumWheelCenterDisplacementBound=0;
  const targetMaxWheelCenterDisplacementPerInterval=0.05;
  const minAdaptiveIntervals=4,maxAdaptiveIntervals=64;
  const baselineIntermediateSampleFractions=[0.25,0.5,0.75];
  const endpointTolerance={position:1e-5,heading:1e-6};
  const segmentDiagnostics=[];
  const snapshot=state=>({
   x:state.x,y:state.y,psi:state.psi,psiT:state.psiT,
   gear:state.gear===-1?-1:1,steer:state.steer===undefined?0:state.steer,
   gearKnown:state.gearKnown!==false,steerKnown:state.steerKnown!==false
  });
  for(let index=0;index<states.length;index++){
   const state=states[index];
   if(!state||![state.x,state.y,state.psi,state.psiT].every(Number.isFinite)){
    throw new Error('Route state '+index+' is invalid for per-wheel clearance analysis.');
   }
   const gear=state.gear===undefined?1:state.gear;
   const steer=state.steer===undefined?0:state.steer;
   if(gear!==1&&gear!==-1)throw new Error('Route state '+index+' has an invalid gear for per-wheel clearance analysis.');
   if(!Number.isFinite(steer)||Math.abs(steer)>=Math.PI/2){
    throw new Error('Route state '+index+' has an invalid steering angle for per-wheel clearance analysis.');
   }
   let segmentStartDistance=distanceAlongRoute,stepDistance=null;
   if(index>0){
    stepDistance=state.stepDistance;
    if(!Number.isFinite(stepDistance)||stepDistance<=0){
     throw new Error('Route state '+index+' is missing its positive integration stepDistance; per-wheel swept bound cannot be established.');
    }
    const chordDistance=Math.hypot(state.x-previousState.x,state.y-previousState.y);
    if(chordDistance>stepDistance+1e-6){
     throw new Error('Route state '+index+' moves farther than its integration stepDistance; per-wheel swept bound is invalid.');
    }
    segmentStartDistance=distanceAlongRoute;
    distanceAlongRoute+=stepDistance;
   }
   const footprints=rootPhysics.wheelFootprints(state,p,steer);
   if(footprints.length!==10)throw new Error('Per-wheel geometry must contain exactly ten wheel footprints.');
   const currentChecks=footprints.map(wheel=>{
    const envelope=wheelEnvelopePolygon(wheel.center,tireRadius);
    let envelopeDistance=Infinity,envelopeObstacle=null;
    let polygonDistance=Infinity,polygonObstacle=null;
    for(const obstacle of prepared){
     const envelopeGap=polyDist(envelope,obstacle.poly);
     if(envelopeGap<envelopeDistance){envelopeDistance=envelopeGap;envelopeObstacle=obstacle.name;}
     const polygonGap=polyDist(wheel.polygon,obstacle.poly);
     if(polygonGap<polygonDistance){polygonDistance=polygonGap;polygonObstacle=obstacle.name;}
    }
    let record=records.get(wheel.name);
    if(!record){
     record={
      name:wheel.name,vehiclePart:wheel.vehiclePart,axleIndex:wheel.axleIndex,side:wheel.side,
      minimumSampledPolygonClearance:Infinity,minimumSampledPolygonDetail:null,
      minimumSampledEnvelopeClearance:Infinity,minimumSampledEnvelopeDetail:null,
      minimumIntermediatePolygonClearance:Infinity,minimumIntermediatePolygonDetail:null,
      minimumIntermediateEnvelopeClearance:Infinity,minimumIntermediateEnvelopeDetail:null,
      minimumClearanceLowerBound:Infinity,limitingDetail:null
     };
     records.set(wheel.name,record);
    }
    if(polygonDistance<record.minimumSampledPolygonClearance){
     record.minimumSampledPolygonClearance=polygonDistance;
     record.minimumSampledPolygonDetail=Number.isFinite(polygonDistance)?{
      obstacle:polygonObstacle,state:snapshot(state),distanceAlongRoute
     }:null;
    }
    if(envelopeDistance<record.minimumSampledEnvelopeClearance){
     record.minimumSampledEnvelopeClearance=envelopeDistance;
     record.minimumSampledEnvelopeDetail=Number.isFinite(envelopeDistance)?{
      obstacle:envelopeObstacle,state:snapshot(state),distanceAlongRoute
     }:null;
    }
    if(envelopeDistance<record.minimumClearanceLowerBound){
     record.minimumClearanceLowerBound=envelopeDistance;
     record.limitingDetail=Number.isFinite(envelopeDistance)?{
      kind:'sampled-circumscribed-envelope',lowerBound:envelopeDistance,
      obstacle:envelopeObstacle,state:snapshot(state),distanceAlongRoute
     }:null;
    }
    return {wheel,envelopeDistance,envelopeObstacle,polygonDistance,polygonObstacle};
   });
   if(index>0){
    let intermediateSampleStatus='rk4-unavailable',endpointResidual=null;
    let intervalCount=0,sampleFractions=[],maxWheelRate=0;
    let maxCenterDisplacementPerInterval=0,adaptiveCapReached=false;
    const previousByName=new Map(previousChecks.map(item=>[item.wheel.name,item]));
    const ratesByWheel=new Map();
    const applyFullSegmentFallbackBound=reason=>{
     for(const current of currentChecks){
      const previous=previousByName.get(current.wheel.name);
      if(!previous)throw new Error('Wheel identity changed between adjacent route states.');
      if(!Number.isFinite(previous.envelopeDistance))continue;
      const rate=wheelCenterDisplacementRatePerMeter(current.wheel,p,previousState,state);
      const lowerBound=previous.envelopeDistance-rate*stepDistance;
      const record=records.get(current.wheel.name);
      if(lowerBound<record.minimumClearanceLowerBound){
       record.minimumClearanceLowerBound=lowerBound;
       record.limitingDetail={
        kind:'segment-circumscribed-envelope-lower-bound',lowerBound,
        obstacleAtReference:previous.envelopeObstacle,referenceDistance:previous.envelopeDistance,
        displacementRate:rate,stepDistance,distanceAlongRoute:segmentStartDistance,
        from:snapshot(previousState),to:snapshot(state),gear,steer,
        fallbackReason:reason
       };
      }
     }
    };
    const segmentGear=gear,segmentSteer=steer;
    let controlStartPoseChecked=false;
    if(prepared.length>0&&
       (initialSteeringSelectionAtSegmentStart||steeringChangedAtSegmentStart)){
     // Steering is selected at the stationary start boundary of this spatial
     // segment. Check the new tire headings before integrating any distance.
     const controlStartFootprints=rootPhysics.wheelFootprints(previousState,p,segmentSteer);
     if(controlStartFootprints.length!==10){
      throw new Error('Control-start wheel geometry must contain exactly ten footprints.');
     }
     for(const wheel of controlStartFootprints){
      let polygonDistance=Infinity,polygonObstacle=null;
      for(const obstacle of prepared){
       const gap=polyDist(wheel.polygon,obstacle.poly);
       if(gap<polygonDistance){polygonDistance=gap;polygonObstacle=obstacle.name;}
      }
      const record=records.get(wheel.name);
      if(!record)throw new Error('Wheel identity changed during control-start pose checking.');
      if(polygonDistance<record.minimumIntermediatePolygonClearance){
       record.minimumIntermediatePolygonClearance=polygonDistance;
       record.minimumIntermediatePolygonDetail=Number.isFinite(polygonDistance)?{
        obstacle:polygonObstacle,
        state:snapshot({...previousState,gear:segmentGear,steer:segmentSteer,gearKnown:true,steerKnown:true}),
        distanceAlongRoute:segmentStartDistance,sampleFraction:0,controlStartPose:true
       }:null;
      }
     }
     controlStartPoseChecked=true;
     controlStartPosesAnalyzed++;
     controlStartWheelChecks+=controlStartFootprints.length;
    }
    if(prepared.length===0){
     intermediateSampleStatus='skipped-no-obstacles';
     segmentsSkippedForNoObstacles++;
    }else if(typeof rootPhysics.rk4==='function'){
     // A gear change is an instantaneous control boundary in this spatial model:
     // the following segment uses its destination gear from the shared start pose.
     // Stop time, clutch engagement and drivetrain transients are not modelled.
     // Intermediate samples are accepted only when the reconstructed endpoint
     // matches the actual route node within the documented numerical tolerance.
     const segmentDt=stepDistance/0.7;
     const reconstructedEnd=rootPhysics.rk4(
      previousState,segmentGear*0.7,segmentSteer,segmentDt,p
     );
     endpointResidual={
      position:Math.hypot(reconstructedEnd.x-state.x,reconstructedEnd.y-state.y),
      tractorHeading:Math.abs(norm(reconstructedEnd.psi-state.psi)),
      trailerHeading:Math.abs(norm(reconstructedEnd.psiT-state.psiT))
     };
     const endpointMatches=
      endpointResidual.position<=endpointTolerance.position&&
      endpointResidual.tractorHeading<=endpointTolerance.heading&&
      endpointResidual.trailerHeading<=endpointTolerance.heading;
     if(endpointMatches){
      for(const current of currentChecks){
       const previous=previousByName.get(current.wheel.name);
       if(!previous)throw new Error('Wheel identity changed between adjacent route states.');
       const rate=wheelCenterDisplacementRatePerMeter(current.wheel,p,previousState,state);
       if(!Number.isFinite(rate)||rate<0)throw new Error('Wheel displacement-rate bound is invalid.');
       ratesByWheel.set(current.wheel.name,rate);
       maxWheelRate=Math.max(maxWheelRate,rate);
      }
      const requiredIntervals=Math.max(
       minAdaptiveIntervals,
       Math.ceil(maxWheelRate*stepDistance/targetMaxWheelCenterDisplacementPerInterval)
      );
      intervalCount=Math.min(maxAdaptiveIntervals,requiredIntervals);
      adaptiveCapReached=requiredIntervals>maxAdaptiveIntervals;
      if(adaptiveCapReached)segmentsAtAdaptiveSamplingCap++;
      maxAdaptiveIntervalsUsed=Math.max(maxAdaptiveIntervalsUsed,intervalCount);
      maxCenterDisplacementPerInterval=maxWheelRate*stepDistance/intervalCount;
      maximumWheelCenterDisplacementBound=Math.max(
       maximumWheelCenterDisplacementBound,maxCenterDisplacementPerInterval
      );
      sampleFractions=Array.from({length:intervalCount-1},(_,sampleIndex)=>
       (sampleIndex+1)/intervalCount
      );
      const samplesByWheel=new Map();
      for(const current of currentChecks){
       const previous=previousByName.get(current.wheel.name);
       samplesByWheel.set(current.wheel.name,[{
        fraction:0,envelopeDistance:previous.envelopeDistance,
        envelopeObstacle:previous.envelopeObstacle
       }]);
      }
      for(const fraction of sampleFractions){
       const intermediateState=rootPhysics.rk4(
        previousState,segmentGear*0.7,segmentSteer,segmentDt*fraction,p
       );
       const intermediateDistance=segmentStartDistance+stepDistance*fraction;
       const intermediateFootprints=rootPhysics.wheelFootprints(
        intermediateState,p,segmentSteer
       );
       if(intermediateFootprints.length!==10){
        throw new Error('Intermediate wheel geometry must contain exactly ten footprints.');
       }
       for(const wheel of intermediateFootprints){
        const envelope=wheelEnvelopePolygon(wheel.center,tireRadius);
        let envelopeDistance=Infinity,envelopeObstacle=null;
        let polygonDistance=Infinity,polygonObstacle=null;
        for(const obstacle of prepared){
         const envelopeGap=polyDist(envelope,obstacle.poly);
         if(envelopeGap<envelopeDistance){
          envelopeDistance=envelopeGap;
          envelopeObstacle=obstacle.name;
         }
         const polygonGap=polyDist(wheel.polygon,obstacle.poly);
         if(polygonGap<polygonDistance){
          polygonDistance=polygonGap;
          polygonObstacle=obstacle.name;
         }
        }
        const record=records.get(wheel.name);
        if(!record)throw new Error('Wheel identity changed during intermediate swept sampling.');
        if(polygonDistance<record.minimumIntermediatePolygonClearance){
         record.minimumIntermediatePolygonClearance=polygonDistance;
         record.minimumIntermediatePolygonDetail=Number.isFinite(polygonDistance)?{
          obstacle:polygonObstacle,state:snapshot({...intermediateState,gear:segmentGear,steer:segmentSteer}),
          distanceAlongRoute:intermediateDistance,sampleFraction:fraction
         }:null;
        }
        if(envelopeDistance<record.minimumIntermediateEnvelopeClearance){
         record.minimumIntermediateEnvelopeClearance=envelopeDistance;
         record.minimumIntermediateEnvelopeDetail=Number.isFinite(envelopeDistance)?{
          obstacle:envelopeObstacle,state:snapshot({...intermediateState,gear:segmentGear,steer:segmentSteer}),
          distanceAlongRoute:intermediateDistance,sampleFraction:fraction
         }:null;
        }
        const wheelSamples=samplesByWheel.get(wheel.name);
        if(!wheelSamples)throw new Error('Wheel identity changed during adaptive swept sampling.');
        wheelSamples.push({fraction,envelopeDistance,envelopeObstacle});
       }
       intermediateSamplesAnalyzed++;
      }
      for(const current of currentChecks){
       const wheelName=current.wheel.name;
       const samples=samplesByWheel.get(wheelName);
       samples.push({
        fraction:1,envelopeDistance:current.envelopeDistance,
        envelopeObstacle:current.envelopeObstacle
       });
       const rate=ratesByWheel.get(wheelName);
       const halfIntervalDisplacementBound=rate*stepDistance/(2*intervalCount);
       const record=records.get(wheelName);
       for(let sampleIndex=0;sampleIndex<samples.length-1;sampleIndex++){
        const a=samples[sampleIndex],b=samples[sampleIndex+1];
        const reference=a.envelopeDistance<=b.envelopeDistance?a:b;
        const lowerBound=Math.min(a.envelopeDistance,b.envelopeDistance)-halfIntervalDisplacementBound;
        if(lowerBound<record.minimumClearanceLowerBound){
         record.minimumClearanceLowerBound=lowerBound;
         record.limitingDetail={
          kind:'adaptive-segment-circumscribed-envelope-lower-bound',lowerBound,
          obstacleAtReference:reference.envelopeObstacle,referenceDistance:reference.envelopeDistance,
          displacementRate:rate,stepDistance,intervalCount,
          intervalFractionStart:a.fraction,intervalFractionEnd:b.fraction,
          maximumCenterDisplacementBound:rate*stepDistance/intervalCount,
          halfIntervalDisplacementBound,
          distanceAlongRoute:segmentStartDistance+stepDistance*a.fraction,
          from:snapshot(previousState),to:snapshot(state),gear,steer
         };
        }
       }
      }
      intermediateSampleStatus='rk4-adaptive-intermediate-samples';
      segmentsWithIntermediateKinematicSamples++;
     }else{
      intermediateSampleStatus='kinematic-endpoint-mismatch';
      segmentsWithoutIntermediateKinematicSamples++;
      applyFullSegmentFallbackBound('kinematic-endpoint-mismatch');
     }
    }else{
     intermediateSampleStatus='rk4-unavailable';
     segmentsWithoutIntermediateKinematicSamples++;
     applyFullSegmentFallbackBound('rk4-unavailable');
    }
    const gearTransitionKind=initialGearSelectionAtSegmentStart?
     'initial-gear-selection':gearChangedAtSegmentStart?'gear-change':'none';
    const gearTransition=gearTransitionKind==='none'?null:{
     kind:gearTransitionKind,
     from:previousGearKnown?previousGear:null,
     to:gear,
     at:'segment-start',
     dwellTimeModelled:false,
     spatialSegmentUsesGear:segmentGear
    };
    segmentDiagnostics.push({
     fromSample:index-1,toSample:index,stepDistance,
     status:intermediateSampleStatus,endpointResidual,
     intervalCount,sampleCount:sampleFractions.length,
     sampleFractions,adaptiveCapReached,
     gearChangedAtSegmentStart,initialGearSelectionAtSegmentStart,
     steeringChangedAtSegmentStart,initialSteeringSelectionAtSegmentStart,
     gearTransition,controlStartPoseChecked,
     motionModel:'constant-control-spatial-segment; gear-change dwell not modelled',
     maxWheelCenterDisplacementPerInterval:maxCenterDisplacementPerInterval,
     maxWheelCenterRatePerMeter:maxWheelRate
    });
   }
   previousState=state;
   previousChecks=currentChecks;
  }
  const wheels=[...records.values()];
  const limitingWheel=wheels.reduce((best,wheel)=>
   Number.isFinite(wheel.minimumClearanceLowerBound)&&
   (!best||wheel.minimumClearanceLowerBound<best.minimumClearanceLowerBound)?wheel:best,null);
  return {
   statesAnalyzed:states.length,segmentsAnalyzed:Math.max(0,states.length-1),
   distanceAlongRoute,wheels,limitingWheel,
   minimumClearanceLowerBound:limitingWheel?limitingWheel.minimumClearanceLowerBound:Infinity,
   segmentsWithIntermediateKinematicSamples,segmentsWithoutIntermediateKinematicSamples,
   segmentsSkippedForNoObstacles,intermediateSamplesAnalyzed,
   segmentsWithGearChangeAtStart,segmentsWithInitialGearSelection,
   segmentsWithSteeringChangeAtStart,segmentsWithInitialSteeringSelection,
   controlStartPosesAnalyzed,controlStartWheelChecks,
   intermediateSampleFractions:baselineIntermediateSampleFractions,
   segmentsAtAdaptiveSamplingCap,maxAdaptiveIntervalsUsed,maximumWheelCenterDisplacementBound,
   targetMaxWheelCenterDisplacementPerInterval,minAdaptiveIntervals,maxAdaptiveIntervals,
   segmentDiagnostics,
   intermediateSamplingMethod:'adaptive-rk4-samples-with-per-interval-circumscribed-envelope-lower-bound',
   envelope:'circumscribed-16-gon-around-rectangular-tire-footprint',
   method:'per-wheel center displacement rate bound refined by adaptive RK4 samples and half-interval envelope-distance bounds',
   note:'Нижняя оценка использует модельную верхнюю границу скорости центра колеса на каждом интервале и минимум расстояний в его концах. Она применима только при согласованной RK4-реконструкции сегмента и корректных параметрах модели; при достижении лимита число интервалов ограничивается 64, что отражается в диагностике. Это не сертифицированная непрерывная проверка реального автопоезда.'
  };
 }

 function push(heap,n){
  let i=heap.length;heap.push(n);
  while(i){const p=(i-1)>>1;if(heap[p].f<=n.f)break;heap[i]=heap[p];i=p;heap[i]=n;}
 }
 function pop(heap){
  const top=heap[0],last=heap.pop();
  if(heap.length){let i=0;heap[0]=last;for(;;){let l=i*2+1,r=l+1,m=i;if(l<heap.length&&heap[l].f<heap[m].f)m=l;if(r<heap.length&&heap[r].f<heap[m].f)m=r;if(m===i)break;[heap[i],heap[m]]=[heap[m],heap[i]];i=m;}}
  return top;
 }
 function plan(options={}){
  const inputErrors=validatePlanOptions(options);
  if(inputErrors.length)return {ok:false,reason:'invalid-input',errors:inputErrors,expanded:0};
  const {start,goal,params,obstacles}=options;
  const maxExpansions=options.maxExpansions||24000;
  const bounds=options.bounds||{minX:-20,maxX:42,minY:-20,maxY:20};
  const step=0.20,angleStep=Math.PI/60,primitive=0.6,ds=0.05;
  const maxSteer=options.maxSteer||Math.PI/7;
  const minClearance=Number.isFinite(options.minClearance)?Math.max(0,options.minClearance):0.30;
  const positionTolerance=options.positionTolerance??0.9;
  const headingTolerance=options.headingTolerance??12*Math.PI/180;
  const maxTerminalArticulation=options.maxTerminalArticulation??12*Math.PI/180;
  const goalOptions={positionTolerance,headingTolerance,maxTerminalArticulation};
  const checkClearance=createClearanceChecker(obstacles,params);
  const key=q=>[
   Math.round(q.x/step),Math.round(q.y/step),
   Math.round(norm(q.psi)/angleStep),Math.round(norm(q.psiT)/angleStep)
  ].join(':');
  const h=q=>{
   const e=goalErrors(q,goal);
   return e.position+0.45*(e.headingError??0)+0.35*e.articulation;
  };
  const startCheck=checkClearance(start,0);
  const startClearance=startCheck.distance;
  const snapshot=(q,gear=1,steer=0)=>({
   x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear,steer
  });
  const startDetail=Number.isFinite(startClearance)?
   {...startCheck,state:snapshot(start,1,0)}:null;
  const initialSweptDetail=Number.isFinite(startClearance)?{
   kind:'initial-pose',
   lowerBound:startClearance,
   state:snapshot(start,1,0),
   referenceVehiclePart:startCheck.vehiclePart,
   referenceObstacle:startCheck.obstacle
  }:null;
  if(startClearance===0)return {
   ok:false,reason:'start-collision',expanded:0,startClearance,
   minimumClearanceDetail:startDetail
  };
  if(Number.isFinite(startClearance)&&startClearance<minClearance-1e-9){
   return {
    ok:false,reason:'start-clearance-insufficient',expanded:0,startClearance,
    requiredClearance:minClearance,minimumClearanceDetail:startDetail
   };
  }
  const requiredClearance=minClearance;
  const initial={
   q:{...start},g:0,f:h(start),parent:null,gear:1,steer:0,samples:[],firstTurnChosen:false,
   clearance:startClearance,clearanceDetail:startDetail,
   sweptLowerBound:startClearance,sweptLowerBoundDetail:initialSweptDetail
  };
  const open=[];push(open,initial);
  const best=new Map([[key(start),0]]);
  let expanded=0,finish=null,closest=initial;
  const quality=q=>{
   const e=goalErrors(q,goal);
   return e.position+0.5*(e.headingError??0)+0.5*Math.max(0,e.articulation-maxTerminalArticulation);
  };
  let closestQuality=quality(start);
  const steering=[-maxSteer,-0.24,-0.12,0,0.12,0.24,maxSteer];
  const controls=[];
  for(const gear of [-1,1])for(const steer of steering)controls.push([gear,steer]);
  while(open.length&&expanded<maxExpansions){
   const cur=pop(open),ck=key(cur.q);
   if(cur.g>(best.get(ck)??Infinity)+1e-8)continue;
   expanded++;
   const errors=goalErrors(cur.q,goal);
   const dist=errors.position;
   const currentQuality=quality(cur.q);
   if(currentQuality<closestQuality){closest=cur;closestQuality=currentQuality;}
   if(options.onProgress&&(expanded===1||expanded%250===0)){
    try{options.onProgress({expanded,maxExpansions,goalDistance:dist,headingError:errors.headingError,articulationAngle:errors.articulation,closest:{x:closest.q.x,y:closest.q.y}});}catch(_){}
   }
   if(goalReached(cur.q,goal,goalOptions)&&(!options.firstActionConstraint||cur.firstTurnChosen)){finish=cur;break;}
   let activeControls=controls;
   const firstActionConstraint=options.firstActionConstraint;
   if(firstActionConstraint&&!cur.firstTurnChosen){
    activeControls=controls.filter(([gear,steer])=>{
     if(!cur.parent){
      if(gear!==firstActionConstraint.initialGear)return false;
      return firstActionConstraint.firstTurn==='left'?steer>=0.08:steer<=-0.08;
     }
     if(Math.abs(steer)<0.08)return true;
     return firstActionConstraint.firstTurn==='left'?steer>0:steer<0;
    }).sort((a,b)=>Math.abs(a[1])-Math.abs(b[1]));
   }
   for(const [gear,steer] of activeControls){
    let q={...cur.q},valid=true,minD=Infinity,minClearanceDetail=null,samples=[];
    let currentClearance=cur.clearance,currentClearanceDetail=cur.clearanceDetail;
    let sweptLowerBound=cur.sweptLowerBound,sweptLowerBoundDetail=cur.sweptLowerBoundDetail,travelled=0;
    // Steering can rotate the front tire envelopes before the tractor translates.
    // Check that control-start pose explicitly instead of reusing the previous node's clearance.
    const controlStartCheck=checkClearance(q,steer);
    if(controlStartCheck.distance<requiredClearance-1e-9)continue;
    currentClearance=controlStartCheck.distance;
    currentClearanceDetail=Number.isFinite(controlStartCheck.distance)?
     {...controlStartCheck,state:snapshot(q,gear,steer)}:null;
    if(controlStartCheck.distance<minD){
     minD=controlStartCheck.distance;
     minClearanceDetail=currentClearanceDetail;
    }
    if(controlStartCheck.distance<sweptLowerBound-1e-12){
     sweptLowerBound=controlStartCheck.distance;
     sweptLowerBoundDetail={
      kind:'control-start-pose',lowerBound:controlStartCheck.distance,
      state:snapshot(q,gear,steer),
      referenceVehiclePart:controlStartCheck.vehiclePart||null,
      referenceObstacle:controlStartCheck.obstacle||null
     };
    }
    while(travelled<primitive-1e-9){
     const remaining=primitive-travelled;
     let travel=Math.min(ds,remaining),displacementRate=0;
     if(Number.isFinite(currentClearance)){
      const margin=currentClearance-requiredClearance;
      if(margin<=1e-9){valid=false;break;}
      displacementRate=sweptDisplacementPerMeter(steer,params,q,travel);
      travel=Math.min(travel,0.8*margin/displacementRate);
     }
     if(travel<5e-3){valid=false;break;}
     const segmentStart={...q};
     const next=options.integrate(q,gear*0.7,steer,travel/0.7,params);
     if(next.x<bounds.minX||next.x>bounds.maxX||next.y<bounds.minY||next.y>bounds.maxY||
        Math.abs(norm(next.psi-next.psiT))>Math.PI*0.92){valid=false;break;}
     const c=checkClearance(next,steer);
     if(c.distance<requiredClearance-1e-9){valid=false;break;}
     const segmentLowerBound=Number.isFinite(currentClearance)?currentClearance-displacementRate*travel:Infinity;
     if(segmentLowerBound<sweptLowerBound-1e-12){
      sweptLowerBound=segmentLowerBound;
      sweptLowerBoundDetail={
       kind:'segment-lower-bound',
       lowerBound:segmentLowerBound,
       from:snapshot(segmentStart,gear,steer),
       to:snapshot(next,gear,steer),
       stepDistance:travel,
       displacementRate,
       referenceDistance:currentClearance,
       referenceVehiclePart:currentClearanceDetail?.vehiclePart||null,
       referenceObstacle:currentClearanceDetail?.obstacle||null
      };
     }
     if(c.distance<minD){
      minD=c.distance;
      minClearanceDetail=Number.isFinite(c.distance)?
       {...c,state:snapshot(next,gear,steer)}:null;
     }
     q=next;
     currentClearance=c.distance;
     currentClearanceDetail=Number.isFinite(c.distance)?
      {...c,state:snapshot(next,gear,steer)}:null;
     travelled+=travel;
     samples.push({x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear,steer,stepDistance:travel});
    }
    if(!valid)continue;
    const nk=key(q);
    const gearPenalty=gear<0?0.55:0;
    const steerPenalty=Math.abs(steer)*0.08+(cur.steer!==steer?0.10:0);
    const ng=cur.g+primitive+gearPenalty+steerPenalty;
    if(ng>=(best.get(nk)??Infinity))continue;
    best.set(nk,ng);
    push(open,{
     q,g:ng,f:ng+h(q),parent:cur,gear,steer,minClearance:minD,
     firstTurnChosen:cur.firstTurnChosen||Math.abs(steer)>=0.08,
     minClearanceDetail,samples,clearance:currentClearance,
     clearanceDetail:currentClearanceDetail,
     sweptLowerBound,sweptLowerBoundDetail
    });
   }
  }
  if(!finish){
   const errors=goalErrors(closest.q,goal);
   // A non-empty valid frontier means the expansion budget stopped the search;
   // do not report that as proof that no route exists.
   const frontierRemains=open.some(node=>
    node.g<=(best.get(key(node.q))??Infinity)+1e-8
   );
   return {ok:false,reason:frontierRemains?'search-limit':'no-path',expanded,
    searchLimitReached:frontierRemains,searchExhausted:!frontierRemains,
    closest:{x:closest.q.x,y:closest.q.y,psi:closest.q.psi,psiT:closest.q.psiT},
    goalDistance:errors.position,headingError:errors.headingError,articulationAngle:errors.articulation};
  }
  const chain=[];for(let n=finish;n&&n.parent;n=n.parent)chain.push(n);
  chain.reverse();
  const states=[{x:start.x,y:start.y,psi:start.psi,psiT:start.psiT,
   gear:1,steer:0,gearKnown:false,steerKnown:false,stepDistance:0}];
  for(const n of chain)states.push(...n.samples);
  let pathMinimum=startClearance;
  let minimumClearanceDetail=startDetail;
  for(const node of chain){
   if(node.minClearance<pathMinimum){
    pathMinimum=node.minClearance;
    minimumClearanceDetail=node.minClearanceDetail;
   }
  }
  const trajectoryClearance=analyzeTrajectoryClearance(states,obstacles,params);
  const minimumClearanceLowerBound=Math.min(
   finish.sweptLowerBound,trajectoryClearance.minimumClearanceLowerBound
  );
  return {
   ok:true,states,expanded,length:finish.g,
   minimumClearance:pathMinimum,minimumClearanceDetail,
   minimumClearanceLowerBound,
   sweptClearanceDetail:finish.sweptLowerBoundDetail,
   trajectoryClearance,requiredClearance,
   terminalErrors:goalErrors(finish.q,goal)
  };
 }
 function articulationExposure(states){
  if(!Array.isArray(states)||states.length===0)throw new Error('Route states are required for articulation exposure.');
  let distance=0,peak=null;
  for(let i=0;i<states.length;i++){
   const current=states[i];
   if(!current||![current.x,current.y,current.psi,current.psiT].every(Number.isFinite)){
    throw new Error('Route state '+i+' has invalid coordinates or headings for articulation exposure.');
   }
   if(i>0){
    const previous=states[i-1];
    distance+=Math.hypot(current.x-previous.x,current.y-previous.y);
   }
   const signedAngle=norm(current.psi-current.psiT);
   const magnitude=Math.abs(signedAngle);
   if(!peak||magnitude>peak.maxAngleRadians){
    const gear=(current.gear??1)<0?-1:1;
    peak={
     maxAngleRadians:magnitude,
     maxAngleDegrees:magnitude*180/Math.PI,
     signedAngleRadians:signedAngle,
     signedAngleDegrees:signedAngle*180/Math.PI,
     stateIndex:i,
     segmentIndex:Math.max(0,i-1),
     distanceAlongRoute:distance,
     gear,
     state:{
      x:current.x,y:current.y,psi:current.psi,psiT:current.psiT,
      gear,steer:Number.isFinite(current.steer)?current.steer:0
     }
    };
   }
  }
  return peak;
 }
 function recommendStart(states,options={}){
  if(!Array.isArray(states)||states.length<2)return {ok:false,reason:'route-too-short'};
  const steeringThreshold=Number.isFinite(options.steeringThreshold)&&options.steeringThreshold>0?options.steeringThreshold:0.08;
  let distance=0;
  const first=states[1];
  const initialGear=(first.gear??1)<0?-1:1;
  let firstTurn=null;
  let firstTurnDistance=null;
  let firstTurnGear=null;
  let tractorYawDirection=null;
  for(let i=1;i<states.length;i++){
   const previous=states[i-1],current=states[i];
   distance+=Math.hypot(current.x-previous.x,current.y-previous.y);
   const steer=Number(current.steer)||0;
   if(Math.abs(steer)>=steeringThreshold){
    firstTurn=steer>0?'left':'right';
    firstTurnDistance=distance;
    firstTurnGear=(current.gear??initialGear)<0?-1:1;
    // Positive yaw is counter-clockwise (left); reverse gear flips yaw for the same steering angle.
    tractorYawDirection=steer*firstTurnGear>0?'left':'right';
    break;
   }
  }
  return {
   ok:true,
   initialGear,
   initialSteer:Number(first.steer)||0,
   firstTurn,
   steeringWheelDirection:firstTurn,
   firstTurnDistance,
   firstTurnGear,
   tractorYawDirection,
   firstTurnAfterGearChange:firstTurnGear!==null&&firstTurnGear!==initialGear
  };
 }

 
 function compareStartCandidates(candidates,options={}){
  if(!Array.isArray(candidates)||candidates.length===0){
   return {ok:false,reason:'no-candidates'};
  }
  const costTolerance=options.costTolerance===undefined?0.25:options.costTolerance;
  if(!Number.isFinite(costTolerance)||costTolerance<0){
   return {ok:false,reason:'invalid-cost-tolerance'};
  }
  const firstTurnDistanceTolerance=options.firstTurnDistanceTolerance===undefined?0.25:options.firstTurnDistanceTolerance;
  if(!Number.isFinite(firstTurnDistanceTolerance)||firstTurnDistanceTolerance<0){
   return {ok:false,reason:'invalid-first-turn-distance-tolerance'};
  }
  const assessed=[];
  for(let i=0;i<candidates.length;i++){
   const candidate=candidates[i];
   if(!candidate||!Number.isFinite(candidate.cost)||candidate.cost<0){
    return {ok:false,reason:'invalid-candidate-cost',candidateIndex:i};
   }
   const recommendation=recommendStart(candidate.states,options);
   if(!recommendation.ok){
    return {ok:false,reason:'invalid-candidate-route',candidateIndex:i};
   }
   assessed.push({
    index:i,
    label:typeof candidate.label==='string'&&candidate.label?candidate.label:'вариант '+(i+1),
    cost:candidate.cost,
    recommendation
   });
  }
  assessed.sort((a,b)=>a.cost-b.cost||a.index-b.index);
  const best=assessed[0];
  const comparable=assessed.slice(1).filter(candidate=>
   candidate.cost-best.cost<=costTolerance+1e-12
  );
  const ambiguousFields=[];
  if(comparable.some(candidate=>candidate.recommendation.initialGear!==best.recommendation.initialGear)){
   ambiguousFields.push('initialGear');
  }
  if(comparable.some(candidate=>candidate.recommendation.firstTurn!==best.recommendation.firstTurn)){
   ambiguousFields.push('firstTurn');
  }
  if(comparable.some(candidate=>candidate.recommendation.firstTurnGear!==best.recommendation.firstTurnGear)){
   ambiguousFields.push('turnGear');
  }
  if(comparable.some(candidate=>candidate.recommendation.tractorYawDirection!==best.recommendation.tractorYawDirection)){
   ambiguousFields.push('tractorYawDirection');
  }
  if(comparable.some(candidate=>{
   const a=best.recommendation.firstTurnDistance,b=candidate.recommendation.firstTurnDistance;
   return a===null||b===null?a!==b:!Number.isFinite(a)||!Number.isFinite(b)||Math.abs(a-b)>firstTurnDistanceTolerance+1e-9;
  })){
   ambiguousFields.push('firstTurnDistance');
  }
  const ambiguous=ambiguousFields.length>0;
  const summarize=candidate=>({
   index:candidate.index,label:candidate.label,cost:candidate.cost,
   initialGear:candidate.recommendation.initialGear,
   firstTurn:candidate.recommendation.firstTurn,
   steeringWheelDirection:candidate.recommendation.steeringWheelDirection??candidate.recommendation.firstTurn,
   firstTurnGear:candidate.recommendation.firstTurnGear,
   tractorYawDirection:candidate.recommendation.tractorYawDirection,
   firstTurnDistance:candidate.recommendation.firstTurnDistance,
   firstTurnAfterGearChange:candidate.recommendation.firstTurnAfterGearChange
  });
  return {
   ok:true,
   status:ambiguous?'ambiguous':assessed.length===1?'single-candidate':'preferred',
   recommendation:ambiguous?null:best.recommendation,
   bestCandidate:{...summarize(best),recommendation:best.recommendation},
   comparableAlternatives:comparable.map(summarize),
   ambiguousFields,
   candidatesCompared:assessed.length,
   costTolerance
  };
 }

 function scoreCandidateRoute(route){
  const states=route.states;
  let pathLength=0,gearChanges=0;
  let previousGear=(states[1]?.gear??1)<0?-1:1;
  for(let i=1;i<states.length;i++){
   if(i>0){
    const previous=states[i-1],current=states[i];
    pathLength+=Math.hypot(current.x-previous.x,current.y-previous.y);
   }
   if(i>1){
    const gear=(states[i].gear??previousGear)<0?-1:1;
    if(gear!==previousGear)gearChanges++;
    previousGear=gear;
   }
  }
  const peak=articulationExposure(states);
  const clearance=route.minimumClearanceLowerBound;
  const clearanceMargin=Number.isFinite(clearance)?
   Math.max(0,clearance-route.requiredClearance):Infinity;
  const gearChangePenalty=gearChanges*0.75;
  const articulationPenalty=Math.max(0,peak.maxAngleDegrees-20)*0.02;
  const clearancePenalty=Number.isFinite(clearanceMargin)?
   0.20/Math.max(0.05,clearanceMargin+0.05):0;
  return {
   cost:pathLength+gearChangePenalty+articulationPenalty+clearancePenalty,
   pathLength,gearChanges,maxArticulationDegrees:peak.maxAngleDegrees,
   minimumClearanceLowerBound:Number.isFinite(clearance)?clearance:null,
   components:{pathLength,gearChangePenalty,articulationPenalty,clearancePenalty}
  };
 }
 function planAlternatives(options={}){
  const inputErrors=validatePlanOptions(options);
  if(inputErrors.length)return {ok:false,reason:'invalid-input',errors:inputErrors,expanded:0};
  if(options.firstActionConstraint!==undefined){
   return {ok:false,reason:'invalid-input',errors:['planAlternatives самостоятельно задаёт ограничения первых действий.'],expanded:0};
  }
  const perCandidateMaxExpansions=options.alternativeMaxExpansions??options.maxExpansions??24000;
  if(!Number.isInteger(perCandidateMaxExpansions)||perCandidateMaxExpansions<1||perCandidateMaxExpansions>500000){
   return {ok:false,reason:'invalid-input',errors:['Лимит расширений каждого альтернативного поиска должен быть целым числом от 1 до 500000.'],expanded:0};
  }
  const costTolerance=options.costTolerance??0.25;
  if(!Number.isFinite(costTolerance)||costTolerance<0){
   return {ok:false,reason:'invalid-input',errors:['Допуск сравнения стоимости должен быть конечным неотрицательным числом.'],expanded:0};
  }
  const constraints=[
   {initialGear:1,firstTurn:'left',label:'вперёд, руль влево'},
   {initialGear:1,firstTurn:'right',label:'вперёд, руль вправо'},
   {initialGear:-1,firstTurn:'left',label:'назад, руль влево'},
   {initialGear:-1,firstTurn:'right',label:'назад, руль вправо'}
  ];
  const baseOptions={...options};
  delete baseOptions.alternativeMaxExpansions;
  delete baseOptions.costTolerance;
  delete baseOptions.firstActionConstraint;
  const candidateCount=constraints.length+1;
  const candidates=[],failures=[];
  let expanded=0;
  function attempt(label,constraint,candidateIndex){
   const planOptions={
    ...baseOptions,
    maxExpansions:perCandidateMaxExpansions,
    onProgress:progress=>{
     if(typeof options.onProgress==='function'){
      try{
       options.onProgress({
        ...progress,
        candidateIndex,
        candidateCount,
        expanded:expanded+progress.expanded,
        maxExpansions:perCandidateMaxExpansions*candidateCount
       });
      }catch(_){}
     }
    }
   };
   if(constraint)planOptions.firstActionConstraint=constraint;
   const result=plan(planOptions);
   expanded+=result.expanded||0;
   if(!result.ok){
    failures.push({label,candidateIndex,reason:result.reason,expanded:result.expanded||0});
    return;
   }
   const recommendation=recommendStart(result.states);
   const score=scoreCandidateRoute(result);
   candidates.push({states:result.states,cost:score.cost,label,route:result,recommendation,score});
  }
  attempt('свободный поиск',null,1);
  for(let index=0;index<constraints.length;index++){
   const constraint=constraints[index];
   attempt(constraint.label,{initialGear:constraint.initialGear,firstTurn:constraint.firstTurn},index+2);
  }
  const incompleteActions=failures.filter(failure=>
   failure.candidateIndex>1&&failure.reason==='search-limit'
  );
  if(incompleteActions.length){
   return {
    ok:false,reason:'search-incomplete',expanded,
    candidatesFound:candidates.length,candidatesAttempted:candidateCount,
    candidateSummaries:candidates.map(candidate=>({
     label:candidate.label,cost:candidate.cost,
     initialGear:candidate.recommendation.initialGear,
     firstTurn:candidate.recommendation.firstTurn,
     steeringWheelDirection:candidate.recommendation.steeringWheelDirection??candidate.recommendation.firstTurn,
     firstTurnGear:candidate.recommendation.firstTurnGear,
     tractorYawDirection:candidate.recommendation.tractorYawDirection,
     pathLength:candidate.score.pathLength,
     gearChanges:candidate.score.gearChanges,
     maxArticulationDegrees:candidate.score.maxArticulationDegrees,
     minimumClearanceLowerBound:candidate.score.minimumClearanceLowerBound,
     scoreComponents:candidate.score.components
    })),
    failures,incompleteActions,recommendation:null,
    note:'Не все четыре ограниченных начальных действия были исследованы до исчерпания поискового фронтира; рекомендация заблокирована.'
   };
  }
  if(!candidates.length){
   return {
    ok:false,reason:'no-candidate-route',expanded,
    candidatesFound:0,candidatesAttempted:candidateCount,failures
   };
  }
  const comparison=compareStartCandidates(
   candidates.map(candidate=>({states:candidate.states,cost:candidate.cost,label:candidate.label})),
   {costTolerance,firstTurnDistanceTolerance:options.firstTurnDistanceTolerance}
  );
  if(!comparison.ok){
   return {ok:false,reason:'candidate-comparison-failed',expanded,candidatesFound:candidates.length,failures,comparison};
  }
  const candidateSummaries=candidates.map(candidate=>({
   label:candidate.label,cost:candidate.cost,
   initialGear:candidate.recommendation.initialGear,
   firstTurn:candidate.recommendation.firstTurn,
   steeringWheelDirection:candidate.recommendation.steeringWheelDirection??candidate.recommendation.firstTurn,
   firstTurnGear:candidate.recommendation.firstTurnGear,
   tractorYawDirection:candidate.recommendation.tractorYawDirection,
   pathLength:candidate.score.pathLength,
   gearChanges:candidate.score.gearChanges,
   maxArticulationDegrees:candidate.score.maxArticulationDegrees,
   minimumClearanceLowerBound:candidate.score.minimumClearanceLowerBound,
   scoreComponents:candidate.score.components
  }));
  if(comparison.status==='ambiguous'){
   return {
    ok:false,reason:'ambiguous-start',status:comparison.status,
    expanded,candidatesFound:candidates.length,candidatesAttempted:candidateCount,
    candidateSummaries,
    candidateRoutes:candidates.map((candidate,index)=>({
     index,label:candidate.label,cost:candidate.cost,
     states:candidate.states,
     metrics:candidate.score,
     trajectoryClearance:candidate.route.trajectoryClearance,
     recommendation:candidate.recommendation
    })),
    failures,comparison
   };
  }
  const selected=candidates[comparison.bestCandidate.index];
  return {
   ...selected.route,
   ok:true,status:comparison.status,
   recommendation:comparison.recommendation||selected.recommendation,
   recommendationComparison:comparison,
   selectedCandidate:comparison.bestCandidate.index,
   candidatesFound:candidates.length,candidatesAttempted:candidateCount,
   candidateSummaries,failures,
   expanded
  };
 }


 function recommendationAction(result){
  if(!result||!result.ok)return null;
  const recommendation=result.recommendation||
   result.recommendationComparison?.recommendation||
   result.recommendationComparison?.bestCandidate?.recommendation||
   result.bestCandidate?.recommendation;
  if(!recommendation||![-1,1].includes(recommendation.initialGear))return null;
  return {
   initialGear:recommendation.initialGear,
   firstTurn:recommendation.firstTurn??null,
   steeringWheelDirection:recommendation.steeringWheelDirection??recommendation.firstTurn??null,
   firstTurnGear:recommendation.firstTurnGear??null,
   tractorYawDirection:recommendation.tractorYawDirection??null,
   firstTurnAfterGearChange:recommendation.firstTurnAfterGearChange??null,
   firstTurnDistance:Number.isFinite(recommendation.firstTurnDistance)?recommendation.firstTurnDistance:null
  };
 }
 function compareRecommendationSensitivity(nominal,reserve,options={}){
  if(!nominal||!reserve)return {ok:false,reason:'invalid-scenarios'};
  const firstTurnDistanceTolerance=options.firstTurnDistanceTolerance===undefined?0.25:options.firstTurnDistanceTolerance;
  if(!Number.isFinite(firstTurnDistanceTolerance)||firstTurnDistanceTolerance<0){
   return {ok:false,reason:'invalid-first-turn-distance-tolerance'};
  }
  const nominalAction=recommendationAction(nominal);
  const reserveAction=recommendationAction(reserve);
  const clearanceEvidenceFor=result=>{
   const sampled=result.minimumClearanceDetail&&typeof result.minimumClearanceDetail==='object'?
    result.minimumClearanceDetail:null;
   const swept=result.sweptClearanceDetail&&typeof result.sweptClearanceDetail==='object'?
    result.sweptClearanceDetail:null;
   const sampledDistance=Number.isFinite(sampled?.distance)?sampled.distance:null;
   const lowerBound=Number.isFinite(result.minimumClearanceLowerBound)?
    result.minimumClearanceLowerBound:null;
   const obstacle=typeof swept?.referenceObstacle==='string'&&swept.referenceObstacle?
    swept.referenceObstacle:
    typeof sampled?.obstacle==='string'&&sampled.obstacle?sampled.obstacle:null;
   const vehiclePart=typeof swept?.referenceVehiclePart==='string'&&swept.referenceVehiclePart?
    swept.referenceVehiclePart:
    typeof sampled?.vehiclePart==='string'&&sampled.vehiclePart?sampled.vehiclePart:null;
   return {
    minimumClearanceLowerBound:lowerBound,
    limitingObstacle:obstacle,
    limitingVehiclePart:vehiclePart,
    sampled:sampled?{
     distance:sampledDistance,
     obstacle:typeof sampled.obstacle==='string'?sampled.obstacle:null,
     vehiclePart:typeof sampled.vehiclePart==='string'?sampled.vehiclePart:null,
     component:typeof sampled.component==='string'?sampled.component:null
    }:null,
    swept:swept?{
     kind:typeof swept.kind==='string'?swept.kind:null,
     lowerBound:Number.isFinite(swept.lowerBound)?swept.lowerBound:null,
     obstacle:typeof swept.referenceObstacle==='string'?swept.referenceObstacle:null,
     vehiclePart:typeof swept.referenceVehiclePart==='string'?swept.referenceVehiclePart:null
    }:null
   };
  };
  const nominalClearance=clearanceEvidenceFor(nominal);
  const reserveClearance=clearanceEvidenceFor(reserve);
  const clearanceEvidence={
   nominal:nominalClearance,
   reserve:reserveClearance,
   limitingObstacleChanged:nominalClearance.limitingObstacle!==reserveClearance.limitingObstacle,
   limitingVehiclePartChanged:nominalClearance.limitingVehiclePart!==reserveClearance.limitingVehiclePart
  };
  if(!nominalAction&&!reserveAction){
   return {
    ok:true,status:'ambiguous',stable:false,
    changedFields:['recommendation'],nominalAction:null,reserveAction:null,
    clearanceEvidence
   };
  }
  if(!nominalAction||!reserveAction){
   return {
    ok:true,status:'changed',stable:false,
    changedFields:['recommendation'],nominalAction,reserveAction,
    clearanceEvidence
   };
  }
  const fields=['initialGear','firstTurn','firstTurnGear','tractorYawDirection','firstTurnAfterGearChange'];
  const changedFields=fields.filter(field=>nominalAction[field]!==reserveAction[field]);
  const nominalDistance=nominalAction.firstTurnDistance,reserveDistance=reserveAction.firstTurnDistance;
  const distanceChanged=nominalDistance===null||reserveDistance===null?
   nominalDistance!==reserveDistance:
   !Number.isFinite(nominalDistance)||!Number.isFinite(reserveDistance)||
    Math.abs(nominalDistance-reserveDistance)>firstTurnDistanceTolerance+1e-9;
  if(distanceChanged)changedFields.push('firstTurnDistance');
  return {
   ok:true,status:changedFields.length?'changed':'stable',
   stable:changedFields.length===0,changedFields,nominalAction,reserveAction,
   firstTurnDistanceTolerance,clearanceEvidence
  };
 }
 function planAlternativesWithReserve(options={}){
  const inputErrors=validatePlanOptions(options);
  if(inputErrors.length)return {ok:false,reason:'invalid-input',errors:inputErrors,expanded:0};
  if(options.firstActionConstraint!==undefined){
   return {ok:false,reason:'invalid-input',errors:['Проверка резерва самостоятельно задаёт альтернативные первые действия.'],expanded:0};
  }
  const uncertainty=options.measurementUncertainty??0;
  if(!Number.isFinite(uncertainty)||uncertainty<0||uncertainty>5){
   return {ok:false,reason:'invalid-input',errors:['measurementUncertainty должен быть конечным числом от 0 до 5 м на каждую сторону.'],expanded:0};
  }
  const nominalRequiredClearance=options.minClearance??0.30;
  const reserveRequiredClearance=nominalRequiredClearance+2*uncertainty;
  if(reserveRequiredClearance>10){
   return {ok:false,reason:'invalid-input',errors:['Требуемый зазор вместе с резервом неопределённости превышает 10 м.'],expanded:0};
  }
  const perCandidateMaxExpansions=options.alternativeMaxExpansions??options.maxExpansions??24000;
  if(!Number.isInteger(perCandidateMaxExpansions)||perCandidateMaxExpansions<1||perCandidateMaxExpansions>500000){
   return {ok:false,reason:'invalid-input',errors:['Лимит расширений каждого альтернативного поиска должен быть целым числом от 1 до 500000.'],expanded:0};
  }
  const baseOptions={...options};
  delete baseOptions.measurementUncertainty;
  const originalProgress=options.onProgress;
  const scenarioCount=uncertainty>0?2:1;
  const totalBudget=perCandidateMaxExpansions*5*scenarioCount;
  function runScenario(scenario,minClearance,expandedOffset){
   return planAlternatives({
    ...baseOptions,
    minClearance,
    onProgress:typeof originalProgress==='function'?progress=>{
     try{
      originalProgress({
       ...progress,scenario,
       expanded:expandedOffset+progress.expanded,
       maxExpansions:totalBudget
      });
     }catch(_){}
    }:undefined
   });
  }
  const nominal=runScenario('nominal',nominalRequiredClearance,0);
  if(uncertainty===0){
   return {
    ...nominal,
    sensitivity:{
     status:'zero-reserve',measurementUncertainty:0,
     nominalRequiredClearance,reserveRequiredClearance:nominalRequiredClearance,
     note:'Дополнительный резерв неопределённости равен нулю; отдельный повторный поиск не выполнялся.'
    }
   };
  }
  if(!nominal.ok&&nominal.reason!=='ambiguous-start'){
   return {
    ...nominal,
    sensitivity:{
     status:nominal.reason==='search-incomplete'?'nominal-search-incomplete':'nominal-unavailable',measurementUncertainty:uncertainty,
     nominalRequiredClearance,reserveRequiredClearance,
     nominalReason:nominal.reason
    }
   };
  }
  const nominalExpanded=nominal.expanded||0;
  const reserve=runScenario('reserve',reserveRequiredClearance,nominalExpanded);
  const totalExpanded=nominalExpanded+(reserve.expanded||0);
  if(!reserve.ok&&reserve.reason==='search-incomplete'){
   return {
    ok:false,reason:'search-incomplete',expanded:totalExpanded,
    candidatesFound:reserve.candidatesFound??0,
    nominalCandidateSummaries:nominal.candidateSummaries||[],
    reserveCandidateSummaries:reserve.candidateSummaries||[],
    incompleteActions:reserve.incompleteActions||[],
    reserveFailure:{reason:reserve.reason,failures:reserve.failures||[]},
    requiredClearance:reserveRequiredClearance,
    sensitivity:{
     status:'reserve-search-incomplete',measurementUncertainty:uncertainty,
     nominalRequiredClearance,reserveRequiredClearance,
     nominalStatus:nominal.status,reserveStatus:reserve.reason
    }
   };
  }
  if(!reserve.ok){
   if(reserve.reason==='ambiguous-start'){
    return {
     ...reserve,expanded:totalExpanded,
     nominalCandidateSummaries:nominal.candidateSummaries||[],
     sensitivity:{
      status:'reserve-ambiguous',measurementUncertainty:uncertainty,
      nominalRequiredClearance,reserveRequiredClearance,
      nominalStatus:nominal.status,reserveStatus:reserve.status
     }
    };
   }
   return {
    ok:false,reason:'uncertainty-no-route',expanded:totalExpanded,
    candidatesFound:0,candidatesAttempted:reserve.candidatesAttempted||5,
    candidateSummaries:nominal.candidateSummaries||[],
    nominalCandidateSummaries:nominal.candidateSummaries||[],
    reserveFailure:{reason:reserve.reason,failures:reserve.failures||[]},
    requiredClearance:reserveRequiredClearance,
    sensitivity:{
     status:'no-route-under-reserve',measurementUncertainty:uncertainty,
     nominalRequiredClearance,reserveRequiredClearance,
     nominalStatus:nominal.status,reserveStatus:reserve.reason
    }
   };
  }
  const comparison=compareRecommendationSensitivity(nominal,reserve,options);
  if(comparison.status==='ambiguous'){
   return {
    ...reserve,ok:false,reason:'ambiguous-start',expanded:totalExpanded,
    nominalCandidateSummaries:nominal.candidateSummaries||[],
    sensitivity:{
     status:'ambiguous-across-scenarios',measurementUncertainty:uncertainty,
     nominalRequiredClearance,reserveRequiredClearance,
     nominalStatus:nominal.status,reserveStatus:reserve.status,
     comparison
    }
   };
  }
  if(comparison.status==='changed'){
   return {
    ok:false,reason:'uncertainty-sensitive-start',expanded:totalExpanded,
    candidatesFound:reserve.candidatesFound,candidatesAttempted:reserve.candidatesAttempted,
    candidateSummaries:reserve.candidateSummaries||[],
    nominalCandidateSummaries:nominal.candidateSummaries||[],
    reserveCandidateSummaries:reserve.candidateSummaries||[],
    conservativeCandidateRoutes:reserve.ok&&Array.isArray(reserve.states)&&reserve.states.length>=2?[{
     index:0,label:'Консервативный вариант с резервом погрешности',
     cost:Number(reserve.cost??reserve.score?.cost??0),
     states:reserve.states,
     metrics:{
      minimumClearanceLowerBound:reserve.minimumClearanceLowerBound,
      sweptClearanceDetail:reserve.sweptClearanceDetail
     },
     recommendation:reserve.recommendation
    }]:[],
    sensitivity:{
     status:'start-action-changed',measurementUncertainty:uncertainty,
     nominalRequiredClearance,reserveRequiredClearance,
     nominalStatus:nominal.status,reserveStatus:reserve.status,
     nominalCandidatesFound:nominal.candidatesFound,
     reserveCandidatesFound:reserve.candidatesFound,
     comparison
    }
   };
  }
  return {
   ...reserve,expanded:totalExpanded,
   sensitivity:{
    status:'stable-start-action',measurementUncertainty:uncertainty,
    nominalRequiredClearance,reserveRequiredClearance,
    nominalStatus:nominal.status,reserveStatus:reserve.status,
    nominalCandidatesFound:nominal.candidatesFound,
    reserveCandidatesFound:reserve.candidatesFound,
    candidateCoverageChanged:nominal.candidatesFound!==reserve.candidatesFound,
    comparison
   }
  };
 }


function planAlternativesWithGeometryUncertainty(options={}){
 const inputErrors=validatePlanOptions(options);
 if(inputErrors.length)return {ok:false,reason:'invalid-input',errors:inputErrors,expanded:0};
 const obstaclePositionUncertainty=options.obstaclePositionUncertainty??0;
 const obstacleSizeUncertainty=options.obstacleSizeUncertainty??0;
 const vehicleFootprintUncertainty=options.vehicleFootprintUncertainty??0;
 const bounds=[
  ['obstaclePositionUncertainty',obstaclePositionUncertainty,0,2],
  ['obstacleSizeUncertainty',obstacleSizeUncertainty,0,2],
  ['vehicleFootprintUncertainty',vehicleFootprintUncertainty,0,1]
 ];
 for(const [name,value,min,max] of bounds){
  if(!Number.isFinite(value)||value<min||value>max){
   return {ok:false,reason:'invalid-input',errors:[name+' должен быть конечным числом от '+min+' до '+max+' м.'],expanded:0};
  }
 }
 const baseFootprint=options.params.footprintUncertainty??0;
 const expandedFootprint=baseFootprint+vehicleFootprintUncertainty;
 if(expandedFootprint>1){
  return {ok:false,reason:'invalid-input',errors:['Суммарное расширение контура автопоезда не должно превышать 1 м на каждую внешнюю грань.'],expanded:0};
 }
 const gateGeometry=options.gateGeometry??null;
 const nominalizedObstacles=options.obstacles.map(obstacle=>{
  if(!Array.isArray(obstacle?.nominalPolygon)||!Array.isArray(obstacle?.polygon))return obstacle;
  return {
   ...obstacle,
   polygon:obstacle.nominalPolygon.map(point=>({x:point.x,y:point.y}))
  };
 });
 const perObstaclePoseEnvelopeCount=options.obstacles.filter(obstacle=>{
  if(!Array.isArray(obstacle?.nominalPolygon)||!Array.isArray(obstacle?.polygon))return false;
  if(obstacle.nominalPolygon.length!==obstacle.polygon.length)return true;
  return obstacle.nominalPolygon.some((point,index)=>
   Math.hypot(point.x-obstacle.polygon[index].x,point.y-obstacle.polygon[index].y)>1e-9
  );
 }).length;
 let obstaclesForNominal=nominalizedObstacles;
 let obstaclesForEnvelope=options.obstacles;
 let gateBoundsActive=false;
 let gateGeometryUncertainty=null;
 if(gateGeometry!==null){
  if(!gateGeometry||typeof gateGeometry!=='object'||Array.isArray(gateGeometry)){
   return {ok:false,reason:'invalid-input',errors:['gateGeometry должен быть объектом геометрии ворот.'],expanded:0};
  }
  try{
   const nominalGatePosts=createGateObstacles({
    ...gateGeometry,widthUncertainty:0,positionUncertainty:0,headingUncertaintyDeg:0
   });
   const boundedGatePosts=createGateObstacles(gateGeometry);
   const nominalStaticObstacles=nominalizedObstacles.filter(obstacle=>obstacle?.kind!=='gate-jamb');
   const boundedStaticObstacles=options.obstacles.filter(obstacle=>obstacle?.kind!=='gate-jamb');
   obstaclesForNominal=[...nominalStaticObstacles,...nominalGatePosts];
   obstaclesForEnvelope=[...boundedStaticObstacles,...boundedGatePosts];
   const widthUncertainty=gateGeometry.widthUncertainty??0;
   const positionUncertainty=gateGeometry.positionUncertainty??0;
   const headingUncertaintyDeg=gateGeometry.headingUncertaintyDeg??0;
   gateBoundsActive=widthUncertainty>0||positionUncertainty>0||headingUncertaintyDeg>0;
   gateGeometryUncertainty={
    enabled:true,
    nominalClearWidth:gateGeometry.clearWidth,
    conservativeClearWidth:gateGeometry.clearWidth-widthUncertainty,
    widthUncertainty,
    positionUncertaintyPerAxis:positionUncertainty,
    headingUncertaintyDeg,
    model:'joint bounded envelope: inward jamb shift for width plus radial per-jamb expansion for simultaneous centre X/Y and heading error'
   };
  }catch(error){
   return {ok:false,reason:'invalid-input',errors:[error&&error.message?error.message:'Некорректная геометрия ворот.'],expanded:0};
  }
 }
 const obstacleExpansionPerSide=obstaclePositionUncertainty+obstacleSizeUncertainty/2;
 const expandedObstacles=obstaclesForEnvelope.map(obstacle=>{
  if(Array.isArray(obstacle.polygon)){
   // A radial offset of sqrt(2) times the per-axis bound contains every
   // simultaneous X/Y translation within the configured rectangular error box.
   return {
    ...obstacle,
    polygon:inflateConvexPolygon(obstacle.polygon,Math.SQRT2*obstacleExpansionPerSide)
   };
  }
  return {
   ...obstacle,
   w:obstacle.w+2*obstacleExpansionPerSide,
   h:obstacle.h+2*obstacleExpansionPerSide
  };
 });
 const hasGeometryBounds=obstaclePositionUncertainty>0||
  obstacleSizeUncertainty>0||vehicleFootprintUncertainty>0||
  perObstaclePoseEnvelopeCount>0||gateBoundsActive;
 const baseOptions={...options,obstacles:obstaclesForNominal};
 delete baseOptions.obstaclePositionUncertainty;
 delete baseOptions.obstacleSizeUncertainty;
 delete baseOptions.vehicleFootprintUncertainty;
 delete baseOptions.gateGeometry;
 const originalProgress=options.onProgress;
 const perCandidateMaxExpansions=options.alternativeMaxExpansions??options.maxExpansions??24000;
 const innerScenarioCount=(options.measurementUncertainty??0)>0?2:1;
 const totalBudget=perCandidateMaxExpansions*5*innerScenarioCount*(hasGeometryBounds?2:1);
 function runScenario(geometryScenario,scenarioOptions,expandedOffset){
  return planAlternativesWithReserve({
   ...scenarioOptions,
   onProgress:typeof originalProgress==='function'?progress=>{
    try{
     originalProgress({
      ...progress,geometryScenario,
      expanded:expandedOffset+progress.expanded,
      maxExpansions:totalBudget
     });
    }catch(_){}
   }:undefined
  });
 }
 const nominal=runScenario('nominal-geometry',baseOptions,0);
 const geometryBounds={
  obstaclePositionUncertainty,obstacleSizeUncertainty,
  vehicleFootprintUncertainty,obstacleExpansionPerSide,
  footprintExpansionPerEdge:vehicleFootprintUncertainty,
  gateGeometryUncertainty,
  perObstaclePoseEnvelopeCount,
  obstacleModel:'camera obstacles with nominalPolygon use that contour in the nominal scenario and polygon in the bounded scenario; global bounds are added only to the bounded contour; axis-aligned rectangles expand per axis and convex polygons receive a conservative radial offset',
  vehicleModel:'all three rectangular body contours expanded outward on every edge',
  kinematicParameterUncertaintyIncluded:false
 };
 if(!nominal.ok){
  return {
   ...nominal,
   geometrySensitivity:{
    ...geometryBounds,
    status:nominal.reason==='ambiguous-start'?'nominal-ambiguous':
     nominal.reason==='search-incomplete'?'nominal-search-incomplete':'nominal-unavailable',
    envelopeEvaluated:false
   }
  };
 }
 if(!hasGeometryBounds){
  return {
   ...nominal,
   geometrySensitivity:{...geometryBounds,status:'zero-geometry-bounds'}
  };
 }
 const expandedParams={...options.params,footprintUncertainty:expandedFootprint};
 const parameterErrors=rootPhysics.validateVehicleParams(expandedParams);
 if(parameterErrors.length){
  return {ok:false,reason:'invalid-input',errors:parameterErrors,expanded:nominal.expanded||0};
 }
 const expandedOptions={
  ...baseOptions,
  params:expandedParams,
  obstacles:expandedObstacles
 };
 const robust=runScenario('expanded-geometry-envelope',expandedOptions,nominal.expanded||0);
 const totalExpanded=(nominal.expanded||0)+(robust.expanded||0);
 if(!robust.ok&&robust.reason==='search-incomplete'){
  return {
   ok:false,reason:'geometry-search-incomplete',expanded:totalExpanded,
   candidatesFound:robust.candidatesFound??0,
   nominalCandidateSummaries:nominal.candidateSummaries||[],
   envelopeCandidateSummaries:robust.candidateSummaries||[],
   incompleteActions:robust.incompleteActions||[],
   envelopeFailure:{reason:robust.reason,failures:robust.failures||[]},
   geometrySensitivity:{
    ...geometryBounds,status:'envelope-search-incomplete',
    nominalStatus:nominal.status,envelopeStatus:robust.reason,
    incompleteActions:robust.incompleteActions||[]
   }
  };
 }
 if(!robust.ok){
  if(robust.reason==='uncertainty-sensitive-start'){
   return {
    ok:false,reason:'geometry-sensitive-start',expanded:totalExpanded,
    nominalCandidateSummaries:nominal.candidateSummaries||[],
    envelopeCandidateSummaries:robust.reserveCandidateSummaries||robust.candidateSummaries||[],
    geometrySensitivity:{
     ...geometryBounds,status:'start-action-changed-under-clearance-reserve',
     nominalStatus:nominal.status,envelopeStatus:robust.reason,
     comparison:robust.sensitivity?.comparison||null,
     clearanceSensitivity:robust.sensitivity||null
    }
   };
  }
  return {
   ok:false,reason:'geometry-envelope-no-route',expanded:totalExpanded,
   nominalCandidateSummaries:nominal.candidateSummaries||[],
   envelopeFailure:{reason:robust.reason,failures:robust.failures||[]},
   requiredClearance:robust.requiredClearance??null,
   geometrySensitivity:{
    ...geometryBounds,status:robust.reason==='ambiguous-start'?'ambiguous-under-envelope':'no-route-under-envelope',
    nominalStatus:nominal.status,envelopeStatus:robust.reason,
    comparison:robust.sensitivity?.comparison||null,
    clearanceSensitivity:robust.sensitivity||null
   }
  };
 }
 const comparison=compareRecommendationSensitivity(nominal,robust,options);
 if(comparison.status!=='stable'){
  return {
   ok:false,reason:'geometry-sensitive-start',expanded:totalExpanded,
   nominalCandidateSummaries:nominal.candidateSummaries||[],
   envelopeCandidateSummaries:robust.candidateSummaries||[],
   geometrySensitivity:{
    ...geometryBounds,status:'start-action-changed',
    nominalStatus:nominal.status,envelopeStatus:robust.status,comparison
   }
  };
 }
 return {
  ...robust,expanded:totalExpanded,
  geometrySensitivity:{
   ...geometryBounds,status:'stable-within-bounded-envelope',
   nominalStatus:nominal.status,envelopeStatus:robust.status,
   nominalCandidatesFound:nominal.candidatesFound,
   envelopeCandidatesFound:robust.candidatesFound,
   candidateCoverageChanged:nominal.candidatesFound!==robust.candidatesFound,
   comparison
  }
 };
}


function parameterEndpointVectors(count){
 if(!Number.isInteger(count)||count<0||count>31){
  throw new Error('Число активных параметров должно быть целым числом от 0 до 31.');
 }
 if(count<=3){
  return Array.from({length:1<<count},(_,mask)=>
   Array.from({length:count},(_,index)=>(mask>>index)&1)
  );
 }
 // Distinct non-zero vectors over GF(2)^k are pairwise independent.
 // Use 8, 16, or 32 orthogonal rows for 4–7, 8–15, or 16–31 dimensions.
 const width=count<=7?3:count<=15?4:5;
 const rowCount=1<<width;
 const columnMasks=Array.from({length:rowCount-1},(_,index)=>index+1).slice(0,count);
 const rows=[];
 const parity=value=>{
  let result=0;
  for(let n=value;n;n>>=1)result^=n&1;
  return result;
 };
 for(let row=0;row<rowCount;row++){
  rows.push(columnMasks.map(mask=>parity(row&mask)));
 }
 // Add a joint all-high endpoint in addition to pairwise coverage.
 const allHigh=Array(count).fill(1);
 if(!rows.some(row=>row.every((value,index)=>value===allHigh[index])))rows.push(allHigh);
 return rows;
}

function planAlternativesWithVehicleParameterBounds(options={}){
 const inputErrors=validatePlanOptions(options);
 if(inputErrors.length)return {ok:false,reason:'invalid-input',errors:inputErrors,expanded:0};
 const suppliedBounds=options.vehicleParameterBounds??{};
 if(!suppliedBounds||typeof suppliedBounds!=='object'||Array.isArray(suppliedBounds)){
  return {ok:false,reason:'invalid-input',errors:['vehicleParameterBounds должен быть объектом диапазонов параметров.'],expanded:0};
 }
 const allowedBounds={
  wheelbase:[1,15],
  hitchOffset:[0,5],
  kingpinAxles:[0.5,30],
  trailerLength:[4,30],
  trailerWidth:[1.5,5],
  trailerAxleSpacing:[0.3,5],
  tractorSteeringTrack:[1,3.5],
  tractorFrontTrack:[1,3.5],
  tractorRearTrack:[1,3.5],
  trailerTrack:[1,4.5],
  tireLength:[0.2,1.5],
  tireWidth:[0.1,0.8],
  cabOffset:[0,6],
  cabFront:[0.5,3.5],
  cabRear:[0.3,2.5],
  cabWidth:[1.8,2.55],
  mirrorOutboard:[0,1],
  mirrorLongitudinal:[-2,8],
  mirrorLength:[0.2,2]
 };
 const unknownKeys=Object.keys(suppliedBounds).filter(key=>!Object.prototype.hasOwnProperty.call(allowedBounds,key));
 if(unknownKeys.length){
  return {ok:false,reason:'invalid-input',errors:['Неизвестные параметры неопределённости: '+unknownKeys.join(', ')+'.'],expanded:0};
 }
 const baseParams={...options.params};
 const activeDimensions=[];
 const boundsSummary={};
 for(const key of Object.keys(allowedBounds)){
  if(!Object.prototype.hasOwnProperty.call(suppliedBounds,key))continue;
  const bound=suppliedBounds[key],limits=allowedBounds[key],nominal=baseParams[key];
  if(!bound||typeof bound!=='object'||!Number.isFinite(bound.min)||!Number.isFinite(bound.max)||
     bound.min>bound.max||bound.min>nominal||bound.max<nominal||
     bound.min<limits[0]||bound.max>limits[1]){
   return {ok:false,reason:'invalid-input',errors:[key+': диапазон должен быть конечным, содержать номинальное значение и оставаться в пределах '+limits[0]+'–'+limits[1]+'.'],expanded:0};
  }
  boundsSummary[key]={min:bound.min,max:bound.max,nominal};
  if(bound.max-bound.min>1e-12)activeDimensions.push({key,min:bound.min,max:bound.max,nominal});
 }
 const baseOptions={...options};
 delete baseOptions.vehicleParameterBounds;
 const perCandidateMaxExpansions=options.alternativeMaxExpansions??options.maxExpansions??24000;
 if(!Number.isInteger(perCandidateMaxExpansions)||perCandidateMaxExpansions<1||perCandidateMaxExpansions>500000){
  return {ok:false,reason:'invalid-input',errors:['Лимит расширений каждого альтернативного поиска должен быть целым числом от 1 до 500000.'],expanded:0};
 }
 const reserveScenarioCount=(options.measurementUncertainty??0)>0?2:1;
 const hasGeometryBounds=(options.obstaclePositionUncertainty??0)>0||
  (options.obstacleSizeUncertainty??0)>0||
  (options.vehicleFootprintUncertainty??0)>0;
 const gateBounds=options.gateGeometry;
 const hasGateBounds=!!gateBounds&&[
  gateBounds.widthUncertainty??0,
  gateBounds.positionUncertainty??0,
  gateBounds.headingUncertaintyDeg??0
 ].some(value=>Number.isFinite(value)&&value>0);
 const geometryScenarioCount=hasGeometryBounds||hasGateBounds?2:1;
 if(activeDimensions.length>31){
  return {ok:false,reason:'invalid-input',errors:['Поддерживается не более 31 активного параметра неопределённости.'],expanded:0};
 }
 const endpointRows=parameterEndpointVectors(activeDimensions.length);
 const endpointScenarios=endpointRows.map((row,index)=>{
  const parameterValues={},endpointSides={},failedParameters=[];
  for(let dimensionIndex=0;dimensionIndex<activeDimensions.length;dimensionIndex++){
   const dimension=activeDimensions[dimensionIndex];
   const side=row[dimensionIndex]?'max':'min';
   const value=side==='max'?dimension.max:dimension.min;
   parameterValues[dimension.key]=value;
   endpointSides[dimension.key]=side;
   failedParameters.push({key:dimension.key,side,value});
  }
  return {
   name:'совместная комбинация '+String(index+1).padStart(2,'0')+'/'+endpointRows.length,
   kind:'endpoint-combination',parameterValues,endpointSides,failedParameters
  };
 });
 // Endpoint-only checks can miss an interior change in the planner's discrete
 // candidate ranking. Check the midpoint of each interval independently too.
 // The nominal scenario already covers a midpoint that equals the nominal value.
 const midpointScenarios=activeDimensions.flatMap(dimension=>{
  const value=(dimension.min+dimension.max)/2;
  if(Math.abs(value-dimension.nominal)<=1e-12)return [];
  return [{
   name:'середина диапазона '+dimension.key,
   kind:'one-factor-midpoint',
   parameterValues:{[dimension.key]:value},
   endpointSides:{[dimension.key]:'midpoint'},
   failedParameters:[{key:dimension.key,side:'midpoint',value}]
  }];
 });
 const scenarios=[...endpointScenarios,...midpointScenarios];
 const totalScenarioCount=scenarios.length+1;
 const totalBudget=perCandidateMaxExpansions*5*reserveScenarioCount*geometryScenarioCount*totalScenarioCount;
 let expandedOffset=0;
 function runScenario(name,params,index){
  const scenarioOptions={...baseOptions,params,alternativeMaxExpansions:perCandidateMaxExpansions};
  if(typeof options.onProgress==='function'){
   scenarioOptions.onProgress=progress=>{
    try{
     options.onProgress({
      ...progress,
      parameterScenario:name,
      parameterScenarioIndex:index,
      parameterScenarioCount:totalScenarioCount,
      expanded:expandedOffset+(progress.expanded||0),
      maxExpansions:totalBudget
     });
    }catch(_){}
   };
  }
  const result=planAlternativesWithGeometryUncertainty(scenarioOptions);
  expandedOffset+=result.expanded||0;
  return result;
 }
 const nominal=runScenario('номинальные размеры',baseParams,1);
 if(!activeDimensions.length){
  return {
   ...nominal,expanded:expandedOffset,
   parameterSensitivity:{
    status:'not-varied',coverage:'none',interiorCoverage:'none',
    suppliedBounds:boundsSummary,testedScenarioCount:0,midpointScenarioCount:0,
    testedMidpointScenarioCount:0,activeParameterCount:0,
    kinematicParameterUncertaintyIncluded:false,
    allParameterCombinationsEvaluated:false,pairwiseEndpointCoverage:false,
    note:'Все заданные диапазоны имеют нулевую ширину или не заданы; неопределённость размеров автопоезда не проверялась.'
   }
  };
 }
 const coverage=activeDimensions.length<=3?
  'full-endpoint-combinations':'pairwise-endpoint-covering-array';
 const allParameterCombinationsEvaluated=activeDimensions.length<=3;
 const pairwiseEndpointCoverage=activeDimensions.length>=2;
 const interiorCoverage='one-factor-midpoints';
 const coverageNote=(allParameterCombinationsEvaluated?
  'Проверены все сочетания нижних и верхних границ активных параметров.':
  (activeDimensions.length<=7?
   'Для 4–7 активных параметров применён 8-строчный парный covering array на GF(2)^3 плюс совместный верхний угол; каждая пара проверена во всех четырёх сочетаниях min/max.':
   (activeDimensions.length<=15?
    'Для 8–15 активных параметров применён 16-строчный парный covering array на GF(2)^4 плюс совместный верхний угол; каждая пара проверена во всех четырёх сочетаниях min/max.':
    'Для 16–31 активного параметра применён 32-строчный парный covering array на GF(2)^5 плюс совместный верхний угол; каждая пара проверена во всех четырёх сочетаниях min/max.'))+
   ' Полные многопараметрические комбинации не перебирались.')+
  ' Дополнительно проверяется середина каждого активного диапазона по одному параметру за раз; совместные промежуточные отклонения и непрерывный интервал не проверяются.';
 const testedMidpoints=()=>completed.filter(item=>item.kind==='one-factor-midpoint').length;
 if(!nominal.ok){
  return {
   ...nominal,expanded:expandedOffset,
   parameterSensitivity:{
    status:'nominal-unavailable',coverage,interiorCoverage,
    suppliedBounds:boundsSummary,activeParameterCount:activeDimensions.length,
    scenarioCount:scenarios.length,testedScenarioCount:0,
    midpointScenarioCount:midpointScenarios.length,testedMidpointScenarioCount:0,
    allParameterCombinationsEvaluated,pairwiseEndpointCoverage,
    nominalReason:nominal.reason,note:coverageNote
   }
  };
 }
 const nominalAction=recommendationAction(nominal);
 const completed=[];
 for(let index=0;index<scenarios.length;index++){
  const scenario=scenarios[index];
  const scenarioParams={...baseParams,...scenario.parameterValues};
  const {name:scenarioName,kind:scenarioKind,parameterValues,endpointSides,failedParameters}=scenario;
  const geometryErrors=rootPhysics.validateVehicleParams(scenarioParams);
  if(geometryErrors.length){
   return {
    ok:false,reason:'parameter-scenario-unavailable',expanded:expandedOffset,candidatesFound:0,
    parameterSensitivity:{
     status:'joint-scenario-invalid',coverage,interiorCoverage,
     suppliedBounds:boundsSummary,activeParameterCount:activeDimensions.length,
     scenarioCount:scenarios.length,testedScenarioCount:completed.length,
     midpointScenarioCount:midpointScenarios.length,testedMidpointScenarioCount:testedMidpoints(),
     completedScenarios:completed,failedScenario:scenarioName,failedParameters,
     parameterValues,endpointSides,failureReason:'invalid-vehicle-geometry',
     validationErrors:geometryErrors,nominalAction,allParameterCombinationsEvaluated,
     pairwiseEndpointCoverage,note:coverageNote
    }
   };
  }
  const result=runScenario(scenarioName,scenarioParams,index+2);
  if(!result.ok){
   return {
    ok:false,reason:'parameter-scenario-unavailable',expanded:expandedOffset,
    candidatesFound:result.candidatesFound??0,
    parameterSensitivity:{
     status:'joint-scenario-unavailable',coverage,interiorCoverage,
     suppliedBounds:boundsSummary,activeParameterCount:activeDimensions.length,
     scenarioCount:scenarios.length,testedScenarioCount:completed.length,
     midpointScenarioCount:midpointScenarios.length,testedMidpointScenarioCount:testedMidpoints(),
     completedScenarios:completed,failedScenario:scenarioName,failedParameters,
     parameterValues,endpointSides,failureReason:result.reason,nominalAction,
     allParameterCombinationsEvaluated,pairwiseEndpointCoverage,note:coverageNote
    }
   };
  }
  const comparison=compareRecommendationSensitivity(nominal,result);
  const summary={
   name:scenarioName,kind:scenarioKind,parameterValues,endpointSides,
   status:comparison.status,changedFields:comparison.changedFields||[],
   recommendation:recommendationAction(result),
   candidatesFound:result.candidatesFound??0,
   geometryStatus:result.geometrySensitivity?.status??'not-reported'
  };
  completed.push(summary);
  if(comparison.status!=='stable'){
   return {
    ok:false,reason:'parameter-sensitive-start',expanded:expandedOffset,
    candidatesFound:result.candidatesFound??0,
    parameterSensitivity:{
     status:'start-action-changed',coverage,interiorCoverage,
     suppliedBounds:boundsSummary,activeParameterCount:activeDimensions.length,
     scenarioCount:scenarios.length,testedScenarioCount:completed.length,
     midpointScenarioCount:midpointScenarios.length,testedMidpointScenarioCount:testedMidpoints(),
     completedScenarios:completed,failedScenario:scenarioName,failedParameters,
     parameterValues,endpointSides,nominalAction,scenarioAction:recommendationAction(result),
     changedFields:comparison.changedFields||[],
     allParameterCombinationsEvaluated,pairwiseEndpointCoverage,note:coverageNote
    }
   };
  }
 }
 return {
  ...nominal,expanded:expandedOffset,
  parameterSensitivity:{
   status:allParameterCombinationsEvaluated?
    'stable-across-joint-endpoint-combinations':
    'stable-across-pairwise-endpoint-scenarios',
   coverage,interiorCoverage,suppliedBounds,
   activeParameterCount:activeDimensions.length,
   scenarioCount:scenarios.length,testedScenarioCount:completed.length,
   midpointScenarioCount:midpointScenarios.length,testedMidpointScenarioCount:testedMidpoints(),
   completedScenarios:completed,nominalAction,
   allParameterCombinationsEvaluated,pairwiseEndpointCoverage,
   kinematicParameterUncertaintyIncluded:true,
   note:coverageNote
  }
 };
}

return Object.freeze({plan,planAlternatives,planAlternativesWithReserve,planAlternativesWithGeometryUncertainty,planAlternativesWithVehicleParameterBounds,parameterEndpointVectors,compareRecommendationSensitivity,clearance,createClearanceChecker,sweptDisplacementPerMeter,sweptDisplacementRatesPerMeter,analyzeTrajectoryClearance,analyzeWheelTrajectoryClearance,polygons,goalErrors,goalReached,articulationExposure,recommendStart,compareStartCandidates,validatePlanOptions,validateObstacles,createGateObstacles,inflateConvexPolygon,inflateObstaclePoseUncertainty});
});
