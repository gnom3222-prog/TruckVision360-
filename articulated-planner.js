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
 function obstaclePoly(o){return[
  {x:o.x-o.w/2,y:o.y-o.h/2},{x:o.x+o.w/2,y:o.y-o.h/2},
  {x:o.x+o.w/2,y:o.y+o.h/2},{x:o.x-o.w/2,y:o.y+o.h/2}
 ];}
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
   if(!Number.isFinite(o.x)||!Number.isFinite(o.y)||
      !Number.isFinite(o.w)||!Number.isFinite(o.h)||o.w<=0||o.h<=0){
    errors.push('Препятствие '+(index+1)+' должно иметь конечные координаты и положительные ширину/высоту.');
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
  const obstacleErrors=validateObstacles(obstacles);
  if(obstacleErrors.length)throw new Error('Некорректные препятствия: '+obstacleErrors.join(' '));
  const prepared=obstacles.map(o=>{
   const poly=obstaclePoly(o);
   return {poly,bounds:polygonBounds(poly),name:o.name||'препятствие'};
  });
  const bodyNames=['тягач','кабина','полуприцеп'];
  return function(q){
   const bodies=polygons(q,p),bodyBounds=bodies.map(polygonBounds);
   let min=Infinity,hit=null,vehiclePart=null,bodyIndex=null;
   for(const obstacle of prepared){
    for(let i=0;i<bodies.length;i++){
     if(aabbDistance(bodyBounds[i],obstacle.bounds)>=min)continue;
     const d=polyDist(bodies[i],obstacle.poly);
     if(d<min){
      min=d;
      hit=obstacle.name;
      vehiclePart=bodyNames[i]||('часть '+(i+1));
      bodyIndex=i;
     }
    }
   }
   return {distance:min,hit,obstacle:hit,vehiclePart,bodyIndex};
  };
 }
 function sweptDisplacementPerMeter(steer,p,q=null,maxDistance=0.05){
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
  const tractorRadius=Math.hypot(5+footprintUncertainty,(2.55+2*footprintUncertainty)/2);
  const cabRadiusFromRearAxle=2.1+Math.hypot(1.35+footprintUncertainty,(2.25+2*footprintUncertainty)/2);
  // vehiclePolygons() places the trailer body relative to the kingpin, while
  // the articulated yaw reference is the equivalent axle-group center.
  // Include the same outward footprint envelope used by the clearance checker.
  const trailerFrontFromAxle=kingpinAxles+1.5+footprintUncertainty;
  const trailerRearFromAxle=kingpinAxles-(trailerLength-1.5+footprintUncertainty);
  if(q&&(!Number.isFinite(q.psi)||!Number.isFinite(q.psiT))){
   throw new Error('Articulation state is invalid for swept-clearance checking.');
  }
  // For fixed steering, each trailer corner's velocity in trailer coordinates
  // is a 2x2 linear map of [cos(beta), sin(beta)]. Its spectral norm is the
  // exact maximum speed over every articulation angle. Evaluate all four
  // corners and both gear signs to cover forward and reverse motion.
  const hitchYawCoupling=hitchOffset*tractorYawPerMeter;
  const trailerWidthWithUncertainty=trailerWidth+2*footprintUncertainty;
  const trailerLongitudinalOffsets=[trailerFrontFromAxle,trailerRearFromAxle];
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
     const largestEigenvalue=0.5*(
      column1+column2+Math.hypot(column1-column2,2*cross)
     );
     trailerPointRate=Math.max(trailerPointRate,Math.sqrt(Math.max(0,largestEigenvalue)));
    }
   }
  }
  return Math.max(
   1+tractorRadius*tractorYawPerMeter,
   1+cabRadiusFromRearAxle*tractorYawPerMeter,
   trailerPointRate
  );
 }
 function clearance(q,obstacles,p){return createClearanceChecker(obstacles,p)(q);}
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
  const startCheck=checkClearance(start);
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
     const c=checkClearance(next);
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
     samples.push({x:q.x,y:q.y,psi:q.psi,psiT:q.psiT,gear,steer});
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
   return {ok:false,reason:'no-path',expanded,
    closest:{x:closest.q.x,y:closest.q.y,psi:closest.q.psi,psiT:closest.q.psiT},
    goalDistance:errors.position,headingError:errors.headingError,articulationAngle:errors.articulation};
  }
  const chain=[];for(let n=finish;n&&n.parent;n=n.parent)chain.push(n);
  chain.reverse();
  const states=[{x:start.x,y:start.y,psi:start.psi,psiT:start.psiT,gear:1,steer:0}];
  for(const n of chain)states.push(...n.samples);
  let pathMinimum=startClearance;
  let minimumClearanceDetail=startDetail;
  for(const node of chain){
   if(node.minClearance<pathMinimum){
    pathMinimum=node.minClearance;
    minimumClearanceDetail=node.minClearanceDetail;
   }
  }
  return {
   ok:true,states,expanded,length:finish.g,
   minimumClearance:pathMinimum,
   minimumClearanceDetail,
   minimumClearanceLowerBound:finish.sweptLowerBound,
   sweptClearanceDetail:finish.sweptLowerBoundDetail,
   requiredClearance,
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
  const ambiguous=ambiguousFields.length>0;
  const summarize=candidate=>({
   index:candidate.index,label:candidate.label,cost:candidate.cost,
   initialGear:candidate.recommendation.initialGear,
   firstTurn:candidate.recommendation.firstTurn,
   firstTurnGear:candidate.recommendation.firstTurnGear,
   tractorYawDirection:candidate.recommendation.tractorYawDirection
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
   {initialGear:1,firstTurn:'left',label:'вперёд, первый поворот влево'},
   {initialGear:1,firstTurn:'right',label:'вперёд, первый поворот вправо'},
   {initialGear:-1,firstTurn:'left',label:'назад, первый поворот влево'},
   {initialGear:-1,firstTurn:'right',label:'назад, первый поворот вправо'}
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
    failures.push({label,reason:result.reason,expanded:result.expanded||0});
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
  if(!candidates.length){
   return {
    ok:false,reason:'no-candidate-route',expanded,
    candidatesFound:0,candidatesAttempted:candidateCount,failures
   };
  }
  const comparison=compareStartCandidates(
   candidates.map(candidate=>({states:candidate.states,cost:candidate.cost,label:candidate.label})),
   {costTolerance}
  );
  if(!comparison.ok){
   return {ok:false,reason:'candidate-comparison-failed',expanded,candidatesFound:candidates.length,failures,comparison};
  }
  const candidateSummaries=candidates.map(candidate=>({
   label:candidate.label,cost:candidate.cost,
   initialGear:candidate.recommendation.initialGear,
   firstTurn:candidate.recommendation.firstTurn,
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
    candidateSummaries,failures,comparison
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
   firstTurnGear:recommendation.firstTurnGear??null,
   tractorYawDirection:recommendation.tractorYawDirection??null
  };
 }
 function compareRecommendationSensitivity(nominal,reserve){
  if(!nominal||!reserve)return {ok:false,reason:'invalid-scenarios'};
  const nominalAction=recommendationAction(nominal);
  const reserveAction=recommendationAction(reserve);
  if(!nominalAction&&!reserveAction){
   return {
    ok:true,status:'ambiguous',stable:false,
    changedFields:['recommendation'],nominalAction:null,reserveAction:null
   };
  }
  if(!nominalAction||!reserveAction){
   return {
    ok:true,status:'changed',stable:false,
    changedFields:['recommendation'],nominalAction,reserveAction
   };
  }
  const fields=['initialGear','firstTurn','firstTurnGear','tractorYawDirection'];
  const changedFields=fields.filter(field=>nominalAction[field]!==reserveAction[field]);
  return {
   ok:true,status:changedFields.length?'changed':'stable',
   stable:changedFields.length===0,changedFields,nominalAction,reserveAction
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
     status:'nominal-unavailable',measurementUncertainty:uncertainty,
     nominalRequiredClearance,reserveRequiredClearance,
     nominalReason:nominal.reason
    }
   };
  }
  const nominalExpanded=nominal.expanded||0;
  const reserve=runScenario('reserve',reserveRequiredClearance,nominalExpanded);
  const totalExpanded=nominalExpanded+(reserve.expanded||0);
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
  const comparison=compareRecommendationSensitivity(nominal,reserve);
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
 const obstacleExpansionPerSide=obstaclePositionUncertainty+obstacleSizeUncertainty/2;
 const expandedObstacles=options.obstacles.map(obstacle=>({
  ...obstacle,
  w:obstacle.w+2*obstacleExpansionPerSide,
  h:obstacle.h+2*obstacleExpansionPerSide
 }));
 const baseOptions={...options};
 delete baseOptions.obstaclePositionUncertainty;
 delete baseOptions.obstacleSizeUncertainty;
 delete baseOptions.vehicleFootprintUncertainty;
 const originalProgress=options.onProgress;
 const perCandidateMaxExpansions=options.alternativeMaxExpansions??options.maxExpansions??24000;
 const innerScenarioCount=(options.measurementUncertainty??0)>0?2:1;
 const totalBudget=perCandidateMaxExpansions*5*innerScenarioCount*2;
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
  obstacleModel:'axis-aligned rectangles expanded by bounded center and full-size tolerances',
  vehicleModel:'all three rectangular body contours expanded outward on every edge',
  kinematicParameterUncertaintyIncluded:false
 };
 if(!nominal.ok){
  return {
   ...nominal,
   geometrySensitivity:{...geometryBounds,status:'nominal-unavailable'}
  };
 }
 if(obstaclePositionUncertainty===0&&obstacleSizeUncertainty===0&&vehicleFootprintUncertainty===0){
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
 if(!robust.ok){
  return {
   ok:false,reason:'geometry-envelope-no-route',expanded:totalExpanded,
   nominalCandidateSummaries:nominal.candidateSummaries||[],
   envelopeFailure:{reason:robust.reason,failures:robust.failures||[]},
   requiredClearance:robust.requiredClearance??null,
   geometrySensitivity:{
    ...geometryBounds,status:robust.reason==='ambiguous-start'?'ambiguous-under-envelope':'no-route-under-envelope',
    nominalStatus:nominal.status,envelopeStatus:robust.reason
   }
  };
 }
 const comparison=compareRecommendationSensitivity(nominal,robust);
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

return Object.freeze({plan,planAlternatives,planAlternativesWithReserve,planAlternativesWithGeometryUncertainty,compareRecommendationSensitivity,clearance,createClearanceChecker,sweptDisplacementPerMeter,polygons,goalErrors,goalReached,articulationExposure,recommendStart,compareStartCandidates,validatePlanOptions});
});
