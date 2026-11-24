(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionCameraGeometry=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';

 const dist=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
 const MAX_CONDITION_ESTIMATE=1e8;
 const CONDITION_WARNING_ESTIMATE=1e5;

 function validQuad(q,minArea=1000,minEdge=20){
  if(!Array.isArray(q)||q.length!==4||q.some(p=>!p||!Number.isFinite(p.x)||!Number.isFinite(p.y)))return false;
  let sign=0,area=0;
  for(let i=0;i<4;i++){
   const a=q[i],b=q[(i+1)%4],c=q[(i+2)%4];
   const cross=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);
   if(Math.abs(cross)<1e-4)return false;
   const current=Math.sign(cross);
   if(sign&&current!==sign)return false;
   sign=current;
   area+=a.x*b.y-b.x*a.y;
   if(dist(a,b)<=minEdge)return false;
  }
  return Math.abs(area)/2>minArea;
 }

 function solveLinear(A,b){
  if(!Array.isArray(A)||!Array.isArray(b)||A.length!==b.length||A.length===0||
     A.some(r=>!Array.isArray(r)||r.length!==A.length||r.some(v=>!Number.isFinite(v)))||
     b.some(v=>!Number.isFinite(v)))throw new Error('Недостаточно корректных данных для калибровки.');
  const n=A.length,m=A.map((r,i)=>r.concat(b[i]));
  const scale=Math.max(1,...A.flatMap(r=>r.map(Math.abs)));
  for(let j=0;j<n;j++){
   let p=j;
   for(let r=j+1;r<n;r++)if(Math.abs(m[r][j])>Math.abs(m[p][j]))p=r;
   if(Math.abs(m[p][j])<scale*1e-12)throw new Error('Калибровочные точки расположены неправильно или почти на одной линии.');
   [m[j],m[p]]=[m[p],m[j]];
   const z=m[j][j];
   for(let k=j;k<=n;k++)m[j][k]/=z;
   for(let r=0;r<n;r++)if(r!==j){
    const factor=m[r][j];
    for(let k=j;k<=n;k++)m[r][k]-=factor*m[j][k];
   }
  }
  return m.map(r=>r[n]);
 }

 function matrixNormInf(A){
  return Math.max(...A.map(row=>row.reduce((sum,value)=>sum+Math.abs(value),0)));
 }

 function conditionEstimateInf(A){
  const n=A.length,inverse=Array.from({length:n},()=>Array(n).fill(0));
  for(let column=0;column<n;column++){
   const unit=Array(n).fill(0);
   unit[column]=1;
   const solution=solveLinear(A,unit);
   for(let row=0;row<n;row++)inverse[row][column]=solution[row];
  }
  return matrixNormInf(A)*matrixNormInf(inverse);
 }

 function normalizePoints(points){
  const center={
   x:points.reduce((sum,p)=>sum+p.x,0)/points.length,
   y:points.reduce((sum,p)=>sum+p.y,0)/points.length
  };
  const meanDistance=points.reduce((sum,p)=>sum+dist(p,center),0)/points.length;
  if(!Number.isFinite(meanDistance)||meanDistance<1e-12)throw new Error('Калибровочные точки расположены слишком близко друг к другу.');
  const scale=Math.SQRT2/meanDistance;
  const transform=[
   [scale,0,-scale*center.x],
   [0,scale,-scale*center.y],
   [0,0,1]
  ];
  const inverse=[
   [1/scale,0,center.x],
   [0,1/scale,center.y],
   [0,0,1]
  ];
  return {
   points:points.map(p=>({x:scale*(p.x-center.x),y:scale*(p.y-center.y)})),
   transform,
   inverse
  };
 }

 function multiply3(a,b){
  return Array.from({length:3},(_,row)=>
   Array.from({length:3},(_,column)=>
    a[row][0]*b[0][column]+a[row][1]*b[1][column]+a[row][2]*b[2][column]
   )
  );
 }

 function solve(A,b){
  return solveLinear(A,b);
 }

 function homography(src,width,height){
  if(!Array.isArray(src)||src.length!==4||!Number.isFinite(width)||!Number.isFinite(height)||width<=0||height<=0)throw new Error('Размеры эталона или калибровочные точки некорректны.');
  if(!validQuad(src,0,0))throw new Error('Калибровочные точки должны образовывать выпуклый четырёхугольник.');
  const dst=[{x:0,y:0},{x:width,y:0},{x:width,y:height},{x:0,y:height}];
  const normalizedSource=normalizePoints(src);
  const normalizedDestination=normalizePoints(dst);
  const A=[],b=[];
  for(let i=0;i<4;i++){
   const {x,y}=normalizedSource.points[i],{x:X,y:Y}=normalizedDestination.points[i];
   A.push([x,y,1,0,0,0,-X*x,-X*y]);b.push(X);
   A.push([0,0,0,x,y,1,-Y*x,-Y*y]);b.push(Y);
  }
  const conditionEstimate=conditionEstimateInf(A);
  if(!Number.isFinite(conditionEstimate)||conditionEstimate>MAX_CONDITION_ESTIMATE){
   throw new Error('Калибровочная геометрия плохо обусловлена. Увеличьте площадь эталона в кадре, избегайте почти прямого ракурса и повторите калибровку.');
  }
  const values=solveLinear(A,b);
  const normalizedTransform=[
   [values[0],values[1],values[2]],
   [values[3],values[4],values[5]],
   [values[6],values[7],1]
  ];
  const denormalized=multiply3(
   normalizedDestination.inverse,
   multiply3(normalizedTransform,normalizedSource.transform)
  );
  const divisor=denormalized[2][2];
  const matrixScale=Math.max(...denormalized.flat().map(Math.abs));
  if(!Number.isFinite(divisor)||Math.abs(divisor)<=matrixScale*1e-12){
   throw new Error('Перспективное преобразование нельзя устойчиво нормализовать. Измените положение эталона и повторите калибровку.');
  }
  const h=denormalized.flat().map(value=>value/divisor);
  const result=[h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7]];
  Object.defineProperty(result,'conditionEstimate',{value:conditionEstimate,enumerable:false});
  return result;
 }

 function map(point,h){
  if(!point||!Number.isFinite(point.x)||!Number.isFinite(point.y)||!Array.isArray(h)||h.length!==8||h.some(v=>!Number.isFinite(v)))throw new Error('Точка или параметры перспективной коррекции некорректны.');
  const [a,b,c,d,e,f,g,k]=h,z=g*point.x+k*point.y+1;
  if(Math.abs(z)<1e-9)throw new Error('Невозможно исправить перспективу.');
  const result={x:(a*point.x+b*point.y+c)/z,y:(d*point.x+e*point.y+f)/z};
  if(!Number.isFinite(result.x)||!Number.isFinite(result.y))throw new Error('Перспективная коррекция дала некорректный результат.');
  return result;
 }

 function measureQuad(referencePoints,referenceWidth,referenceHeight,measuredPoints){
  if(!validQuad(referencePoints,0,0)||!validQuad(measuredPoints,0,0))throw new Error('Четыре угла эталона и проёма должны образовывать правильные четырёхугольники.');
  const transform=homography(referencePoints,referenceWidth,referenceHeight);
  // The projective denominator is affine in image coordinates. If its sign
  // changes across a convex measured quad, the quad crosses the homography
  // horizon and its mapped dimensions are not physically meaningful.
  const g=transform[6],k=transform[7];
  const denominators=measuredPoints.map(({x,y})=>g*x+k*y+1);
  const denominatorScale=Math.max(1,...measuredPoints.map(({x,y})=>Math.abs(g*x)+Math.abs(k*y)+1));
  const denominatorTolerance=denominatorScale*1e-9;
  if(denominators.some(value=>!Number.isFinite(value)||Math.abs(value)<=denominatorTolerance)||
     denominators.some(value=>Math.sign(value)!==Math.sign(denominators[0]))){
   throw new Error('Контур измерения пересекает линию вырождения перспективного преобразования. Проверьте порядок углов и повторите калибровку с эталоном рядом с проёмом и в той же плоскости.');
  }
  const points=measuredPoints.map(point=>map(point,transform));
  const width=(dist(points[0],points[1])+dist(points[3],points[2]))/2;
  const height=(dist(points[0],points[3])+dist(points[1],points[2]))/2;
  if(!Number.isFinite(width)||!Number.isFinite(height)||width<=0||height<=0)throw new Error('Не удалось рассчитать размеры проёма.');
  return {
   width,
   height,
   points,
   conditionEstimate:transform.conditionEstimate,
   conditioningWarning:transform.conditionEstimate>CONDITION_WARNING_ESTIMATE
  };
 }

 // First-order uncertainty propagation. sigmaPixels is the assumed independent
 // one-standard-deviation error for each manually selected x/y coordinate.
 // This is sensitivity to point selection, not total camera measurement accuracy.
 function estimateUncertainty(options){
  const {
   referencePoints,referenceWidth,referenceHeight,measuredPoints,
   sigmaPixels=2,sigmaReferenceWidth=0,sigmaReferenceHeight=0
  }=options||{};
  if(!Number.isFinite(sigmaPixels)||sigmaPixels<0||sigmaPixels>20||
     !Number.isFinite(sigmaReferenceWidth)||sigmaReferenceWidth<0||
     !Number.isFinite(sigmaReferenceHeight)||sigmaReferenceHeight<0){
   throw new Error('Параметры оценки неопределённости должны быть неотрицательными числами.');
  }
  const base=measureQuad(referencePoints,referenceWidth,referenceHeight,measuredPoints);
  let varianceWidth=0,varianceHeight=0;
  const add=(plus,minus,epsilon,sigma)=>{
   const dw=(plus.width-minus.width)/(2*epsilon);
   const dh=(plus.height-minus.height)/(2*epsilon);
   varianceWidth+=(dw*sigma)**2;
   varianceHeight+=(dh*sigma)**2;
  };
  const pixelStep=0.25;
  if(sigmaPixels>0){
   for(let i=0;i<4;i++){
    for(const axis of ['x','y']){
     const refPlus=referencePoints.map(p=>({...p}));
     const refMinus=referencePoints.map(p=>({...p}));
     refPlus[i][axis]+=pixelStep;
     refMinus[i][axis]-=pixelStep;
     add(
      measureQuad(refPlus,referenceWidth,referenceHeight,measuredPoints),
      measureQuad(refMinus,referenceWidth,referenceHeight,measuredPoints),
      pixelStep,sigmaPixels
     );
     const gatePlus=measuredPoints.map(p=>({...p}));
     const gateMinus=measuredPoints.map(p=>({...p}));
     gatePlus[i][axis]+=pixelStep;
     gateMinus[i][axis]-=pixelStep;
     add(
      measureQuad(referencePoints,referenceWidth,referenceHeight,gatePlus),
      measureQuad(referencePoints,referenceWidth,referenceHeight,gateMinus),
      pixelStep,sigmaPixels
     );
    }
   }
  }
  if(sigmaReferenceWidth>0){
   const epsilon=Math.min(Math.max(referenceWidth*1e-5,1e-6),referenceWidth*0.25);
   add(
    measureQuad(referencePoints,referenceWidth+epsilon,referenceHeight,measuredPoints),
    measureQuad(referencePoints,referenceWidth-epsilon,referenceHeight,measuredPoints),
    epsilon,sigmaReferenceWidth
   );
  }
  if(sigmaReferenceHeight>0){
   const epsilon=Math.min(Math.max(referenceHeight*1e-5,1e-6),referenceHeight*0.25);
   add(
    measureQuad(referencePoints,referenceWidth,referenceHeight+epsilon,measuredPoints),
    measureQuad(referencePoints,referenceWidth,referenceHeight-epsilon,measuredPoints),
    epsilon,sigmaReferenceHeight
   );
  }
  return {
   ...base,
   sigmaWidth:Math.sqrt(varianceWidth),
   sigmaHeight:Math.sqrt(varianceHeight),
   uncertaintyModel:'linearized-independent-coordinate-errors'
  };
 }

 return Object.freeze({
  validQuad,solve,homography,map,measureQuad,estimateUncertainty,
  MAX_CONDITION_ESTIMATE,CONDITION_WARNING_ESTIMATE
 });
});
