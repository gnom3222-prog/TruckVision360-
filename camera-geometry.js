(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionCameraGeometry=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';

 const dist=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);

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

 function solve(A,b){
  if(!Array.isArray(A)||A.length!==8||!Array.isArray(b)||b.length!==8||A.some(r=>!Array.isArray(r)||r.length!==8||r.some(v=>!Number.isFinite(v)))||b.some(v=>!Number.isFinite(v)))throw new Error('Недостаточно корректных данных для калибровки.');
  const m=A.map((r,i)=>r.concat(b[i]));
  const scale=Math.max(1,...m.flatMap(r=>r.slice(0,8).map(Math.abs)));
  for(let j=0;j<8;j++){
   let p=j;
   for(let r=j+1;r<8;r++)if(Math.abs(m[r][j])>Math.abs(m[p][j]))p=r;
   if(Math.abs(m[p][j])<scale*1e-12)throw new Error('Калибровочные точки расположены неправильно или почти на одной линии.');
   [m[j],m[p]]=[m[p],m[j]];
   const z=m[j][j];
   for(let k=j;k<9;k++)m[j][k]/=z;
   for(let r=0;r<8;r++)if(r!==j){
    const factor=m[r][j];
    for(let k=j;k<9;k++)m[r][k]-=factor*m[j][k];
   }
  }
  return m.map(r=>r[8]);
 }

 function homography(src,width,height){
  if(!Array.isArray(src)||src.length!==4||!Number.isFinite(width)||!Number.isFinite(height)||width<=0||height<=0)throw new Error('Размеры эталона или калибровочные точки некорректны.');
  const A=[],b=[],dst=[{x:0,y:0},{x:width,y:0},{x:width,y:height},{x:0,y:height}];
  for(let i=0;i<4;i++){
   const {x,y}=src[i],{x:X,y:Y}=dst[i];
   A.push([x,y,1,0,0,0,-X*x,-X*y]);b.push(X);
   A.push([0,0,0,x,y,1,-Y*x,-Y*y]);b.push(Y);
  }
  return solve(A,b);
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
  return {width,height,points};
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

 return Object.freeze({validQuad,solve,homography,map,measureQuad,estimateUncertainty});
});
