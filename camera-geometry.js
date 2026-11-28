(function(root,factory){
 'use strict';
 const api=factory();
 if(typeof module==='object'&&module.exports)module.exports=api;
 if(root)root.TruckVisionCameraGeometry=api;
})(typeof globalThis!=='undefined'?globalThis:this,function(){
 'use strict';

 const dist=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
 const MAX_CONDITION_ESTIMATE=1e7;
 const CONDITION_WARNING_ESTIMATE=1e5;
 const MAX_QUAD_EDGE_SPREAD_RATIO=0.05;
 const MAX_QUAD_ANGLE_ERROR_DEG=5;
 const MAX_QUAD_DIAGONAL_SPREAD_RATIO=0.05;
 const NONLINEAR_UNCERTAINTY_WARNING_RATIO=0.25;

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

 function angleBetweenDeg(a,b){
  const denominator=Math.hypot(a.x,a.y)*Math.hypot(b.x,b.y);
  if(denominator<1e-12)return Infinity;
  const cosine=Math.max(-1,Math.min(1,(a.x*b.x+a.y*b.y)/denominator));
  return Math.acos(cosine)*180/Math.PI;
 }
 function parallelDeviationDeg(a,b){
  const angle=angleBetweenDeg(a,b);
  return Number.isFinite(angle)?Math.min(angle,180-angle):Infinity;
 }
 function quadShapeDiagnostics(points){
  if(!validQuad(points,0,0))throw new Error('Для проверки формы нужны четыре конечные точки выпуклого четырёхугольника.');
  const top={x:points[1].x-points[0].x,y:points[1].y-points[0].y};
  const right={x:points[2].x-points[1].x,y:points[2].y-points[1].y};
  const bottom={x:points[2].x-points[3].x,y:points[2].y-points[3].y};
  const left={x:points[3].x-points[0].x,y:points[3].y-points[0].y};
  const widthTop=dist(points[0],points[1]);
  const widthBottom=dist(points[3],points[2]);
  const heightLeft=dist(points[0],points[3]);
  const heightRight=dist(points[1],points[2]);
  const width=(widthTop+widthBottom)/2;
  const height=(heightLeft+heightRight)/2;
  const diagonalA=dist(points[0],points[2]);
  const diagonalB=dist(points[1],points[3]);
  const widthSpreadRatio=Math.abs(widthTop-widthBottom)/Math.max(width,1e-9);
  const heightSpreadRatio=Math.abs(heightLeft-heightRight)/Math.max(height,1e-9);
  const diagonalSpreadRatio=Math.abs(diagonalA-diagonalB)/Math.max((diagonalA+diagonalB)/2,1e-9);
  const edgeVectors=[
   top,right,
   {x:points[3].x-points[2].x,y:points[3].y-points[2].y},
   {x:points[0].x-points[3].x,y:points[0].y-points[3].y}
  ];
  const cornerAngles=edgeVectors.map((edge,index)=>
   angleBetweenDeg(edge,edgeVectors[(index+1)%edgeVectors.length])
  );
  const maxOrthogonalityErrorDeg=Math.max(...cornerAngles.map(angle=>Math.abs(90-angle)));
  const widthParallelismErrorDeg=parallelDeviationDeg(top,bottom);
  const heightParallelismErrorDeg=parallelDeviationDeg(left,right);
  const maxParallelismErrorDeg=Math.max(widthParallelismErrorDeg,heightParallelismErrorDeg);
  const warning=widthSpreadRatio>MAX_QUAD_EDGE_SPREAD_RATIO||
   heightSpreadRatio>MAX_QUAD_EDGE_SPREAD_RATIO||
   diagonalSpreadRatio>MAX_QUAD_DIAGONAL_SPREAD_RATIO||
   maxOrthogonalityErrorDeg>MAX_QUAD_ANGLE_ERROR_DEG||
   maxParallelismErrorDeg>MAX_QUAD_ANGLE_ERROR_DEG;
  return {
   widthTop,widthBottom,heightLeft,heightRight,width,height,
   diagonalA,diagonalB,widthSpreadRatio,heightSpreadRatio,diagonalSpreadRatio,
   maxOrthogonalityErrorDeg,widthParallelismErrorDeg,heightParallelismErrorDeg,
   maxParallelismErrorDeg,warning,
   limits:{
    maxEdgeSpreadRatio:MAX_QUAD_EDGE_SPREAD_RATIO,
    maxDiagonalSpreadRatio:MAX_QUAD_DIAGONAL_SPREAD_RATIO,
    maxAngleErrorDeg:MAX_QUAD_ANGLE_ERROR_DEG
   }
  };
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
  const shapeDiagnostics=quadShapeDiagnostics(points);
  const width=shapeDiagnostics.width;
  const height=shapeDiagnostics.height;
  if(!Number.isFinite(width)||!Number.isFinite(height)||width<=0||height<=0)throw new Error('Не удалось рассчитать размеры проёма.');
  return {
   width,
   height,
   points,
   shapeDiagnostics,
   shapeWarning:shapeDiagnostics.warning,
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
  if(!Number.isFinite(sigmaPixels)||sigmaPixels<0.5||sigmaPixels>20||
     !Number.isFinite(sigmaReferenceWidth)||sigmaReferenceWidth<0.0001||
     !Number.isFinite(sigmaReferenceHeight)||sigmaReferenceHeight<0.0001){
   throw new Error('Задайте положительную неопределённость отметки угла (0,5–20 px) и размера эталона (не менее 0,1 мм). Нулевые допуски не принимаются.');
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

  // Cross-check the first-order estimate with finite +/-1σ perturbations of
  // each independent input separately. This is axis-wise sensitivity analysis,
  // not a joint Monte Carlo sample and not a deterministic error bound.
  let finiteVarianceWidth=0,finiteVarianceHeight=0;
  let maxAxisEvenShiftWidth=0,maxAxisEvenShiftHeight=0,nonlinearSensitivityChecks=0;
  let nonlinearSensitivityFailure=null;
  const addFinitePerturbation=(plus,minus)=>{
   const plusWidth=plus.width-base.width,minusWidth=minus.width-base.width;
   const plusHeight=plus.height-base.height,minusHeight=minus.height-base.height;
   finiteVarianceWidth+=(plusWidth*plusWidth+minusWidth*minusWidth)/2;
   finiteVarianceHeight+=(plusHeight*plusHeight+minusHeight*minusHeight)/2;
   maxAxisEvenShiftWidth=Math.max(maxAxisEvenShiftWidth,Math.abs(plusWidth+minusWidth)/2);
   maxAxisEvenShiftHeight=Math.max(maxAxisEvenShiftHeight,Math.abs(plusHeight+minusHeight)/2);
   nonlinearSensitivityChecks++;
  };
  const checkFinitePerturbation=(plusOptions,minusOptions)=>{
   try{
    addFinitePerturbation(
     measureQuad(plusOptions.referencePoints,plusOptions.referenceWidth,plusOptions.referenceHeight,plusOptions.measuredPoints),
     measureQuad(minusOptions.referencePoints,minusOptions.referenceWidth,minusOptions.referenceHeight,minusOptions.measuredPoints)
    );
   }catch(error){
    if(!nonlinearSensitivityFailure)nonlinearSensitivityFailure=error&&error.message?error.message:'sigma-perturbation-invalid';
   }
  };
  if(sigmaPixels>0){
   for(let i=0;i<4;i++){
    for(const axis of ['x','y']){
     const refPlus=referencePoints.map(p=>({...p}));
     const refMinus=referencePoints.map(p=>({...p}));
     refPlus[i][axis]+=sigmaPixels;
     refMinus[i][axis]-=sigmaPixels;
     checkFinitePerturbation(
      {referencePoints:refPlus,referenceWidth,referenceHeight,measuredPoints},
      {referencePoints:refMinus,referenceWidth,referenceHeight,measuredPoints}
     );
     const gatePlus=measuredPoints.map(p=>({...p}));
     const gateMinus=measuredPoints.map(p=>({...p}));
     gatePlus[i][axis]+=sigmaPixels;
     gateMinus[i][axis]-=sigmaPixels;
     checkFinitePerturbation(
      {referencePoints,referenceWidth,referenceHeight,measuredPoints:gatePlus},
      {referencePoints,referenceWidth,referenceHeight,measuredPoints:gateMinus}
     );
    }
   }
  }
  if(sigmaReferenceWidth>0){
   checkFinitePerturbation(
    {referencePoints,referenceWidth:referenceWidth+sigmaReferenceWidth,referenceHeight,measuredPoints},
    {referencePoints,referenceWidth:referenceWidth-sigmaReferenceWidth,referenceHeight,measuredPoints}
   );
  }
  if(sigmaReferenceHeight>0){
   checkFinitePerturbation(
    {referencePoints,referenceWidth,referenceHeight:referenceHeight+sigmaReferenceHeight,measuredPoints},
    {referencePoints,referenceWidth,referenceHeight:referenceHeight-sigmaReferenceHeight,measuredPoints}
   );
  }
  const sigmaWidth=Math.sqrt(varianceWidth),sigmaHeight=Math.sqrt(varianceHeight);
  const axiswiseSensitivityAvailable=nonlinearSensitivityFailure===null&&nonlinearSensitivityChecks>0;
  const nonlinearSigmaWidth=axiswiseSensitivityAvailable?Math.sqrt(finiteVarianceWidth):null;
  const nonlinearSigmaHeight=axiswiseSensitivityAvailable?Math.sqrt(finiteVarianceHeight):null;
  const maxAxisAsymmetryRatio=axiswiseSensitivityAvailable?Math.max(
   maxAxisEvenShiftWidth/Math.max(sigmaWidth,nonlinearSigmaWidth,1e-9),
   maxAxisEvenShiftHeight/Math.max(sigmaHeight,nonlinearSigmaHeight,1e-9)
  ):null;
  const relativeMismatch=(linearized,finite)=>{
   if(!Number.isFinite(finite))return Infinity;
   return Math.abs(finite-linearized)/Math.max(Math.abs(finite),Math.abs(linearized),1e-9);
  };
  const nonlinearSigmaRatioWidth=axiswiseSensitivityAvailable?
   (sigmaWidth>1e-12?nonlinearSigmaWidth/sigmaWidth:(nonlinearSigmaWidth<=1e-12?1:Infinity)):null;
  const nonlinearSigmaRatioHeight=axiswiseSensitivityAvailable?
   (sigmaHeight>1e-12?nonlinearSigmaHeight/sigmaHeight:(nonlinearSigmaHeight<=1e-12?1:Infinity)):null;
  const nonlinearMismatchWidth=axiswiseSensitivityAvailable?relativeMismatch(sigmaWidth,nonlinearSigmaWidth):null;
  const nonlinearMismatchHeight=axiswiseSensitivityAvailable?relativeMismatch(sigmaHeight,nonlinearSigmaHeight):null;

  // Screen interactions between independent ±1σ inputs with a deterministic
  // pairwise covering array. The 32 rows cover all four sign combinations for
  // every pair of the 18 input dimensions; an additional all-high row is a
  // stress case. This is not Monte Carlo, a confidence interval, or a bound.
  const jointInputs=[];
  for(let i=0;i<4;i++)for(const axis of ['x','y']){
   jointInputs.push({kind:'reference-point',index:i,axis,sigma:sigmaPixels});
  }
  for(let i=0;i<4;i++)for(const axis of ['x','y']){
   jointInputs.push({kind:'measured-point',index:i,axis,sigma:sigmaPixels});
  }
  jointInputs.push(
   {kind:'reference-width',sigma:sigmaReferenceWidth},
   {kind:'reference-height',sigma:sigmaReferenceHeight}
  );
  const jointParity=value=>{
   let result=0;
   for(let n=value;n;n>>=1)result^=n&1;
   return result;
  };
  const jointMasks=Array.from({length:31},(_,index)=>index+1).slice(0,jointInputs.length);
  const jointRows=Array.from({length:32},(_,row)=>
   jointMasks.map(mask=>jointParity(row&mask))
  );
  const allHigh=Array(jointInputs.length).fill(1);
  if(!jointRows.some(row=>row.every((value,index)=>value===allHigh[index])))jointRows.push(allHigh);
  let jointSquaredWidth=0,jointSquaredHeight=0;
  let jointMaxAbsDeviationWidth=0,jointMaxAbsDeviationHeight=0;
  let jointSensitivityChecks=0,jointSensitivityFailure=null;
  for(let rowIndex=0;rowIndex<jointRows.length;rowIndex++){
   const row=jointRows[rowIndex];
   const perturbedReference=referencePoints.map(point=>({...point}));
   const perturbedMeasured=measuredPoints.map(point=>({...point}));
   let perturbedReferenceWidth=referenceWidth,perturbedReferenceHeight=referenceHeight;
   for(let inputIndex=0;inputIndex<jointInputs.length;inputIndex++){
    const input=jointInputs[inputIndex];
    const delta=(row[inputIndex]?1:-1)*input.sigma;
    if(input.kind==='reference-point')perturbedReference[input.index][input.axis]+=delta;
    else if(input.kind==='measured-point')perturbedMeasured[input.index][input.axis]+=delta;
    else if(input.kind==='reference-width')perturbedReferenceWidth+=delta;
    else if(input.kind==='reference-height')perturbedReferenceHeight+=delta;
   }
   try{
    const perturbed=measureQuad(
     perturbedReference,perturbedReferenceWidth,perturbedReferenceHeight,perturbedMeasured
    );
    const widthDelta=perturbed.width-base.width;
    const heightDelta=perturbed.height-base.height;
    if(rowIndex<32){
     jointSquaredWidth+=widthDelta*widthDelta;
     jointSquaredHeight+=heightDelta*heightDelta;
    }
    jointMaxAbsDeviationWidth=Math.max(jointMaxAbsDeviationWidth,Math.abs(widthDelta));
    jointMaxAbsDeviationHeight=Math.max(jointMaxAbsDeviationHeight,Math.abs(heightDelta));
    jointSensitivityChecks++;
   }catch(error){
    jointSensitivityFailure=error&&error.message?error.message:'joint-sigma-perturbation-invalid';
    break;
   }
  }
  const jointSensitivityAvailable=jointSensitivityFailure===null&&jointSensitivityChecks===jointRows.length;
  const jointRmsDeviationWidth=jointSensitivityAvailable?Math.sqrt(jointSquaredWidth/32):null;
  const jointRmsDeviationHeight=jointSensitivityAvailable?Math.sqrt(jointSquaredHeight/32):null;
  const jointMismatchWidth=jointSensitivityAvailable&&axiswiseSensitivityAvailable?
   relativeMismatch(nonlinearSigmaWidth,jointRmsDeviationWidth):null;
  const jointMismatchHeight=jointSensitivityAvailable&&axiswiseSensitivityAvailable?
   relativeMismatch(nonlinearSigmaHeight,jointRmsDeviationHeight):null;
  const nonlinearSensitivityAvailable=axiswiseSensitivityAvailable&&jointSensitivityAvailable;
  const nonlinearWarningReasons=[];
  if(!axiswiseSensitivityAvailable)nonlinearWarningReasons.push('finite-perturbation-unavailable');
  if(!jointSensitivityAvailable)nonlinearWarningReasons.push('joint-sigma-perturbation-unavailable');
  if(axiswiseSensitivityAvailable&&Math.max(nonlinearMismatchWidth,nonlinearMismatchHeight)>NONLINEAR_UNCERTAINTY_WARNING_RATIO){
   nonlinearWarningReasons.push('linearized-and-finite-sensitivity-disagree');
  }
  if(axiswiseSensitivityAvailable&&maxAxisAsymmetryRatio>NONLINEAR_UNCERTAINTY_WARNING_RATIO){
   nonlinearWarningReasons.push('asymmetric-one-sigma-response');
  }
  if(jointSensitivityAvailable&&axiswiseSensitivityAvailable&&
     Math.max(jointMismatchWidth,jointMismatchHeight)>NONLINEAR_UNCERTAINTY_WARNING_RATIO){
   nonlinearWarningReasons.push('joint-and-axiswise-sensitivity-disagree');
  }
  return {
   ...base,
   sigmaWidth,
   sigmaHeight,
   uncertaintyModel:'linearized-independent-coordinate-errors',
   nonlinearSensitivityModel:'axiswise-one-sigma-finite-perturbations+pairwise-joint-screen',
   nonlinearSensitivityAvailable,
   nonlinearSensitivityChecks,
   nonlinearSensitivityFailure,
   axiswiseSensitivityAvailable,
   jointSensitivityModel:'32-row-pairwise-independent-±1σ-plus-all-high-stress-case',
   jointSensitivityAvailable,
   jointSensitivityChecks,
   jointSensitivityFailure,
   jointRmsDeviationWidth,
   jointRmsDeviationHeight,
   jointMaxAbsDeviationWidth:jointSensitivityAvailable?jointMaxAbsDeviationWidth:null,
   jointMaxAbsDeviationHeight:jointSensitivityAvailable?jointMaxAbsDeviationHeight:null,
   jointMismatchWidth,
   jointMismatchHeight,
   nonlinearSigmaWidth,
   nonlinearSigmaHeight,
   nonlinearSigmaRatioWidth,
   nonlinearSigmaRatioHeight,
   nonlinearMismatchWidth,
   nonlinearMismatchHeight,
   maxAxisAsymmetryRatio,
   nonlinearWarning:nonlinearWarningReasons.length>0,
   nonlinearWarningReasons
  };
 }
 function summarizeRepeatability(readings,options){
  const {widthTolerance,heightTolerance}=options||{};
  if(!Array.isArray(readings)||readings.length<3||readings.length>10){
   throw new Error('Для оценки повторяемости нужны от 3 до 10 отдельно зафиксированных кадров.');
  }
  if(!Number.isFinite(widthTolerance)||widthTolerance<=0||widthTolerance>0.5||
     !Number.isFinite(heightTolerance)||heightTolerance<=0||heightTolerance>0.5){
   throw new Error('Допуски повторяемости ширины и высоты должны быть положительными значениями до 0,5 м.');
  }
  for(const reading of readings){
   if(!reading||![reading.width,reading.height,reading.sigmaWidth,reading.sigmaHeight].every(Number.isFinite)||
      reading.width<=0||reading.height<=0||reading.sigmaWidth<=0||reading.sigmaHeight<=0){
    throw new Error('Каждое повторное измерение должно содержать положительные размеры и оценки чувствительности 1σ.');
   }
  }
  const stats=key=>{
   const values=readings.map(reading=>reading[key]);
   const mean=values.reduce((sum,value)=>sum+value,0)/values.length;
   const variance=values.reduce((sum,value)=>sum+(value-mean)**2,0)/(values.length-1);
   return {
    mean,
    sampleStdDev:Math.sqrt(variance),
    range:Math.max(...values)-Math.min(...values),
    maxDeviation:Math.max(...values.map(value=>Math.abs(value-mean)))
   };
  };
  const width=stats('width'),height=stats('height');
  const maxSigmaWidth=Math.max(...readings.map(reading=>reading.sigmaWidth));
  const maxSigmaHeight=Math.max(...readings.map(reading=>reading.sigmaHeight));
  // Keep each frame's local 2σ sensitivity separate: shared reference-size
  // error may be correlated across captures and must not be reduced by 1/sqrt(n).
  const widthEnvelopeBound=Math.max(...readings.map(reading=>
   Math.abs(reading.width-width.mean)+2*reading.sigmaWidth
  ));
  const heightEnvelopeBound=Math.max(...readings.map(reading=>
   Math.abs(reading.height-height.mean)+2*reading.sigmaHeight
  ));
  const withinTolerance=width.range<=widthTolerance&&height.range<=heightTolerance;
  return {
   count:readings.length,
   meanWidth:width.mean,
   meanHeight:height.mean,
   sampleStdDevWidth:width.sampleStdDev,
   sampleStdDevHeight:height.sampleStdDev,
   rangeWidth:width.range,
   rangeHeight:height.range,
   maxDeviationWidth:width.maxDeviation,
   maxDeviationHeight:height.maxDeviation,
   widthTolerance,
   heightTolerance,
   maxSigmaWidth,
   maxSigmaHeight,
   widthEnvelopeBound,
   heightEnvelopeBound,
   withinTolerance,
   status:withinTolerance?'repeatable-within-operator-tolerance':'repeatability-exceeds-operator-tolerance',
   uncertaintyModel:'observed-range-plus-per-frame-2sigma-screening'
  };
 }

 function registerGateOnMap(measurement,anchor,headingRadians,repeatability){
  if(!measurement||!Array.isArray(measurement.points)||measurement.points.length!==4||
     !Number.isFinite(measurement.width)||measurement.width<=0||measurement.width>50||
     !Number.isFinite(measurement.height)||measurement.height<=0||measurement.height>50){
   throw new Error('Измерение ворот должно содержать четыре угла и положительные конечные ширину и высоту.');
  }
  if(!anchor||!Number.isFinite(anchor.x)||!Number.isFinite(anchor.y)||
     Math.abs(anchor.x)>10000||Math.abs(anchor.y)>10000||!Number.isFinite(headingRadians)){
   throw new Error('Для привязки ворот задайте конечные координаты центра и направление линии ворот.');
  }
  if(!validQuad(measurement.points,0,0)){
   throw new Error('Измеренные углы ворот должны образовывать выпуклый четырёхугольник.');
  }
  const points=measurement.points;
  const shapeDiagnostics=quadShapeDiagnostics(points);
  const derivedWidth=shapeDiagnostics.width;
  const derivedHeight=shapeDiagnostics.height;
  if(shapeDiagnostics.warning){
   throw new Error('Форма измеренного проёма заметно отличается от прямоугольника. Проверьте четыре угла и положение эталона в той же плоскости.');
  }
  if(Math.abs(measurement.width-derivedWidth)>Math.max(0.01,derivedWidth*0.01)){
   throw new Error('Ширина измерения не совпадает с шириной, рассчитанной по четырём углам. Повторите измерение.');
  }
  if(Math.abs(measurement.height-derivedHeight)>Math.max(0.01,derivedHeight*0.01)){
   throw new Error('Высота измерения не совпадает с высотой, рассчитанной по четырём углам. Повторите измерение.');
  }
  const sigmaWidth=measurement.sigmaWidth;
  const sigmaHeight=measurement.sigmaHeight;
  if(!Number.isFinite(sigmaWidth)||sigmaWidth<0||sigmaWidth>25||
     !Number.isFinite(sigmaHeight)||sigmaHeight<0||sigmaHeight>25){
   throw new Error('Оценки 1σ ширины и высоты должны быть конечными и неотрицательными.');
  }
  if(!repeatability||repeatability.withinTolerance!==true||
     !Number.isInteger(repeatability.count)||repeatability.count<3||repeatability.count>10||
     repeatability.status!=='repeatable-within-operator-tolerance'||
     ![repeatability.meanWidth,repeatability.meanHeight,
       repeatability.maxDeviationWidth,repeatability.maxDeviationHeight,
       repeatability.widthTolerance,repeatability.heightTolerance,
       repeatability.widthEnvelopeBound,repeatability.heightEnvelopeBound].every(Number.isFinite)||
     repeatability.meanWidth<=0||repeatability.meanHeight<=0||
     repeatability.maxDeviationWidth<0||repeatability.maxDeviationHeight<0||
     repeatability.widthTolerance<=0||repeatability.widthTolerance>0.5||
     repeatability.heightTolerance<=0||repeatability.heightTolerance>0.5||
     repeatability.maxDeviationWidth>repeatability.widthTolerance||
     repeatability.maxDeviationHeight>repeatability.heightTolerance||
     repeatability.widthEnvelopeBound<=0||repeatability.heightEnvelopeBound<=0||
     repeatability.widthEnvelopeBound+1e-9<2*sigmaWidth||
     repeatability.heightEnvelopeBound+1e-9<2*sigmaHeight){
   throw new Error('Повторяемость ворот не подтверждена. Нужны минимум три согласованных кадра и корректные оболочки неопределённости.');
  }
  // The final frozen frame must be one member of the accepted series. Do not
  // let a caller pair an unrelated contour with a fabricated repeat summary.
  if(Math.abs(repeatability.meanWidth-measurement.width)>repeatability.maxDeviationWidth+1e-8||
     Math.abs(repeatability.meanHeight-measurement.height)>repeatability.maxDeviationHeight+1e-8){
   throw new Error('Последний контур не согласуется с серией повторных измерений. Повторите серию.');
  }
  const clearWidth=repeatability.meanWidth;
  const clearHeight=repeatability.meanHeight;
  const jointScreenValues=[
   measurement.jointRmsDeviationWidth,
   measurement.jointRmsDeviationHeight,
   measurement.jointMaxAbsDeviationWidth,
   measurement.jointMaxAbsDeviationHeight,
   measurement.jointMismatchWidth,
   measurement.jointMismatchHeight
  ];
  const jointScreenMetricsValid=jointScreenValues.every(value=>Number.isFinite(value)&&value>=0)&&
   measurement.jointMaxAbsDeviationWidth+1e-9>=measurement.jointRmsDeviationWidth&&
   measurement.jointMaxAbsDeviationHeight+1e-9>=measurement.jointRmsDeviationHeight&&
   measurement.jointMismatchWidth<=NONLINEAR_UNCERTAINTY_WARNING_RATIO&&
   measurement.jointMismatchHeight<=NONLINEAR_UNCERTAINTY_WARNING_RATIO;
  if(measurement.nonlinearSensitivityAvailable!==true||measurement.nonlinearWarning===true||
     measurement.jointSensitivityAvailable!==true||measurement.jointSensitivityChecks!==33||
     measurement.jointSensitivityFailure!==null||
     measurement.jointSensitivityModel!=='32-row-pairwise-independent-±1σ-plus-all-high-stress-case'||
     !jointScreenMetricsValid){
   throw new Error('Нелинейная проверка неопределённости не пройдена. Повторите калибровку с более крупным эталоном и устойчивым ракурсом; измерение нельзя переносить на карту.');
  }
  const widthUncertaintyBound=repeatability.widthEnvelopeBound;
  const heightUncertaintyBound=repeatability.heightEnvelopeBound;
  if(widthUncertaintyBound>2||heightUncertaintyBound>2){
   throw new Error('Оболочка неопределённости ширины или высоты превышает поддерживаемый предел 2 м. Измерение нельзя переносить на карту.');
  }
  if(widthUncertaintyBound>=clearWidth-0.05||heightUncertaintyBound>=clearHeight-0.05){
   throw new Error('Неопределённость сопоставима с размером проёма. Измерение нельзя переносить на карту.');
  }
  const leftMid={
   x:(points[0].x+points[3].x)/2,
   y:(points[0].y+points[3].y)/2
  };
  const rightMid={
   x:(points[1].x+points[2].x)/2,
   y:(points[1].y+points[2].y)/2
  };
  const sourceAxisLength=dist(leftMid,rightMid);
  if(!Number.isFinite(sourceAxisLength)||sourceAxisLength<1e-9){
   throw new Error('Невозможно определить ось измеренного проёма.');
  }
  const heading=Math.atan2(Math.sin(headingRadians),Math.cos(headingRadians));
  const ux=Math.cos(heading),uy=Math.sin(heading),half=clearWidth/2;
  return {
   centerX:anchor.x,
   centerY:anchor.y,
   headingRadians:heading,
   headingDeg:heading*180/Math.PI,
   clearWidth,
   clearHeight,
   widthSigma:sigmaWidth,
   heightSigma:sigmaHeight,
   widthUncertaintyBound,
   heightUncertaintyBound,
   widthUncertaintyMultiplier:2,
   widthUncertaintyModel:repeatability.uncertaintyModel,
   repeatability:{
    count:repeatability.count,
    meanWidth:repeatability.meanWidth,
    meanHeight:repeatability.meanHeight,
    sampleStdDevWidth:repeatability.sampleStdDevWidth,
    sampleStdDevHeight:repeatability.sampleStdDevHeight,
    rangeWidth:repeatability.rangeWidth,
    rangeHeight:repeatability.rangeHeight,
    maxDeviationWidth:repeatability.maxDeviationWidth,
    maxDeviationHeight:repeatability.maxDeviationHeight,
    widthTolerance:repeatability.widthTolerance,
    heightTolerance:repeatability.heightTolerance,
    heightEnvelopeBound:repeatability.heightEnvelopeBound
   },
   nonlinearSensitivityModel:measurement.nonlinearSensitivityModel,
   nonlinearMismatchWidth:measurement.nonlinearMismatchWidth,
   maxAxisAsymmetryRatio:measurement.maxAxisAsymmetryRatio,
   jointSensitivityModel:measurement.jointSensitivityModel,
   jointSensitivityAvailable:measurement.jointSensitivityAvailable,
   jointSensitivityChecks:measurement.jointSensitivityChecks,
   jointSensitivityFailure:measurement.jointSensitivityFailure,
   jointRmsDeviationWidth:measurement.jointRmsDeviationWidth,
   jointRmsDeviationHeight:measurement.jointRmsDeviationHeight,
   jointMaxAbsDeviationWidth:measurement.jointMaxAbsDeviationWidth,
   jointMaxAbsDeviationHeight:measurement.jointMaxAbsDeviationHeight,
   jointMismatchWidth:measurement.jointMismatchWidth,
   jointMismatchHeight:measurement.jointMismatchHeight,
   openingStart:{x:anchor.x-half*ux,y:anchor.y-half*uy},
   openingEnd:{x:anchor.x+half*ux,y:anchor.y+half*uy},
   sourceAxisLength,
   shapeDiagnostics,
   mapRegistration:'manual-center-and-heading'
  };
 }

 return Object.freeze({
  validQuad,solve,homography,map,measureQuad,estimateUncertainty,summarizeRepeatability,registerGateOnMap,quadShapeDiagnostics,
  MAX_CONDITION_ESTIMATE,CONDITION_WARNING_ESTIMATE,
  MAX_QUAD_EDGE_SPREAD_RATIO,MAX_QUAD_ANGLE_ERROR_DEG,MAX_QUAD_DIAGONAL_SPREAD_RATIO,
  NONLINEAR_UNCERTAINTY_WARNING_RATIO
 });
});
