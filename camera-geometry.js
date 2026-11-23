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

 return Object.freeze({validQuad,solve,homography,map});
});
