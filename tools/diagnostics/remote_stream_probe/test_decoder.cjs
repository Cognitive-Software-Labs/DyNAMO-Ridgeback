const assert=require('node:assert/strict');
const {decodeDynamoMarker}=require('./client.js');
function pixels(id){
  const p=new Uint8ClampedArray(360*40*4);
  function put(x,y,color){p.set([...color,255],(y*360+x)*4);}
  put(10,10,[0,255,255]);put(350,10,[255,0,255]);
  for(let i=0;i<16;i++)for(let row=0;row<2;row++){
    let v=(((id>>i)&1)^row)?255:0;put(30+i*20,10+20*row,[v,v,v]);
  }
  return p;
}
for(const id of [0,1,19,255,1024,65535])assert.equal(decodeDynamoMarker(pixels(id)),id);
const partial=pixels(12);partial.set([128,128,128],(10*360+30)*4);
assert.equal(decodeDynamoMarker(partial),null);
assert.equal(decodeDynamoMarker(new Uint8ClampedArray(360*40*4)),null);
console.log('Marker decoder: known IDs, partial updates and missing marker passed');
