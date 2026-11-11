/* Save as a bookmark URL prefixed with javascript: and invoke on the RDP tab.
   Reads only the small colored marker; no network calls or screen permissions. */
function decodeDynamoMarker(p,width=360) {
  const at=(x,y)=>[...p.slice((y*width+x)*4,(y*width+x)*4+3)];
  const cyan=at(10,10),magenta=at(350,10);
  if(!(cyan[0]<100&&cyan[1]>140&&cyan[2]>140&&magenta[0]>140&&magenta[1]<100&&magenta[2]>140))return null;
  let id=0;
  for(let i=0;i<16;i++){
    const a=at(30+i*20,10),b=at(30+i*20,30),va=a.reduce((s,x)=>s+x,0)/3,vb=b.reduce((s,x)=>s+x,0)/3;
    if(!((va<80&&vb>175)||(va>175&&vb<80)))return null;
    if(va>128)id|=1<<i;
  }
  return id;
}
if(typeof module!=='undefined'&&module.exports)module.exports={decodeDynamoMarker};

(() => {
  if(typeof document==='undefined')return;
  if(window.dynamoDeliveryProbe){window.dynamoDeliveryProbe.stop();return;}
  const box=document.createElement('div');
  Object.assign(box.style,{position:'fixed',right:'12px',top:'12px',background:'#17202a',color:'white',padding:'12px',zIndex:2147483647,font:'14px monospace',maxWidth:'460px'});
  box.textContent='RDP probe: click the CENTER of the CYAN square in the remote marker window. Esc stops and downloads results.';
  document.body.appendChild(box);
  let anchor=null,alive=true,last=null,events=[],invalid=0,raf=0,start=performance.now(),captureFrames=0;
  const bitmap=document.createElement('canvas');bitmap.width=360;bitmap.height=40;
  const ctx=bitmap.getContext('2d',{willReadFrequently:true});
  function stop(){
    if(!alive)return;alive=false;cancelAnimationFrame(raf);document.removeEventListener('click',pick,true);document.removeEventListener('keydown',key,true);
    const elapsed=(performance.now()-start)/1000;
    const data={metric:'unique remote marker IDs observed in Guacamole canvas',wall_seconds:elapsed,unique_updates:events.length,
      delivered_updates_per_second:events.length/elapsed,invalid_samples:invalid,client_animation_callbacks:captureFrames,events,
      limitations:'Marker delivery is not simulator viewport FPS. Browser refresh limits sampling; invalid or duplicate samples are excluded. No latency or complete-screen-frame claim.'};
    const link=document.createElement('a');link.href=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));
    link.download='dynamo-rdp-delivery.json';link.click();URL.revokeObjectURL(link.href);box.remove();delete window.dynamoDeliveryProbe;
  }
  function key(e){if(e.key==='Escape')stop();}
  function pick(e){
    if(box.contains(e.target))return;
    const canvases=[...document.querySelectorAll('canvas')].filter(c=>{const r=c.getBoundingClientRect();return c.width>400&&r.width>300&&e.clientX>=r.left&&e.clientX<=r.right&&e.clientY>=r.top&&e.clientY<=r.bottom;});
    if(!canvases.length){box.textContent='No remote desktop canvas at that point. Click the cyan marker.';return;}
    const base=canvases.sort((a,b)=>b.width*b.height-a.width*a.height)[0],r=base.getBoundingClientRect(),scale=r.width/base.width;
    anchor={x:e.clientX-10*scale,y:e.clientY-10*scale,scale};start=performance.now();events=[];last=null;invalid=0;captureFrames=0;
    e.preventDefault();e.stopPropagation();document.removeEventListener('click',pick,true);
  }
  function sample(){
    if(!alive)return;
    if(anchor){
      captureFrames++;ctx.clearRect(0,0,360,40);
      for(const c of document.querySelectorAll('canvas')){
        const r=c.getBoundingClientRect();if(!r.width||!r.height||r.right<anchor.x||r.bottom<anchor.y||r.left>anchor.x+360*anchor.scale||r.top>anchor.y+40*anchor.scale)continue;
        ctx.drawImage(c,(r.left-anchor.x)/anchor.scale,(r.top-anchor.y)/anchor.scale,r.width/anchor.scale,r.height/anchor.scale);
      }
      try{
        const p=ctx.getImageData(0,0,360,40).data,id=decodeDynamoMarker(p);
        if(id!==null&&id!==last){events.push({wall_ms:performance.now()-start,id});last=id;}
        else if(id===null)invalid++;
        const sec=(performance.now()-start)/1000;
        box.textContent=`RDP marker: ${(events.length/Math.max(sec,.001)).toFixed(1)} unique updates/s; ${invalid} invalid samples. Esc saves JSON. This is not simulator FPS.`;
        if(sec>=30){stop();return;}
      }catch(error){box.textContent='Canvas pixels unavailable: '+error.message+' — Esc exits.';}
    }
    raf=requestAnimationFrame(sample);
  }
  document.addEventListener('click',pick,true);document.addEventListener('keydown',key,true);
  window.dynamoDeliveryProbe={stop};raf=requestAnimationFrame(sample);
})();
