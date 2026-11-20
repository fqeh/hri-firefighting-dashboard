// Actual noVNC and ROS-backed dashboard recording; launch C paused first.
const {chromium}=require('playwright');
const fs=require('node:fs/promises');
const assert=require('node:assert/strict');
(async()=>{
 const output=process.argv[2]||'results/phase2-visible-demo';await fs.mkdir(output,{recursive:true});
 const browser=await chromium.launch();
 const context=await browser.newContext({viewport:{width:1440,height:1000},recordVideo:{dir:output,size:{width:1280,height:900}}});
 const page=await context.newPage();const errors=[];page.on('pageerror',e=>errors.push(e.message));
 const state=async()=>(await context.request.get('http://127.0.0.1:8080/api/state')).json();
 const wait=async(predicate)=>{const deadline=Date.now()+90000;while(Date.now()<deadline){const s=await state();if(predicate(s))return s;await page.waitForTimeout(250);}throw Error('Live demo condition timed out');};
 let result={status:'failed'};
 try{
  await page.goto('http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale');
  await page.waitForFunction(()=>document.documentElement.classList.contains('noVNC_connected'));
  const canvas=page.locator('#noVNC_container canvas');await canvas.waitFor();await page.waitForTimeout(1500);
  const box=await canvas.boundingBox();await page.mouse.click(box.x+box.width*26/1280,box.y+box.height*733/800);
  const before=await wait(s=>s.topics['/forest/routes']?.data.edges.includes('trail_a'));
  await page.screenshot({path:output+'/before.png'});
  const after=await wait(s=>{const r=s.topics['/forest/routes']?.data;return r&&r.edge_states.trail_a==='blocked'&&r.edges.includes('trail_b')&&!r.edges.includes('trail_a');});
  const pause=await context.request.post('http://127.0.0.1:8080/api/control',{data:{command:'pause'}});assert.equal(pause.status(),202);
  await page.waitForTimeout(1000);await page.screenshot({path:output+'/after.png'});
  const dashboard=await context.newPage();dashboard.on('pageerror',e=>errors.push(e.message));
  await dashboard.goto('http://127.0.0.1:5173');await dashboard.getByRole('heading',{name:'Evacuation route assessment'}).waitFor();
  await dashboard.screenshot({path:output+'/dashboard.png',fullPage:true});
  const image=await context.request.get('http://127.0.0.1:8080/api/uav-camera.png');
  if(image.status()===200)await fs.writeFile(output+'/uav-camera.png',await image.body());
  assert.deepEqual(errors,[]);
  result={status:'passed',before:before.topics['/forest/routes'].data,after:after.topics['/forest/routes'].data,
   clock_at_blockage:after.topics['/clock'].data.seconds,errors,paused_for_inspection:true};
 }catch(e){result.error=e.stack;process.exitCode=1;}
 finally{await fs.writeFile(output+'/result.json',JSON.stringify(result,null,2));await context.close();await browser.close();console.log(JSON.stringify(result,null,2));}
})();
