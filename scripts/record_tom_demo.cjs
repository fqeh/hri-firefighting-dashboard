// Record the real Gazebo desktop and ROS-backed dashboard for the matched pair.
// Requires the optional GUI image, running Vite on 5173, and Playwright.
const {chromium}=require('playwright');
const fs=require('node:fs/promises');
const {execFile}=require('node:child_process');
const {promisify}=require('node:util');
const execute=promisify(execFile);
const assert=require('node:assert/strict');

(async()=>{
 const output=process.argv[2]||'results/phase3-visible-demo';
 await fs.mkdir(output,{recursive:true});
 const browser=await chromium.launch();
 const results={};
 try {
  for(const condition of ['known','unaware']){
   const dir=`${output}/${condition}`;await fs.mkdir(dir,{recursive:true});
   const control=await execute('python3',['scripts/forestctl.py','start','--belief',condition,'--seed','7']);
   await fs.writeFile(dir+'/launch.log',control.stdout+control.stderr);
   const context=await browser.newContext({viewport:{width:1440,height:1000},recordVideo:{dir,size:{width:1280,height:900}}});
   const errors=[];const page=await context.newPage();page.on('pageerror',e=>errors.push(e.message));
   const wait=async(fn,timeout=120000)=>{const end=Date.now()+timeout;while(Date.now()<end){try{const value=await fn();if(value)return value;}catch{}await new Promise(r=>setTimeout(r,300));}throw Error('Live demonstration timed out');};
   try{
    await wait(async()=>{const r=await context.request.get('http://127.0.0.1:6080/vnc.html',{timeout:2000});return r.ok();});
    await page.goto('http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale');
    await page.waitForFunction(()=>document.documentElement.classList.contains('noVNC_connected'));
    await page.locator('#noVNC_container canvas').waitFor();
    // noVNC can connect before Gazebo creates its window. Wait for the native
    // orange toolbar, not merely an empty connected X desktop.
    await page.waitForFunction(()=>{
      const canvas=document.querySelector('#noVNC_container canvas');
      if(!canvas)return false;
      const p=canvas.getContext('2d').getImageData(500,40,1,1).data;
      return p[0]>220&&p[1]>50&&p[1]<160&&p[2]<80;
    },undefined,{timeout:60000});
    await page.waitForTimeout(1500);
    await page.screenshot({path:dir+'/initial-world.png'});
    await execute('python3',['scripts/forestctl.py','resume']);
    const state=await wait(async()=>{
      const response=await context.request.get('http://127.0.0.1:8080/api/state',{timeout:2000});
      if(!response.ok())return null;const s=await response.json();const b=s.topics['/hri/belief']?.data;
      return b?.last_movement_evidence?.origin==='fork'&&b?.beliefs?.trail_a?.hazard_observed?s:null;
    });
    await execute('python3',['scripts/forestctl.py','pause']);
    await page.waitForTimeout(700);await page.screenshot({path:dir+'/branch-world.png'});
    const b=state.topics['/hri/belief'].data,edge=b.beliefs.trail_a;
    assert.equal(b.last_movement_evidence.target,condition==='known'?'trail_b':'trail_a');
    assert.ok(condition==='known'?edge.posterior.open_safe<.1:edge.posterior.open_safe>.7);
    const dashboard=await context.newPage();dashboard.on('pageerror',e=>errors.push(e.message));
    await dashboard.goto('http://127.0.0.1:5173');
    await dashboard.getByText('Believes open and safe',{exact:true}).waitFor();
    await dashboard.getByText('Believes blocked',{exact:true}).waitFor();
    await dashboard.getByText('Believes open but hazardous',{exact:true}).waitFor();
    await dashboard.getByText('Unknown or stale',{exact:true}).waitFor();
    await dashboard.screenshot({path:dir+'/dashboard.png',fullPage:true});
    const ui=await dashboard.locator('body').innerText();
    assert.ok(!ui.includes('crew_initial_knowledge'));
    assert.deepEqual(errors,[]);
    results[condition]={status:'passed',belief:edge,movement:b.last_movement_evidence,
        clock:state.topics['/clock'].data.seconds,errors};
    await fs.writeFile(dir+'/state.json',JSON.stringify(state,null,2));
   }finally{await context.close();}
  }
  await fs.writeFile(output+'/result.json',JSON.stringify({status:'passed',conditions:results},null,2));
  console.log(JSON.stringify({status:'passed',conditions:results},null,2));
 }catch(error){
  await fs.writeFile(output+'/result.json',JSON.stringify({status:'failed',error:error.stack,conditions:results},null,2));
  throw error;
 }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
