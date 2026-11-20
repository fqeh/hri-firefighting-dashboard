// Native Gazebo and dashboard recording of physical stop, warning and approved detour.
const {chromium}=require('playwright');
const fs=require('node:fs/promises');
const {promisify}=require('node:util');
const execute=promisify(require('node:child_process').execFile);
const assert=require('node:assert/strict');
(async()=>{
 const output=process.argv[2]||'results/phase5-visible';await fs.mkdir(output,{recursive:true});
 const launched=await execute('python3',['scripts/scenarioctl.py','start','--scenario','oil_facility','--case','blockage']);
 await fs.writeFile(output+'/launch.log',launched.stdout+launched.stderr);
 const browser=await chromium.launch();const context=await browser.newContext({viewport:{width:1440,height:1000},recordVideo:{dir:output,size:{width:1280,height:900}}});
 const world=await context.newPage(),dashboard=await context.newPage();const history=[],captures={},errors=[];
 for(const page of [world,dashboard])page.on('pageerror',e=>errors.push(e.message));
 const wait=async(fn,timeout=180000)=>{const end=Date.now()+timeout;while(Date.now()<end){try{const v=await fn();if(v)return v;}catch{}await new Promise(r=>setTimeout(r,200));}throw Error('Industrial demonstration timed out');};
 const state=async()=>{const r=await context.request.get('http://127.0.0.1:8080/api/state',{timeout:2500});return r.ok()?r.json():null;};
 const command=async(data)=>{const r=await context.request.post('http://127.0.0.1:8080/api/control',{data});assert.equal(r.status(),202,await r.text());};
 const capture=async(name,s)=>{captures[name]=s;await fs.writeFile(`${output}/${name}.json`,JSON.stringify(s,null,2));await world.screenshot({path:`${output}/${name}-gazebo.png`});await dashboard.screenshot({path:`${output}/${name}-dashboard.png`,fullPage:true});};
 try{
  await wait(async()=>(await context.request.get('http://127.0.0.1:6080/vnc.html',{timeout:2000})).ok());
  await world.goto('http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale');
  await world.waitForFunction(()=>document.documentElement.classList.contains('noVNC_connected'));
  await world.waitForFunction(()=>{const c=document.querySelector('#noVNC_container canvas');if(!c)return false;const p=c.getContext('2d').getImageData(500,40,1,1).data;return p[0]>220&&p[1]>50&&p[1]<160&&p[2]<80;},undefined,{timeout:60000});
  await dashboard.goto('http://127.0.0.1:5173');
  await execute('python3',['scripts/scenarioctl.py','resume']);
  await wait(async()=>{const s=await state();return s?.topics['/hri/facility']&&s.connected&&s.topics['/hri/navigation']?.data.state==='ready';});
  await dashboard.getByRole('button',{name:'Request route recommendation',exact:true}).click();
  const selected=await wait(async()=>{const s=await state();return s?.topics['/hri/robot_route']?.data.request_id>0?s:false;});
  await dashboard.waitForFunction(revision=>document.querySelector('[data-route-revision]')?.dataset.routeRevision===String(revision),selected.topics['/hri/robot_route'].data.revision);
  await dashboard.getByRole('button',{name:'Approve recommended route',exact:true}).click();
  await wait(async()=>{const s=await state();return s?.topics['/hri/navigation']?.data.state==='navigating'&&(s.topics['/robot/odom']?.data.position?.[0]??0)>.1;},30000);
  await wait(async()=>{
   const s=await state();if(!s)return false;
   const t=s.topics['/clock']?.data.seconds||0,r=s.topics['/hri/robot_route']?.data,p=s.topics['/robot/odom']?.data.position;
   history.push(s);
   if(r&&t<7&&!captures.initial)await capture('initial',s);
   if(r?.blocked_edges?.includes('corridor_b')&&s.topics['/hri/warning'])return s;
   return false;
  });
  await dashboard.getByRole('heading',{name:'Explicit warning',exact:true}).waitFor();
  // Native rendering and SSE lag the sensor event. Capture the displayed
  // warning and route after both views have received their next frames.
  await new Promise(r=>setTimeout(r,1500));
  await capture('blocked-warning',await state());
  await dashboard.getByRole('button',{name:'Acknowledge warning',exact:true}).click();
  await dashboard.getByRole('button',{name:'Approve recommended route',exact:true}).click();
  const final=await wait(async()=>{const s=await state();if(s)history.push(s);return s?.topics['/robot/odom']?.data.position?.[1]>2?s:false;});
  await capture('approved-detour',final);
  const arrived=await wait(async()=>{const s=await state();if(s)history.push(s);const p=s?.topics['/robot/odom']?.data.position;return p&&Math.hypot(p[0]-15,p[1]-6)<.4&&s.topics['/hri/navigation']?.data.state==='succeeded'?s:false;},240000);
  await capture('assembly-arrival',arrived);
  await command({command:'cancel_waypoint'});
  await wait(async()=>{const s=await state();return Math.abs(s?.topics['/robot/odom']?.data.linear_speed??1)<.01;});
  await execute('python3',['scripts/scenarioctl.py','pause']);
  await new Promise(r=>setTimeout(r,6500));
  const session=JSON.parse(await fs.readFile('results/scenario-session.json','utf8'));
  const log=await fs.readFile(session.output+'/simulator.log','utf8');
  const critical=log.split('\n').filter(l=>/\[ERROR\]|\[Err\]|Traceback|process has died|Segmentation fault|error sending response/.test(l));
  assert.deepEqual(critical,[]);assert.deepEqual(errors,[]);
  assert.ok(captures.initial&&captures['blocked-warning']&&captures['approved-detour']);
  assert.ok(final.topics['/hri/belief'].data.action_evidence_count>0);
  await fs.writeFile(output+'/result.json',JSON.stringify({status:'passed',critical_log_lines:critical,errors,runtime_log:session.output+'/simulator.log',captures:Object.keys(captures),final_robot_pose:arrived.topics['/robot/odom'].data.position,source:'Native Gazebo, ROS sensor data, physical Nav2 movement and real dashboard approval controls'},null,2));
  console.log('Passed: visible facility, physical blockage/stop, operator posterior/warning, acknowledgement and approved Nav2 detour.');
 }catch(error){await fs.writeFile(output+'/result.json',JSON.stringify({status:'failed',error:error.stack,errors},null,2));throw error;}
 finally{await fs.writeFile(output+'/history.json',JSON.stringify(history));await context.close();await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
