const {chromium}=require('playwright');
const fs=require('node:fs/promises');
(async()=>{
 const browser=await chromium.launch();const page=await browser.newPage({viewport:{width:1440,height:1100}});
 const errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.goto('http://127.0.0.1:5173');
 await page.getByRole('heading',{name:'Evacuation route assessment'}).waitFor();
 await page.getByText('UAV camera paused or awaiting a fresh frame. Press Play to resume.',{exact:true}).waitFor();
 await page.screenshot({path:'results/phase2-visible-demo/dashboard-paused.png',fullPage:true});
 await fs.writeFile('results/phase2-visible-demo/dashboard-check.json',JSON.stringify({status:errors.length?'failed':'passed',errors,paused_camera_fallback:true},null,2));
 await browser.close();if(errors.length)process.exitCode=1;
})();
