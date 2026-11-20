/* Optional real-browser integration test. Requires Playwright, a running GUI
 * service (6080/8080), and Vite (5173). See docs/PHASE1_VALIDATION.md. */
const { chromium } = require('playwright');
const fs = require('node:fs/promises');
const path = require('node:path');
const assert = require('node:assert/strict');

(async () => {
  const output = process.argv[2] || 'results/phase1-browser';
  await fs.mkdir(output, { recursive: true });
  const browser = await chromium.launch({ headless: true });
  const context = await browser.newContext({ viewport: { width: 1440, height: 1000 } });
  const errors = [];
  let result = { status: 'failed', browser: browser.version(), errors };
  const state = async () => (await context.request.get('http://127.0.0.1:8080/api/state')).json();
  const control = async (command) => {
    const response = await context.request.post('http://127.0.0.1:8080/api/control', { data: { command } });
    assert.equal(response.status(), 202);
  };
  try {
    const gui = await context.newPage();
    gui.on('pageerror', error => errors.push(`gui: ${error.message}`));
    await gui.goto('http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale');
    await gui.waitForFunction(() => document.documentElement.classList.contains('noVNC_connected'), { timeout: 30000 });
    await gui.waitForFunction(() => {
      const canvas = document.querySelector('#noVNC_container canvas');
      if (!canvas || canvas.width < 600) return false;
      const pixels = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height).data;
      const colors = new Set();
      for (let i = 0; i < pixels.length; i += 120) colors.add(`${pixels[i]},${pixels[i+1]},${pixels[i+2]}`);
      return colors.size > 50;
    });
    await gui.screenshot({ path: path.join(output, 'gazebo-browser.png') });
    const page = await context.newPage();
    page.on('pageerror', error => errors.push(`dashboard: ${error.message}`));
    await page.goto('http://127.0.0.1:5173');
    await page.getByText('Stop latch released · confirmed', { exact: true }).waitFor();
    const initial = await state();
    assert.equal(initial.connected, true);
    await page.getByRole('button', { name: 'Stop robot' }).click();
    await page.getByText('Stop latch active · confirmed', { exact: true }).waitFor();
    let stopped = await state();
    assert.equal(stopped.topics['/robot/emergency_stop_active'].data.active, true);
    await page.waitForFunction(async () => {
      const value = await (await fetch('/api/state')).json();
      return value.topics['/dashboard/emergency_stop_result']?.data.success === true;
    });
    stopped = await state();
    await page.screenshot({ path: path.join(output, 'dashboard-stop.png'), fullPage: true });
    await page.getByRole('button', { name: 'Release stop', exact: true }).click();
    await page.getByText('Stop latch released · confirmed', { exact: true }).waitFor();
    await control('pause');
    await page.waitForTimeout(1200);
    const paused = (await state()).topics['/clock'].data.seconds;
    await page.waitForTimeout(1200);
    assert.equal((await state()).topics['/clock'].data.seconds, paused);
    await page.getByRole('button', { name: 'Stop robot' }).click();
    await page.getByText('Stop latch active · confirmed', { exact: true }).waitFor();
    assert.equal((await state()).topics['/clock'].data.seconds, paused);
    await page.getByRole('button', { name: 'Release stop', exact: true }).click();
    await page.getByText('Stop latch released · confirmed', { exact: true }).waitFor();
    await control('resume');
    let resumed = paused;
    const deadline = Date.now() + 15000;
    while (Date.now() < deadline && resumed <= paused) {
      await page.waitForTimeout(200);
      resumed = (await state()).topics['/clock'].data.seconds;
    }
    assert.ok(resumed > paused, 'simulation clock did not resume');
    const camera = await context.request.get('http://127.0.0.1:8080/api/camera.png');
    assert.equal(camera.status(), 200);
    assert.deepEqual((await camera.body()).subarray(0, 8), Buffer.from([137,80,78,71,13,10,26,10]));
    assert.deepEqual(errors, []);
    result = { ...result, status: 'passed', noVNC: 'connected, nonblank actual desktop canvas',
      stop_service_result: stopped.topics['/dashboard/emergency_stop_result'].data,
      confirmed_stop_latch: stopped.topics['/robot/emergency_stop_active'].data,
      stop_and_release_while_clock_paused: true, paused_sim_time: paused,
      resumed_sim_time: resumed, camera_png: true };
  } catch (error) {
    result.error = error.stack;
    process.exitCode = 1;
  } finally {
    await control('resume').catch(() => {});
    await fs.writeFile(path.join(output, 'browser-check.json'), JSON.stringify(result, null, 2) + '\n');
    console.log(JSON.stringify(result, null, 2));
    await browser.close();
  }
})();
