import {afterEach,beforeEach,it,expect,vi} from 'vitest';
import {render,screen,fireEvent,cleanup,waitFor} from '@testing-library/react';
import App from './App';
const hooks=vi.hoisted(()=>({telemetry:{state:null as any,online:true,error:''},session:{session:null as any,error:''},send:vi.fn().mockResolvedValue(undefined),save:vi.fn().mockResolvedValue(undefined)}));
vi.mock('./transport',()=>({useTelemetry:()=>hooks.telemetry,useSession:()=>hooks.session,useScene:()=>({scene:null,error:''}),sendControl:hooks.send,saveEquipmentContents:hooks.save}));
afterEach(()=>{cleanup();hooks.send.mockClear();localStorage.clear();});
beforeEach(()=>{vi.spyOn(HTMLCanvasElement.prototype,'getContext').mockReturnValue(null);});  // jsdom has no WebGL
function setup(){
  const t=(data:any)=>({data,received_at:100,age_seconds:0,stale:false,source:'observation',sim_stamp:10});
  hooks.telemetry.state={schema_version:1,epoch:'run',connected:true,updated_at:100,topics:{'/clock':t({seconds:10}),'/robot/odom':t({position:[2,3,0]}),'/hri/facility':t({destinations:{assembly:'East assembly point'},corridors:['corridor_b']}),'/hri/robot_route':t({revision:8,navigation_allowed:true,recommended:['a','b']})},warnings:[],timeline:[],validation_errors:[]};
  hooks.session.session={schema_version:1,session:{scenario:'oil_facility',case:'blockage',seed:7,policy:'B4',engine:'expected_loss',paused:false},busy:false,operation:null,error:null,parameters:{},cases:{forest:['F'],oil_facility:['blockage']},policies:['B1','B4'],experiment:null,artifacts:[],evaluation_enabled:false};
}
it('switches scenarios through backend and never labels battery as measured',async()=>{
  setup();render(<App/>);fireEvent.change(screen.getByLabelText('Scenario'),{target:{value:'forest'}});fireEvent.click(screen.getByRole('button',{name:'Load scenario & policy'}));
  await waitFor(()=>expect(hooks.send).toHaveBeenCalledWith(expect.objectContaining({command:'switch',scenario:'forest',case:'F'}),true));expect(screen.getByText('Not modeled')).toBeTruthy();
});
it('approves the displayed route revision, then disables an unavailable recommendation',async()=>{
  setup();const view=render(<App/>);fireEvent.click(screen.getByRole('tab',{name:'Robot'}));fireEvent.click(screen.getByRole('button',{name:'Approve recommended route'}));await waitFor(()=>expect(hooks.send).toHaveBeenCalledWith({command:'approve_route',revision:8},false));
  hooks.telemetry.state={...hooks.telemetry.state,topics:{...hooks.telemetry.state.topics,'/hri/robot_route':{...hooks.telemetry.state.topics['/hri/robot_route'],data:{revision:9,navigation_allowed:false}}}};
  view.rerender(<App/>);expect((screen.getByRole('button',{name:'Approve recommended route'}) as HTMLButtonElement).disabled).toBe(true);
});
it('pause does not erase the run while a scenario restart hides previous telemetry',()=>{
  setup();const view=render(<App/>);hooks.session.session={...hooks.session.session,busy:true,operation:'pause'};view.rerender(<App/>);expect(screen.getByText(/2.0, 3.0/)).toBeTruthy();
  hooks.session.session={...hooks.session.session,operation:'switch'};view.rerender(<App/>);expect(screen.queryByText(/2.0, 3.0/)).toBeNull();
});
it('shows and injects the configured tank aftermath, and renders its recorded timeline event',async()=>{
  setup();
  hooks.session.session={...hooks.session.session,parameters:{incidents:[{id:'hazard_debris_tk01_aftermath',kind:'debris',edge:'north_road',display_name:'Tank TK-01 damage aftermath'}]}};
  hooks.telemetry.state.timeline=[{topic:'/hri/events',received_at:101,data:{stamp:10.1,source:'industrial',event:'tank_damage_aftermath_created',detail:'Configured debris; no process telemetry.'}}];
  render(<App/>);
  expect(screen.getByRole('option',{name:'Tank TK-01 damage aftermath'})).toBeTruthy();
  fireEvent.click(screen.getByRole('button',{name:'Inject test incident'}));
  await waitFor(()=>expect(hooks.send).toHaveBeenCalledWith({command:'inject',hazard_id:'hazard_debris_tk01_aftermath'},false));
  expect(screen.getByText('tank damage aftermath created')).toBeTruthy();
  expect(screen.getByText(/no process telemetry/)).toBeTruthy();
});
it('keeps one screen: configuration sections on the left, the world in the middle, the explanation on the right',()=>{
  setup();render(<App/>);
  expect(screen.getByRole('button',{name:'Load scenario & policy'})).toBeTruthy();
  fireEvent.click(screen.getByRole('tab',{name:'Experiment'}));
  expect(screen.getByRole('button',{name:'Start experiment'})).toBeTruthy();
  expect(screen.queryByRole('button',{name:'Load scenario & policy'})).toBeNull();  // one section at a time
  fireEvent.click(screen.getByRole('tab',{name:'Experiment'}));  // clicking the open section collapses the sidebar
  expect(screen.queryByRole('button',{name:'Start experiment'})).toBeNull();
  expect(screen.getByRole('tab',{name:'3D web view'})).toBeTruthy();
  expect(screen.getByText('Should the robot warn the operator?')).toBeTruthy();
});
it('labels warning policies like scenario cases',()=>{
  setup();render(<App/>);
  expect(screen.getByRole('option',{name:'B4 · Belief-aware (ours)'})).toBeTruthy();
  expect((screen.getByRole('option',{name:'B1 · Show everything, never alert'}) as HTMLOptionElement).value).toBe('B1');
});
it('presets only fill the form; nothing reaches the simulator until Load',async()=>{
  setup();render(<App/>);
  fireEvent.click(screen.getByRole('button',{name:'Crew unaware'}));
  expect(hooks.send).not.toHaveBeenCalled();
  expect((screen.getByLabelText('Scenario') as HTMLSelectElement).value).toBe('forest');
  expect((screen.getByLabelText('Scenario case') as HTMLSelectElement).value).toBe('F');
  expect((screen.getByLabelText('Crew observation radius') as HTMLInputElement).value).toBe('1');
  expect(screen.getByText(/Not loaded yet/)).toBeTruthy();
  fireEvent.click(screen.getByRole('button',{name:'Load scenario & policy'}));
  await waitFor(()=>expect(hooks.send).toHaveBeenCalledWith(expect.objectContaining({command:'switch',scenario:'forest',case:'F',policy:'B4',human_radius:1,map_rate:0,human_lapse:0,manual_incidents:false,seed:7}),true));
});
it('keeps the warning and event log on the right, next to the explanation',()=>{
  setup();render(<App/>);
  expect(screen.getByRole('tablist',{name:'Right panel'})).toBeTruthy();
  fireEvent.click(screen.getByRole('tab',{name:'Log'}));
  expect(screen.getByText('LIVE EVENT LOG · SIMULATION TIME')).toBeTruthy();
  expect(screen.queryByRole('heading',{name:'Should the robot warn the operator?'})).toBeNull();  // explanation hidden while the log is open
});
it('shows which incidents are already in the run and only offers the rest',()=>{
  setup();
  hooks.session.session={...hooks.session.session,parameters:{incidents:[{id:'hazard_tree',kind:'tree',edge:'trail_a',manual_only:false,display_name:'Fallen tree on Trail A'},{id:'hazard_smoke',kind:'smoke',edge:'trail_a',manual_only:true,display_name:'Dense smoke over Trail A'}]}};
  hooks.telemetry.state.timeline=[{topic:'/hri/events',received_at:101,data:{stamp:8,source:'scenario',event:'incident_injected',id:'hazard_tree'}}];
  render(<App/>);
  const tree=screen.getByRole('option',{name:'Fallen tree on Trail A — in the simulation since 8.0 s'}) as HTMLOptionElement;
  expect(tree.disabled).toBe(true);
  expect((screen.getByLabelText('Test incident') as HTMLSelectElement).value).toBe('hazard_smoke');
  fireEvent.click(screen.getByRole('button',{name:'Inject test incident'}));
  expect(hooks.send).toHaveBeenCalledWith({command:'inject',hazard_id:'hazard_smoke'},false);
});
