import {afterEach,it,expect,vi} from 'vitest';
import {render,screen,fireEvent,cleanup,waitFor} from '@testing-library/react';
import App from './App';
const hooks=vi.hoisted(()=>({telemetry:{state:null as any,online:true,error:''},session:{session:null as any,error:''},send:vi.fn().mockResolvedValue(undefined),save:vi.fn().mockResolvedValue(undefined)}));
vi.mock('./transport',()=>({useTelemetry:()=>hooks.telemetry,useSession:()=>hooks.session,sendControl:hooks.send,saveEquipmentContents:hooks.save}));
afterEach(()=>{cleanup();hooks.send.mockClear();});
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
  setup();const view=render(<App/>);fireEvent.click(screen.getByRole('button',{name:'Approve recommended route'}));await waitFor(()=>expect(hooks.send).toHaveBeenCalledWith({command:'approve_route',revision:8},false));
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
