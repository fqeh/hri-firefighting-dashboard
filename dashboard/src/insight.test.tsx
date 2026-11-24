import {afterEach,describe,it,expect} from 'vitest';
import {render,screen,cleanup,fireEvent} from '@testing-library/react';
import {DecisionPanel,LookoutStatus,CrewBeliefPanel,EscalationAlert} from './insight';
import {vi} from 'vitest';
import {WorldMap} from './WorldMap';
import {type Dict,type Topic} from './contracts';
afterEach(cleanup);
const topic=(data:Dict,source:Topic['source']='observation'):Topic=>({data,received_at:100,age_seconds:.1,stale:false,source,sim_stamp:10});
const params={nodes:{start:[0,0],fork:[4,0],trail_a:[10,0],trail_b:[4,6]},edges:[{id:'approach',source:'start',target:'fork'},{id:'trail_a',source:'fork',target:'trail_a'},{id:'trail_b',source:'fork',target:'trail_b'}]};
const posterior={open_safe:.8,blocked:.1,open_hazardous:.05,unknown_stale:.05};
const topics=(intentPath:string[]):Record<string,Topic>=>({
  '/hri/rescue_route':topic({nodes:params.nodes,edge_states:{approach:'unknown',trail_a:'blocked',trail_b:'unknown'}},'inference'),
  '/hri/belief':topic({edge:'trail_a',beliefs:{trail_a:{posterior,p_mismatch:.8}}},'inference'),
  '/hri/human_intent':topic({path:intentPath}),
  '/hri/observations':topic({hazards:[{id:'tree',kind:'tree',edge:'trail_a',position:[7,0]}]}),
  '/human/odom':topic({position:[2,0,0]},'simulation_truth'),
});
const decision=(approaching:boolean,action:string):Dict=>({hazard_id:'tree',edge:'trail_a',action,policy:'B4',classification:approaching?'probably_unaware':'likely_aware_or_avoiding',
  context:{probability:.8,approaching,task_relevant:true,distance:5,urgency:0},threshold:approaching?{probability:.39,action:'SOFT_CUE'}:null,
  losses:{SILENT:approaching?8.4:0,SOFT_CUE:5.1,EXPLICIT_WARNING:6.6},baselines:{B1:'SILENT',B2:'SILENT',B3:'EXPLICIT_WARNING',B4:approaching?'SOFT_CUE':'SILENT'}});

describe('presentation panels',()=>{
  it('names the lookout state explicitly and walks observe → infer → decide → warn',()=>{
    const view=render(<LookoutStatus topics={{}} decision={{action:'SILENT',policy:'B4',classification:'no_relevant_hazard'}} warnings={[]}/>);
    expect(screen.getByTestId('lookout-status').textContent).toMatch(/Monitoring.*speaks only when a hazard is on their route/);
    expect(view.container.querySelectorAll('.pipeline li.idle')).toHaveLength(2);
    view.rerender(<LookoutStatus topics={topics(['start','fork','trail_a'])} decision={decision(true,'EXPLICIT_WARNING')} warnings={[]}/>);
    expect(screen.getByText('≠ False belief detected')).toBeTruthy();
    expect(screen.getByText('UAV sees Trail A blocked (tree)')).toBeTruthy();
    expect(screen.getByText('Crew thinks it is open · 80%')).toBeTruthy();
    expect(screen.getByText('Explicit warning · B4 (above 39%)')).toBeTruthy();
    const warning={warning:{id:'B4-1',action:'explicit',stamp:11.5,edge:'trail_a'},acknowledged:false,dismissed:false};
    view.rerender(<LookoutStatus topics={topics(['start','fork','trail_a'])} decision={decision(true,'EXPLICIT_WARNING')} warnings={[warning]}/>);
    expect(screen.getByText('Warning sent')).toBeTruthy();
    expect(screen.getByText('Explicit warning at 11.5 s · awaiting acknowledgement')).toBeTruthy();
  });
  it('escalates to the operator when every escape route is blocked, and records the dispatch',()=>{
    const esc=(extra:Dict)=>topic({active:true,since:31.5,blocked_edges:['trail_a','trail_b','trail_c','trail_d'],message:'Every escape route to the safety zone is blocked.',operator_dispatched_at:null,...extra},'inference');
    const send=vi.fn();
    const view=render(<EscalationAlert topic={esc({})} send={send} disabled={false}/>);
    expect(screen.getByText('Blocked: Trail A · Trail B · Trail C · Trail D')).toBeTruthy();
    fireEvent.click(screen.getByRole('button',{name:'Send operator'}));
    expect(send).toHaveBeenCalledWith({command:'send_operator'});
    view.rerender(<EscalationAlert topic={esc({operator_dispatched_at:40})} send={send} disabled={false}/>);
    expect(screen.getByText('Operator dispatched at 40.0 s')).toBeTruthy();
    view.rerender(<EscalationAlert topic={topic({active:false},'inference')} send={send} disabled={false}/>);
    expect(screen.queryByTestId('escalation')).toBeNull();
    cleanup();
    render(<LookoutStatus topics={{'/hri/escalation':esc({})}} decision={{action:'SILENT',policy:'B4'}} warnings={[]}/>);
    expect(screen.getByText('No escape route')).toBeTruthy();
  });
  it('says when smoke hides part of the routes from the sensors',()=>{
    const smoky={'/hri/observations':topic({hazards:[],occluders:[{id:'hazard_smoke',kind:'smoke',edge:'trail_a',position:[7,-2,0],radius:3.5}]})};
    render(<LookoutStatus topics={smoky} decision={{action:'SILENT',policy:'B4'}} warnings={[]}/>);
    expect(screen.getByText('Smoke hides Trail A from the UAV')).toBeTruthy();
    expect(screen.getByTestId('lookout-status').textContent).toMatch(/cannot warn about hazards it cannot see/);
  });
  it('shows the crew’s belief about the trail in focus as four hypotheses',()=>{
    render(<CrewBeliefPanel topics={topics(['start','fork','trail_a'])} decision={decision(true,'SOFT_CUE')}/>);
    expect(screen.getByRole('meter',{name:'Open and safe'}).getAttribute('aria-valuenow')).toBe('0.8');
    expect(screen.getByText(/still heading that way: 80% likely/)).toBeTruthy();
    expect(screen.getByText(/not their hidden true belief/)).toBeTruthy();
  });
  it('shows the threshold, the lowest-loss action and what each baseline would do',()=>{
    const view=render(<DecisionPanel decision={decision(true,'SOFT_CUE')}/>);
    expect(screen.getByTestId('threshold').textContent).toBe('speak above 39%');
    expect(view.container.querySelector('.loss-row.best')?.getAttribute('data-action')).toBe('SOFT_CUE');
    expect(view.container.querySelector('[data-policy="B3"] em')?.textContent).toBe('WARN');
    view.rerender(<DecisionPanel decision={decision(false,'SILENT')}/>);
    expect(view.container.querySelector('[data-policy="B4"] em')?.textContent).toBe('SILENT');
    expect(screen.queryByTestId('threshold')).toBeNull();
  });
  it('waits quietly when no hazard has been observed',()=>{
    render(<DecisionPanel decision={{action:'SILENT',classification:'no_relevant_hazard',policy:'B4'}}/>);
    expect(screen.getByText(/stays silent and keeps watching/)).toBeTruthy();
  });
  it('labels a fire cluster once instead of once per patch',()=>{
    const hazards=[0,1,2].map(i=>({id:`fire_${i}`,kind:'fire',edge:'trail_a',position:[7+i*.2,0],radius:1}));
    const view=render(<WorldMap topics={{'/hri/observations':topic({hazards})}} parameters={params} showTruth={false}/>);
    const labels=[...view.container.querySelectorAll('.hazard-label')].map(n=>n.textContent);
    expect(labels).toEqual(['FIRE ×3']);
  });
});
