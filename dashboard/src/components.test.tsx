import {afterEach,describe,it,expect,vi} from 'vitest';
import {render,screen,fireEvent,cleanup} from '@testing-library/react';
import {WarningCard,BeliefPanel,Camera,EquipmentRegistryPanel} from './components';
import {WorldMap} from './WorldMap';
import {type Dict,type Topic} from './contracts';
afterEach(cleanup);
const topic=(data:Dict,source:Topic['source']='observation'):Topic=>({data,received_at:100,age_seconds:.1,stale:false,source,sim_stamp:10});
describe('operational components',()=>{
  it('shows an updated posterior and keeps inference separate from truth',()=>{
    const view=render(<BeliefPanel topic={topic({edge:'trail_a',posterior:{open_safe:.8,blocked:.1,open_hazardous:.05,unknown_stale:.05},p_mismatch:.8,uncertainty:.6},'inference')} decision={{}}/>);
    expect(screen.getByTestId('mismatch').textContent).toBe('80%');
    view.rerender(<BeliefPanel topic={topic({edge:'trail_a',posterior:{open_safe:.1,blocked:.8,open_hazardous:.05,unknown_stale:.05},p_mismatch:.1,uncertainty:.6},'inference')} decision={{}}/>);
    expect(screen.getByTestId('mismatch').textContent).toBe('10%');expect(screen.getByText(/not their hidden true belief/)).toBeTruthy();
  });
  it('acknowledges the exact warning and dismisses without claiming hazard removal',()=>{
    const send=vi.fn();render(<WarningCard record={{warning:{id:'B4-7',hazard_id:'tree',edge:'trail_a',action:'explicit',stamp:8,explanation:'Observed tree'},acknowledged:false,dismissed:false}} send={send} disabled={false}/>);
    fireEvent.click(screen.getByRole('button',{name:'Acknowledge'}));expect(send).toHaveBeenCalledWith({command:'acknowledge_warning',warning_id:'B4-7',hazard_id:'tree'});
    fireEvent.click(screen.getByRole('button',{name:'Dismiss'}));expect(send).toHaveBeenCalledWith({command:'dismiss_warning',warning_id:'B4-7'});expect(screen.getByText(/Dismissal does not clear/)).toBeTruthy();
  });
  it('renders physical hazard updates and an unavailable edge',()=>{
    const params={nodes:{a:[0,0],b:[4,0]},edges:[{id:'road',source:'a',target:'b'}],goal:'b'};
    const view=render(<WorldMap topics={{}} parameters={params} showTruth={false}/>);
    expect(view.container.querySelector('[data-edge="road"]')?.getAttribute('data-state')).toBe('unknown');
    view.rerender(<WorldMap topics={{'/hri/rescue_route':topic({nodes:params.nodes,edge_states:{road:'blocked'},recommended:[]}),'/hri/observations':topic({hazards:[{id:'tree',kind:'tree',position:[2,0],radius:1}]}),'/human/odom':topic({position:[1,0,0]},'simulation_truth')}} parameters={params} showTruth={false}/>);
    expect(view.container.querySelector('[data-edge="road"]')?.getAttribute('data-state')).toBe('blocked');expect(view.container.querySelector('[data-hazard="tree"]')).toBeTruthy();expect(view.container.querySelector('[data-entity="Crew"]')).toBeTruthy();
  });
  it('handles missing camera and displays error without crashing',()=>{render(<Camera online={true} uav={true}/>);expect(screen.getByText('Waiting for camera frames')).toBeTruthy();});
  it('shows equipment configuration separately from unmodeled measurements and saves metadata',async()=>{
    const onSave=vi.fn().mockResolvedValue(undefined);
    render(<EquipmentRegistryPanel topic={topic({source:'configuration',source_statement:'This is not a sensor feed.',equipment:[
      {equipment_id:'TK-01',category:'storage_tank',gazebo_model:'tank_0',geometry_reference:'world::tank_0',position_m:[6,3.6,1.6],
        contents:{label:'Unspecified product',material_class:'unknown',source:'Not supplied'},
        inventory:{status:'not_modeled',value:null,unit:null},temperature:{status:'not_modeled',value:null,unit:'degC'},
        pressure:{status:'not_modeled',value:null,unit:'kPa'},integrity:{status:'not_assessed'},leak:{status:'not_modeled'},
        safety_systems:{status:'not_modeled'},sensors:[]}
    ]},'configuration')} disabled={false} onSave={onSave}/>);
    expect(screen.getAllByText(/not modeled/i).length).toBeGreaterThan(0);
    expect(screen.getByText(/not a sensor feed/i)).toBeTruthy();
    fireEvent.click(screen.getByText('Edit contents metadata'));
    fireEvent.change(screen.getByLabelText('Contents label'),{target:{value:'Operator entered label'}});
    fireEvent.click(screen.getByRole('button',{name:'Save metadata'}));
    await vi.waitFor(()=>expect(onSave).toHaveBeenCalledWith({equipment_id:'TK-01',contents:{
      label:'Operator entered label',material_class:'unknown',source:'Not supplied'
    }}));
  });
});
