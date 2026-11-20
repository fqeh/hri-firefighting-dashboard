import {describe,it,expect} from 'vitest';
import {parseState,type State,type Topic,type Dict,parseSession} from './contracts';
export const topic=(data:Dict,source:Topic['source']='observation'):Topic=>({data,received_at:100,age_seconds:.1,stale:false,source,sim_stamp:10});
export const state=():State=>({schema_version:1,epoch:'test-fixture',connected:true,updated_at:100,topics:{'/clock':topic({seconds:10})},timeline:[],warnings:[],validation_errors:[]});
describe('wire contracts',()=>{
  it('accepts a partial stream without inventing unavailable topics',()=>expect(parseState(state()).topics['/robot/odom']).toBeUndefined());
  it('rejects malformed envelope, nonfinite numbers and bogus posteriors',()=>{
    expect(()=>parseState({topics:[]})).toThrow();const s=state();s.topics['/clock'].data.seconds=Infinity;expect(()=>parseState(s)).toThrow();
    s.topics['/clock'].data.seconds=10;s.topics['/hri/belief']=topic({posterior:{open_safe:.8,blocked:.8,open_hazardous:0,unknown_stale:0}});expect(()=>parseState(s)).toThrow(/posterior/);
  });
  it('rejects unknown schema and malformed topic envelopes',()=>{expect(()=>parseState({...state(),schema_version:2})).toThrow();const s=state();s.topics['/clock'].stale='false' as never;expect(()=>parseState(s)).toThrow();});
  it('validates equipment configuration without treating it as observation data',()=>{
    const s=state();s.topics['/hri/equipment_registry']=topic({schema_version:1,equipment:[{
      equipment_id:'TK-01',gazebo_model:'tank_0',position_m:[6,3.6,1.6],
      contents:{label:'Unspecified product',material_class:'unknown',source:'Not supplied'}
    }]},'configuration');
    expect(parseState(s).topics['/hri/equipment_registry'].source).toBe('configuration');
    s.topics['/hri/equipment_registry'].data.equipment=[{equipment_id:'TK-01'}];
    expect(()=>parseState(s)).toThrow(/equipment/);
  });
  it('rejects external artifact links',()=>expect(()=>parseSession({schema_version:1,session:{},parameters:{},busy:false,cases:{forest:[],oil_facility:[]},evaluation_enabled:false,artifacts:[{name:'fake',url:'https://example.com'}]})).toThrow());
});
