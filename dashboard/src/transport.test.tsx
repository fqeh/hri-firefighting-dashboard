import {afterEach,it,expect,vi} from 'vitest';
import {renderHook,act,cleanup} from '@testing-library/react';
import {useTelemetry,sendControl,saveEquipmentContents} from './transport';
class Events {
  static CLOSED=2;static instances:Events[]=[];onmessage:((e:{data:string})=>void)|null=null;onerror:(()=>void)|null=null;readyState=1;
  constructor(){Events.instances.push(this);}close(){this.readyState=2;}
}
const packet=(epoch='one')=>({schema_version:1,epoch,connected:true,updated_at:100,topics:{},timeline:[],warnings:[],validation_errors:[]});
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();Events.instances=[];});
it('reconnects after transport failure and replaces previous epoch',()=>{
  vi.useFakeTimers();vi.stubGlobal('EventSource',Events);const {result}=renderHook(useTelemetry);
  act(()=>Events.instances[0].onmessage?.({data:JSON.stringify(packet())}));expect(result.current.online).toBe(true);
  act(()=>Events.instances[0].onerror?.());expect(result.current.online).toBe(false);
  act(()=>vi.advanceTimersByTime(600));expect(Events.instances).toHaveLength(2);
  act(()=>Events.instances[1].onmessage?.({data:JSON.stringify(packet('two'))}));expect(result.current.state?.epoch).toBe('two');expect(result.current.online).toBe(true);
});
it('treats silent connections and malformed payloads as unavailable',()=>{
  vi.useFakeTimers();vi.stubGlobal('EventSource',Events);const {result}=renderHook(useTelemetry);
  act(()=>Events.instances[0].onmessage?.({data:JSON.stringify(packet())}));act(()=>vi.advanceTimersByTime(5000));expect(result.current.online).toBe(false);
  act(()=>vi.advanceTimersByTime(1000));act(()=>Events.instances.at(-1)?.onmessage?.({data:'{"topics":null}'}));expect(result.current.error).toMatch(/Rejected telemetry/);
});
it('sends typed commands and surfaces server rejection',async()=>{
  const fetch=vi.fn().mockResolvedValue({ok:true,json:async()=>({})});vi.stubGlobal('fetch',fetch);
  await sendControl({command:'approve_route',revision:3});expect(JSON.parse(fetch.mock.calls[0][1].body)).toEqual({command:'approve_route',revision:3});
  fetch.mockResolvedValue({ok:false,json:async()=>({error:'Stale route'})});await expect(sendControl({command:'approve_route',revision:2})).rejects.toThrow('Stale route');
});
it('saves equipment configuration metadata through the session API',async()=>{
  const fetch=vi.fn().mockResolvedValue({ok:true,json:async()=>({accepted:true})});vi.stubGlobal('fetch',fetch);
  const update={equipment_id:'TK-01',contents:{label:'Unspecified product',material_class:'unknown',source:'Operator supplied'}};
  await saveEquipmentContents(update);
  expect(fetch.mock.calls[0][0]).toBe('/api/session/equipment-registry');
  expect(JSON.parse(fetch.mock.calls[0][1].body)).toEqual(update);
  fetch.mockResolvedValue({ok:false,json:async()=>({error:'Stop the active experiment'})});
  await expect(saveEquipmentContents(update)).rejects.toThrow('Stop the active experiment');
});
