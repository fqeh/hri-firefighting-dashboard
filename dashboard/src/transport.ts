import {useEffect,useState} from 'react';
import {parseState,parseSession,parseScene,type State,type Session,type Scene,type Control,type SessionControl} from './contracts';

export async function sendControl(control:Control|SessionControl,session=false):Promise<void>{
  const response=await fetch(session?'/api/session/control':'/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(control),signal:AbortSignal.timeout(8000)});
  const body=await response.json();
  if(!response.ok)throw Error(typeof body.error==='string'?body.error:`Control rejected (${response.status})`);
}

export type EquipmentContentsUpdate={equipment_id:string;contents:{label:string;material_class:string;source:string}};
export async function saveEquipmentContents(update:EquipmentContentsUpdate):Promise<void>{
  const response=await fetch('/api/session/equipment-registry',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(update),signal:AbortSignal.timeout(8000)});
  const body=await response.json();
  if(!response.ok)throw Error(typeof body.error==='string'?body.error:`Registry update rejected (${response.status})`);
}

export function useTelemetry(){
  const [state,setState]=useState<State|null>(null),[online,setOnline]=useState(false),[error,setError]=useState('');
  useEffect(()=>{
    let disposed=false,source:EventSource|null=null,lastMessage=0,timer:ReturnType<typeof setTimeout>|undefined,attempt=0;
    let received=false,polling=false,pollTimer:ReturnType<typeof setTimeout>|undefined;
    const accept=(raw:unknown)=>{const data=parseState(raw);lastMessage=Date.now();attempt=0;setState(data);setOnline(true);setError('');};
    // Some proxies (e.g. Cloudflare quick tunnels used to share the demo) hold back
    // SSE bodies entirely; the gateway serves the same envelope at /api/state.
    async function poll(){
      if(disposed)return;
      try{
        const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(4000)});
        if(!response.ok)throw Error(`HTTP ${response.status}`);
        const body:unknown=await response.json();
        try{accept(body);}catch(e){setOnline(false);setError(`Rejected telemetry: ${String(e)}`);}
      }catch{if(!disposed){setOnline(false);setError('ROS gateway unreachable. Retrying…');}}
      finally{if(!disposed)pollTimer=setTimeout(()=>void poll(),500);}
    }
    function connect(){
      if(disposed)return;
      source=new EventSource('/api/events');lastMessage=Date.now();received=false;
      const fail=(reason:string)=>{
        source?.close();setOnline(false);setError(reason);
        if(timer)clearTimeout(timer);
        timer=setTimeout(connect,Math.min(10000,500*2**Math.min(attempt++,5)));
      };
      source.onmessage=e=>{received=true;try{accept(JSON.parse(e.data));}catch(e){fail(`Rejected telemetry: ${String(e)}`);}};
      source.onerror=()=>fail('ROS gateway disconnected. Reconnecting automatically…');
    }
    connect();
    const heartbeat=setInterval(()=>{
      if(polling||Date.now()-lastMessage<=4000||source?.readyState===EventSource.CLOSED)return;
      source?.close();
      if(!received){polling=true;void poll();return;}  // stream opened but never delivered: switch to polling
      setOnline(false);setError('Telemetry heartbeat timed out. Reconnecting…');timer=setTimeout(connect,1000);
    },1000);
    return()=>{disposed=true;source?.close();clearInterval(heartbeat);if(timer)clearTimeout(timer);if(pollTimer)clearTimeout(pollTimer);};
  },[]);
  return {state,online,error};
}

export function useSession(){
  const [session,setSession]=useState<Session|null>(null),[error,setError]=useState('');
  useEffect(()=>{
    let disposed=false;const controller=new AbortController();let timer:ReturnType<typeof setTimeout>;
    async function poll(){
      try{
        const response=await fetch('/api/session',{signal:AbortSignal.any([controller.signal,AbortSignal.timeout(4000)])});
        if(!response.ok)throw Error('Session supervisor unavailable');
        const data=parseSession(await response.json());if(!disposed){setSession(data);setError('');}
      }catch{if(!disposed)setError('Session supervisor offline. Start scripts/dashboard_session.py to enable scenario and experiment controls.');}
      finally{if(!disposed)timer=setTimeout(poll,1000);}
    }
    void poll();return()=>{disposed=true;controller.abort();clearTimeout(timer);};
  },[]);
  return {session,error};
}

// World geometry changes only when a scenario is (re)loaded, so it is fetched per run.
export function useScene(key:string){
  const [scene,setScene]=useState<Scene|null>(null),[error,setError]=useState('');
  useEffect(()=>{
    if(!key)return;
    let disposed=false,timer:ReturnType<typeof setTimeout>|undefined;const controller=new AbortController();
    async function load(attempt=0){
      try{
        const response=await fetch('/api/session/scene',{signal:AbortSignal.any([controller.signal,AbortSignal.timeout(10000)])});
        const body=await response.json();
        if(!response.ok)throw Error(typeof body.error==='string'?body.error:`Scene unavailable (${response.status})`);
        const data=parseScene(body);if(!disposed){setScene(data);setError('');}
      }catch(e){if(!disposed){setError(String(e instanceof Error?e.message:e));timer=setTimeout(()=>void load(attempt+1),Math.min(15000,1000*2**attempt));}}
    }
    void load();return()=>{disposed=true;controller.abort();if(timer)clearTimeout(timer);};
  },[key]);
  return {scene,error};
}
