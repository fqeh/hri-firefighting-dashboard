import {useEffect,useState} from 'react';
import {parseState,parseSession,type State,type Session,type Control,type SessionControl} from './contracts';

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
    function connect(){
      if(disposed)return;
      source=new EventSource('/api/events');lastMessage=Date.now();
      const fail=(reason:string)=>{
        source?.close();setOnline(false);setError(reason);
        if(timer)clearTimeout(timer);
        timer=setTimeout(connect,Math.min(10000,500*2**Math.min(attempt++,5)));
      };
      source.onmessage=e=>{try{const data=parseState(JSON.parse(e.data));lastMessage=Date.now();attempt=0;setState(data);setOnline(true);setError('');}catch(e){fail(`Rejected telemetry: ${String(e)}`);}};
      source.onerror=()=>fail('ROS gateway disconnected. Reconnecting automatically…');
    }
    connect();
    const heartbeat=setInterval(()=>{if(Date.now()-lastMessage>4000&&source?.readyState!==EventSource.CLOSED){source?.close();setOnline(false);setError('Telemetry heartbeat timed out. Reconnecting…');timer=setTimeout(connect,1000);}},1000);
    return()=>{disposed=true;source?.close();clearInterval(heartbeat);if(timer)clearTimeout(timer);};
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
