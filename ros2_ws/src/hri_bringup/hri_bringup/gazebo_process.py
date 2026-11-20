"""Own the Gazebo child and request service-based shutdown outside signal handlers.

The simulation command and telemetry are unchanged. Gazebo's ServerControl stop
service avoids interrupting a render/transport thread inside its signal handler.
Signals are a bounded fallback if the service is unavailable.
"""
import os,signal,subprocess,sys,time


def main():
    stopping=False
    def requested(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,requested);signal.signal(signal.SIGTERM,requested)
    process=subprocess.Popen(sys.argv[1:],start_new_session=True)
    while process.poll() is None and not stopping:time.sleep(.05)
    if process.poll() is None:
        try:
            result=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl',
                '--reptype','gz.msgs.Boolean','--timeout','2000','--req','stop: true'],capture_output=True,text=True,timeout=3)
            print('Gazebo service shutdown:',result.stdout.strip() or result.stderr.strip(),flush=True)
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
                if process.poll() is not None:break
                os.killpg(process.pid,sig)
                try:process.wait(timeout=2)
                except subprocess.TimeoutExpired:pass
    raise SystemExit(process.returncode if process.returncode is not None else 1)
