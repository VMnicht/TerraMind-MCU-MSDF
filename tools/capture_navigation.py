"""Capture/validate the physical navigation UART, with raw evidence and summary."""
import argparse
import json
import math
from pathlib import Path
import time
import serial
from navigation_protocol import Parser,StreamState,SIMULATED

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--port',required=True)
    ap.add_argument('--seconds',type=float,default=30);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--idle-stop',type=float,default=0,help='Stop after this much silence, after the first frame')
    args=ap.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    parser=Parser();stream=StreamState();valid=simulated=0;first=last=None;ages=[];times=[]
    with serial.Serial(args.port,460800,timeout=.02) as port,args.output.with_suffix('.bin').open('wb') as raw:
        start=time.monotonic();print('Serial capture ready: '+args.port,flush=True)
        while time.monotonic()-start<args.seconds:
            data=port.read(min(65536,max(1,port.in_waiting)));now=time.monotonic()
            raw.write(data)
            for p in parser.feed(data,now):
                usable,_=stream.accept(p,now)
                if first is None:first=now
                last=now;times.append(p.state_time)
                valid+=int(usable);simulated+=bool(p.status&SIMULATED)
                if usable:ages.append(p.age_ms)
            if args.idle_stop and last is not None and now-last>args.idle_stop:break
    result=dict(port=args.port,frames=parser.frames,valid_measurements=valid,simulated_frames=simulated,
        crc_errors=parser.crc_errors,version_errors=parser.version_errors,discarded_bytes=parser.discarded,
        incomplete_timeouts=parser.timeouts,missing=stream.missing,duplicates=stream.duplicates,sessions=stream.sessions,
        receive_hz=(parser.frames-1)/(last-first) if first is not None and last>first else 0,
        measurement_hz=(len(times)-1)/(times[-1]-times[0]) if times and times[-1]>times[0] else 0,
        min_age_ms=min(ages) if ages else None,max_age_ms=max(ages) if ages else None,
        last_packet=vars(stream.latest) if stream.latest else None)
    args.output.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
