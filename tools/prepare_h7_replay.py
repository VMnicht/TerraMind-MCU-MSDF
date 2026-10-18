"""Export verified mode-3 capture into Flash arrays and a separate Keil test target.

Uses the documented +Y/-X/+Z, [0,-0.15,0] m installation. Raw logs are read only.
Reference results come from the unchanged offline aiding executable.
"""
from __future__ import annotations
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from kf_gins_bridge import (KfOptions, VEHICLE_CONFIG, prepare_log, _parse_log,
                            _config, _write_rows, _aiding_rows, _aided_executable)
from sync_timeline import analyze_sync, GPS_WEEK_MS

ROOT = Path(__file__).resolve().parents[1]

def number(x):
    if not math.isfinite(x):
        raise ValueError('Nonfinite exported value')
    return format(x, '.17g')

def vector(xs):
    return '{' + ','.join(number(x) for x in xs) + '}'

def create_project(folder: Path):
    original = ROOT/'MDK-ARM/TerraMind_MSDF.uvprojx'
    tree = ET.parse(original)
    for node in tree.iter('IncludePath'):
        if node.text:
            node.text = ';'.join(str((original.parent/p.replace('\\','/')).resolve())
                                 for p in node.text.split(';') if p)
    source = (ROOT/'Core/Src/freertos.c').read_text()
    source = source.replace('../../TerraMind_FW/App/app_tasks.h',
                            (ROOT/'TerraMind_FW/App/app_tasks.h').as_posix())
    source = source.replace('#include "../../TerraMind_FW/App/app_navigation.h"', '')
    source = source.replace('  AppNavigation_StartTask();', '')
    source = source.replace('const osThreadAttr_t defaultTask_attributes = {',
                            'static StaticTask_t replay_task_cb;\n'
                            'static StackType_t replay_task_stack[6144];\n'
                            'const osThreadAttr_t defaultTask_attributes = {')
    source = source.replace('.stack_size = 1536 * 4,',
                            '.cb_mem = &replay_task_cb,\n  .cb_size = sizeof(replay_task_cb),\n'
                            '  .stack_mem = replay_task_stack,\n  .stack_size = sizeof(replay_task_stack),')
    (folder/'replay_freertos.c').write_text(source)
    for node in tree.iter('FilePath'):
        resolved = (original.parent/node.text.replace('\\','/')).resolve()
        if resolved.name == 'app_tasks.cpp':
            resolved = ROOT/'tests/navigation/h7_benchmark.cpp'
        elif resolved == ROOT/'Core/Src/freertos.c':
            resolved = folder/'replay_freertos.c'
        node.text = str(resolved)
    target = tree.find('./Targets/Target')
    target.find('TargetName').text = 'Navigation Replay 480MHz'
    common = target.find('./TargetOption/TargetCommonOption')
    common.find('OutputDirectory').text = str(folder/'firmware')+'\\'
    common.find('OutputName').text = 'navigation_replay'
    common.find('ListingPath').text = str(folder/'firmware')+'\\'
    for tag in ('BeforeCompile','BeforeMake','AfterMake'):
        for node in common.findall('./'+tag+'/*'):
            if node.tag.startswith('RunUserProg'): node.text='0'
    group = ET.SubElement(target.find('Groups'), 'Group')
    ET.SubElement(group,'GroupName').text='Replay fixture (test only)'
    files=ET.SubElement(group,'Files');f=ET.SubElement(files,'File')
    ET.SubElement(f,'FileName').text='replay_data.cpp'
    ET.SubElement(f,'FileType').text='8'
    ET.SubElement(f,'FilePath').text=str(folder/'replay_data.cpp')
    for node in target.findall('./TargetOption/TargetArmAds/Cads/VariousControls/IncludePath'):
        node.text += ';'+str(ROOT/'tests/navigation')+';'+str(ROOT/'TerraMind_FW/Navigation')
    ET.indent(tree)
    tree.write(folder/'navigation_replay.uvprojx',encoding='utf-8',xml_declaration=True)

def export(log: Path, folder: Path):
    log=log.resolve();folder=folder.resolve();folder.mkdir(parents=True,exist_ok=True)
    options=KfOptions(axis_forward='+y',axis_right='-x',axis_down='+z',lever_right_m=-.15,
                      heading_offset_deg=-90,config_path=str(VEHICLE_CONFIG),
                      heading_aiding=True,velocity_aiding=True)
    data=prepare_log(log,options)
    raw,_,_,rejected,_=_parse_log(log)
    lines=log.read_text(encoding='ascii').splitlines()
    trailing_partial=bool(lines and lines[-1].startswith('I,') and len(lines[-1].split(','))!=13)
    if rejected != int(trailing_partial):
        raise ValueError('Only one trailing partial IMU record may be excluded; invalid interior input')
    if not data.hardware_sync or any(s.capture_ticks is None for s in raw):
        raise ValueError('H7 replay requires complete hardware timestamps')
    sync=analyze_sync(log)
    def arrival(ticks): return (sync.tick_to_gps_ms(ticks)-data.week*GPS_WEEK_MS)/1000
    imu_arrivals={(t.mcu_ms,t.count):arrival(t.uart_end_ticks) for t in sync.imu}
    gnss_arrivals={};pending=None
    for line in lines:
        if line.startswith('N,'):
            _,mcu,message=line.split(',',2);pending=None
            if message.startswith(('#BESTNAVA,','#UNIHEADINGA,')):
                header=message.split(';',1)[0].split(',')
                pending=(int(mcu),int(header[5]))
        elif line.startswith('R,'):
            f=line.split(',')
            if pending and int(f[1])==pending[0]:
                epoch=pending[1]/1000;t=arrival((int(f[2])<<32)|int(f[3]))
                gnss_arrivals[epoch]=max(gnss_arrivals.get(epoch,t),t)
            pending=None
    config=_config(data,options,folder)
    def setting(key): return ast.literal_eval(re.search(r'^'+key+r':\s*(.*)$',config,re.M)[1])
    start=float(setting('starttime'))
    # Match the offline file's decimal representation and startup seed selection.
    rows=[tuple(float(format(x,'.12f')) for x in row) for row in data.imu_rows]
    seed=next(i for i,row in enumerate(rows) if row[0]>=start)
    observations=[r for r in _aiding_rows(data,options) if r[0]>start and r[0]>=rows[seed][0] and (r[1] or r[8] or r[15])]
    arrivals=[imu_arrivals[(s.mcu_ms,s.count)] for s in raw[seed:]]
    obs_arrivals=[gnss_arrivals[r[0]] for r in observations]
    for values in (arrivals,obs_arrivals):
        if any(b<a for a,b in zip(values,values[1:])): raise ValueError('Arrival order is not monotonic')
    for r,t in zip(rows[seed:],arrivals):
        if t<r[0]: raise ValueError('IMU arrival precedes measurement')
    if any(t<r[0] or t-r[0]>.19 for r,t in zip(observations,obs_arrivals)):
        raise ValueError('Observation outside the 200 ms replay delay budget')
    out=['#include "replay_data.h"']
    digest=hashlib.sha256(log.read_bytes()).hexdigest()
    out.append('const char replay_source_sha256[]="'+digest+'";')
    for name,key in [('position','initpos'),('velocity','initvel'),('attitude','initatt')]:
        values=setting(key)
        if name=='position': values[:2]=[math.radians(v) for v in values[:2]]
        if name=='attitude': values=[math.radians(v) for v in values]
        out.append('const double replay_initial_'+name+'[3]='+vector(values)+';')
    out.append('const ReplayImu replay_imu[]={')
    for i in range(seed,len(rows)):
        row=rows[i];dt=row[0]-rows[i-1][0] if i else .005
        out.append('{{'+number(row[0])+','+number(dt)+','+vector(row[1:4])+','+vector(row[4:7])+'},'+number(arrivals[i-seed])+'},')
    out.append('};\nconst unsigned replay_imu_count=sizeof(replay_imu)/sizeof(replay_imu[0]);')
    out.append('const ReplayObservation replay_observations[]={')
    for row,t in zip(observations,obs_arrivals):
        fields=int(bool(row[1]))+2*int(bool(row[8]))+4*int(bool(row[15]))
        out.append('{{'+number(row[0])+','+str(fields)+','+vector(row[2:5])+','+vector(row[5:8])+','+
                   vector(row[9:12])+','+vector(row[12:15])+','+','.join(number(v) for v in row[16:19])+'},'+number(t)+'},')
    out.append('};\nconst unsigned replay_observation_count=sizeof(replay_observations)/sizeof(replay_observations[0]);')
    (folder/'replay_data.cpp').write_text('\n'.join(out)+'\n',encoding='ascii')
    _write_rows(folder/'imu.txt',data.imu_rows);_write_rows(folder/'gnss.txt',data.gnss_rows)
    # Independent reference: existing offline path remains unchanged.
    executable=_aided_executable()
    for mask in (1,7):
        selected=[]
        for r in _aiding_rows(data,options):
            r=list(r)
            if mask==1:r[8]=r[15]=0
            if r[1] or r[8] or r[15]:selected.append(r)
        _write_rows(folder/'aiding.txt',selected,'.17g')
        (folder/'run.yaml').write_text(config,encoding='utf-8')
        result=subprocess.run([str(executable),str(folder/'run.yaml'),str(folder/'aiding.txt')],
                              capture_output=True,text=True,check=True)
        (folder/f'reference_{mask}.nav').write_bytes((folder/'KF_GINS_Navresult.nav').read_bytes())
        (folder/f'reference_{mask}.log').write_text(result.stdout+result.stderr)
    manifest=dict(source=str(log),sha256=digest,sidecar_sha256=hashlib.sha256(Path(str(log)+'.capture.json').read_bytes()).hexdigest(),
                  reference_exe_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                  imu_frames=len(rows)-seed,processed_frames=len(rows)-seed-1,observation_epochs=len(observations),
                  excluded_trailing_partial_imu=trailing_partial,seed_original_index=seed,
                  duration_s=rows[-1][0]-rows[seed][0],cpu_hz=480000000,output_hz=100,buffer_delay_s=.2,
                  max_merged_observation_latency_s=max(t-r[0] for t,r in zip(obs_arrivals,observations)),
                  installation=dict(axes=['+y','-x','+z'],lever_frd_m=[0,-.15,0],heading_offset_deg=-90),
                  startup_calibration_applied=True,notes=data.warnings)
    (folder/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    create_project(folder)
    print(json.dumps({k:v for k,v in manifest.items() if k not in ('notes','installation')},indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--output',type=Path,default=ROOT/'build-navigation/h7-replay')
    args=parser.parse_args();export(args.log,args.output)
