"""Flash and measure the isolated replay firmware over a specified CMSIS-DAP probe.

Requires pyocd, pyelftools, and intelhex (installed in build-navigation/probe-venv
for this workstation). Requires a verified full original Flash backup manifest.
Does not alter option bytes, unlock read protection, or erase the entire chip.
"""
import argparse
import ctypes as C
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
from elftools.elf.elffile import ELFFile
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
from pyocd.flash.file_programmer import FileProgrammer

class State(C.LittleEndianStructure):
    _fields_=[(n,C.c_double*3) for n in ('position','velocity','attitude','gyro_bias','accel_bias','gyro_scale','accel_scale')]
class Output(C.LittleEndianStructure):
    _fields_=[('sequence',C.c_uint64)]+[(n,C.c_double) for n in ('state_time_s','publish_time_s','age_s','last_gnss_time_s')]+[
        ('state',State),('quaternion_bn',C.c_double*4),('std',C.c_double*21),('issue_flags',C.c_uint32)]
class Stats(C.LittleEndianStructure):
    _fields_=[(n,C.c_uint64) for n in ('imu_accepted','imu_processed','gnss_accepted','gnss_updates','outputs')]+[
        (n,C.c_uint32) for n in ('imu_queue_depth','gnss_queue_depth','imu_queue_peak','gnss_queue_peak','rejected_inputs',
        'imu_overflows','gnss_overflows','late_gnss','numerical_faults','skipped_output_slots','issue_flags',
        'last_process_cycles','max_process_cycles')]+[('branch_max_cycles',C.c_uint32*4),('total_process_cycles',C.c_uint64),
        ('branch_count',C.c_uint64*4)]+[(n,C.c_uint64) for n in ('position_updates','velocity_updates','velocity_rejected','heading_updates','heading_rejected')]+[
        (n,C.c_double) for n in ('velocity_nis','heading_nis','last_state_time_s','last_publish_time_s','max_output_age_s')]+[
        ('last_error',C.c_uint8),('running',C.c_uint8),('faulted',C.c_uint8)]
class Pass(C.LittleEndianStructure):
    _fields_=[('stats',Stats),('last_output',Output)]+[(n,C.c_double) for n in ('elapsed_s','processing_hz','output_hz','max_publish_interval_s')]+[
        ('stack_free_bytes',C.c_uint32),('branches_over_5ms',C.c_uint32)]
class Report(C.LittleEndianStructure):
    _fields_=[(n,C.c_uint32) for n in ('phase','cpu_hz','context_bytes','error','icache_enabled','dcache_enabled','source_sha256')]+[
        ('fast',Pass),('realtime',Pass),('serial',C.c_uint32*5),('serial_max_cycles',C.c_uint32),('last_packet',C.c_uint8*80)]

def decode(value):
    if isinstance(value,C.Array):return [decode(v) for v in value]
    if isinstance(value,C.Structure):return {n:decode(getattr(value,n)) for n,*_ in value._fields_}
    return value

def symbol(path,name):
    with path.open('rb') as f:
        elf=ELFFile(f);sym=elf.get_section_by_name('.symtab').get_symbol_by_name(name)[0]
        return sym['st_value'],sym['st_size']

def verify_abi(path):
    # Armclang uses short enums here; host-native struct packing alone is unsafe.
    expected={'NavStats':Stats,'NavOutput':Output,'ReplayPassReport':Pass,'ReplayReport':Report}
    with path.open('rb') as f:
        dwarf=ELFFile(f).get_dwarf_info()
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                name=die.attributes.get('DW_AT_name')
                if not name or name.value.decode() not in expected:continue
                name=name.value.decode();model=expected[name]
                if die.tag=='DW_TAG_typedef':die=die.get_DIE_from_attribute('DW_AT_type')
                if die.tag!='DW_TAG_structure_type' or 'DW_AT_byte_size' not in die.attributes:continue
                if die.attributes['DW_AT_byte_size'].value!=C.sizeof(model):raise ValueError('ABI size mismatch: '+name)
                for member in die.iter_children():
                    if member.tag=='DW_TAG_member':
                        field=member.attributes['DW_AT_name'].value.decode()
                        if getattr(model,field).offset!=member.attributes['DW_AT_data_member_location'].value:
                            raise ValueError('ABI offset mismatch: '+name+'.'+field)
                del expected[name]
                if not expected:return
    raise ValueError('Missing ABI debug types: '+str(expected))

def check_final_states(result,folder):
    reference=[float(v) for v in (folder/'reference_7.nav').read_text().splitlines()[-1].split()]
    for name in ('fast','realtime'):
        state=result[name]['last_output']['state']
        position=max(abs(state['position'][j]-(math.radians(reference[2+j]) if j<2 else reference[2+j]))*
                     (6378137 if j<2 else 1) for j in range(3))
        velocity=max(abs(state['velocity'][j]-reference[5+j]) for j in range(3))
        attitude=max(abs(math.atan2(math.sin(state['attitude'][j]-math.radians(reference[8+j])),
                                   math.cos(state['attitude'][j]-math.radians(reference[8+j])))) for j in range(3))
        result[name]['final_reference_difference']={'position_m':position,'velocity_mps':velocity,'attitude_rad':attitude}
        if position>1e-3 or velocity>1e-5 or attitude>1e-6:
            raise ValueError('Board final state differs from offline reference: '+name)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--probe',required=True)
    p.add_argument('--backup-manifest',type=Path,required=True)
    p.add_argument('--name',default='measurement')
    p.add_argument('--no-flash',action='store_true',help='Verify existing image and reset for another measurement')
    args=p.parse_args();folder=args.folder.resolve()
    backup=json.loads(args.backup_manifest.read_text())
    data=Path(backup['backup']).read_bytes()
    if backup['probe']!=args.probe or len(data)!=2097152 or hashlib.sha256(data).hexdigest()!=backup['sha256']:
        raise ValueError('Original Flash backup verification failed')
    axf=folder/'firmware/navigation_replay.axf';hexfile=folder/'firmware/navigation_replay.hex'
    address,size=symbol(axf,'g_nav_replay_report');boot,_=symbol(axf,'g_boot_stage')
    if size!=C.sizeof(Report):raise ValueError(f'Report ABI mismatch: ELF {size}, decoder {C.sizeof(Report)}')
    verify_abi(axf)
    manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    board=folder/'board';board.mkdir(exist_ok=True)
    with ConnectHelper.session_with_chosen_probe(unique_id=args.probe,target_override='stm32h743xx',frequency=1000000,
        options={'auto_unlock':False,'resume_on_disconnect':True,'connect_mode':'attach','reset_type':'sysresetreq',
                 'hide_programming_progress':True}) as session:
        t=session.target;t.halt()
        chip=t.read32(0x5c001000);flash_kib=t.read16(0x1ff1e880)
        if chip&0xfff!=0x450 or flash_kib!=2048:raise ValueError('Unexpected MCU / Flash capacity')
        if not args.no_flash:
            print('Programming isolated replay firmware; original backup verified.',flush=True)
            FileProgrammer(session,chip_erase='sector',keep_unwritten=True,trust_crc=False).program(str(hexfile))
        t.halt();expected=IntelHex(str(hexfile))
        for start,end in expected.segments():
            actual=bytes(t.read_memory_block8(start,end-start))
            if actual!=bytes(expected.tobinarray(start=start,end=end-1)):
                raise RuntimeError('Flash readback verification failed')
        print('Flash readback verified. Resetting and running without halting.',flush=True)
        t.reset(Target.ResetType.SYSRESETREQ)
        deadline=time.monotonic()+180;previous=-1
        while time.monotonic()<deadline:
            phase=t.read32(address)
            if phase!=previous:print('Phase:',phase,flush=True);previous=phase
            if phase in (3,4):break
            time.sleep(1)
        raw=bytes(t.read_memory_block8(address,size))
        result=decode(Report.from_buffer_copy(raw));result['device_id']=hex(chip)
        result['readback_utc']=datetime.now(timezone.utc).isoformat()
        result['boot_stage']=hex(t.read32(boot));result['core_state']=str(t.get_state())
        source=bytes(t.read_memory_block8(result['source_sha256'],65)).split(b'\0')[0].decode('ascii')
        if source!=manifest['sha256']:raise ValueError('Board source fingerprint mismatch')
        result['source_sha256']=source
        result['firmware_sha256']=hashlib.sha256(hexfile.read_bytes()).hexdigest()
        result['probe']=args.probe
        result['original_backup']=backup
        for name in ('fast','realtime'):
            s=result[name]['stats'];hz=result['cpu_hz']
            if hz and s['imu_processed']:
                result[name]['average_process_us']=s['total_process_cycles']/s['imu_processed']/hz*1e6
                result[name]['max_process_us']=s['max_process_cycles']/hz*1e6
                result[name]['branch_max_us']=[v/hz*1e6 for v in s['branch_max_cycles']]
        (board/(args.name+'.bin')).write_bytes(raw)
        if result['phase']==3:check_final_states(result,folder)
        (board/(args.name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        if result['phase']!=3:raise RuntimeError('Replay did not complete; report saved')

if __name__=='__main__':main()
