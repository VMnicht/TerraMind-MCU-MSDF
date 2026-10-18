"""Read live H7 diagnostics over SWD without halting; optional verified deployment.

Use the pyocd environment described in doc/navigation_vehicle_test.md.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import struct
import time
from elftools.elf.elffile import ELFFile
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
from pyocd.flash.file_programmer import FileProgrammer

def unwrap(die):
    while die.tag in ('DW_TAG_typedef','DW_TAG_const_type','DW_TAG_volatile_type'):
        die=die.get_DIE_from_attribute('DW_AT_type')
    return die

def size(die):
    die=unwrap(die)
    if 'DW_AT_byte_size' in die.attributes:return die.attributes['DW_AT_byte_size'].value
    if die.tag=='DW_TAG_pointer_type':return die.cu['address_size']
    if die.tag=='DW_TAG_array_type':return count(die)*size(die.get_DIE_from_attribute('DW_AT_type'))
    raise ValueError('Unsupported type size: '+die.tag)

def count(die):
    child=next(c for c in die.iter_children() if c.tag=='DW_TAG_subrange_type')
    if 'DW_AT_count' in child.attributes:return child.attributes['DW_AT_count'].value
    return child.attributes['DW_AT_upper_bound'].value+1

def decode(die,data,offset=0):
    die=unwrap(die)
    if die.tag=='DW_TAG_structure_type':
        return {c.attributes['DW_AT_name'].value.decode():decode(c.get_DIE_from_attribute('DW_AT_type'),data,
            offset+c.attributes['DW_AT_data_member_location'].value) for c in die.iter_children() if c.tag=='DW_TAG_member'}
    if die.tag=='DW_TAG_array_type':
        element=die.get_DIE_from_attribute('DW_AT_type');step=size(element)
        return [decode(element,data,offset+i*step) for i in range(count(die))]
    n=size(die);raw=data[offset:offset+n]
    encoding=die.attributes.get('DW_AT_encoding')
    if encoding and encoding.value==4:return struct.unpack('<f' if n==4 else '<d',raw)[0]
    return int.from_bytes(raw,'little',signed=bool(encoding and encoding.value in (5,6)))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--probe',required=True);ap.add_argument('--flash',action='store_true')
    ap.add_argument('--backup-manifest',type=Path);ap.add_argument('--seconds',type=float,default=15)
    ap.add_argument('--image',type=Path,default=Path('MDK-ARM/TerraMind_MSDF/TerraMind_MSDF.axf'))
    ap.add_argument('--output',type=Path,default=Path('build-navigation/vehicle-board.json'))
    args=ap.parse_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    image=args.image;hexfile=image.with_suffix('.hex');symbols={};types={}
    names=('g_navigation_debug','g_gyro_calibration_debug','g_um982_debug_position','g_um982_debug_heading',
           'g_um982_debug_stats','g_monitor_debug_tx','g_boot_stage','g_um982_debug_flags','g_imu_debug_stage','g_capture_debug')
    with image.open('rb') as f:
        elf=ELFFile(f);table=elf.get_section_by_name('.symtab')
        for name in names:
            s=table.get_symbol_by_name(name)[0];symbols[name]=(s['st_value'],s['st_size'])
        for cu in elf.get_dwarf_info().iter_CUs():
            for die in cu.iter_DIEs():
                name=die.attributes.get('DW_AT_name')
                if die.tag=='DW_TAG_variable' and name and name.value.decode() in names and 'DW_AT_type' in die.attributes:
                    types[name.value.decode()]=die.get_DIE_from_attribute('DW_AT_type')
        with ConnectHelper.session_with_chosen_probe(unique_id=args.probe,target_override='stm32h743xx',frequency=1000000,
            options={'auto_unlock':False,'resume_on_disconnect':True,'connect_mode':'attach','reset_type':'sysresetreq',
                     'hide_programming_progress':True}) as session:
            target=session.target
            if target.read32(0x5c001000)&0xfff!=0x450 or target.read16(0x1ff1e880)!=2048:raise ValueError('Unexpected MCU')
            if args.flash:
                if not args.backup_manifest:raise ValueError('Verified backup manifest required')
                backup=json.loads(args.backup_manifest.read_text());data=Path(backup['backup']).read_bytes()
                if backup['probe']!=args.probe or len(data)!=2097152 or hashlib.sha256(data).hexdigest()!=backup['sha256']:
                    raise ValueError('Original Flash backup verification failed')
                target.halt();print('Programming vehicle firmware (real inputs).',flush=True)
                FileProgrammer(session,chip_erase='sector',keep_unwritten=True,trust_crc=False).program(str(hexfile))
                expected=IntelHex(str(hexfile))
                for start,end in expected.segments():
                    if bytes(target.read_memory_block8(start,end-start))!=expected.tobinarray(start=start,end=end-1).tobytes():
                        raise ValueError('Flash readback mismatch')
                target.reset(Target.ResetType.SYSRESETREQ);print('Flash verified; running.',flush=True)
            records=[];start=time.monotonic()
            while time.monotonic()-start<args.seconds:
                record={'elapsed_s':time.monotonic()-start}
                for name,(address,n) in symbols.items():
                    raw=bytes(target.read_memory_block8(address,n));record[name]=decode(types[name],raw)
                records.append(record)
                print(json.dumps({k:record[k] for k in ('elapsed_s','g_navigation_debug','g_gyro_calibration_debug')},ensure_ascii=False),flush=True)
                time.sleep(1)
    result={'utc':datetime.now(timezone.utc).isoformat(),'image':str(image.resolve()),
            'image_sha256':hashlib.sha256(hexfile.read_bytes()).hexdigest(),'flashed':args.flash,'records':records}
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__':main()
