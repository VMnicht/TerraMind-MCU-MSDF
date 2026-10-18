import math
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from navigation_protocol import *

GOLDEN=bytes.fromhex('AA55 019D 01000000 0000000000408F40 000000000000E03F 0000000000000040 '
    '0000A041 00000000 0000803F 00000000 00000000 00000000 DB0FC93F F304353F '
    '00000000 00000000 F304353F C800 3D23')
def alter(frame,offset,data):
    b=bytearray(frame);b[offset:offset+len(data)]=data;b[78:]=crc16(b[2:78]).to_bytes(2,'little');return bytes(b)

class ProtocolTest(unittest.TestCase):
    def test_golden(self):
        self.assertEqual(len(GOLDEN),80);self.assertEqual(crc16(b'123456789'),0x29b1)
        p=Packet.decode(GOLDEN);self.assertEqual(p.sequence,1);self.assertEqual(p.validity(),'')
        self.assertAlmostEqual(p.attitude[2],math.pi/2,places=6)
    def test_all_fragmentations(self):
        for split in range(81):
            p=Parser();a=p.feed(GOLDEN[:split],0);a+=p.feed(GOLDEN[split:],.001)
            self.assertEqual(len(a),1);self.assertEqual(p.buffer,b'')
    def test_noise_embedded_headers_corruption_and_bound(self):
        p=Parser();bad=bytearray(GOLDEN);bad[10:12]=b'\xaa\x55'
        frames=p.feed(b'noise'*10000+bad+GOLDEN+GOLDEN[:30],0)
        self.assertEqual(len(frames),1);self.assertGreater(p.crc_errors,0);self.assertLess(len(p.buffer),80)
        p.feed(b'',.101);self.assertEqual(p.timeouts,1)
        self.assertEqual(len(p.feed(GOLDEN,.102)),1)
    def test_version_and_nonfinite(self):
        p=Parser();self.assertEqual(len(p.feed(alter(GOLDEN,2,b'\x02')+GOLDEN,0)),1)
        self.assertEqual(p.version_errors,1)
        for offset,data in [(16,struct.pack('<d',float('nan'))),(60,struct.pack('<f',2.0))]:
            self.assertTrue(Packet.decode(alter(GOLDEN,offset,data)).validity())
    def test_validity_and_freshness(self):
        s=StreamState();p=Packet.decode(GOLDEN);self.assertEqual(s.accept(p,0),(True,False))
        self.assertEqual(s.accept(p,.09),(False,False));self.assertNotEqual(s.health(.101),'导航正常')
        heartbeat=Packet.decode(alter(alter(GOLDEN,3,b'\x04'),4,struct.pack('<I',2)))
        self.assertFalse(s.accept(heartbeat,1)[0]);self.assertIn('等待',s.health(1.1))
        self.assertIn('连接超时',s.health(4.1))
        for age in (501,65535):self.assertEqual(Packet.decode(alter(GOLDEN,76,struct.pack('<H',age))).validity(),'结果过期')
        self.assertEqual(Packet.decode(alter(GOLDEN,3,b'\x03')).validity(),'导航故障')
    def test_wrap_reset_and_skips(self):
        s=StreamState()
        s.accept(Packet.decode(alter(GOLDEN,4,struct.pack('<I',0xffffffff))),0)
        _,reset=s.accept(Packet.decode(alter(GOLDEN,4,struct.pack('<I',0))),.01);self.assertFalse(reset)
        s.accept(Packet.decode(alter(GOLDEN,4,struct.pack('<I',3))),.02);self.assertEqual(s.missing,2)
        _,reset=s.accept(Packet.decode(GOLDEN),.03);self.assertTrue(reset)
        self.assertEqual(s.sessions,1)
        _,reset=s.accept(Packet.decode(alter(demo_frame(2,999),3,b'\x9d')),.04);self.assertTrue(reset)
    def test_demo_marked(self):
        self.assertTrue(Packet.decode(demo_frame(10,1000)).status&SIMULATED)

if __name__=='__main__':unittest.main()
