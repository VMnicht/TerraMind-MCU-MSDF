"""Live navigation v1 dashboard: python tools/navigation_monitor.py [--demo]."""
from __future__ import annotations
import argparse
from collections import deque
import csv
import hashlib
import json
from datetime import datetime
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from navigation_protocol import Parser, StreamState, Packet, demo_frame, SIMULATED, TIME_LOCKED, GNSS_RECENT

CSV_HEADER = ['received_monotonic_s','sequence','status','state_time_s','latitude_rad','longitude_rad',
              'height_m','v_n_mps','v_e_mps','v_d_mps','roll_rad','pitch_rad','yaw_rad',
              'q_w','q_x','q_y','q_z','age_ms','validity']

class Reader(threading.Thread):
    def __init__(self, mode, source, directory, events):
        super().__init__(daemon=True)
        self.mode, self.source, self.directory, self.events = mode, source, directory, events
        self.stop_event = threading.Event()
        self.ui_dropped = 0

    def emit(self, event):
        try: self.events.put_nowait(event)
        except queue.Full:
            try: self.events.get_nowait()
            except queue.Empty: pass
            self.ui_dropped += 1
            self.events.put_nowait(event)

    def run(self):
        raw = csv_file = port = None
        prefix = None; digest = hashlib.sha256(); byte_count = 0; error = None
        try:
            parser = Parser()
            if self.mode == 'serial':
                import serial
                port = serial.Serial(self.source, 460800, timeout=.02, write_timeout=.1)
            elif self.mode == 'replay':
                port = Path(self.source).open('rb')
            self.directory.mkdir(parents=True, exist_ok=True)
            prefix = self.directory / ('navigation_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            metadata = dict(protocol='TerraMind navigation v1',baud=460800,mode=self.mode,source=str(self.source),
                            started=datetime.now().isoformat(),finished=False)
            prefix.with_suffix('.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
            raw = prefix.with_suffix('.bin').open('wb')
            csv_file = prefix.with_suffix('.csv').open('w', encoding='utf-8-sig', newline='')
            writer = csv.writer(csv_file); writer.writerow(CSV_HEADER)
            self.emit(('ready', str(prefix)))
            start = time.monotonic(); sequence = 0; last_flush = start
            while not self.stop_event.is_set():
                if self.mode == 'demo':
                    data = demo_frame(sequence, 1000+sequence*.01); sequence += 1
                    self.stop_event.wait(max(0, start+sequence*.01-time.monotonic()))
                elif self.mode == 'replay':
                    data = port.read(80)
                    if not data: break
                    self.stop_event.wait(.01)
                else:
                    data = port.read(min(4096, max(1, port.in_waiting)))
                now = time.monotonic()
                if data:
                    raw.write(data);digest.update(data);byte_count += len(data)
                for p in parser.feed(data, now):
                    writer.writerow([format(now,'.6f'),p.sequence,p.status,p.state_time,p.latitude,p.longitude,
                        p.height,*p.velocity,*p.attitude,*p.quaternion,p.age_ms,p.validity() or 'valid'])
                    self.emit(('packet', p, now))
                self.emit(('stats', parser.frames, parser.crc_errors, parser.version_errors,
                           parser.discarded, parser.timeouts, self.ui_dropped))
                if now-last_flush >= 1:
                    raw.flush(); csv_file.flush(); last_flush = now
        except Exception as exc:
            error = str(exc)
            self.emit(('error', str(exc)))
        finally:
            for resource in (port, raw, csv_file):
                if resource is not None:
                    try: resource.close()
                    except Exception: pass
            if prefix is not None:
                metadata.update(finished=error is None,ended=datetime.now().isoformat(),error=error,
                    bytes=byte_count,sha256=digest.hexdigest(),frames=parser.frames,crc_errors=parser.crc_errors,
                    version_errors=parser.version_errors,resync_discarded_bytes=parser.discarded,
                    partial_timeouts=parser.timeouts,display_queue_dropped=self.ui_dropped)
                try: prefix.with_suffix('.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
                except OSError as exc: self.emit(('error','记录清单写入失败：'+str(exc)))
            self.emit(('closed',))

class Dashboard:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.reader = None; self.events = queue.Queue(maxsize=2048)
        self.stream = StreamState(); self.path = deque(maxlen=12000)
        self.speeds = deque(maxlen=3000); self.times = deque(maxlen=1000)
        self.origin = None; self.current = None; self.error = ''; self.mode = ''
        self.parser_stats = (0,0,0,0,0,0)
        root.title('TerraMind · H7 导航结果监视器'); root.geometry('1180x820'); root.minsize(1000,700)
        root.configure(bg='#101827')
        style = ttk.Style(); style.theme_use('clam')
        style.configure('TFrame', background='#101827'); style.configure('TLabel', background='#101827', foreground='#dce6f4')
        style.configure('TButton', padding=7); style.configure('TLabelframe', background='#101827', bordercolor='#334155')
        style.configure('TLabelframe.Label', background='#101827', foreground='#94a3b8')
        style.configure('Value.TLabel', font=('Consolas',17,'bold'), foreground='#f8fafc')
        top = ttk.Frame(root, padding=14); top.pack(fill='x')
        ttk.Label(top,text='H7 导航 / 100 Hz',font=('Microsoft YaHei UI',18,'bold')).pack(side='left')
        self.port = tk.StringVar(); self.ports = ttk.Combobox(top,textvariable=self.port,width=13)
        self.ports.pack(side='left',padx=(26,6)); ttk.Button(top,text='刷新',command=self.refresh).pack(side='left')
        ttk.Label(top,text='460800 · 8N1').pack(side='left',padx=10)
        self.connect_button = ttk.Button(top,text='连接',command=self.connect); self.connect_button.pack(side='left')
        ttk.Button(top,text='打开记录',command=self.replay).pack(side='right')
        ttk.Button(top,text='演示',command=lambda:self.start('demo','')).pack(side='right',padx=6)
        self.health = tk.Label(root,text='未连接',anchor='w',bg='#263245',fg='#e2e8f0',font=('Microsoft YaHei UI',14,'bold'),padx=16,pady=10)
        self.health.pack(fill='x',padx=14)
        self.flags = tk.StringVar(value='USART3 / PB10 TX → USB 串口 RX · 共地 · 上电静止等待初始化')
        ttk.Label(root,textvariable=self.flags,padding=(16,8)).pack(fill='x')
        values = ttk.Frame(root,padding=(14,0));values.pack(fill='x')
        self.values = {}
        labels = [('speed','水平速度 · m/s'),('heading','车头航向 · °'),('roll','横滚 / 俯仰 · °'),('age','结果年龄 / 接收频率')]
        for column,(name,label) in enumerate(labels):
            box = ttk.LabelFrame(values,text=label,padding=10);box.grid(row=0,column=column,sticky='nsew',padx=4)
            values.columnconfigure(column,weight=1)
            v=tk.StringVar(value='—');self.values[name]=v
            ttk.Label(box,textvariable=v,style='Value.TLabel').pack(anchor='w')
        charts = ttk.Frame(root,padding=14);charts.pack(fill='both',expand=True)
        charts.columnconfigure(0,weight=3);charts.columnconfigure(1,weight=2);charts.rowconfigure(0,weight=1)
        self.map = tk.Canvas(charts,bg='#0b1220',highlightthickness=1,highlightbackground='#334155')
        self.map.grid(row=0,column=0,sticky='nsew',padx=(0,10))
        right = ttk.Frame(charts);right.grid(row=0,column=1,sticky='nsew');right.rowconfigure(0,weight=1);right.rowconfigure(1,weight=1);right.columnconfigure(0,weight=1)
        self.compass=tk.Canvas(right,bg='#0b1220',highlightthickness=0);self.compass.grid(row=0,column=0,sticky='nsew')
        self.graph=tk.Canvas(right,bg='#0b1220',highlightthickness=0);self.graph.grid(row=1,column=0,sticky='nsew',pady=(10,0))
        self.details = tk.StringVar(value='等待结果帧。设备初始化期间约每秒发送一帧状态。')
        ttk.Label(root,textvariable=self.details,font=('Consolas',11),padding=(18,0),justify='left').pack(fill='x')
        self.counters=tk.StringVar();ttk.Label(root,textvariable=self.counters,padding=(18,8)).pack(fill='x')
        footer=ttk.Frame(root,padding=(14,0,14,12));footer.pack(fill='x')
        self.record=tk.StringVar(value='连接后自动保存原始 .bin 和解析 .csv 至 '+str(args.output))
        ttk.Label(footer,textvariable=self.record).pack(side='left')
        ttk.Button(footer,text='清空轨迹',command=self.clear).pack(side='right')
        self.refresh();root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(20,self.poll)
        if args.demo: root.after(100,lambda:self.start('demo',''))
        elif args.replay: root.after(100,lambda:self.start('replay',args.replay))
        elif args.port: self.port.set(args.port);root.after(100,self.connect)
        if args.quit_after: root.after(int(args.quit_after*1000),self.close)

    def refresh(self):
        try:
            from serial.tools import list_ports
            ports=[p.device for p in list_ports.comports()]
            self.ports['values']=ports
            if not self.port.get() and ports:self.port.set(ports[0])
        except ImportError:
            self.flags.set('缺少 pyserial：运行 python -m pip install -r tools/requirements.txt；演示与文件回放仍可用。')

    def clear(self):
        self.path.clear();self.speeds.clear();self.origin=None

    def stop(self):
        if self.reader:
            self.reader.stop_event.set();self.reader.join(timeout=1)
            if self.reader.is_alive(): return False
            self.reader=None
        self.connect_button.configure(text='连接');return True

    def start(self,mode,source):
        if not self.stop():return
        self.events=queue.Queue(maxsize=2048);self.stream=StreamState();self.times.clear();self.clear()
        self.current=None;self.error='';self.mode=mode;self.parser_stats=(0,0,0,0,0,0)
        self.reader=Reader(mode,source,self.args.output,self.events);self.reader.start()
        self.connect_button.configure(text='断开')

    def connect(self):
        if self.reader and self.reader.is_alive():self.stop();return
        if not self.port.get():messagebox.showinfo('选择串口','请选择接到 USART3 / PB10 的 USB 串口。');return
        self.start('serial',self.port.get())

    def replay(self):
        path=filedialog.askopenfilename(title='打开导航结果二进制记录',filetypes=[('Navigation binary','*.bin')])
        if path:self.start('replay',path)

    def poll(self):
        for _ in range(3000):
            try:event=self.events.get_nowait()
            except queue.Empty:break
            if event[0]=='packet':
                p,when=event[1:];usable,reset=self.stream.accept(p,when)
                if reset:self.clear()
                self.times.append(when)
                if usable:
                    self.current=p
                    if self.origin is None:self.origin=(p.latitude,p.longitude)
                    east=(p.longitude-self.origin[1])*6378137*math.cos(self.origin[0])
                    north=(p.latitude-self.origin[0])*6378137
                    self.path.append((east,north));self.speeds.append((when,math.hypot(*p.velocity[:2])))
            elif event[0]=='stats':self.parser_stats=event[1:]
            elif event[0]=='ready':self.record.set('正在记录：'+event[1]+'.bin / .csv')
            elif event[0]=='error':self.error=event[1]
            elif event[0]=='closed':self.connect_button.configure(text='连接')
        now=time.monotonic();self.render(now)
        self.root.after(50,self.poll)

    def render(self,now):
        state=self.stream.health(now)
        if self.error:state='连接 / 记录错误：'+self.error
        elif not self.reader:state='已断开' if self.stream.latest else '未连接'
        elif not self.reader.is_alive():state='记录回放结束' if self.mode=='replay' else '连接已关闭'
        p=self.stream.latest
        simulation=bool(p and p.status&SIMULATED)
        prefix='【模拟数据】 ' if simulation or self.mode=='demo' else ('【记录回放 · 100 Hz 节奏】 ' if self.mode=='replay' else '')
        normal=state=='导航正常' and not simulation
        self.health.configure(text=prefix+state,bg='#124c41' if normal else '#633b24')
        while self.times and now-self.times[0]>1:self.times.popleft()
        hz=len(self.times)
        if p:
            fix={0:'无解',1:'单点',2:'差分',3:'浮点',4:'固定',7:'未知'}.get(p.status>>5,'保留')
            self.flags.set(f'RTK {fix}  |  PPS {"锁定" if p.status&TIME_LOCKED else "未锁定"}  |  GNSS {"近期更新" if p.status&GNSS_RECENT else "未近期更新"}  |  帧序号 {p.sequence}  |  测量时刻 {p.state_time:.3f} s')
            self.values['age'].set(f'{p.age_ms if p.age_ms!=65535 else "未知"} ms / {hz} Hz')
        else:self.values['age'].set('—')
        q=self.current
        if q:
            self.values['speed'].set(f'{math.hypot(*q.velocity[:2]):.3f}')
            self.values['heading'].set(f'{math.degrees(q.attitude[2])%360:.2f}')
            self.values['roll'].set(f'{math.degrees(q.attitude[0]):.2f} / {math.degrees(q.attitude[1]):.2f}')
            self.details.set(f'最近有效结果  纬度 {math.degrees(q.latitude):.9f}°  经度 {math.degrees(q.longitude):.9f}°  椭球高 {q.height:.3f} m\n'
                f'速度 N/E/D  {q.velocity[0]: .4f} / {q.velocity[1]: .4f} / {q.velocity[2]: .4f} m/s    '
                f'四元数 w/x/y/z  '+ ' / '.join(f'{x:.6f}' for x in q.quaternion))
        else:
            for name in ('speed','heading','roll'):self.values[name].set('—')
            self.details.set('等待有效测量。无效状态帧不会更新轨迹、速度或姿态。')
        frames,crc,version,discarded,timeout,dropped=self.parser_stats
        self.counters.set(f'完整帧 {frames}    CRC 错误 {crc}    版本错误 {version}    重同步丢弃 {discarded} B    半帧超时 {timeout}    '
            f'序号缺口 {self.stream.missing}    重复 {self.stream.duplicates}    新会话 {self.stream.sessions}    显示队列丢弃 {dropped}')
        self.draw_map();self.draw_compass();self.draw_speed(now)

    def draw_map(self):
        c=self.map;c.delete('all');w,h=c.winfo_width(),c.winfo_height()
        c.create_text(18,18,text='局部 EN 轨迹 · IMU 测量中心 · m',fill='#b6c5da',anchor='nw')
        if not self.path:return
        points=list(self.path);xs=[p[0] for p in points];ys=[p[1] for p in points]
        cx=(min(xs)+max(xs))/2;cy=(min(ys)+max(ys))/2
        scale=min(max(1,w-70)/max(10,max(xs)-min(xs)),max(1,h-90)/max(10,max(ys)-min(ys)))
        def point(e,n):return w/2+(e-cx)*scale,h/2+15-(n-cy)*scale
        for frac in (.25,.5,.75):
            c.create_line(frac*w,45,frac*w,h-30,fill='#1b2b41');c.create_line(30,frac*h,w-25,frac*h,fill='#1b2b41')
        stride=max(1,len(points)//2000);coords=[v for p in points[::stride] for v in point(*p)]
        if len(coords)>=4:c.create_line(*coords,fill='#38bdf8',width=2)
        x,y=point(*points[-1]);c.create_oval(x-5,y-5,x+5,y+5,fill='#4ade80',outline='')
        c.create_text(18,h-18,text=f'E {points[-1][0]:.2f}   N {points[-1][1]:.2f}     ↑ 北   → 东',fill='#94a3b8',anchor='w')

    def draw_compass(self):
        c=self.compass;c.delete('all');w,h=c.winfo_width(),c.winfo_height();x,y=w/2,h/2+8;r=max(12,min(w,h)/2-35)
        c.create_text(14,15,text='车头航向 · 真北顺时针',fill='#b6c5da',anchor='nw')
        c.create_oval(x-r,y-r,x+r,y+r,outline='#334155',width=2)
        for text,dx,dy in [('N',0,-1),('E',1,0),('S',0,1),('W',-1,0)]:
            c.create_text(x+dx*(r+13),y+dy*(r+13),text=text,fill='#94a3b8')
        if self.current:
            a=self.current.attitude[2];c.create_line(x,y,x+math.sin(a)*r*.8,y-math.cos(a)*r*.8,fill='#38bdf8',width=4,arrow=tk.LAST)

    def draw_speed(self,now):
        c=self.graph;c.delete('all');w,h=c.winfo_width(),c.winfo_height()
        c.create_text(14,15,text='水平速度 · 最近 30 s',fill='#b6c5da',anchor='nw')
        points=[p for p in self.speeds if now-p[0]<=30]
        if not points:return
        maximum=max(1,max(v for _,v in points)*1.1)
        coords=[v for t,s in points for v in (35+(t-now+30)/30*max(1,w-55),h-25-s/maximum*max(1,h-65))]
        if len(coords)>=4:c.create_line(*coords,fill='#4ade80',width=2)
        c.create_text(14,40,text=f'{maximum:.1f} m/s',fill='#94a3b8',anchor='nw')

    def close(self):
        if not self.stop():
            self.root.after(100,self.close);return
        self.root.destroy()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port');parser.add_argument('--demo',action='store_true');parser.add_argument('--replay')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'captures/navigation')
    parser.add_argument('--quit-after',type=float,default=0,help='Automated GUI smoke-test duration')
    args=parser.parse_args();root=tk.Tk();Dashboard(root,args);root.mainloop()

if __name__=='__main__':main()
