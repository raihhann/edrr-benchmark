#!/usr/bin/env python3
"""SSH orchestration only: sensor serialization and navigation run on the robot."""
import argparse
import concurrent.futures
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import threading
import time
import uuid
from go1_local_quality import assess,save

PROJECT=Path(__file__).resolve().parent.parent
if PROJECT.name!='Go1_Project':PROJECT=Path('/home/fsti/ROS/Go1_Project')
HOSTS={'pi':'pi@192.168.123.161','13':'unitree@192.168.123.13',
       '14':'unitree@192.168.123.14','15':'unitree@192.168.123.15'}


def valid_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,99}',value):raise ValueError('Bag name: 1–100 letters, digits, underscores or hyphens')
    return value


def options(role,args):
    if role=='14':
        return ['-o','ConnectTimeout=30','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=4',
                '-o','ControlMaster=no','-o','ControlPath=none','-o','ControlPersist=no']
    return ['-o','ConnectTimeout=5','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=2',
            '-o','ControlMaster=auto','-o','ControlPersist=3600',
            '-o','ControlPath='+args.ssh_control_pattern.format(role=role)]


def ssh(role,args,words,**kwargs):
    return subprocess.run(['ssh',*options(role,args),'-o','BatchMode=yes',HOSTS[role],shlex.join(words)],**kwargs)


class Peer:
    def __init__(self,role,args,name,folder):
        self.role=role;self.args=args;self.ready=False;self.finalized=False;self.error=None
        self.target_reached=False;self.process=None;self.lock=threading.Lock();self.probes={};self.clock=None
        self.log=(folder/(role+'_ssh.log')).open('w')
        max_seconds=(max(10.,3*args.distance/args.speed+10.) if args.navigate else args.duration)+args.startup_timeout+30
        words=['bash','MA_Raihan/local_capture.sh','--run-id',name,'--max-seconds',str(max_seconds),
               '--startup-timeout',str(args.startup_timeout),'--distance',str(args.distance),'--speed',str(args.speed)]
        words+=['--realsense-controller',args.realsense_controller]
        if args.navigate and role=='pi':words+=['--navigate']
        for flag in ['include_lidar','include_realsense','skip_lidar','skip_realsense']:
            if getattr(args,flag):words+=['--'+flag.replace('_','-')]
        for camera in args.optional_camera:words+=['--optional-camera',camera]
        self.process=subprocess.Popen(['ssh',*options(role,args),'-o','BatchMode=yes',HOSTS[role],shlex.join(words)],
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
        self.thread=threading.Thread(target=self.reader,daemon=True);self.thread.start()

    def reader(self):
        for line in self.process.stdout:
            self.log.write(line);self.log.flush()
            try:message=json.loads(line)
            except ValueError:continue
            event=message.get('event')
            if event=='ready':
                self.ready=True;print('[READY] '+self.role+': actual required messages and recorder subscriptions verified',flush=True)
            elif event=='error':self.error=message.get('message','Remote error')
            elif event=='target_reached':self.target_reached=True;print('[INFO] Target reached: %.3f m'%message['distance_m'],flush=True)
            elif event=='finalized':self.finalized=True;self.error=self.error or message.get('error')
            elif event=='clock':
                start=self.probes.pop(message['probe_id'],None)
                if start is not None:
                    finish=time.time_ns();rtt=finish-start
                    probe=dict(offset_ns=message['remote_ns']-(start+finish)//2,rtt_ns=rtt)
                    if self.clock is None or rtt<self.clock['rtt_ns']:self.clock=probe
            elif event=='motion_started':print('[INFO] Walking %.2f m at %.2f m/s'%(message['target_m'],message['speed_m_s']),flush=True)
        self.process.wait()
        if not self.finalized:self.error=self.error or 'SSH/runtime exited before finalization'

    def send(self,command,**values):
        if self.process.poll() is not None:return
        with self.lock:
            try:self.process.stdin.write(json.dumps(dict(command=command,**values))+'\n');self.process.stdin.flush()
            except (BrokenPipeError,OSError):pass

    def probe(self):
        token=uuid.uuid4().hex;self.probes[token]=time.time_ns();self.send('clock_probe',probe_id=token)


def authenticate(args):
    for role,target in HOSTS.items():
        print('[INFO] SSH '+target,flush=True)
        subprocess.run(['ssh',*options(role,args),target,'true'],check=True,timeout=90)


def detect_realsense(args):
    if args.realsense_controller!='auto':return args.realsense_controller
    code="import glob,os;print(' '.join(p for p in glob.glob('/sys/bus/usb/devices/*') if os.path.isfile(p+'/idVendor') and os.path.isfile(p+'/idProduct') and open(p+'/idVendor').read().strip()=='8086' and open(p+'/idProduct').read().strip().lower()=='0b5c'))"
    def query(role):
        response=ssh(role,args,['python2','-c',code],capture_output=True,text=True,timeout=45)
        if response.returncode:raise RuntimeError('Cannot detect RealSense on '+role+': '+response.stderr[-500:])
        return role if response.stdout.strip() else None
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        found=[role for role in pool.map(query,HOSTS) if role]
    if len(found)!=1:raise RuntimeError('Expected one RealSense USB host, found '+str(found)+'. Specify --realsense-controller explicitly for multiple cameras.')
    args.realsense_controller=found[0]
    print('[INFO] RealSense USB detected on '+found[0],flush=True)
    return found[0]


def preflight_control(args):
    if not args.navigate:return
    code="import rosgraph;master=rosgraph.Master('/ma_v5_control_preflight');master.lookupService('/set_mode');topics=dict(master.getPublishedTopics(''));assert '/ma_raihan/robot/odom' in topics, 'Stamped odometry is not published'"
    command='source "$HOME/MA_Raihan/ros_ws/devel/setup.bash"; export ROS_MASTER_URI=http://localhost:11311; python2 -c '+shlex.quote(code)
    response=ssh('pi',args,['bash','-c',command],capture_output=True,text=True,timeout=20)
    if response.returncode:
        raise RuntimeError('Pi navigation driver/odometry is unavailable. On Pi run: bash ~/start_pi.sh --control --lidar. No recording or motion was started.')


def start_nodes(args):
    # This is explicit opt-in. No control startup is used for stationary capture.
    def start(role):
        script='start_pi.sh' if role=='pi' else 'start_'+role+'.sh'
        words=['bash',script]
        if role=='pi':
            words+=['--lidar' if args.include_lidar else '--no-lidar']
            if args.navigate:words+=['--control']
        result=ssh(role,args,words,timeout=90)
        if result.returncode:raise RuntimeError(role+': startup failed')
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(start,HOSTS))


def download(role,args,name,folder):
    destination=folder/role
    def verify(directory):
        manifest=json.loads((directory/'manifest.json').read_text())
        if 'audit.json' not in manifest:raise RuntimeError('Audit missing from manifest')
        for filename,data in manifest.items():
            path=directory/filename
            if Path(filename).name!=filename or path.is_symlink() or path.resolve().parent!=directory.resolve():raise RuntimeError('Unsafe manifest entry')
            digest=hashlib.sha256()
            with path.open('rb') as handle:
                for block in iter(lambda:handle.read(1024*1024),b''):digest.update(block)
            if path.stat().st_size!=data['bytes'] or digest.hexdigest()!=data['sha256']:raise RuntimeError('Checksum mismatch: '+filename)
        return manifest
    if destination.exists():
        verify(destination);print('[INFO] '+role+': existing download verified',flush=True);return
    print('[INFO] '+role+': auditing native bag on controller',flush=True)
    result=ssh(role,args,['bash','MA_Raihan/local_audit.sh',name],capture_output=True,text=True,timeout=900)
    if result.returncode:raise RuntimeError(role+': native audit failed: '+result.stderr[-1500:])
    incoming=folder/('.incoming_'+role+'_'+uuid.uuid4().hex[:8]);incoming.mkdir()
    remote=HOSTS[role]+':MA_Raihan/captures/'+name
    print('[INFO] '+role+': downloading (remote original retained)',flush=True)
    subprocess.run(['scp',*options(role,args),'-o','BatchMode=yes','-r',remote,str(incoming)],check=True,timeout=1800)
    staged=incoming/name;manifest=verify(staged)
    (staged/'download_verified.json').write_text(json.dumps(dict(verified=True,files=manifest),indent=2))
    staged.rename(destination)
    print('[INFO] '+role+': download SHA256 verified',flush=True)


def merge_downloads(folder):
    interpreter=PROJECT/'.venv-native-bags/bin/python'
    if not interpreter.is_file():interpreter=Path(sys.executable)
    subprocess.run([str(interpreter),str(Path(__file__).with_name('convert_native_db3.py')),str(folder)],check=True)
    report=json.loads((folder/'db3_report.json').read_text())
    return folder/report['database']


def cleanup_downloaded_recordings(args,name,folder):
    # Conversion/merge verification must finish before any remote deletion.
    report=json.loads((folder/'db3_report.json').read_text())
    if not report.get('verified'):raise RuntimeError('ROS2 output has not been verified')
    for filename,digest in report['output_sha256'].items():
        path=folder/'combined_ros2'/filename
        if Path(filename).name!=filename or path.is_symlink():raise RuntimeError('Unsafe ROS2 output')
        with path.open('rb') as handle:actual=hashlib.file_digest(handle,'sha256').hexdigest()
        if actual!=digest:raise RuntimeError('ROS2 file changed; remote originals retained')
    native=json.loads((folder/'merge_report.json').read_text())
    for role in HOSTS:
        path=folder/role/'capture.bag'
        with path.open('rb') as handle:actual=hashlib.file_digest(handle,'sha256').hexdigest()
        if actual!=native['source_sha256'][role]:raise RuntimeError('Local source changed; remote originals retained')
    helper=Path(__file__).with_name('remote_cleanup.py').read_text()
    receipts={}
    for role in HOSTS:
        response=ssh(role,args,['python2','-c',helper,name,native['source_sha256'][role]],capture_output=True,text=True,timeout=120)
        if response.returncode:
            receipts[role]=dict(status='retained_error',error=response.stderr[-1500:])
            print('[WARNING] '+role+': cleanup failed; remote file retained',flush=True)
        else:
            receipts[role]=json.loads(response.stdout)
            print('[CLEANUP] '+role+': '+receipts[role]['status'],flush=True)
        (folder/'remote_cleanup.json').write_text(json.dumps(receipts,indent=2))
    return receipts


def cli(argv):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--navigate',action='store_true');mode.add_argument('--record-only',action='store_true')
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--distance',type=float,default=1.)
    parser.add_argument('--speed',type=float,default=.2)
    parser.add_argument('--bag-name');parser.add_argument('--out',type=Path,default=PROJECT/'Bags/go1_bags')
    parser.add_argument('--fetch-only',metavar='RUN_ID');parser.add_argument('--audit-only',type=Path)
    parser.add_argument('--merge-only',type=Path,help='Merge an already downloaded dataset without contacting the robot')
    parser.add_argument('--keep-remote',action='store_true',help='Keep controller recordings after verified ROS2 export')
    parser.add_argument('--no-fetch',action='store_true');parser.add_argument('--start-nodes',action='store_true')
    parser.add_argument('--startup-timeout',type=float,default=30.)
    parser.add_argument('--min-camera-fps',type=float,default=15.)
    parser.add_argument('--max-loss-percent',type=float,default=1.)
    for flag in ['include-lidar','include-realsense','skip-lidar','skip-realsense']:parser.add_argument('--'+flag,action='store_true')
    parser.add_argument('--realsense-controller',default='auto',choices=['auto','pi','13','14','15'])
    parser.add_argument('--optional-camera',action='append',default=[],choices=['front','chin','left','right','rear_down'])
    parser.add_argument('--ssh-control-pattern',default=str(PROJECT/('.runtime/native_'+uuid.uuid4().hex[:8]+'_{role}')))
    args=parser.parse_args(argv)
    for name in ['duration','distance','speed','startup_timeout','min_camera_fps','max_loss_percent']:
        if not math.isfinite(getattr(args,name)):parser.error(name+' must be finite')
    if not 0<args.duration<=3600 or not 0<args.startup_timeout<=120:parser.error('duration (0,3600], startup-timeout (0,120]')
    if args.navigate and not (0<args.distance<=100 and 0<args.speed<=.5):parser.error('Navigation distance (0,100] m; speed (0,0.5] m/s')
    if args.navigate and max(10.,3*args.distance/args.speed+10.)+args.startup_timeout+30>4000:
        parser.error('Requested navigation would exceed the 4000-second session hard limit')
    if args.min_camera_fps<=0 or not 0<=args.max_loss_percent<=100:parser.error('Invalid quality thresholds')
    if args.include_lidar and args.skip_lidar or args.include_realsense and args.skip_realsense:parser.error('Cannot include and skip the same sensor')
    if sum(bool(value) for value in [args.fetch_only,args.audit_only,args.merge_only])>1:parser.error('Choose fetch-only, audit-only or merge-only')
    if args.fetch_only:valid_name(args.fetch_only)
    if args.bag_name:valid_name(args.bag_name)
    return args


def main(argv=None):
    args=cli(sys.argv[1:] if argv is None else argv)
    if args.merge_only:
        merge_downloads(args.merge_only.expanduser().resolve());return 0
    if args.audit_only:
        result=assess(args.audit_only,min_camera_fps=args.min_camera_fps,max_loss_percent=args.max_loss_percent)
        save(result,args.audit_only);print('[QUALITY] '+result['verdict']);return 2 if result['verdict']=='INVALID' else 3 if result['verdict']=='UNVERIFIED' else 0
    (PROJECT/'.runtime').mkdir(mode=0o700,exist_ok=True)
    args.out=args.out.expanduser().resolve();args.out.mkdir(parents=True,exist_ok=True)
    name=args.fetch_only or args.bag_name or datetime.datetime.now().strftime('%Y%m%d_%H%M%S_go1_native_')+uuid.uuid4().hex[:6]
    folder=args.out/name
    if not args.fetch_only:folder.mkdir(exist_ok=False)
    else:folder.mkdir(exist_ok=True)
    authenticate(args)
    if args.include_realsense and not args.fetch_only:detect_realsense(args)
    if args.fetch_only:
        context=json.loads((folder/'experiment.json').read_text()) if (folder/'experiment.json').exists() else dict(run_id=name,navigate=False,distance_m=None,clock_probes={},coordinated_finish=False,error=None)
    else:
        if args.start_nodes:start_nodes(args)
        preflight_control(args)
        if args.include_realsense:
            print('[INFO] Starting/verifying RealSense on '+args.realsense_controller+' (camera only)',flush=True)
            response=ssh(args.realsense_controller,args,['bash','MA_Raihan/scripts/start_realsense.sh'],timeout=150)
            if response.returncode:raise RuntimeError('RealSense startup failed; no recording or navigation started')
        context=dict(schema_version=1,run_id=name,navigate=args.navigate,duration_s=args.duration,
                     distance_m=args.distance if args.navigate else None,speed_m_s=args.speed if args.navigate else None,
                     include_lidar=args.include_lidar,include_realsense=args.include_realsense,realsense_controller=args.realsense_controller,
                     optional_cameras=args.optional_camera,
                     coordinated_finish=False,error=None,clock_probes={},format='One combined ROS2 SQLite db3; native ROS1 sources retained on laptop')
        (folder/'experiment.json').write_text(json.dumps(context,indent=2))
        peers={};heartbeat_stop=threading.Event()
        def heartbeat():
            while not heartbeat_stop.wait(.4):
                for peer in list(peers.values()):peer.send('heartbeat')
        worker=threading.Thread(target=heartbeat,daemon=True);worker.start()
        try:
            for role in HOSTS:peers[role]=Peer(role,args,name,folder)
            deadline=time.monotonic()+args.startup_timeout+10;last_probe=0.
            while not all(peer.ready for peer in peers.values()):
                for role,peer in peers.items():
                    if peer.error:raise RuntimeError(role+': '+peer.error)
                if time.monotonic()>deadline:raise RuntimeError('Timed out waiting for all native recorders')
                if time.monotonic()-last_probe>.5:
                    last_probe=time.monotonic()
                    for peer in peers.values():peer.probe()
                time.sleep(.05)
            for peer in peers.values():peer.send('assessment_start')
            print('[INFO] All controllers ready; original published rates, no sampling.',flush=True)
            if args.navigate:peers['pi'].send('navigate')
            until=time.monotonic()+(max(10.,3*args.distance/args.speed+10.) if args.navigate else args.duration)
            while True:
                for role,peer in peers.items():
                    if peer.error:raise RuntimeError(role+': '+peer.error)
                if args.navigate and peers['pi'].target_reached:break
                if time.monotonic()>=until:
                    if args.navigate:raise RuntimeError('Navigation time limit exceeded')
                    break
                time.sleep(.05)
            # Preserve two seconds after a successful motion stop, still monitored.
            if args.navigate:
                deadline=time.monotonic()+2.
                while time.monotonic()<deadline:
                    for role,peer in peers.items():
                        if peer.error:raise RuntimeError(role+': '+peer.error)
                    time.sleep(.05)
        except BaseException as error:
            context['error']=str(error) or type(error).__name__;print('[ERROR] '+context['error'],flush=True)
        finally:
            # Stop Pi first; no camera/audit/transfer can delay its local zero command.
            for role in ['pi','13','14','15']:
                if role in peers:peers[role].send('finish')
            deadline=time.monotonic()+30
            while any(peer.process.poll() is None for peer in peers.values()) and time.monotonic()<deadline:time.sleep(.1)
            heartbeat_stop.set();worker.join(timeout=2)
            context['coordinated_finish']=len(peers)==4 and all(peer.finalized and not peer.error for peer in peers.values())
            context['clock_probes']={role:peer.clock for role,peer in peers.items() if peer.clock}
            context['finished_ns']=time.time_ns()
            for peer in peers.values():
                if peer.process.poll() is None:peer.process.terminate()
                peer.thread.join(timeout=2);peer.log.close()
    (folder/'experiment.json').write_text(json.dumps(context,indent=2))
    if args.no_fetch:
        print('[INFO] Bags retained on controllers: ~/MA_Raihan/captures/'+name)
        print('Fetch later: python3 NavigationScripts/nav_and_collect_v5.py --fetch-only '+name)
        return 2 if context.get('error') else 3
    transfer_errors={}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        futures={pool.submit(download,role,args,name,folder):role for role in HOSTS}
        for future in concurrent.futures.as_completed(futures):
            role=futures[future]
            try:future.result()
            except Exception as error:transfer_errors[role]=str(error);print('[ERROR] '+role+': '+str(error),flush=True)
    if transfer_errors:
        context['transfer_errors']=transfer_errors;(folder/'experiment.json').write_text(json.dumps(context,indent=2))
    result=assess(folder,min_camera_fps=args.min_camera_fps,max_loss_percent=args.max_loss_percent)
    if transfer_errors:result['transfer_errors']=transfer_errors
    save(result,folder)
    print('[QUALITY] '+result['verdict']+' | rerun recommended: '+str(result['rerun_recommended']),flush=True)
    for issue in result['issues']:print('  '+issue['level'].upper()+': '+str(issue.get('role') or '')+' '+str(issue.get('topic') or '')+' '+issue['message'])
    merge_error=None
    if not transfer_errors:
        try:
            merged=merge_downloads(folder)
            context['combined_bag']=str(merged.relative_to(folder))
            context['format']='One combined ROS2 SQLite db3; native ROS1 sources retained on laptop'
            (folder/'experiment.json').write_text(json.dumps(context,indent=2))
            if not args.keep_remote:cleanup_downloaded_recordings(args,name,folder)
            else:print('[INFO] Remote originals retained (--keep-remote)',flush=True)
        except Exception as error:
            merge_error=str(error);print('[ERROR] Merge failed; source bags retained: '+merge_error,flush=True)
    print('Dataset: '+str(folder))
    if merge_error:return 3
    if transfer_errors:print('[INFO] Merge deferred until every source bag is downloaded')
    return 2 if result['verdict']=='INVALID' else 3 if result['verdict']=='UNVERIFIED' else 0
