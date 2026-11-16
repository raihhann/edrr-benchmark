#!/usr/bin/env python3
"""Atomic, fsync-verified installation; does not start nodes or motion."""
import base64,concurrent.futures,hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
HOSTS={'pi':'pi@192.168.123.161','13':'unitree@192.168.123.13','14':'unitree@192.168.123.14','15':'unitree@192.168.123.15'}
REMOTE=r'''
import os,sys,json,hashlib,base64,tempfile,shutil,subprocess
root=os.path.expanduser('~/MA_Raihan');backup=root+'/updates/realsense_repair_20261116'
if not os.path.isdir(backup):os.makedirs(backup)
for target,row in json.load(sys.stdin).items():
 if target.startswith('/') or '..' in target.split('/'):raise ValueError('Unsafe target')
 data=base64.b64decode(row['base64'])
 if hashlib.sha256(data).hexdigest()!=row['sha256']:raise ValueError('Input digest mismatch')
 if target.endswith('.py'):compile(data,target,'exec')
 if target.endswith('.launch'):
  import xml.etree.ElementTree as ET
  ET.fromstring(data)
 dest=root+'/'+target;directory=os.path.dirname(dest)
 if not os.path.isdir(directory):os.makedirs(directory)
 saved=backup+'/'+target.replace('/','_')
 if os.path.isfile(dest) and os.path.getsize(dest)>0 and not os.path.exists(saved):shutil.copy2(dest,saved)
 fd,tmp=tempfile.mkstemp(prefix='.verified_',dir=directory)
 with os.fdopen(fd,'wb') as handle:handle.write(data);handle.flush();os.fsync(handle.fileno())
 if target.endswith('.sh'):subprocess.check_call(['bash','-n',tmp])
 os.chmod(tmp,0755);os.rename(tmp,dest)
 fd=os.open(directory,os.O_RDONLY);os.fsync(fd);os.close(fd)
 with open(dest,'rb') as handle:actual=hashlib.sha256(handle.read()).hexdigest()
 if actual!=row['sha256']:raise ValueError('Installed hash mismatch')
 print(target+': '+actual)
'''
def deploy(role):
 names=['local_capture.py','local_audit.py','local_paths.py','local_startup.sh','camera_recovery.py','start_realsense.sh','realsense_readiness.py']
 payload={'scripts/'+name:dict(base64=base64.b64encode((ROOT/name).read_bytes()).decode(),sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()) for name in names}
 for name in ['realsense.launch','realsense_usb2.launch']:
  data=(ROOT/name).read_bytes();payload['config/'+name]=dict(base64=base64.b64encode(data).decode(),sha256=hashlib.sha256(data).hexdigest())
 import shlex
 result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ControlMaster=no','-o','ControlPath=none','-o','ConnectTimeout=30',HOSTS[role],'python2 -c '+shlex.quote(REMOTE)],input=json.dumps(payload),text=True,capture_output=True,check=True,timeout=90)
 return role+': '+result.stdout
if __name__=='__main__':
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  for report in pool.map(deploy,HOSTS):print(report)
