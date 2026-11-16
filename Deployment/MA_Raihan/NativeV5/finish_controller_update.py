#!/usr/bin/env python3
"""Finish native helper deployment, without starting nodes or motion."""
from pathlib import Path
import hashlib
import shlex
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parent
HOSTS={'pi':'pi@192.168.123.161','13':'unitree@192.168.123.13','14':'unitree@192.168.123.14','15':'unitree@192.168.123.15'}
FILES=['local_capture.py','local_paths.py','local_audit.py','local_startup.sh','camera_recovery.py','local_capture.sh','local_audit.sh']

def main():
    # Stage and validate before replacing any file; backups from initial install remain.
    session_dir=Path(tempfile.mkdtemp(prefix="go1_update_"))
    for role,host in HOSTS.items():
        connection=["-o","ConnectTimeout=5","-o","ServerAliveInterval=3","-o","ServerAliveCountMax=2",
                    "-o","ControlMaster=auto","-o","ControlPersist=120","-o","ControlPath="+str(session_dir/role)]
        if role=='14':
            connection=['-o','ConnectTimeout=30','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=4',
                        '-o','ControlMaster=no','-o','ControlPath=none','-o','ControlPersist=no','-o','BatchMode=yes']
        print("Connecting to "+host,flush=True)
        subprocess.run(["ssh",*connection,host,"true"],check=True,timeout=90)
        for name in FILES:
            source=ROOT/name
            data=source.read_bytes();digest=hashlib.sha256(data).hexdigest()
            directory='"$HOME/MA_Raihan"' if name in ['local_capture.sh','local_audit.sh'] else '"$HOME/MA_Raihan/scripts"'
            verify=('python2 -m py_compile "$tmp"' if name.endswith('.py') else 'bash -n "$tmp"')
            command='set -e; dest='+directory+'/'+shlex.quote(name)+'; tmp=$(mktemp "${dest}.pending.XXXXXX"); cat > "$tmp"; '
            command+='actual=$(sha256sum "$tmp"); test "${actual%% *}" = '+shlex.quote(digest)+'; '+verify+'; '
            command+='chmod 755 "$tmp"; mv "$tmp" "$dest"; sha256sum "$dest"'
            subprocess.run(['ssh',*connection,'-o','BatchMode=yes',host,command],input=data,check=True,timeout=120)
        print('Verified helper update: '+role,flush=True)
    print('All controller helpers verified. No nodes or motion were started.')

if __name__=='__main__':main()
