#!/usr/bin/env python3
import json,sqlite3,sys,math,bisect,statistics
from pathlib import Path
import cv2,numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag=Path(sys.argv[1]);output=Path(sys.argv[2])
report=json.loads((bag/'capture_report.json').read_text())
c=sqlite3.connect('file:'+str(next(bag.glob('*.db3')))+'?mode=ro',uri=True)
result={'bag':str(bag),'sqlite':c.execute('pragma quick_check').fetchone()[0],'capture_status':report['status'],'missing':report['topics_without_messages'],'failed':report['conversion_failures'],'dropped':report['local_queue_drops'],'topics':{}}
previews={};stamps={}
for tid,name,typ in c.execute('select id,name,type from topics').fetchall():
 cls=get_message(typ);rows=c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)).fetchall()
 s=dict(report['topics'][name]);result['topics'][name]=s
 decoded=[deserialize_message(raw,cls) for _,raw in rows]
 source=[];mismatch=0
 for (time,_),msg in zip(rows,decoded):
  if hasattr(msg,'header'):t=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  elif typ=='std_msgs/msg/String' and name.startswith('/ma_raihan/robot/'):
   obj=json.loads(msg.data)['message'];h=obj['header']['stamp'];t=h['secs']*10**9+h['nsecs']
  else:continue
  source.append(t);mismatch+=time!=t
 if source:stamps[name]=source;s['timestamp_mismatches']=mismatch
 if typ=='sensor_msgs/msg/CompressedImage':
  invalid=0;shapes=set();types=set();depth_values=[]
  for msg in decoded:
   image=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
   if image is None:invalid+=1;continue
   shapes.add(tuple(image.shape));types.add(str(image.dtype))
   if '/image_rect/' in name and name not in previews:previews[name]=image
   if '/depth/' in name:
    valid=image[(image>0)&(image<65535)]
    if valid.size:depth_values.append(float(np.median(valid)))
  s.update(invalid_images=invalid,shapes=sorted(shapes),pixel_types=sorted(types))
  if depth_values:s['median_valid_depth_mm']=statistics.median(depth_values)
 if name.endswith('/motor_states'):
  msgs=[json.loads(m.data)['message'] for m in decoded]
  s['motor_count']=sorted(set(len(m['motor_state']) for m in msgs))
  for field in ['q','dq','ddq','tauEst','temperature']:
   vals=[motor[field] for m in msgs for motor in m['motor_state']]
   s[field]={'min':min(vals),'max':max(vals)}
 if name.endswith('/low_state'):
  msgs=[json.loads(m.data)['message']['state'] for m in decoded]
  ticks=[m['tick'] for m in msgs];s['tick_first_last']=[ticks[0],ticks[-1]];s['ticks_advance']=all(b>a for a,b in zip(ticks,ticks[1:]));s['preserved_fields']=sorted(msgs[0])
 if typ=='sensor_msgs/msg/BatteryState' and decoded:
  s['charge_fraction']=[min(m.percentage for m in decoded),max(m.percentage for m in decoded)]
  s['voltage_V']=[min(m.voltage for m in decoded),max(m.voltage for m in decoded)]
  s['current_A']=[min(m.current for m in decoded),max(m.current for m in decoded)]
 if typ=='sensor_msgs/msg/PointCloud2' and decoded:
  values=[]
  for m in decoded:
   z=np.ndarray((m.width*m.height,),np.float32,buffer=m.data,offset=8,strides=(m.point_step,))
   valid=z[np.isfinite(z)&(z>0)]
   if valid.size:values.append(float(np.median(valid)))
  s['median_valid_z_sdk_m']=statistics.median(values) if values else None
imu=stamps.get('/ma_raihan/robot/imu',[])
for name,times in stamps.items():
 if '/image_rect/' not in name or not imu:continue
 distances=[]
 for t in times:
  if not imu[0]<=t<=imu[-1]:continue
  i=bisect.bisect_left(imu,t);distances.append(min(abs(t-a) for a in imu[max(0,i-1):i+1])/1e6)
 if distances:result['topics'][name]['nearest_imu_header_ms']={'median':statistics.median(distances),'max':max(distances)}
output.write_text(json.dumps(result,indent=2))
if previews:
 blocks=[]
 for name,image in sorted(previews.items()):
  tile=image.copy();cv2.putText(tile,name.split('/cameras/')[1].split('/')[0],(10,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2);blocks.append(tile)
 if len(blocks)==4:cv2.imwrite(str(output.with_suffix('.png')),np.vstack([np.hstack(blocks[:2]),np.hstack(blocks[2:])]))
print('Audit:',output,'status:',result['capture_status'],'missing:',result['missing'])
for n,s in result['topics'].items():
 if '/image_rect/' in n:print(n,round(s['source_hz'],2),'FPS',round(s['median_age_ms'],1),'median ms',round(s['max_age_ms'],1),'max ms','invalid',s.get('invalid_images'))
print('motor',result['topics'].get('/ma_raihan/robot/motor_states'))
