import sqlite3,json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
bag=Path('/home/tzb/bags/grasp_mission_20261116_192535')
names=['/local_costmap/costmap','/global_costmap/costmap','/plan','/explore/corridor_centerline','/explore/corridor_boundary','/explore/corridor_selection','/selected_viewpoint_vis_cloud','/explore/waypoint','/explore/control_path','/odometry']
names += ['/tf_static']
rows={n:[] for n in names}
for db in sorted(bag.glob('*.db3')):
 c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
 ts={i:(n,t) for i,n,t in c.execute('select id,name,type from topics') if n in rows}
 if ts:
  for tid,stamp,data in c.execute('select topic_id,timestamp,data from messages where topic_id in ('+','.join(map(str,ts))+')'):
   name,typ=ts[tid];rows[name].append((stamp,data,typ))
 c.close()
for n in rows: rows[n].sort(key=lambda r:r[0])
print('COUNTS', {n:len(v) for n,v in rows.items()},flush=True)
def decode(r):return deserialize_message(r[1],get_message(r[2]))
def nearest(n,t):return min(rows[n],key=lambda r:abs(r[0]-t)) if rows[n] else None
# Find a real nonempty centerline with nearby plan, map and selection.
best=None
for r in rows['/explore/corridor_centerline'][::max(1,len(rows['/explore/corridor_centerline'])//200)]:
 m=decode(r)
 if len(m.poses)<3:continue
 t=r[0];p=nearest('/plan',t);g=nearest('/local_costmap/costmap',t);sel=nearest('/explore/corridor_selection',t)
 if not p or not g or len(decode(p).poses)<3:continue
 score=abs(p[0]-t)+abs(g[0]-t)+(abs(sel[0]-t) if sel else 10**10)
 if best is None or score<best[0]:best=(score,t)
if best is None:raise RuntimeError('No usable overlapping centerline/path/map')
t=best[1]; chosen={n:nearest(n,t) for n in names}; msgs={n:decode(r) for n,r in chosen.items() if r}
out=Path('/home/tzb/nav_explore_ws/captures/navigation_bag_20261116');out.mkdir(exist_ok=True)
meta={'source_bag':str(bag),'reference_time_shanghai':datetime.fromtimestamp(t/1e9,ZoneInfo('Asia/Shanghai')).isoformat(),'topics':{n:{'offset_seconds':(r[0]-t)/1e9,'frame':getattr(getattr(msgs[n],'header',None),'frame_id',None)} for n,r in chosen.items() if r}}
(out/'metadata.json').write_text(json.dumps(meta,indent=2))
for n,m in msgs.items():
 if 'PointCloud2' not in str(type(m)):
  (out/(n.strip('/').replace('/','_')+'.json')).write_text(json.dumps(message_to_ordereddict(m)))
g=msgs['/local_costmap/costmap'];frame=g.header.frame_id
from scipy.spatial.transform import Rotation
edges={}
for row in rows['/tf_static']:
 for tr in decode(row).transforms:
  q=tr.transform.rotation;v=tr.transform.translation
  mat=np.eye(4);mat[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();mat[:3,3]=[v.x,v.y,v.z]
  edges.setdefault(tr.child_frame_id,{})[tr.header.frame_id]=mat
  edges.setdefault(tr.header.frame_id,{})[tr.child_frame_id]=np.linalg.inv(mat)
def transform(source):
 queue=[(source,np.eye(4))];seen=set()
 for node,mat in queue:
  if node==frame:return mat
  if node in seen:continue
  seen.add(node)
  queue.extend((dest,edge@mat) for dest,edge in edges.get(node,{}).items() if dest not in seen)
 raise RuntimeError(f'Missing TF {source} to {frame}')
def xy(points,source):
 pts=np.array([[p.x,p.y,p.z,1] for p in points]);return (transform(source)@pts.T).T[:,:2]
meta['transforms_to_map']={f:transform(f).tolist() for f in ['odom_init',frame]}
(out/'metadata.json').write_text(json.dumps(meta,indent=2))
arr=np.array(g.data).reshape(g.info.height,g.info.width);np.save(out/'costmap_raw.npy',arr)
x=g.info.origin.position.x;y=g.info.origin.position.y;res=g.info.resolution
extent=[x,x+g.info.width*res,y,y+g.info.height*res]
def base(title):
 fig,ax=plt.subplots(figsize=(10,8),layout='constrained');cmap=plt.get_cmap('magma').copy();cmap.set_bad('#bcbcbc')
 im=ax.imshow(np.ma.masked_less(arr,0),origin='lower',extent=extent,cmap=cmap,vmin=0,vmax=100,interpolation='nearest');fig.colorbar(im,ax=ax,label='Occupancy cost (0–100); gray = unknown',shrink=.75)
 ax.set(xlabel=f'X (m), frame: {frame}',ylabel='Y (m)',title=title);ax.set_aspect('equal');return fig,ax
def path(ax,n,color,label,lw=2.5):
 m=msgs.get(n)
 if m is None or not m.poses:return
 
 pts=xy([p.pose.position for p in m.poses],m.header.frame_id);ax.plot(*pts.T,color=color,lw=lw,label=label)
fig,ax=base('Recorded local costmap');fig.savefig(out/'01_costmap.png',dpi=180);plt.close(fig)
fig,ax=base('Planned path on recorded costmap');path(ax,'/plan','#00e5ff','Planned path');ax.legend();fig.savefig(out/'02_costmap_planned_path.png',dpi=180);plt.close(fig)
fig,ax=base('Exploration selection and corridor centerline constraint')
path(ax,'/explore/corridor_centerline','#ffe600','Corridor centerline',3)
path(ax,'/explore/control_path','#00e5ff','Exploration control path',2)
bound=msgs.get('/explore/corridor_boundary')
if bound and bound.polygon.points:
 
 pts=xy(bound.polygon.points,bound.header.frame_id);pts=np.vstack([pts,pts[0]]);ax.plot(*pts.T,'--',color='#44ff88',label='Corridor boundary')
markers=msgs.get('/explore/corridor_selection');print('MARKERS',[(m.ns,m.id,m.type,len(m.points),m.text[:100]) for m in markers.markers] if markers else [],flush=True)
if markers:
 labels=set()
 for m in markers.markers:
  if m.action!=0:continue
  # Marker points are transformed below
  if m.type in [1,2,3,7,8]:
   pts=[[p.x+m.pose.position.x,p.y+m.pose.position.y] for p in m.points] if m.points else [[m.pose.position.x,m.pose.position.y]]
   pts=np.array(pts);mat=transform(m.header.frame_id or frame);pts=(mat@np.column_stack([pts,np.zeros(len(pts)),np.ones(len(pts))]).T).T[:,:2];label=({0:'Observed candidate points',1:'Eligible candidate points',3:'Selected target'}.get(m.id,m.ns or 'Selection markers'));ax.scatter(*pts.T,s=35,color=(m.color.r,m.color.g,m.color.b),edgecolors='white',linewidths=.4,label=label if label not in labels else None);labels.add(label)
  elif m.type in [4,5] and m.points:
   pts=xy(m.points,m.header.frame_id or frame);ax.plot(*pts.T,color=(m.color.r,m.color.g,m.color.b),lw=1.2)
w=msgs.get('/explore/waypoint')
if w:
 wp=xy([w.point],w.header.frame_id)[0];ax.scatter(wp[0],wp[1],marker='*',s=220,color='#ff5c35',edgecolor='white',label='Exploration waypoint',zorder=9)
ax.set_xlim(extent[:2]);ax.set_ylim(extent[2:]);ax.legend(loc='best',fontsize=8);fig.savefig(out/'03_exploration_centerline.png',dpi=180);plt.close(fig)
print('OUTPUT',out);print(json.dumps(meta,indent=2))
