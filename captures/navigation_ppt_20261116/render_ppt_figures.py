import pickle,json,math
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Patch,Polygon,Circle
from matplotlib.lines import Line2D
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':14,'axes.unicode_minus':False,'svg.fonttype':'path'})
cache=Path('/tmp/navigation_rows.pkl')
if cache.exists():
 rows=pickle.load(cache.open('rb'))
else:
 import sqlite3
 bag=Path('/home/tzb/bags/grasp_mission_20261116_192535')
 names=['/local_costmap/costmap','/global_costmap/costmap','/plan','/explore/corridor_centerline','/explore/corridor_boundary','/explore/corridor_selection','/selected_viewpoint_vis_cloud','/explore/waypoint','/explore/control_path','/odometry','/tf_static']
 rows={n:[] for n in names}
 for db in sorted(bag.glob('*.db3')):
  c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
  ts={i:(n,t) for i,n,t in c.execute('select id,name,type from topics') if n in rows}
  if ts:
   for tid,stamp,data in c.execute('select topic_id,timestamp,data from messages where topic_id in ('+','.join(map(str,ts))+')'):
    name,typ=ts[tid];rows[name].append((stamp,data,typ))
  c.close()
 for n in rows:rows[n].sort(key=lambda r:r[0])
def dec(r):return deserialize_message(r[1],get_message(r[2]))
def near(n,t):return min(rows[n],key=lambda r:abs(r[0]-t))
r=rows['/explore/corridor_centerline'][17];t=r[0]
chosen={n:near(n,t) for n in rows if rows[n]};msgs={n:dec(v) for n,v in chosen.items()}
# Confirm the recorded static relationship before overlaying the two frames.
valid=False
for rr in rows['/tf_static']:
 for tr in dec(rr).transforms:
  if {tr.header.frame_id,tr.child_frame_id}=={'map','odom_init'}:
   q=tr.transform.rotation;v=tr.transform.translation
   assert np.allclose([v.x,v.y,v.z,q.x,q.y,q.z,abs(q.w)],[0,0,0,0,0,0,1]);valid=True
assert valid,'No recorded identity map/odom_init transform'
out=Path('/home/tzb/nav_explore_ws/captures/navigation_ppt_20261116')
g=msgs['/local_costmap/costmap'];a=np.array(g.data).reshape(g.info.height,g.info.width);np.save(out/'costmap_raw.npy',a)
ox=g.info.origin.position.x;oy=g.info.origin.position.y;res=g.info.resolution
extent=[ox,ox+g.info.width*res,oy,oy+g.info.height*res]
def pts(n):return np.array([[p.pose.position.x,p.pose.position.y] for p in msgs[n].poses])
center=pts('/explore/corridor_centerline');plan=pts('/plan')
od=msgs['/odometry'].pose.pose;robot=np.array([od.position.x,od.position.y]);q=od.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
w=msgs['/explore/waypoint'].point;wp=np.array([w.x,w.y])
bound=np.array([[p.x,p.y] for p in msgs['/explore/corridor_boundary'].polygon.points])
sel=msgs['/explore/corridor_selection'];status=next(m.text for m in sel.markers if m.text)
assert 'INNER_WALL_PAIR' in status and 'FOLLOW' in status
xlim=(5.25,8.0);ylim=(-1.4,.4)
navy='#18324d';blue='#147ad6';orange='#ec7634';green='#119b80'
def setup(num,title,subtitle):
 fig=plt.figure(figsize=(13.3333,7.5),facecolor='white');ax=fig.add_axes([.07,.18,.70,.62])
 fig.text(.055,.925,num,color=blue,fontsize=18,weight='bold');fig.text(.11,.92,title,color=navy,fontsize=26,weight='bold')
 fig.text(.11,.86,subtitle,color='#596b7e',fontsize=13)
 ax.set(xlim=xlim,ylim=ylim,xlabel='X / m',ylabel='Y / m');ax.set_aspect('equal');ax.set_facecolor('#f7f9fc')
 ax.spines[['top','right']].set_visible(False)
 ax.spines[['left','bottom']].set_color('#c6cfd9');ax.tick_params(colors='#66768a',labelsize=11);ax.grid(color='#dbe3eb',lw=.5,alpha=.4,zorder=0)
 fig.text(.07,.06,'历史实测数据  ·  2026-11-16 19:29:14（北京时间）  ·  地图分辨率 0.02 m',fontsize=10,color='#718096')
 return fig,ax
def legend(ax,handles):ax.legend(handles=handles,loc='upper left',bbox_to_anchor=(1.02,1),frameon=False,fontsize=12,labelspacing=1.2,borderaxespad=0)
def save(fig,name):
 fig.savefig(out/(name+'.png'),dpi=240,facecolor='white');fig.savefig(out/(name+'.svg'),facecolor='white');plt.close(fig)
def robot_plot(ax):
 ax.add_patch(Circle(robot,.065,color=navy,zorder=12));ax.annotate('',robot+.25*np.array([math.cos(yaw),math.sin(yaw)]),robot,arrowprops=dict(arrowstyle='-|>',color=navy,lw=2),zorder=13)
def walls(ax,light=False):
 arr=np.ma.masked_where(a<=0,a)
 ax.imshow(arr,origin='lower',extent=extent,cmap=ListedColormap(['#d0d7df' if light else '#45576a']),vmin=0,vmax=100,interpolation='nearest',zorder=1)
fig,ax=setup('01','洞内环境代价地图','双侧墙壁观测清晰，突出通行空间与高代价区域')
cmap=ListedColormap(['#f7f9fc','#ffc97b','#273d53']);norm=BoundaryNorm([-.5,.5,99.5,100.5],3)
ax.imshow(a,origin='lower',extent=extent,cmap=cmap,norm=norm,interpolation='nearest',zorder=1)
legend(ax,[Patch(facecolor='#f7f9fc',edgecolor='#cad3dd',label='自由空间：0'),Patch(facecolor='#ffc97b',label='高代价栅格：99'),Patch(facecolor='#273d53',label='占据栅格：100')])
ax.annotate('左侧墙壁',xy=(6.8,-.08),xytext=(6.7,.23),ha='center',color=navy,arrowprops=dict(arrowstyle='-',color=navy))
ax.annotate('右侧墙壁',xy=(6.8,-1.02),xytext=(6.7,-1.31),ha='center',color=navy,arrowprops=dict(arrowstyle='-',color=navy))
save(fig,'01_costmap')
fig,ax=setup('02','代价地图上的路径规划','突出规划路径、机器人位置与局部规划终点')
walls(ax,True);ax.plot(*plan.T,color='white',lw=9,zorder=5);ax.plot(*plan.T,color=blue,lw=4,zorder=6)
robot_plot(ax);ax.scatter(*plan[-1],s=180,marker='o',facecolor='white',edgecolor=blue,lw=3,zorder=8)
ax.annotate('规划终点',xy=plan[-1],xytext=plan[-1]+[.25,.25],color=blue,arrowprops=dict(arrowstyle='-',color=blue))
legend(ax,[Line2D([],[],color=blue,lw=4,label='规划路径 /plan'),Line2D([],[],marker='o',color=navy,lw=0,label='机器人位置与朝向'),Line2D([],[],marker='o',markerfacecolor='white',markeredgecolor=blue,lw=0,label='规划终点'),Patch(facecolor='#d0d7df',label='地图障碍区域')])
save(fig,'02_planned_path')
fig,ax=setup('03','探索选点与中线约束','候选点筛选 → 双壁中线引导 → 探索目标选择')
walls(ax);ax.add_patch(Polygon(bound,closed=True,facecolor='#cceee7',edgecolor=green,ls='--',lw=1.7,alpha=.55,zorder=2))
for m in sel.markers:
 if m.action!=0 or m.type!=8:continue
 p=np.array([[v.x+m.pose.position.x,v.y+m.pose.position.y] for v in m.points])
 if not len(p):continue
 ax.scatter(*p.T,s=18 if m.id==0 else 33,c='#a7b2bf' if m.id==0 else green,alpha=.6 if m.id==0 else .95,zorder=3 if m.id==0 else 4)
ax.plot(*center.T,color=orange,lw=3,zorder=6);ax.scatter(*wp,s=290,marker='*',c='#d5484d',edgecolors='white',lw=1.2,zorder=10);robot_plot(ax)
legend(ax,[Line2D([],[],marker='o',color='#a7b2bf',lw=0,label='观测候选点：101'),Line2D([],[],marker='o',color=green,lw=0,label='约束内候选点：40'),Patch(facecolor='#cceee7',edgecolor=green,ls='--',label='走廊边界约束'),Line2D([],[],color=orange,lw=3,label='双壁拟合中线'),Line2D([],[],marker='*',markersize=16,color='#d5484d',lw=0,label='探索目标点')])
ax.annotate('探索目标',xy=wp,xytext=wp+[.25,.3],color='#b43439',arrowprops=dict(arrowstyle='-',color='#b43439'),zorder=11)
save(fig,'03_exploration_centerline')
meta={'source_bag':'/home/tzb/bags/grasp_mission_20261116_192535','reference_time':datetime.fromtimestamp(t/1e9,ZoneInfo('Asia/Shanghai')).isoformat(),'registration_status':status,'map_odom_init_transform':'recorded static identity','topics':{n:{'offset_sec':(r[0]-t)/1e9} for n,r in chosen.items() if n!='/tf_static'},'display_crop_map_m':{'x':xlim,'y':ylim},'note':'Recorded values retained. No synthetic walls, inflation, or paths.'}
(out/'metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2))
for n in ['/plan','/explore/corridor_centerline','/explore/corridor_boundary','/explore/corridor_selection','/explore/waypoint']:(out/(n.strip('/').replace('/','_')+'.json')).write_text(json.dumps(message_to_ordereddict(msgs[n]),ensure_ascii=False))
print(out);print(meta['reference_time']);print(status)
