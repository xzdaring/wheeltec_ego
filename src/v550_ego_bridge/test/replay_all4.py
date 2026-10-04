# 录包分析/隔离回放工具。回放只发布规划输入，不启动跟踪器或发布速度。
import sys
BAG_DB=sys.argv[1] if len(sys.argv)>1 else '/ros_workspace/rosbags/rosbag_all4/rosbag_all4_0.db3'
import sqlite3,time,subprocess,signal,os
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav_msgs.msg import Path
rclpy.init();n=rclpy.create_node('all4_replay_analysis');last=[];jumps=[];received=[]
def cb(m):
 p=np.array([(a.pose.position.x,a.pose.position.y) for a in m.poses]);key=(m.header.stamp.sec,m.header.stamp.nanosec);received.append(len(p))
 if last and len(p) and len(last[0]) and key==last[1]:
  d=max(cKDTree(p).query(last[0])[0].max(),cKDTree(last[0]).query(p)[0].max())
  if d>.4:jumps.append(round(d,3))
 last[:]=[p,key]
n.create_subscription(Path,'/visual_local_trajectory',cb,10)
c=sqlite3.connect(BAG_DB);pubs={}
for tid,name,typ in c.execute('select id,name,type from topics'):
 if name not in ['/map','/current_pose','/ego_obstacles','/ego_reference_request','/clock']:continue
 q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if name in ['/map','/ego_reference_request'] else qos_profile_sensor_data if name=='/ego_obstacles' else 10
 t=get_message(typ);pubs[tid]=(n.create_publisher(t,name,q),t)
ps=[]
try:
 for pkg,exe in [('ego_planner','motion_plan'),('v550_ego_bridge','global_route.py')]:
  f=open('/tmp/all4_replay_'+exe+'.log','w');ps.append(subprocess.Popen(['ros2','run',pkg,exe,'--ros-args','-p','use_sim_time:=true'],stdout=f,stderr=f,start_new_session=True))
 time.sleep(2);first=c.execute('select min(timestamp) from messages').fetchone()[0];start=time.monotonic()
 for tid,ts,data in c.execute('select topic_id,timestamp,data from messages where timestamp<? order by timestamp',(first+40*10**9,)):
  if tid not in pubs:continue
  deadline=start+(ts-first)/1e9
  while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=min(.01,max(0.,deadline-time.monotonic())))
  pub,t=pubs[tid];pub.publish(deserialize_message(data,t));rclpy.spin_once(n,timeout_sec=0.)
 print('REPLAY_FIRST_40S local_messages',len(received),'nonempty',sum(v>0 for v in received),'jumps_over_0.4m',jumps,flush=True)
finally:
 for p in ps:os.killpg(p.pid,signal.SIGINT)
 for p in ps:p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown()
