# 录包分析/隔离回放工具。回放只发布规划输入，不启动跟踪器或发布速度。
import sys
BAG_DB=sys.argv[1] if len(sys.argv)>1 else '/ros_workspace/rosbags/rosbag_all4/rosbag_all4_0.db3'
import sqlite3,collections,math,numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
c=sqlite3.connect(BAG_DB);topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')};counts=collections.Counter();warnings=collections.Counter();last={};jumps=[]
for tid,ts,b in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 n,typ=topics[tid];counts[n]+=1
 if n=='/rosout':
  m=deserialize_message(b,typ)
  if m.level>=30:warnings[(m.name,m.msg)]+=1
 if n in ['/ego_global_path','/visual_local_trajectory']:
  m=deserialize_message(b,typ);p=np.array([(a.pose.position.x,a.pose.position.y) for a in m.poses]);key=(m.header.stamp.sec,m.header.stamp.nanosec)
  if n in last and len(p) and len(last[n][1]) and key==last[n][2]:
   prev=last[n][1];d=max(cKDTree(p).query(prev)[0].max(),cKDTree(prev).query(p)[0].max())
   if d>.4:jumps.append((n,ts/1e9,round(d,3),len(prev),len(p)))
  last[n]=(ts,p,key)
print('COUNTS',counts);print('JUMPS',len(jumps),jumps[:30]);print('WARNINGS',warnings.most_common(15))
