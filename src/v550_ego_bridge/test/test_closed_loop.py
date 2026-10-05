# 必须在独立ROS_DOMAIN_ID运行；此测试发布虚拟位置并积分速度，不连接实车。
import os,signal,subprocess,time,math,sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose
from pathlib import Path as FilePath
sys.path.insert(0,str(FilePath(__file__).resolve().parents[1]/'scripts'))
from mecanum_model import wheels,wrap,schedule
rclpy.init();n=Node('v550_kinematic_sim_test');posepub=n.create_publisher(PoseStamped,'/current_pose',10);odompub=n.create_publisher(Odometry,'/odom_combined',10);obspub=n.create_publisher(PointCloud2,'/ego_obstacles',10)
map_pub=n.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
world=OccupancyGrid();world.header.frame_id='map';world.info.width=100;world.info.height=100;world.info.resolution=.1;world.info.origin.position.x=-5.;world.info.origin.position.y=-5.;world.info.origin.orientation.w=1.;world.data=[0]*10000
# 用未知地图复测整条链路，确保不是仅修改RViz颜色而仍拒绝未知目标。
if os.environ.get('V550_TEST_UNKNOWN')=='1':world.data=[-1]*10000
state=[0.,0.,0.];cmd=[0.,0.,0.];cmdtime=[0.];count=[0];lastw=[0.]*4;maxwheel=[0.];lateral=[0.];trajectories=[]
terminal_test=os.environ.get('V550_TEST_TERMINAL')=='1'  # 显式开启all6终点回归，不改变默认三场景测试。
if terminal_test:state[:]=[.00337,.03002,2.97048]  # 平移录包第三个目标到原点，保留摆头时的位置和朝向误差。
def oncmd(m):
 cmd[:]=[m.linear.x,m.linear.y,m.angular.z];cmdtime[0]=time.monotonic();maxwheel[0]=max(maxwheel[0],max(abs(w) for w in wheels(*cmd,.17709)));lateral[0]=max(lateral[0],abs(m.linear.y))
sub=n.create_subscription(Twist,'/cmd_vel',oncmd,10);pathsub=n.create_subscription(Path,'/ego_trajectory',lambda m:trajectories.append(m),10)
client=ActionClient(n,NavigateToPose,'/navigate_to_pose');last=[time.monotonic()];feed_pose=True;feed_cloud=True;obstacles=[(3.,3.,0.)]

def step():
 map_pub.publish(world)
 t=time.monotonic();dt=min(.05,t-last[0]);last[0]=t
 if t-cmdtime[0]<.5:
  x,y,w=cmd;theta=state[2];state[0]+=(math.cos(theta)*x-math.sin(theta)*y)*dt;state[1]+=(math.sin(theta)*x+math.cos(theta)*y)*dt;state[2]=wrap(theta+w*dt)
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=state[0];p.pose.position.y=state[1];p.pose.orientation.z=math.sin(state[2]/2);p.pose.orientation.w=math.cos(state[2]/2)
 if terminal_test:p.pose.position.y+=.001*math.sin(8*t)  # 注入毫米级定位波动，检验3cm边界是否再次触发模式切换。
 if feed_pose:
  posepub.publish(p);o=Odometry();o.header=p.header;o.child_frame_id='base_footprint';o.pose.pose=p.pose;o.twist.twist.linear.x=cmd[0];o.twist.twist.linear.y=cmd[1];o.twist.twist.angular.z=cmd[2];odompub.publish(o)
 if feed_cloud:obspub.publish(point_cloud2.create_cloud_xyz32(p.header,obstacles))
 rclpy.spin_once(n,timeout_sec=.015)
def wait(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:step()
def send(x,y,a):
 g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=n.get_clock().now().to_msg();g.pose.pose.position.x=x;g.pose.pose.position.y=y;g.pose.pose.orientation.z=math.sin(a/2);g.pose.pose.orientation.w=math.cos(a/2)
 f=client.send_goal_async(g);end=time.monotonic()+4
 while not f.done() and time.monotonic()<end:step()
 assert f.done() and f.result().accepted,'goal rejected'
 return f.result()
ps=[]
try:
 for pkg,exe in [('ego_planner','motion_plan'),('v550_ego_bridge','goal_to_path'),('v550_ego_bridge','trajectory_follower.py'),('v550_ego_bridge','global_route.py')]:
  f=open('/tmp/'+exe+'_closed.log','w');ps.append(subprocess.Popen(['ros2','run',pkg,exe],stdout=f,stderr=f,start_new_session=True))
 wait(3);assert client.wait_for_server(timeout_sec=2)
 scenarios=[('lateral',0.,.35,0.),('translation_yaw',.35,.35,math.pi/2),('pure_rotation',None,None,-math.pi/2)]  # 默认保留原来的全链路回归。
 if terminal_test:scenarios=[('all6_noisy_terminal',0.,0.,2.14228187),('terminal_reverse_trim',.06337,.03002,math.pi),('pure_rotation',None,None,-math.pi/2)]  # 验证录包摆头、终点微调和原地转向。
 for label,x,y,a in scenarios:  # 两组场景均检查真实导航动作成功及停车。
  if x is None:x,y=state[:2]
  h=send(x,y,a);res=h.get_result_async();end=time.monotonic()+45
  while not res.done() and time.monotonic()<end:step()
  print(label,'state',state,'error',math.hypot(x-state[0],y-state[1]),wrap(a-state[2]),'status',res.result().status if res.done() else 'TIMEOUT',flush=True)
  assert res.done() and res.result().status==4
  wait(.2);assert max(abs(v) for v in cmd)<1e-8
 assert lateral[0]>(.001 if terminal_test else .02) and maxwheel[0]<=.30001  # 终点微调幅度较小，但必须保留横移能力且不超轮速。
 # 取消必须返回CANCELED并清零速度，不能仅在RViz上移除目标。
 h=send(state[0]+.5,state[1],state[2]);wait(.6);f=h.cancel_goal_async();wait(.5);assert max(abs(v) for v in cmd)<1e-8
 result=h.get_result_async();wait(.3);assert result.done() and result.result().status==5
 print('PASS cancel stop',flush=True)
 h=send(state[0]+.5,state[1],state[2]);wait(.7);feed_cloud=False;wait(1.1);assert max(abs(v) for v in cmd)<1e-8;feed_cloud=True
 print('PASS sensor timeout stop; max wheel=',maxwheel[0], 'max vy=',lateral[0],flush=True)
 # 定位失联和近身障碍也必须停车，独立检查控制器而非只观察规划器输出。
 feed_pose=False;wait(1.1);assert max(abs(v) for v in cmd)<1e-8;feed_pose=True
 obstacles=[(state[0]+.05,state[1],0.)];wait(.4);assert max(abs(v) for v in cmd)<1e-8
 print('PASS pose timeout and footprint collision stop',flush=True)
 h.cancel_goal_async();wait(.4)
 assert any(len(t.poses)>2 for t in trajectories)
 print('ALL CLOSED LOOP CHECKS PASS',flush=True)
finally:
 for p in ps:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in ps:
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
 n.destroy_node();rclpy.shutdown()
