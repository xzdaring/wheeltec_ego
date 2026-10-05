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
for yy in range(48,53):world.data[yy*100+54]=100
state=[0.,0.,0.];cmd=[0.,0.,0.];cmdtime=[0.];count=[0];lastw=[0.]*4;maxwheel=[0.];lateral=[0.];trajectories=[]
def oncmd(m):
 # 前视模式不得发倒车指令；实体矩形墙的距离在积分后独立检查。
 assert m.linear.x>=-1e-7
 cmd[:]=[m.linear.x,m.linear.y,m.angular.z];cmdtime[0]=time.monotonic();maxwheel[0]=max(maxwheel[0],max(abs(w) for w in wheels(*cmd,.17709)));lateral[0]=max(lateral[0],abs(m.linear.y))
sub=n.create_subscription(Twist,'/cmd_vel',oncmd,10);pathsub=n.create_subscription(Path,'/ego_trajectory',lambda m:trajectories.append(m),10)
client=ActionClient(n,NavigateToPose,'/navigate_to_pose');last=[time.monotonic()];feed_pose=True;feed_cloud=True;obstacles=[(.45,-.15+i*.05,0.) for i in range(9)]

def step():
 map_pub.publish(world)
 t=time.monotonic();dt=min(.05,t-last[0]);last[0]=t
 if t-cmdtime[0]<.5:
  x,y,w=cmd;theta=state[2];state[0]+=(math.cos(theta)*x-math.sin(theta)*y)*dt;state[1]+=(math.sin(theta)*x+math.cos(theta)*y)*dt;state[2]=wrap(theta+w*dt)
 # 计算到墙体矩形的欧氏距离，独立于规划器的栅格检查实现。
 dx=max(.4-state[0],0.,state[0]-.5);dy=max(-.2-state[1],0.,state[1]-.3)
 assert math.hypot(dx,dy)>=.20,'robot envelope collided with wall'
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=state[0];p.pose.position.y=state[1];p.pose.orientation.z=math.sin(state[2]/2);p.pose.orientation.w=math.cos(state[2]/2)
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
  f=open('/tmp/'+exe+'_wall.log','w');ps.append(subprocess.Popen(['ros2','run',pkg,exe],stdout=f,stderr=f,start_new_session=True))
 wait(3);assert client.wait_for_server(timeout_sec=2)
 for label,x,y,a in [('wall_detour',.9,0.,0.)]:
  if x is None:x,y=state[:2]
  h=send(x,y,a);res=h.get_result_async();end=time.monotonic()+100
  while not res.done() and time.monotonic()<end:
   step()

  print(label,'state',state,'error',math.hypot(x-state[0],y-state[1]),wrap(a-state[2]),'status',res.result().status if res.done() else 'TIMEOUT',flush=True)
  assert res.done() and res.result().status==4
  wait(.2);assert max(abs(v) for v in cmd)<1e-8
 assert maxwheel[0]<=.30001
 print('WALL DETOUR CLOSED LOOP PASS',flush=True)
finally:
 for p in ps:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in ps:
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
 n.destroy_node();rclpy.shutdown()
