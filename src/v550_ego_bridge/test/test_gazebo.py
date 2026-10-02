import os,signal,subprocess,time,math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
rclpy.init();n=Node('gazebo_acceptance');pose=[None];cmd=[None];paths=[]
ps=n.create_subscription(PoseStamped,'/current_pose',lambda m:pose.__setitem__(0,m),10);cs=n.create_subscription(Twist,'/cmd_vel',lambda m:cmd.__setitem__(0,m),10);ts=n.create_subscription(Path,'/ego_trajectory',lambda m:paths.append(m),10);a=ActionClient(n,NavigateToPose,'/navigate_to_pose')
log=open('/tmp/v550_gazebo_acceptance.log','w');proc=subprocess.Popen(['ros2','launch','v550_ego_bridge','ego_sim.launch.py','use_gui:=false','use_rviz:=false'],stdout=log,stderr=log,start_new_session=True)
def wait_until(fn,secs):
 end=time.monotonic()+secs
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if fn():return True
 return False
try:
 assert wait_until(lambda:pose[0] is not None,45),'No SLAM/current_pose'
 assert a.wait_for_server(timeout_sec=5)
 start=pose[0].pose;g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=start.position.x+.30;g.pose.pose.position.y=start.position.y+.15;g.pose.pose.orientation.z=math.sin(.5/2);g.pose.pose.orientation.w=math.cos(.5/2)
 f=a.send_goal_async(g);assert wait_until(f.done,5) and f.result().accepted
 result=f.result().get_result_async();assert wait_until(result.done,55),'Goal timeout'
 p=pose[0].pose;err=math.hypot(p.position.x-g.pose.pose.position.x,p.position.y-g.pose.pose.position.y);yaw=2*math.atan2(p.orientation.z,p.orientation.w)
 print('GAZEBO status=',result.result().status,'position_error=',err,'yaw_error=',yaw-.5,'trajectories=',len(paths),flush=True)
 assert result.result().status==4 and err<.04 and abs(yaw-.5)<.05
 assert wait_until(lambda:cmd[0] is not None and abs(cmd[0].linear.x)+abs(cmd[0].linear.y)+abs(cmd[0].angular.z)<1e-8,2)
 print('GAZEBO CLOSED LOOP PASS',flush=True)
finally:
 if proc.poll() is None:proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=12)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 n.destroy_node();rclpy.shutdown()
