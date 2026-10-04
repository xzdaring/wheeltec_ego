import time,subprocess,os,signal
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,PointCloud2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
rclpy.init();n=rclpy.create_node('delayed_tf_test');pub=n.create_publisher(LaserScan,'/test_scan',qos_profile_sensor_data);b=TransformBroadcaster(n);got=[]
n.create_subscription(PointCloud2,'/test_cloud',lambda m:got.append(m),qos_profile_sensor_data)
p=subprocess.Popen(['ros2','run','v550_ego_bridge','scan_to_obstacles','--ros-args','-p','scan_topic:=/test_scan','-p','obstacles_topic:=/test_cloud','-p','target_frame:=test_map','-p','transform_timeout:=0.5'],stdout=open('/tmp/all4_tf_node.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
def wait(t):
 end=time.monotonic()+t
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def tf(ns):
 m=TransformStamped();m.header.frame_id='test_map';m.child_frame_id='test_laser';m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9;m.transform.rotation.w=1.;b.sendTransform(m)
def scan(ns):
 m=LaserScan();m.header.frame_id='test_laser';m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9;m.range_min=.1;m.range_max=10.;m.angle_increment=.1;m.ranges=[1.]*8;pub.publish(m)
try:
 wait(2);ns=n.get_clock().now().nanoseconds;tf(ns-3_000_000);wait(.05);scan(ns);wait(.2);assert not got
 tf(ns+3_000_000);wait(.25);assert len(got)==1 and got[0].header.stamp.sec==ns//10**9 and got[0].header.stamp.nanosec==ns%10**9
 scan(ns+10**9);wait(.7);tf(ns+2*10**9);wait(.2);assert len(got)==1,'expired scan replayed'
 print('PASS delayed exact-time TF and expired scan rejection')
finally:
 os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5);n.destroy_node();rclpy.shutdown()
