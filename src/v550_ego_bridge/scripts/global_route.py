#!/usr/bin/env python3
"""用SLAM地图与当前激光构造统一安全地图，再以Dijkstra产生绕障参考路径。"""
import copy,math,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from grid_safety import Grid,inflate

class Router(Node):
    def __init__(self):
        super().__init__('global_route')
        self.physical=self.declare_parameter('collision_radius',.20).value
        self.margin=self.declare_parameter('inflation_radius',.25).value
        if not 0<self.physical<self.margin:raise ValueError('必须保留膨胀安全余量')
        self.map=None;self.cloud=[];self.pose=None;self.request=None;self.last_path=None
        self.pose_at=0.;self.map_at=self.cloud_at=0.;self.last_publish=0.;self.report=''
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(Path,'/ego_global_path',latched)
        self.costpub=self.create_publisher(OccupancyGrid,'/ego_costmap',latched)
        self.status=self.create_publisher(String,'/ego_planning_status',latched)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched)
        self.create_subscription(PointCloud2,'/ego_obstacles',self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/current_pose',self.on_pose,10)
        self.create_subscription(Path,'/ego_reference_request',self.on_request,latched)
        self.create_timer(.2,self.tick)
    def on_map(self,m):
        if m.header.frame_id!='map' or abs(m.info.origin.orientation.z)>1e-6:return
        self.map=m;self.map_at=time.monotonic()
    def on_cloud(self,m):
        if m.header.frame_id!='map':return
        self.cloud=[(float(p[0]),float(p[1])) for p in point_cloud2.read_points(m,field_names=('x','y'),skip_nans=True)];self.cloud_at=time.monotonic()
    def on_pose(self,m):
        if m.header.frame_id=='map':self.pose=m.pose;self.pose_at=time.monotonic()
    def on_request(self,m):
        self.request=m if m.poses else None;self.last_path=None
        # 新目标立即清除旧规划，地图准备好后再允许跟踪。
        empty=Path();empty.header=m.header;self.pub.publish(empty)
    def say(self,s):
        if s!=self.report:self.get_logger().info(s);self.status.publish(String(data=s));self.report=s
    def tick(self):
        now=time.monotonic()
        if self.map is None or now-self.map_at>5 or now-self.cloud_at>1:
            if self.last_path is not None:
                empty=Path();empty.header=self.last_path.header;self.pub.publish(empty);self.last_path=None
            self.say('WAIT_MAP_OR_SCAN: 地图/雷达未就绪或超时');return
        raw=np.array(self.map.data).reshape(self.map.info.height,self.map.info.width)
        occupied=(raw>=50)
        grid=Grid(self.map)
        for x,y in self.cloud:
            ix,iy=grid.cell(x,y)
            if 0<=ix<grid.w and 0<=iy<grid.h:occupied[iy,ix]=True
        cost=copy.deepcopy(self.map);cost.header.stamp=self.get_clock().now().to_msg()
        # 未知代表没有观测，不是实体墙；不向已观测自由区膨胀未知边界。
        # 未知格自身仍禁止通行，真实墙体和当前激光则按完整车体半径膨胀。
        costs=inflate(occupied,grid.r,self.physical,self.margin)
        unknown=raw<0
        if self.pose is not None and now-self.pose_at<.8:
            # 雷达装在车头，SLAM可能把车身当前占据的区域标成未知。
            # 仅豁免当前车体内的未知格；不清除任何实测障碍或其膨胀，不放开后方未知通路。
            yy,xx=np.ogrid[:grid.h,:grid.w]
            under_body=(grid.ox+(xx+.5)*grid.r-self.pose.position.x)**2+(grid.oy+(yy+.5)*grid.r-self.pose.position.y)**2<=self.physical**2
            unknown=unknown & ~under_body
        costs[unknown]=100
        cost.data=costs.ravel().tolist()
        self.costpub.publish(cost);grid=Grid(cost)
        if self.request is None or self.pose is None:return
        start=(self.pose.position.x,self.pose.position.y)
        target=self.request.poses[-1].pose;goal=(target.position.x,target.position.y)
        # 每秒更新一次路线；地图立即发布，控制器始终按最新地图检查。
        if now-self.last_publish<1 and self.last_path is not None:return
        self.last_publish=now
        points=grid.route(start,goal)
        if not points:
            empty=Path();empty.header=self.request.header;self.pub.publish(empty);self.last_path=None
            self.say('NO_SAFE_ROUTE: 起点实体碰撞/目标膨胀占用/通路未知，保持停车');return
        path=Path();path.header=self.request.header
        # 至少6段，避免短路径进入EGO的三次B样条时点数不足。
        for a,b in zip(points,points[1:]):
            count=max(3,math.ceil(math.dist(a,b)/.10))
            for i in range(count):
                p=PoseStamped();p.header=path.header;p.pose.position.x=a[0]+(b[0]-a[0])*i/count;p.pose.position.y=a[1]+(b[1]-a[1])*i/count
                p.pose.orientation.w=1.;path.poses.append(p)
        p=PoseStamped();p.header=path.header;p.pose=target;path.poses.append(p)
        self.pub.publish(path);self.last_path=path;self.say('ROUTE_READY: 全局参考路径已通过统一膨胀地图检查')

def main():
    rclpy.init();n=Router()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
