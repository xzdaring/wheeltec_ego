#!/usr/bin/env python3
"""维护SLAM+激光统一安全地图；文件名兼容旧launch，全局参考已交给原EGO多项式。"""
import copy,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
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
        self.map=None;self.cloud=[]
        self.map_at=self.cloud_at=0.;self.report=''
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costpub=self.create_publisher(OccupancyGrid,'/ego_costmap',latched)
        self.status=self.create_publisher(String,'/ego_planning_status',latched)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched)
        self.create_subscription(PointCloud2,'/ego_obstacles',self.on_cloud,qos_profile_sensor_data)
        # 全局多项式现由motion_plan生成；本节点只维护统一安全地图，不再调用Dijkstra。
        self.create_timer(.2,self.tick)
    def on_map(self,m):
        if m.header.frame_id!='map' or abs(m.info.origin.orientation.z)>1e-6:return
        self.map=m;self.map_at=time.monotonic()
    def on_cloud(self,m):
        if m.header.frame_id!='map':return
        self.cloud=[(float(p[0]),float(p[1])) for p in point_cloud2.read_points(m,field_names=('x','y'),skip_nans=True)];self.cloud_at=time.monotonic()
    def say(self,s):
        if s!=self.report:self.get_logger().info(s);self.status.publish(String(data=s));self.report=s
    def tick(self):
        now=time.monotonic()
        if self.map is None or now-self.map_at>5 or now-self.cloud_at>1:
            self.say('WAIT_MAP_OR_SCAN: 地图/雷达未就绪或超时');return
        raw=np.array(self.map.data).reshape(self.map.info.height,self.map.info.width)
        occupied=(raw>=50)
        grid=Grid(self.map)
        for x,y in self.cloud:
            ix,iy=grid.cell(x,y)
            if 0<=ix<grid.w and 0<=iy<grid.h:occupied[iy,ix]=True
        cost=copy.deepcopy(self.map);cost.header.stamp=self.get_clock().now().to_msg()
        # 未探索不等于障碍：仅膨胀SLAM实测占用及当前激光，允许向未知区规划。
        # 这是乐观规划，执行仍依赖新鲜激光和逐周期碰撞检查；地图之外仍有有限边界。
        costs=inflate(occupied,grid.r,self.physical,self.margin)
        cost.data=costs.ravel().tolist()
        self.costpub.publish(cost)
        self.say('MAP_READY: 统一安全地图可用，全局参考与局部优化由motion_plan负责')

def main():
    rclpy.init();n=Router()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
