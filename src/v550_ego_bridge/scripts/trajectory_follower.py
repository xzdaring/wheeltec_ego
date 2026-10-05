#!/usr/bin/env python3
"""EGO 几何路径 -> V550 SE(2) 运动学轨迹 -> 闭环 /cmd_vel。
几何优化继续由 EGO 完成；这里补车体姿态、轮速时间约束和实测位姿反馈。
"""
import math
import signal
from rclpy.signals import SignalHandlerOptions
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from grid_safety import Grid
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from mecanum_model import wrap, body_velocity, wheels, limit, forward_schedule


def yaw(p):
    q=p.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def stamp(h):
    return h.stamp.sec*1000000000+h.stamp.nanosec


class Follower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        def param(n,v):return self.declare_parameter(n,v).value
        # URDF 轮心给出半轴距/半轮距，STL 外径约74.34mm；实车需重新标定有效半径。
        self.arm=param('half_length',.0788)+param('half_width',.09829)
        self.radius=param('wheel_radius',.03717)
        self.speed=param('wheel_surface_speed',.30)
        self.accel=param('wheel_surface_acceleration',.40)
        self.wmax=param('max_yaw_rate',.6)
        self.footprint=param('collision_radius',.20)
        self.timeout=param('data_timeout',.8)
        self.path_timeout=param('path_timeout',1.5)
        self.xy_tol=param('xy_tolerance',.03)
        self.yaw_tol=param('yaw_tolerance',.04)
        self.terminal_enter=param('terminal_enter_distance',.08)  # 进入终点区域后同时闭环位置和yaw，避免3cm边界反复切换朝向。
        self.terminal_exit=param('terminal_exit_distance',.12)  # 退出距离大于进入距离，定位小幅波动不会反复改变控制模式。
        self.terminal_speed=param('terminal_translation_speed',.03)  # 终点仅允许低速全向微调，不能把此模式用于长距离倒车。
        self.terminal=False  # 每个新导航任务重新建立终点模式，防止继承上一目标状态。
        if not (self.xy_tol < self.terminal_enter < self.terminal_exit) or self.terminal_speed<=0:  # 容差与滞回区间必须有序。
            raise ValueError('终点参数必须满足 xy_tolerance < enter < exit，且微调速度为正')  # 尽早拒绝会引起抖动的参数组合。
        if min(self.arm,self.radius,self.speed,self.accel,self.footprint,self.timeout,self.path_timeout,self.xy_tol,self.yaw_tol,self.wmax)<=0:
            raise ValueError('运动学尺寸、限值及超时必须为正')
        self.grid=None;self.grid_at=0.
        self.pose=None;self.goal=None;self.path=None;self.reference=None;self.cloud=[]
        self.pose_at=self.cloud_at=self.path_at=0.;self.prev=[0.]*4;self.task_id=None
        self.last_tick=time.monotonic();self.last_clock=None;self.clock_advanced_at=self.last_tick
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.vis=self.create_publisher(Path,'/ego_trajectory',10)
        # 此话题仅用于查看计算轮速，不冒充 Gazebo 实际关节状态，也不驱动电机。
        self.wheel_pub=self.create_publisher(JointState,'/ego_wheel_reference',10)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/ego_costmap',self.on_grid,latched)
        self.create_subscription(Path,'/ego_global_path',self.on_goal,latched)
        self.create_subscription(Path,'/visual_local_trajectory',self.on_path,10)
        self.create_subscription(PoseStamped,'/current_pose',self.on_pose,10)
        self.create_subscription(PointCloud2,'/ego_obstacles',self.on_cloud,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)

    def on_grid(self,m):
        if m.header.frame_id=='map':self.grid=Grid(m);self.grid_at=time.monotonic()

    def on_pose(self,m):
        if m.header.frame_id!='map':return
        q=m.pose.orientation
        if not all(math.isfinite(v) for v in [m.pose.position.x,m.pose.position.y,q.x,q.y,q.z,q.w]):return
        self.pose=m.pose;self.pose_at=time.monotonic()

    def on_cloud(self,m):
        if m.header.frame_id!='map':return
        self.cloud=[(float(p[0]),float(p[1])) for p in point_cloud2.read_points(m,field_names=('x','y'),skip_nans=True)]
        self.cloud_at=time.monotonic()

    def stop(self):
        # 失联/碰撞/取消要求立即零速；正常跟踪才使用平滑加减速。
        self.cmd.publish(Twist());self.prev=[0.]*4

    def on_goal(self,m):
        # 同一目标的地图重规划不重置控制器速度；换目标或空路径才立即停车。
        if self.goal is not None and m.poses and stamp(m.header)==self.task_id:
            self.goal=m.poses[-1].pose;return
        self.stop();self.path=None;self.reference=None
        self.terminal=False  # 换目标、取消或完成均清除模式；同任务路径更新不清除。
        self.task_id=stamp(m.header)
        self.goal=m.poses[-1].pose if m.poses and m.header.frame_id=='map' else None
        # 换目标先清空旧轨迹显示，等相同任务 stamp 的局部路径才允许运动。
        empty=Path();empty.header.frame_id='map';self.vis.publish(empty)

    def on_path(self,m):
        if self.goal is None or self.pose is None or m.header.frame_id!='map':return
        if stamp(m.header)!=self.task_id:return  # 延迟到达的上一任务路径不能驱动新任务。
        self.path_at=time.monotonic()
        if not m.poses:
            self.path=None;self.stop();self.vis.publish(m);return
        points=[(p.pose.position.x,p.pose.position.y) for p in m.poses]
        if not all(math.isfinite(x) and math.isfinite(y) for x,y in points):
            self.path=None;self.stop();return
        # EGO只是优化器，不能把穿膨胀区的数值解当作可执行轨迹。
        if self.grid is None or not self.grid.safe([(self.pose.position.x,self.pose.position.y)]+points):
            self.path=None;self.stop();return
        # 局部终点若不是全局终点，不提前强迫车头对齐最终 yaw。
        end=points[-1];dist=math.hypot(end[0]-self.goal.position.x,end[1]-self.goal.position.y)
        total=math.hypot(self.goal.position.x-self.pose.position.x,self.goal.position.y-self.pose.position.y)
        progress=max(0.,min(1.,1-dist/max(total,1e-6)))
        start=yaw(self.pose);target=start+progress*wrap(yaw(self.goal)-start)
        # 可视化姿态与实际前视控制一致，不显示朝终点yaw倒行的旧姿态方案。
        poses,times=forward_schedule(points,start,yaw(self.goal),self.arm,self.speed,self.accel,self.wmax,final=dist<.05)
        if not poses:self.path=None;self.stop();return
        self.path=poses
        out=Path();out.header.frame_id='map';out.header.stamp=m.header.stamp
        t0=self.get_clock().now().nanoseconds
        for p,t in zip(poses,times):
            ps=PoseStamped();ps.header.frame_id='map'
            ns=t0+int(t*1e9);ps.header.stamp.sec=ns//1000000000;ps.header.stamp.nanosec=ns%1000000000
            ps.pose.position.x=p[0];ps.pose.position.y=p[1]
            ps.pose.orientation.z=math.sin(p[2]/2);ps.pose.orientation.w=math.cos(p[2]/2)
            out.poses.append(ps)
        self.reference=out;self.vis.publish(out)

    def tick(self):
        now=time.monotonic();dt=min(.1,max(.001,now-self.last_tick));self.last_tick=now
        clock=self.get_clock().now().nanoseconds
        # /clock 可能只有10Hz而控制20Hz，相同时间戳不等于暂停；持续不推进才停车。
        if self.last_clock is None or clock>self.last_clock:self.clock_advanced_at=now
        if (self.last_clock is not None and clock<self.last_clock) or now-self.clock_advanced_at>self.timeout:
            self.stop();self.last_clock=clock;return
        self.last_clock=clock
        if (self.goal is None or self.pose is None or self.path is None or
            self.grid is None or now-self.grid_at>self.timeout or now-self.pose_at>self.timeout or now-self.cloud_at>self.timeout or now-self.path_at>self.path_timeout):
            self.stop();return
        x=self.pose.position.x;y=self.pose.position.y;theta=yaw(self.pose)
        distance=math.hypot(self.goal.position.x-x,self.goal.position.y-y)
        error=wrap(yaw(self.goal)-theta)
        if distance<=self.xy_tol and abs(error)<=self.yaw_tol:
            self.stop();return
        # 几何跟踪取最近点之后的小前视点；保留 vy，可横移接近目标而不必先掉头。
        nearest=min(range(len(self.path)),key=lambda i:(self.path[i][0]-x)**2+(self.path[i][1]-y)**2)
        index=nearest;length=0.
        while index+1<len(self.path) and length<.12:
            length+=math.hypot(self.path[index+1][0]-self.path[index][0],self.path[index+1][1]-self.path[index][1]);index+=1
        # 前视点不能跨过拐角内侧的膨胀格；缩短前视距离而非沿弦线切墙角。
        while index>nearest and not self.grid.safe([(x,y),self.path[index][:2]]):index-=1
        p=self.path[index];vx,vy=body_velocity(1.2*(p[0]-x),1.2*(p[1]-y),theta)
        # 前方200度雷达不适合长距离倒车：先转向行驶方向，再平移。
        # 保留小幅vy修正（麦轮全向能力），不再沿终点yaw倒着走完整条路径。
        if self.terminal and distance>self.terminal_exit:  # 明显离开终点区才恢复朝行驶方向前进，避免阈值抖动。
            self.terminal=False  # 大位移扰动后重新沿局部路径接近终点。
        elif not self.terminal and distance<=self.terminal_enter:  # 比验收容差更早接管终点位姿。
            self.terminal=True  # 同一目标内保留模式，位置误差跨过3cm也不改追踪朝向。
        if self.terminal:  # 麦轮可独立控制平移与转动，终点处不必先掉头才能修正几毫米位置。
            vx,vy=body_velocity(1.2*(self.goal.position.x-x),1.2*(self.goal.position.y-y),theta)  # 将世界系位置反馈变换到车体系。
            scale=min(1.,self.terminal_speed/max(math.hypot(vx,vy),1e-9))  # 限制平移合速度，保留修正方向。
            vx*=scale  # 允许终点区域内受限的前后微调，后续扫掠碰撞检查仍必须通过。
            vy*=scale  # 保留麦轮横移，同时修正位置以免只转yaw时漂出验收范围。
            angular=error  # 始终对齐用户目标yaw，不再切换到几厘米残差的方位角。
        else:  # 终点区域之外保持原有朝前行驶策略。
            bearing=math.atan2(p[1]-y,p[0]-x)  # 沿当前局部路径前视方向走，避免远距离倒行。
            angular=wrap(bearing-theta)  # 使用最短角度误差控制转向。
            if abs(angular)>.5:vx=vy=0.  # 车头偏离较大时先转向再平移。
            vx=max(0.,vx)  # 常规路径跟踪不允许向后追赶前视点。
        wz=max(-self.wmax,min(self.wmax,1.8*angular))
        (vx,vy,wz),w=limit(vx,vy,wz,self.prev,dt,self.arm,self.speed,self.accel)
        # 用包围整车的圆检查短时扫掠，覆盖任意朝向；只检查中心点会漏掉车角碰撞。
        horizon=max(.4,max(abs(v) for v in w)/self.accel+.15)
        px=x;py=y;heading=theta;swept=[(x,y)]
        for i in range(11):
            if any((ox-px)**2+(oy-py)**2<self.footprint**2 for ox,oy in self.cloud):
                self.stop();return
            step=horizon/10
            px+=(math.cos(heading)*vx-math.sin(heading)*vy)*step
            py+=(math.sin(heading)*vx+math.cos(heading)*vy)*step;heading+=wz*step
            swept.append((px,py))
        # 与全局规划共用余量退出规则；物理碰撞由上面的包络圆始终硬阻止。
        if not self.grid.safe(swept):self.stop();return
        self.prev=w;cmd=Twist();cmd.linear.x=vx;cmd.linear.y=vy;cmd.angular.z=wz;self.cmd.publish(cmd)
        js=JointState();js.header.stamp=self.get_clock().now().to_msg()
        js.name=['lf_wheel','rf_wheel','lb_wheel','rb_wheel'];js.velocity=[v/self.radius for v in w];self.wheel_pub.publish(js)


def main():
    # 保持DDS上下文到finally之后，Ctrl+C/正常SIGTERM退出前仍能发布零速度。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Follower()
    def shutdown_signal(signum, frame):
        # 终端和launch可能先后发SIGINT；第二次信号不能打断发零速和销毁节点。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, shutdown_signal)
    signal.signal(signal.SIGTERM, shutdown_signal)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
