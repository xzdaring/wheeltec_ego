#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

def normalize_scan(scan,count,fov_min,fov_max):
    """
    将真实雷达数据重新分箱为固定点数。

        为什么需要这个函数：
    1. 实车 M10_P 雷达每圈返回的点数可能略有变化。
    2. SLAM Toolbox/Karto 会缓存第一次扫描的角分辨率和点数。
    3. 如果后续扫描点数变化，SLAM 可能拒绝该帧扫描。
    4. 因此需要把每一帧统一成固定点数、固定角度增量。

    参数：
      scan: 原始 sensor_msgs/LaserScan
      count: 输出点数
      fov_min: 保留的最小角度，单位 rad
      fov_max: 保留的最大角度，单位 rad
    """

    # 输出扫描点数必须至少为 2，否则无法形成有效角度范围
    if count<2:
        raise ValueError('scan_points must be at least2')
    # 最大角度必须大于最小角度，否则无法计算角度增量
    if fov_max <= fov_min:
        raise ValueError('fov_max must be greater than fov_min')

        # 原始扫描不能为空
    if not scan.ranges:
        raise ValueError('scan has no ranges')

    # angle_increment 必须为有限值
    if not math.isfinite(scan.angle_increment):
        raise ValueError('invalid angle_increment')

    # 雷达角度增量必须大于 0
    if scan.angle_increment <= 0.0:
        raise ValueError('angle_increment must be positive')

    result = LaserScan()
    # 保留原始数据的时间戳、frame_id 等信息
    # 不要随意修改 frame_id，SLAM 需要通过 TF 找到 laser 坐标系
    result.header = scan.header
# 输出扫描的固定角度范围
    result.angle_min = fov_min
    result.angle_max = fov_max
    # 计算固定角度增量
    result.angle_increment = (fov_max - fov_min) / (count - 1)
    # 保留原始扫描时间和雷达有效量程
    result.scan_time = scan.scan_time
    result.time_increment = 0.0
    result.range_min = scan.range_min
    result.range_max = scan.range_max

    # 初始化输出距离数组
    # 没有激光返回的位置使用 infinity，表示无效点
    ranges = [float('inf')] * count

    # 只有原始 intensities 长度与 ranges 一致时才处理强度
    has_intensity = len(scan.intensities) == len(scan.ranges)
    intensities = [0.0] * count if has_intensity else []

    #遍历原始雷达的所有激光束
    for index,distance in enumerate(scan.ranges):

        if not math.isfinite(distance):
            continue

        if distance<scan.range_min or distance>scan.range_max:
            continue

         # 计算当前激光束在原始雷达坐标系中的角度
        angle = math.atan2(math.sin(scan.angle_min + index * scan.angle_increment), math.cos(scan.angle_min + index * scan.angle_increment))  # 实车是0到2π，转成负π到π后才能保留右前方扫描。

        # 只保留前方设定视场角内的数据
        if angle < fov_min or angle > fov_max:  # 原来两边都比较最大值会过滤几乎全部障碍点。
            continue

        # 将当前角度映射到输出扫描的固定格
        target = int(round(
            (angle-fov_min) / result.angle_increment
        ))
        #防止浮点误差导致目标索引越界
        if target < 0:
            target = 0
        elif target>=count:
            target = count - 1

        # 如果多个原始激光束映射到同一个输出格，
        # 保留最近的障碍物距离，避免漏掉障碍物
        if distance < ranges[target]:
            ranges[target] = distance

            if has_intensity:
                intensities[target] = scan.intensities[index]

    result.ranges = ranges
    result.intensities = intensities

    return result

class ScanNormalizer(Node):
    """
    实车 M10_P 雷达扫描归一化节点。

    输入：
      /scan

    输出：
      /scan_slam

    使用原则：
    1. Nav2 局部和全局代价地图继续使用原始 /scan。
    2. SLAM Toolbox 使用固定点数的 /scan_slam。
    3. /scan_slam 和 /scan 使用相同 header.frame_id。
    """

    def __init__(self):
        super().__init__('v550_scan_normalizer')

        #原始雷达话题
        self.declare_parameter('input_topic','/scan')
         # 输出给 SLAM Toolbox 的话题
        self.declare_parameter('output_topic','/scan_slam')
         # 输出固定点数
        self.declare_parameter('scan_points',800)

        # 实车360°扫描下界为-π，与角度归一化后的范围一致。
        self.declare_parameter(
            'fov_min',
            -3.141592653589793,
        )

        # 实车360°扫描上界为+π，不再裁掉后方观测。
        self.declare_parameter(
            'fov_max',
            3.141592653589793,
        )

        #读取参数
        self.count = self.get_parameter('scan_points').value
        self.fov_min = self.get_parameter('fov_min').value
        self.fov_max = self.get_parameter('fov_max').value

        #检查输出点数
        if self.count < 2:
            raise ValueError('scan_points must be at least 2')
        
        # 检查角度范围
        if self.fov_max <= self.fov_min:
            raise ValueError('fov_max must be greater than fov_min')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # 创建 LaserScan 发布器
        # 使用 sensor data QoS，与常见雷达驱动保持一致
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )

        # 订阅原始激光数据
        self.scan_subscription = self.create_subscription(  # subscriptions是Node只读属性，使用独立成员保存订阅。
            LaserScan,
            input_topic,
            self.receive_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'normalizing {input_topic} -> {output_topic}, '
            f'points={self.count}, '
            f'fov=[{self.fov_min:.4f}, {self.fov_max:.4f}]'
        )
    def receive_scan(self, scan):  # 补齐订阅回调，原文件引用了不存在的方法。
        try:  # 单帧数据错误只丢弃该帧，不能让SLAM的输入节点退出。
            result = normalize_scan(scan, self.count, self.fov_min, self.fov_max)  # 保留时间戳与laser坐标系，只统一角度采样。
        except ValueError as error:  # 捕获归一化函数明确报告的输入错误。
            self.get_logger().warning(f'跳过无效激光: {error}', throttle_duration_sec=2.0)  # 限频提示异常，避免刷屏。
            return  # 无效帧不发布，避免把错误数据交给SLAM。
        self.publisher.publish(result)  # 发布固定800点扫描，恢复SLAM的数据入口。


def main(args=None):  # 入口必须在类定义外；None让rclpy读取正常命令行参数。
    rclpy.init(args=args)  # 不能把Node类作为参数列表传入DDS初始化。
    node = None  # 构造失败时也能清理ROS上下文。
    try:  # 在类已定义完成后创建节点，避免类定义阶段递归执行入口。
        node = ScanNormalizer()  # 构造真实发布器和订阅器。
        rclpy.spin(node)  # 持续处理激光回调。
    except KeyboardInterrupt:  # 支持终端Ctrl+C正常退出。
        pass  # 不将用户主动退出当作节点故障。
    finally:  # 正常或异常退出均释放节点和上下文。
        if node is not None:node.destroy_node()  # 只有构造成功才销毁节点。
        if rclpy.ok():rclpy.shutdown()  # 避免重复关闭已结束的上下文。


if __name__ == '__main__':  # 导入纯函数做录包测试时不启动节点。
    main()  # 仅作为可执行程序运行时进入ROS事件循环。
