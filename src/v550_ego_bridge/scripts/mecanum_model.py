"""V550 X 型麦轮运动学；轮序 LF/RF/LB/RB，x前/y左/z逆时针。
参数来自本项目 URDF/STL 的几何尺寸，不是厂家电机额定参数。
用轮周线速度限幅可保留全向运动，不需要把 vy 强制置零。
"""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_velocity(dx, dy, yaw):
    # map 中的速度必须旋转到车体系，直接发布 map 速度会在转向后走错方向。
    return math.cos(yaw)*dx + math.sin(yaw)*dy, -math.sin(yaw)*dx + math.cos(yaw)*dy


def wheels(vx, vy, wz, arm):
    return [vx-vy-arm*wz, vx+vy+arm*wz, vx+vy-arm*wz, vx-vy+arm*wz]


def chassis(w, arm):
    return ((w[0]+w[1]+w[2]+w[3])/4,
            (-w[0]+w[1]+w[2]-w[3])/4,
            (-w[0]+w[1]-w[2]+w[3])/(4*arm))


def limit(vx, vy, wz, previous, dt, arm, speed, accel):
    # 等比例缩放四轮，保持 vx:vy:wz 的比例；分别裁剪车体速度会破坏运动方向。
    target = wheels(vx, vy, wz, arm)
    scale = max(1., max(abs(v) for v in target)/speed)
    target = [v/scale for v in target]
    delta = max(abs(a-b) for a,b in zip(target, previous))
    gain = min(1., accel*dt/max(delta, 1e-9))
    out = [a+gain*(b-a) for a,b in zip(previous,target)]
    return chassis(out, arm), out


def schedule(points, start_yaw, end_yaw, arm, speed, accel, angular_speed):
    """为 EGO 几何路径生成连续 yaw 和保守时间间隔，满足四轮速度及加速度界。
    与路径切线独立的 yaw 允许保持车头方向横移；零平移时也支持原地转向。
    """
    lengths=[0.]
    for a,b in zip(points,points[1:]): lengths.append(lengths[-1]+math.hypot(b[0]-a[0],b[1]-a[1]))
    if len(points)<2:return [],[]
    poses=[]
    for i,p in enumerate(points):
        u=lengths[i]/lengths[-1] if lengths[-1]>1e-6 else i/(len(points)-1)
        smooth=u*u*(3-2*u)  # 两端 yaw 斜率为零，避免仅最后一点突然改变车头方向。
        poses.append((p[0],p[1],start_yaw+smooth*wrap(end_yaw-start_yaw)))
    times=[0.]; prev=[0.]*4; durations=[]; velocities=[]
    for a,b in zip(poses,poses[1:]):
        vx,vy=body_velocity(b[0]-a[0],b[1]-a[1],(a[2]+b[2])/2)
        dyaw=wrap(b[2]-a[2]); w=wheels(vx,vy,dyaw,arm)
        # yaw变化期间轮速也变化，用旋转无关上界避免仅在中间yaw限速漏掉峰值。
        dt=max(.02,(math.sqrt(2)*math.hypot(b[0]-a[0],b[1]-a[1])+arm*abs(dyaw))/speed,abs(dyaw)/angular_speed)
        durations.append(dt); velocities.append(w)
    # 对整条时间轴缩放，使离散段轮速变化（含起停）满足加速度界。
    factor=1.
    for i,(w,dt) in enumerate(zip(velocities,durations)):
        v=[x/dt for x in w]; period=(dt+durations[i-1])/2 if i else dt/2
        factor=max(factor,math.sqrt(max(abs(x-y) for x,y in zip(v,prev))/(accel*period)))
        prev=v
    factor=max(factor,math.sqrt(max(abs(x) for x in prev)/(accel*durations[-1]/2)))
    for dt in durations: times.append(times[-1]+factor*dt)
    return poses,times
