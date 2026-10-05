#include "planner_interface.h"
#include <traj_utils/polynomial_traj.h>

namespace ego_planner
{

    PlannerInterface::PlannerInterface()
    {

    }

    PlannerInterface::~PlannerInterface()
    {

    }

    void PlannerInterface::initParam(double max_vel,double max_acc,double max_jerk)
    {
        pp_.max_vel_ = max_vel;
        pp_.max_acc_ = max_acc;
        pp_.max_jerk_ = max_jerk;
        pp_.feasibility_tolerance_ = 0.05;
        pp_.ctrl_pt_dist = 0.12;
        pp_.planning_horizen_ = 5.0;
    }
    
    void PlannerInterface::initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d origin,double inflate_values)
    {
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "x_size =" << x_size << " y_size =" << y_size << " z_size" << z_size << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "resolution =" << resolution << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "origin =" << origin << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "inflate_values =" << inflate_values << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        //初始化gridmap地图
        Eigen::Vector2i map_size(x_size,y_size);     
        grid_map_     = std::make_shared<GridMap2D>(resolution,map_size);
        grid_map_->setInflateRadius(inflate_values);

        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam();
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(100, 100));
       
    }

    void PlannerInterface::setPathPoint(std::vector<PathPoint> &plan_traj)
    {
        _global_plan_traj_.clear();
        _global_plan_traj_ = plan_traj;
    }
    
    void PlannerInterface::setObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        for(int i = 0; i < obstacle.size();i++)
        {
            Eigen::Vector2d coord(obstacle[i].x,obstacle[i].y);
            Eigen::Vector2i res;
            res = grid_map_->worldToGrid(coord);
            grid_map_->setObstacle(res, true);
        }

        grid_map_->inflate();

        // for(int i = 0; i < obstacle.size();i++)
        // {
        //     Eigen::Vector3d pos;
        //     pos[0] = obstacle[i].x;
        //     pos[1] = obstacle[i].y;
        //     pos[2] = 1;
        //     grid_map_->getInflateOccupancy(pos);
        //     std::cout << "grid_map_->getInflateOccupancy(pos); =pos " << pos << " ," << grid_map_->getInflateOccupancy(pos) << std::endl;
        // }
       

    }

    void PlannerInterface::getObstacles(std::vector<ObstacleInfo> &obstacle)
    {
        std::vector<Eigen::Vector2d> inflated_cloud = grid_map_->getObstaclePointCloud();

        for (const auto& point : inflated_cloud)
        {
            ObstacleInfo temp;
            temp.x = static_cast<float>(point.x()); // 解析 x
            temp.y = static_cast<float>(point.y()); // 解析 y
            obstacle.push_back(temp);
        }



        //         // 1. 获取膨胀后的障碍物点云（默认）
        // std::vector<Eigen::Vector2d> inflated_cloud = grid_map.getObstaclePointCloud();

        // // 2. 获取原始障碍物点云
        // std::vector<Eigen::Vector2d> raw_cloud = grid_map.getObstaclePointCloud(false);
    }




    void PlannerInterface::setCurrentVehiclePos(PathPoint& cur_pose)
    {
        cur_pose_ = cur_pose;
        // grid_map_->resetGrids();
    }  

    void PlannerInterface::setGridMap(PathPoint& cur_pose)
    {
        grid_map_->setCurPose(cur_pose.x,cur_pose.y); // only once
    }

    void PlannerInterface::resetMap()
    {
        grid_map_->resetMap();
    }

    bool PlannerInterface::getInflateOccupancy(PathPoint& traj)
    {
        Eigen::Vector2d pos;
        pos[0] = traj.x;
        pos[1] = traj.y;
        return grid_map_->getInflateOccupancy(pos);
    }


    void PlannerInterface::makePlan()
    {
        failure_reason_.clear(); // 每轮清空，避免把上次失败原因混入本轮。
        // 每次尝试前清空旧结果，规划失败不能继续输出上一个目标的轨迹。
        // 失败结果为空时仍保留上次成功初值，避免每次失败又退回穿墙直线。
        if (!_plan_traj_results_.empty()) warm_path_ = _plan_traj_results_;
        _plan_traj_results_.clear();
        if (_global_plan_traj_.empty()) {failure_reason_="INPUT: 局部参考为空";return;} // 明确入口失败，不能误报成碰撞。 // 底层parameterizeToBspline要求至少4点。
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "开始规划..." << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        Eigen::Vector3d start_pt;
        Eigen::Vector3d start_vel;
        Eigen::Vector3d start_acc;
        Eigen::Vector3d local_target_pt;
        Eigen::Vector3d local_target_vel;
        vector<Eigen::Vector3d> point_set;


        vector<Eigen::Vector3d> traj_pts;
        
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "point set" << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        for(int i = 0; i< _global_plan_traj_.size();i++)
        {
            Eigen::Vector3d plan_pt(_global_plan_traj_[i].x,_global_plan_traj_[i].y,0.2);
            { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "_global_plan_traj_ x =" << _global_plan_traj_[i].x << " y = " << _global_plan_traj_[i].y << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
            point_set.push_back(plan_pt);
        }
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "point set end" << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。

        start_pt[0] = cur_pose_.x ;//_global_plan_traj_[0].x;
        start_pt[1] = cur_pose_.y;//_global_plan_traj_[0].y;
        start_pt[2] = 0.0;
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "[makePlan] start_pt x = " << start_pt[0] << " start_pt y =" << start_pt[1] << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
        
        local_target_pt[0] = _global_plan_traj_[_global_plan_traj_.size()-1].x;
        local_target_pt[1] = _global_plan_traj_[_global_plan_traj_.size()-1].y;
        local_target_pt[2] = 0;

        // 由里程计车体系速度旋转得到map速度，静止时起始速度必须为零。
        start_vel = Eigen::Vector3d(cur_pose_.vx, cur_pose_.vy, 0.0);
        local_target_vel.setZero();
        // 未提供加速度观测，不再把车头单位向量误当成1m/s²。
        start_acc.setZero();


        auto start = std::chrono::system_clock::now();

        if(point_set.size() < 3) 
        {
            { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
printf("全局轨迹点输入点数过少，不开启优化器，返回，请检查全局轨迹点输入的正确性");
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
            return;
        }
          
        bool plan_success = reboundReplan(start_pt,start_vel, start_acc,local_target_pt,local_target_vel, point_set);
        if (plan_success)
           getTraj();
           
        auto end = std::chrono::system_clock::now();

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
printf("MotionPlanner Total Running Time: %d  ms \n", elapsed.count());
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
    }

    void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results)
    {
        plan_traj_results = _plan_traj_results_;
    }
    
    void PlannerInterface::getAStarPath(vector<vector<Eigen::Vector2d>>& a_star_path)
    {
        a_star_path = a_star_pathes_;

    }


    bool PlannerInterface::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set)
    {
        // 实体/膨胀内端点不可优化，避免A*自行移动端点而改变任务含义。
        if(grid_map_->getInflateOccupancy(start_pt.head<2>()) || grid_map_->getInflateOccupancy(local_target_pt.head<2>())){failure_reason_="ENDPOINT: 起点或局部目标占用";return false;} // 单独标记端点检查。
        // 复用无人机五次多项式初值，以当前实测速度作为起点边界，固定z=0。
        double duration=std::max(1.,2.0*(local_target_pt-start_pt).norm()/pp_.max_vel_);
        auto initial=PolynomialTraj::one_segment_traj_gen(start_pt,start_vel,start_acc,local_target_pt,local_target_vel,Eigen::Vector3d::Zero(),duration);
        initial.init();point_set.clear();
        int count=std::max(6,int(std::ceil((local_target_pt-start_pt).norm()/pp_.ctrl_pt_dist)));
        for(int i=0;i<=count;++i)point_set.push_back(initial.evaluate(duration*double(i)/count));
        // 类似planFromCurrentTraj，先利用旧优化轨迹的未走部分作为warm start。
        // 地面跟踪器按反馈位置而非无人机时间轴执行，因此按最近位置裁剪，边界速度仍取实测。
        if(warm_path_.size()>4){
            size_t nearest=0;
            for(size_t i=1;i<warm_path_.size();++i)
                if(std::hypot(warm_path_[i].x-start_pt.x(),warm_path_[i].y-start_pt.y())<std::hypot(warm_path_[nearest].x-start_pt.x(),warm_path_[nearest].y-start_pt.y()))nearest=i;
            auto last=warm_path_.back();
            if(std::hypot(warm_path_[nearest].x-start_pt.x(),warm_path_[nearest].y-start_pt.y())<.15 && std::hypot(last.x-local_target_pt.x(),last.y-local_target_pt.y())<.5){
                vector<Eigen::Vector3d> seed{start_pt};
                for(size_t i=nearest;i<warm_path_.size();++i)seed.emplace_back(warm_path_[i].x,warm_path_[i].y,0);
                seed.push_back(local_target_pt);
                vector<double> lengths{0.};for(size_t i=1;i<seed.size();++i)lengths.push_back(lengths.back()+(seed[i]-seed[i-1]).norm());
                if(lengths.back()>.05){
                    point_set.clear();size_t j=1;
                    for(int i=0;i<=count;++i){double arc=lengths.back()*i/count;while(j+1<seed.size() && lengths[j]<arc)++j;
                        double u=(arc-lengths[j-1])/std::max(1e-9,lengths[j]-lengths[j-1]);point_set.push_back(seed[j-1]+u*(seed[j]-seed[j-1]));}
                }
            }
        }
        vector<Eigen::Vector3d> start_end_derivatives;
        double ts = duration/count; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(start_acc);

        // 第一轮使用多项式/旧优化曲线，失败再复用原2D A*生成局部初始化；绝不执行A*折线。
        for(int attempt=0;attempt<2;++attempt){
            failure_reason_ += attempt==0 ? "[多项式/旧轨迹] " : "[局部A*初值] "; // 保留每次尝试的上下文。
            if(attempt==1){
                if(!bspline_optimizer_rebound_->a_star_->AstarSearch(.1,start_pt.head<2>(),local_target_pt.head<2>())){failure_reason_+="ASTAR: 搜索无解; ";break;} // 区分搜索失败与优化失败。
                auto guide=bspline_optimizer_rebound_->a_star_->getPath();
                if(guide.size()<2){failure_reason_+="ASTAR: 引导点不足; ";break;} // 防止空路径被当作优化器故障。
                guide.front()=start_pt.head<2>();guide.back()=local_target_pt.head<2>();
                vector<double> arc{0.};for(size_t i=1;i<guide.size();++i)arc.push_back(arc.back()+(guide[i]-guide[i-1]).norm());
                int n=std::max(6,int(std::ceil(arc.back()/pp_.ctrl_pt_dist)));point_set.clear();size_t j=1;
                for(int i=0;i<=n;++i){double d=arc.back()*i/n;while(j+1<guide.size() && arc[j]<d)++j;
                    auto xy=guide[j-1]+(guide[j]-guide[j-1])*(d-arc[j-1])/std::max(1e-9,arc[j]-arc[j-1]);point_set.emplace_back(xy.x(),xy.y(),0.);}
                ts=std::max(.2,2.*arc.back()/pp_.max_vel_/n);
            }
            Eigen::MatrixXd ctrl_pts_3d;
            UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts_3d);
            Eigen::MatrixXd ctrl_pts_2d=ctrl_pts_3d.topRows(2);
            a_star_pathes_=bspline_optimizer_rebound_->initControlPoints(ctrl_pts_2d,true);
            if(!bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts_2d,ts)){failure_reason_+="REBOUND: 样条反弹优化失败; ";continue;} // 记录L-BFGS/内部碰撞检查失败。
            ctrl_pts_3d=bspline_optimizer_rebound_->convert2DTo3D(ctrl_pts_2d);
            UniformBspline pos(ctrl_pts_3d,3,ts);
            pos.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
            double ratio;
            if(!pos.checkFeasibility(ratio,false)){
                Eigen::MatrixXd refined;
                if(!refineTrajAlgo(pos,start_end_derivatives,ratio,ts,refined)){failure_reason_+="REFINE: 时间重分配后精修失败; ";continue;} // 精修失败独立于初次优化。
                pos=UniformBspline(refined,3,ts);
                pos.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
                if(!pos.checkFeasibility(ratio,false)){failure_reason_+="DYNAMICS: 精修后速度/加速度仍超限; ";continue;} // 便于识别限制过严或时间伸缩不足。
            }
            // 原版只检前2/3用于滚动规划；地面跟踪器会收到整段，必须验证完整曲线。
            bool safe=true;
            for(double t=0.;t<=pos.getTimeSum();t+=.02){
                if(grid_map_->getInflateOccupancy(pos.evaluateDeBoorT(t).head<2>())){safe=false;break;}
            }
            if(!safe){failure_reason_+="FULL_COLLISION: 优化后整段曲线仍碰撞; ";continue;} // 原生2/3检查通过不等于整段安全。
            updateTrajInfo(pos);continous_failures_count_=0;
            { // 默认关闭逐帧调试输出，终端只保留ROS失败阶段汇总。
if (false) { // 如需底层数值调试可临时开启，不影响规划逻辑。
std::cout << "EGO_OPTIMIZED: seed=" << (attempt==0?"polynomial_or_previous":"local_astar") << std::endl;
} // 调试输出结束。
} // 用外层块保持原有if/else语义。
            return true;
        }
        ++continous_failures_count_;
        return false;
    }

    bool PlannerInterface::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {

        Eigen::MatrixXd optimal_control_point_temp;
        double t_inc;

        Eigen::MatrixXd ctrl_pts; 

        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        Eigen::MatrixXd ctrl_pts_2d;

        ctrl_pts_2d = ctrl_pts.topRows(2);

        traj = UniformBspline(ctrl_pts, 3, ts);

        

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);

        bspline_optimizer_rebound_->ref_pts_.clear();

        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t).topRows(2));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts_2d, ts, optimal_control_point_temp);

        optimal_control_points = bspline_optimizer_rebound_->convert2DTo3D(optimal_control_point_temp);
         
        return success;
    }

    void PlannerInterface::updateTrajInfo(const UniformBspline &position_traj)
    {
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void PlannerInterface::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                            Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }

        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

    void PlannerInterface::getTraj()
    {
        auto info = &local_data_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        Eigen::VectorXd knots = info->position_traj_.getKnot();

        vector<ego_planner::UniformBspline> traj_;
        double traj_duration_;

        constexpr int spline_order = 3;
        ego_planner::UniformBspline pos_traj(pos_pts, spline_order, 0.1);
        pos_traj.setKnot(knots);
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(traj_[0].getDerivative());
        traj_.push_back(traj_[1].getDerivative());
        traj_duration_ = traj_[0].getTimeSum();


        Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
        _plan_traj_results_.clear();
        for (double t_cur = 0; t_cur < traj_duration_; t_cur += 0.1)
        {
            pos = traj_[0].evaluateDeBoorT(t_cur);
            vel = traj_[1].evaluateDeBoorT(t_cur);
            acc = traj_[2].evaluateDeBoorT(t_cur);
            PathPoint tempPath{};
            tempPath.x = pos(0);
            tempPath.y = pos(1);
            _plan_traj_results_.push_back(tempPath);
        }
        // 固定步长可能跨过最后时刻，显式保留端点以免小车永远差最后几厘米。
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        PathPoint endpoint{}; endpoint.x=pos(0); endpoint.y=pos(1);
        _plan_traj_results_.push_back(endpoint);
    }

}
