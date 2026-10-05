/*
 * @Function:Ego Planner:Trajectory Optimize Base Bspline
 * @Create by:juchunyu@qq.com
 * @Date:2025-08-02 17:40:01
 */

#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <chrono>
#include <string> // 保存失败阶段供ROS节点统一输出，底层不逐帧刷屏。


namespace  ego_planner
{
    struct PathPoint
    {
        float x=0, y=0, z=0, v=0;
        // 当前实测map平面速度；麦轮可横移，不能由yaw假造速度方向。
        float vx=0, vy=0;
    };

    struct ObstacleInfo
    {
        float x;
        float y;
        float z;
    };

    class PlannerInterface
    {
           
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                                Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                                Eigen::Vector3d local_target_vel,vector<Eigen::Vector3d> point_set);
            PlanParameters pp_;
            LocalTrajData local_data_;
            shared_ptr<GridMap2D> grid_map_;
            PathPoint  cur_pose_;

        private:

            BsplineOptimizer::Ptr bspline_optimizer_rebound_;

            int continous_failures_count_{0};
            std::string failure_reason_; // 累积本轮两种初值各自失败的步骤。

            void updateTrajInfo(const UniformBspline &position_traj);

            void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                            double &time_inc);

            bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

            
        private:

           
            std::vector<PathPoint> _global_plan_traj_;
            std::vector<PathPoint> _plan_traj_results_;
            std::vector<PathPoint> warm_path_;
            vector<vector<Eigen::Vector2d>> a_star_pathes_;



        public:
            PlannerInterface();

            ~PlannerInterface();

            void initParam(double max_vel,double max_acc,double max_jerk);

            void initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d org,double inflate_values);

            void setPathPoint(std::vector<PathPoint> &plan_traj);

            void setObstacles(std::vector<ObstacleInfo> &obstacle);

            void setCurrentVehiclePos(PathPoint& cur_pose);

            void makePlan();
            const std::string& failureReason() const {return failure_reason_;} // 只读查询，本轮成功时节点不会打印。
            void resetTrajectory() {_plan_traj_results_.clear();warm_path_.clear();}
            void setOccupancyQuery(std::function<bool(const Eigen::Vector2d&)> query) {grid_map_->setOccupancyQuery(query);}


            void getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results);  

            void getTraj();

            void setGridMap(PathPoint& cur_pose);

            void getAStarPath(vector<vector<Eigen::Vector2d>>& a_star_path);

            void resetMap();

            bool getInflateOccupancy(PathPoint& world_pos);

            void getObstacles(std::vector<ObstacleInfo> &obstacle);
    };


}


#endif
