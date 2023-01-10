//
// Created by ljt666666 on 22-8-27.
//

#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <XmlRpcValue.h>
#include <mutex>
#include <thread>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TargetDetection.h>
#include <dynamic_reconfigure/server.h>
#include <rm_windmill_speed/WindmillConfig.h>
#include "rm_common/filters/lp_filter.h"
#include "std_msgs/Float32.h"
#include <ceres/ceres.h>
#include <Eigen/Core>

using cv::Mat;
using namespace std;
namespace rm_windmill_speed
{
    struct Target
    {
        std::vector<float> points;
        cv::Point3f armor_center_points;
        cv::Point3f r_points;
        int label;
    };

    struct InfoTarget
    {
        ros::Time stamp;
        float speed;
    };

    struct CURVE_FITTING_COST /***曲线拟合损失***/
    {
        CURVE_FITTING_COST (double x, double y) : _x ( x ), _y ( y ) {}
        // 残差的计算
        template <typename T>
        bool operator() (
                const T* params,     // 模型参数，有3维
                T* residual) const     // 残差
        {
            residual[0] = T (_y) - params[0] * ceres::sin(params[1] * T (_x) + params[2]) - params[3]; // f(x) = a * sin(ω * t + θ) + b
            return true;
        }
        const double _x, _y;    // x,y数据
    };

    struct CURVE_FITTING_COST_PHASE/***曲线拟合损失阶段***/
    {
        CURVE_FITTING_COST_PHASE (double x, double y, double a, double omega, double dc) : _x (x), _y (y), _a(a), _omega(omega), _dc(dc){}
        // 残差的计算
        template <typename T>
        bool operator() (
                const T* phase,     // 模型参数，有1维
                T* residual) const     // 残差
        {
            residual[0] = T (_y) - T (_a) * ceres::sin(T(_omega) * T (_x) + phase[0]) - T(_dc); // f(x) = a * sin(ω * t + θ)
            return true;
        }
        const double _x, _y, _a, _omega, _dc;    // x,y数据
    };

    class WindSpeed : public nodelet::Nodelet
{
public:
   WindSpeed():filter_(cutoff_frequency_)
  {
  }
  ~WindSpeed() override
  {
    if (this->my_thread_.joinable())
      my_thread_.join();
  }
  void initialize(ros::NodeHandle& nh);

  void onInit() override;
private:
    std::thread my_thread_;
    rm_msgs::TargetDetectionArray target_array_;
    ros::Publisher target_pub_;
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<rm_windmill_speed::WindmillConfig>* windmill_cfg_srv_;  // server of dynamic config about armor
    dynamic_reconfigure::Server<rm_windmill_speed::WindmillConfig>::CallbackType windmill_cfg_cb_;
    void windmillconfigCB(rm_windmill_speed::WindmillConfig& config, uint32_t level);

    /// transform
    tf2_ros::TransformListener *tf_listener_;
    tf2_ros::Buffer *tf_buffer_;
    ros::Subscriber pose_targets_sub_;
    ros::Publisher pose_targets_pub_;
    bool is_fitting_succeeded_ = false;

    ///windspeed
    bool init_flag_ = false;
    Target prev_fan_{};
    Target last_fan_{};
    ros::Time delat_t_{};
    double wind_speed_{};
    double mean_speed_;
    double diff_threshold_;
    std::vector<Target> objects_;
    ros::Subscriber speed_targets_sub_;
    ros::Publisher OriginMsg_pub_;
    ros::Publisher FilteredMsg_pub_;

    ///predict
    bool re_predict_;
    bool is_start_pred_;
    bool is_params_confirmed_ = false;
    std::deque<InfoTarget> history_info_;
    double max_rmse_;                                               //回归函数最大Cost
    const int max_timespan_ = 20000;                                         //最大时间跨度，大于该时间重置预测器(ms)
    int history_deque_len_cos_;                                  //大符全部参数拟合队列长度
    int history_deque_len_phase_;                                  //大符相位参数拟合队列长度
    double params_[4];
    InfoTarget last_target_;

    ///LowPassFilter
    LowPassFilter filter_;
    double cutoff_frequency_ = -1;


    bool updateFan(Target& object, const InfoTarget& prev_target);

    void speedCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg);

    void speedSolution(InfoTarget& prev_target);

    float getAngle();

    bool updateHistory(const InfoTarget& info_target);

    void predict();

    double evalRMSE(double params[4]);

    void poseCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg);
};

}  // namespace rm_windmill_speed
