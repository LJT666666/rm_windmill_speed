//
// Created by ljt666666 on 22-8-27.
//

#include "../include/windmill_speed.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

using namespace cv;
using namespace std;

PLUGINLIB_EXPORT_CLASS(rm_windmill_speed::WindSpeed, nodelet::Nodelet)

namespace rm_windmill_speed
{
    void WindSpeed::onInit()
    {
      ros::NodeHandle& nh = getMTPrivateNodeHandle();
      static ros::CallbackQueue my_queue;
      nh.setCallbackQueue(&my_queue);
      initialize(nh);
      my_thread_ = std::thread([]()
      {
        ros::SingleThreadedSpinner spinner;
        spinner.spin(&my_queue);
      });
    }

    void WindSpeed::initialize(ros::NodeHandle& nh)
    {
      nh_ = ros::NodeHandle(nh, "windmill_speed");
      it_ = make_shared<image_transport::ImageTransport>(nh_);

      windmill_cfg_srv_ = new dynamic_reconfigure::Server<rm_windmill_speed::WindmillConfig>(ros::NodeHandle(nh_, "windmill_speed"));
      windmill_cfg_cb_ = boost::bind(&WindSpeed::windmillconfigCB, this, _1, _2);
      windmill_cfg_srv_->setCallback(windmill_cfg_cb_);

      image_pub_ = it_->advertise("debug_image", 1);
      cam_sub_ = it_->subscribeCamera("/hk_camera/image_raw", 1, &WindSpeed::callback, this);
      targets_sub_ = nh.subscribe("/prediction", 1, &WindSpeed::speedCallback, this);
      OriginMsg_pub_ = nh.advertise<std_msgs::Float32>("origin_speed", 1);
      FilteredMsg_pub_ = nh.advertise<std_msgs::Float32>("filtered_speed", 1);
      target_pub_ = nh.advertise<decltype(target_array_)>("/processor/result_msg", 1);
      ///LowPassFilter

    }

    void WindSpeed::windmillconfigCB(rm_windmill_speed::WindmillConfig& config, uint32_t level)
    {
        is_filter_readied_ = config.is_filter_readied;
//        target_type_ = config.target_color;
    }

    void WindSpeed::speedCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg)
    {
        if (msg->detections.empty()){
            ROS_INFO("no target!");
            return;
        }

        InfoTarget info_target;
        info_target.stamp = msg->header.stamp;
        for(auto &object : msg->detections){
            int32_t data[5 * 2];
            memcpy(&data[0], &object.pose.orientation.x, sizeof(int32_t) * 2);
            memcpy(&data[2], &object.pose.orientation.y, sizeof(int32_t) * 2);
            memcpy(&data[4], &object.pose.position.x, sizeof(int32_t) * 2);
            memcpy(&data[6], &object.pose.orientation.z, sizeof(int32_t) * 2);
            memcpy(&data[8], &object.pose.orientation.w, sizeof(int32_t) * 2);
            Target target;
            target.label = object.id;
            for(auto &i : data){
                target.points.push_back(float (i));
//                ROS_INFO("i");
//                ROS_INFO("%f", float(i));
            }
            if(object.id == 10){
                if(updateFan(target, info_target)){
                    speedSolution(info_target);
                }
            }
        }
//        if(updateHistory(info_target))
//            if(is_filter_readied_)
//                bool flag = predict();
    }

    bool WindSpeed::updateFan(Target& object, const InfoTarget& prev_target) {
        if(!init_flag_){
            get_Coordinate(object);
            prev_fan_ = object;
            delat_t_ = prev_target.stamp;
            init_flag_ = true;
            filter_.reset();
            return false;
        }else{
            last_fan_ = prev_fan_;
            get_Coordinate(object);
            prev_fan_ = object;
            return true;
        }
    }

    void WindSpeed::speedSolution(InfoTarget& prev_target) {
//        ROS_INFO("point");
//        ROS_INFO("%f", prev_fan_.armor_center_points.x);
//        ROS_INFO("%f", prev_fan_.armor_center_points.y);
//        ROS_INFO("%f", prev_fan_.r_points.x);
//        ROS_INFO("%f", prev_fan_.r_points.y);
//        ROS_INFO("%f", last_fan_.armor_center_points.x);
//        ROS_INFO("%f", last_fan_.armor_center_points.y);
//        ROS_INFO("%f", last_fan_.r_points.x);
//        ROS_INFO("%f", last_fan_.r_points.y);
        float angle = abs(linesOrientation(prev_fan_.armor_center_points, prev_fan_.r_points, last_fan_.armor_center_points, last_fan_.r_points, 1));
        float delat_t = (prev_target.stamp - delat_t_).toSec();
        ROS_INFO("prev_target.stamp:%f", prev_target.stamp.toSec());
        ROS_INFO("delat_t_:%f", delat_t_.toSec());
        ROS_INFO("angle:%f", angle);
        ROS_INFO("delat_t:%f", delat_t);
        delat_t_ = prev_target.stamp;
        if(isnan((angle / delat_t)))
            return;
        float speed = angle / delat_t;
        std_msgs::Float32 origin_msg;
        std_msgs::Float32 filtered_msg;
        float origin_speed = speed;
//        if(origin_speed > 1.5)
//            origin_speed = 0.5;
//        if(origin_speed <= 0)
//            origin_speed = 0;
//            return;
//        if(abs(origin_speed - wind_speed_) > diff_threshold_)
//            origin_speed = wind_speed_;
//        wind_speed_ = origin_speed;
//        if(abs(origin_speed - wind_speed_) > diff_threshold_)
//            origin_speed = wind_speed_;
        wind_speed_ = origin_speed;
        filter_.input(origin_speed, prev_target.stamp);
        float filtered_speed = filter_.output();
        ROS_INFO("origin_speed:%f", origin_speed);
        ROS_INFO("filtered_speed:%f", filtered_speed);
        origin_msg.data = origin_speed;
        filtered_msg.data = filtered_speed;
        prev_target.speed = origin_speed;
        OriginMsg_pub_.publish(origin_msg);
        FilteredMsg_pub_.publish(filtered_msg);
    }

    void WindSpeed::get_Coordinate(Target& object){
        float prev_total_x;
        float prev_total_y;
        for (int i = 0;i < 2;i++){
            prev_total_x += object.points[2 * i];
            prev_total_y += object.points[2 * i + 1];
        }
        for (int i = 0;i < 2;i++){
            prev_total_x += object.points[2 * i + 6];
            prev_total_y += object.points[2 * i + 6 + 1];
        }

        object.armor_center_points.x = prev_total_x * 0.25;
        object.armor_center_points.y = prev_total_y * 0.25;
        object.r_points.x = object.points[4];
        object.r_points.y = object.points[5];
    }

//根据两直线上两点计算两直线夹角
//当flag=0时返回弧度，当flag!=0时返回角度
    float WindSpeed::linesOrientation(const cv::Point2f& A1, const cv::Point2f& A2, const cv::Point2f& B1, const cv::Point2f& B2, int flag)
    {
        //【1】根据直线上两点计算斜率
        float k_line1 = (A2.y - A1.y)/(A2.x - A1.x);
        float k_line2 = (B2.y - B1.y) / (B2.x - B1.x);
        if(B2.x == B1.x)
            return 0;
        //【2】根据两个斜率计算夹角
        float tan_k;                            //直线夹角正切值
        tan_k = (k_line2 - k_line1) / (1 + k_line2*k_line1);
        float lines_arctan = atan(tan_k);            //反正切获取夹角
//        ROS_INFO("kline1:%f", k_line1);
//        ROS_INFO("kline2:%f", k_line2);
//        ROS_INFO("tan_k:%f", tan_k);
//        ROS_INFO("arctan_k:%f", lines_arctan);
        //【3】以角度或弧度形式返回夹角
        if (flag == 0)
        {
            return lines_arctan;
        }
        else
        {
            return lines_arctan* 180.0 / 3.1415926;
        }
    }

    bool WindSpeed::updateHistory(const InfoTarget& info_target){
        if (history_info_.empty() || (info_target.stamp).toSec() - (history_info_[0].stamp).toSec() >= max_timespan_)
        {
            history_info_.clear();
            history_info_.push_back(info_target);
            params_[0] = 0;
            params_[1] = 0;
            params_[2] = 0;
            params_[3] = 0;
            last_target_ = info_target;
            is_params_confirmed_ = false;
            return false;
        }
        int deque_len;
        if (!is_params_confirmed_)
            deque_len = history_deque_len_cos;
        else
            deque_len = history_deque_len_phase;
        if (history_info_.size() < deque_len)
        {
            history_info_.push_back(info_target);
            last_target_ = info_target;
            return false;
        }
        else if (history_info_.size() == deque_len)
        {
            history_info_.pop_front();
            history_info_.push_back(info_target);
        }
        else if (history_info_.size() > deque_len)
        {
            while(history_info_.size() >= deque_len)
                history_info_.pop_front();
            history_info_.push_back(info_target);
        }
        float rotate_speed_sum;
        for (auto target_info : history_info_)
            rotate_speed_sum += target_info.speed;
        mean_speed_ = rotate_speed_sum / history_info_.size();
        return true;
    }

    bool WindSpeed::predict() {
        //拟合函数: f(x) = a * sin(ω * t + θ) + b， 其中a， ω， θ需要拟合.
        //参数未确定时拟合a， ω， θ

        if (!is_params_confirmed_)
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            ceres::Solver::Summary summary;       // 优化信息
            double params_fitting[4] = {1, 1, 1, mean_speed_};
            //旋转方向，逆时针为正
//            if (rotate_speed_sum / fabs(rotate_speed_sum) >= 0)
//                rotate_sign = 1;
//            else
//                rotate_sign = -1;

            for (auto target_info : history_info_)
            {
                problem.AddResidualBlock (     // 向问题中添加误差项
                        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4> (new CURVE_FITTING_COST ((float)(target_info.stamp).toSec(), target_info.speed)),
                        new ceres::CauchyLoss(0.5),
                        params_fitting                 // 待估计参数
                );
            }

            //设置上下限 f(x) = a * sin(ω * t + θ) + b
            //FIXME:参数需根据场上大符实际调整
            problem.SetParameterLowerBound(params_fitting,0,0.7);
            problem.SetParameterUpperBound(params_fitting,0,1.2);
            problem.SetParameterLowerBound(params_fitting,1,1.6);
            problem.SetParameterUpperBound(params_fitting,1,2.2);
            problem.SetParameterLowerBound(params_fitting,2,-CV_PI);
            problem.SetParameterUpperBound(params_fitting,2,CV_PI);
            problem.SetParameterLowerBound(params_fitting,3,0.5);
            problem.SetParameterUpperBound(params_fitting,3,2.5);

            ceres::Solve(options, &problem, &summary);
            double params_tmp[4] = {params_fitting[0], params_fitting[1], params_fitting[2], params_fitting[3]};
            auto rmse = evalRMSE(params_tmp);
            if (rmse > max_rmse_)
            {
                ROS_INFO("rmse:%f", rmse);
                cout<<summary.BriefReport()<<endl;
                ROS_INFO("[BUFF_PREDICT]RMSE is too high, Fitting failed!");
                return false;
            }
            else
            {
                ROS_INFO("[BUFF_PREDICT]Fitting Succeed! RMSE: %f", rmse);
                params_[0] = params_fitting[0];
                params_[1] = params_fitting[1];
                params_[2] = params_fitting[2];
                params_[3] = params_fitting[3];
                is_params_confirmed_ = true;
            }
        }
            //参数确定时拟合θ
        else
        {
            ceres::Problem problem;
            ceres::Solver::Options options;
            ceres::Solver::Summary summary;       // 优化信息
            double phase;
            for (auto target_info : history_info_)
            {
                problem.AddResidualBlock(     // 向问题中添加误差项
                        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1> (
                                new CURVE_FITTING_COST_PHASE ((float)(target_info.stamp).toSec(),
                                                              (target_info.speed - params_[3]), params_[0], params_[1], params_[3])
                        ),
                        new ceres::CauchyLoss(1e1),
                        &phase                 // 待估计参数
                );
            }

            //设置上下限
            problem.SetParameterLowerBound(&phase,0,-CV_PI);
            problem.SetParameterUpperBound(&phase,0,CV_PI);

            ceres::Solve(options, &problem, &summary);
            double params_new[4] = {params_[0], params_[1], phase, params_[3]};
            auto old_rmse = evalRMSE(params_);
            auto new_rmse = evalRMSE(params_new);
            if (new_rmse < old_rmse)
            {
                ROS_INFO("[BUFF_PREDICT]Params Updated! RMSE: %f", new_rmse);
                params_[2] = phase;
            }
            cout<<"RMSE:"<<new_rmse<<endl;
        }

        //f(x) = a * sin(ω * t + θ) + b
        double a = params_[0];
        double omega = params_[1];
        double theta = params_[2];
        double b = params_[3];
        ROS_INFO("Objective function is : f(x) = %f * sin(%f * t + %f) + %f", a, omega, theta, b);
    }

    double WindSpeed::evalRMSE(double params[4])
    {
        double rmse_sum = 0;
        double rmse = 0;
        for (auto target_info : history_info_)
        {
            auto t = (float)(target_info.stamp).toSec() / 1e3;
            auto pred = params[0] * sin (params[1] * t + params[2]) + params[3];
            auto measure = target_info.speed;
            rmse_sum+=pow((pred - measure),2);
        }
        rmse = sqrt(rmse_sum / history_info_.size());
        return rmse;
    }

}  // namespace rm_windmill_speed
