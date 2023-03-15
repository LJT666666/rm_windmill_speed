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
        if(!nh.getParam("is_start_pred", is_start_pred_))
        ROS_WARN("No is_filter_readied specified");
        if(!nh.getParam("diff_threshold", diff_threshold_))
            ROS_WARN("No diff_threshold specified");
        if(!nh.getParam("max_rmse", max_rmse_))
            ROS_WARN("No max_rmse specified");
        if(!nh.getParam("history_deque_len_cos", history_deque_len_cos_))
            ROS_WARN("No history_deque_len_cos specified");
        if(!nh.getParam("history_deque_len_phase", history_deque_len_phase_))
            ROS_WARN("No history_deque_len_phase specified");
        if(!nh.getParam("re_predict", re_predict_))
            ROS_WARN("No re_predict specified");

        std::vector<float> intrinsic;
        std::vector<float> distortion;
        if(!nh.getParam("/windspeed/camera_matrix/data", intrinsic))
            ROS_WARN("No cam_intrinsic_mat_k specified");
        if(!nh.getParam("/windspeed/distortion_coefficients/data", distortion))
            ROS_WARN("No distortion specified");

        cam_intrinsic_mat_k_ = cv::Matx<float, 3, 3>(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5],
                                                    intrinsic[6], intrinsic[7], intrinsic[8]);
        std::cout << "intrinsic maxtric is: " << cam_intrinsic_mat_k_ << std::endl;
        dist_coefficients_ = cv::Matx<float, 1, 5>(distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]);

        windmill_cfg_srv_ = new dynamic_reconfigure::Server<rm_windmill_speed::WindmillConfig>(ros::NodeHandle(nh_, "windmill_speed"));
        windmill_cfg_cb_ = [this](auto && PH1, auto && PH2) { windmillconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); };
        windmill_cfg_srv_->setCallback(windmill_cfg_cb_);

        tf_buffer_ = new tf2_ros::Buffer(ros::Duration(10));
        tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);

//        sensor_msgs::CameraInfoConstPtr camera_info{};
//        pnp_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(
//                "/galaxy_camera/camera_info", 10,
//                [&camera_info](const sensor_msgs::CameraInfoConstPtr& info) -> void { camera_info = info; });

//        ROS_ASSERT(camera_info != nullptr);
//        if (camera_info == nullptr)
//            cout << camera_info->K.data() << endl;
//        memcpy(cam_intrinsic_mat_k_.data, camera_info->K.data(), 9 * sizeof(double));
//        if (camera_info != nullptr)
//            cout << camera_info->K.data() << endl;
//            ROS_INFO("777");
//        while (camera_info == nullptr && ros::ok())
//            ros::spinOnce();  // Spin until camera info is grabbed.
//
//        initializeOnlyOnce(camera_info);

        points_sub_ = nh.subscribe("/processor/result_msg", 1, &WindSpeed::pointsCallback, this);

        speed_targets_sub_ = nh.subscribe("/prediction", 1, &WindSpeed::speedCallback, this);
        OriginMsg_pub_ = nh.advertise<std_msgs::Float32>("origin_speed", 1);
        FilteredMsg_pub_ = nh.advertise<std_msgs::Float32>("filtered_speed", 1);

        track_pub_ = nh.advertise<rm_msgs::TrackData>("/track", 10);

//        pose_targets_sub_ = nh.subscribe("/detection", 1, &WindSpeed::poseCallback, this);
//        pose_targets_pub_ = nh.advertise<decltype(target_array_)>("/windmill_track", 1);
    }

    void WindSpeed::windmillconfigCB(rm_windmill_speed::WindmillConfig& config, uint32_t level)
    {
        is_start_pred_ = config.is_start_pred;
        diff_threshold_ = config.diff_threshold;
        max_rmse_ = config.max_rmse;
        history_deque_len_cos_ = config.history_deque_len_cos;
        history_deque_len_phase_ = config.history_deque_len_phase;
        re_predict_ = config.re_predict;

        angular_velocity_ = config.angular_velocity;
        delay_time_ = config.angular_velocity;

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
//            int32_t data[5 * 2];
//            memcpy(&data[0], &object.pose.orientation.x, sizeof(int32_t) * 2);
//            memcpy(&data[2], &object.pose.orientation.y, sizeof(int32_t) * 2);
//            memcpy(&data[4], &object.pose.position.x, sizeof(int32_t) * 2);
//            memcpy(&data[6], &object.pose.orientation.z, sizeof(int32_t) * 2);
//            memcpy(&data[8], &object.pose.orientation.w, sizeof(int32_t) * 2);
            float data[3 * 2];
            memcpy(&data[0], &object.pose.position.x, sizeof(float) * 2);
            memcpy(&data[2], &object.pose.position.y, sizeof(float) * 2);
            memcpy(&data[4], &object.pose.position.z, sizeof(float) * 2);
            Target target;
            target.label = object.id;
            target.r_points.x = data[0];
            target.r_points.y = data[2];
            target.r_points.z = data[4];
            target.armor_center_points.x = data[1];
            target.armor_center_points.y = data[3];
            target.armor_center_points.z = data[5];

//            ROS_INFO("pts1:%f %f %f", data[0], data[2], data[4]);
//            ROS_INFO("pts2:%f %f %f", data[1], data[3], data[5]);

//            ROS_INFO("id:%d", object.id);
            if(re_predict_){
                history_info_.clear();
                init_flag_ = false;
                is_start_pred_ = true;
                is_params_confirmed_ = false;
                is_fitting_succeeded_ = false;
                re_predict_ = false;
            }
            if(object.id == 10){
                if(updateFan(target, info_target)){
                    speedSolution(info_target);
                }
            }
            if(is_start_pred_)
                if(updateHistory(info_target))
                    predict();
        }

    }

//    void WindSpeed::initializeOnlyOnce(sensor_msgs::CameraInfoConstPtr& camera_info){
//        ROS_INFO("POINTS");
//
//        memcpy(cam_intrinsic_mat_k_.data, camera_info->K.data(), 9 * sizeof(double));
//        dist_coefficients_ = camera_info->D;
//    }

    Target WindSpeed::pnp(const std::vector<Point2f>& points_pic)
    {
        std::vector<Point3d> points_world;

//        //长度为5进入大符模式
//        points_world = {
//                {-0.1125,0.027,0},
//                {-0.1125,-0.027,0},
////                {0,-0.7,-0.05},
//                {0.1125,-0.027,0},
//                {0.1125,0.027,0}};
//        points_world = {
//                {-0.066,-0.027,0},
//                {-0.066,0.027,0},
//                {0.066,0.027,0},
//                {0.066,-0.027,0}};
         points_world = {
         {-0.14,-0.08,0},
         {-0.14,0.08,0},
//         {0,-0.565,-0.05},
         {0.14,0.08,0},
         {0.14,-0.08,0}};

        Mat rvec;
        Mat rmat;
        Mat tvec;
        Eigen::Matrix3d rmat_eigen;
        Eigen::Vector3d R_center_world = {0,-0.7,-0.05};
        Eigen::Vector3d tvec_eigen;
        Eigen::Vector3d coord_camera;

        solvePnP(points_world, points_pic, cam_intrinsic_mat_k_, dist_coefficients_, rvec, tvec, false, SOLVEPNP_ITERATIVE);

        std::array<double, 3> trans_vec = tvec.reshape(1, 1);
        ROS_INFO("x:%f, y:%f, z:%f", trans_vec[0], trans_vec[1], trans_vec[2]);

        Target result;
//        //Pc = R * Pw + T
        cv::Rodrigues(rvec, rmat); /***罗德里格斯变换，把旋转向量转换为旋转矩阵***/
        cv::cv2eigen(rmat, rmat_eigen);/***cv转成eigen格式***/
        cv::cv2eigen(tvec, tvec_eigen);
//
        result.rmat = rmat_eigen;
        result.tvec = tvec_eigen;

        return result;
    }

    void WindSpeed::pointsCallback(const rm_msgs::TargetDetectionArray::Ptr &msg){

        rm_msgs::TrackData track_data;
        track_data.header.frame_id = "base_link";
        track_data.header.stamp = msg->header.stamp;
        track_data.id = 0;
        if(msg->detections[0].id == 0){
            track_pub_.publish(track_data);
            return;
        }

//        if (msg->detections.empty()){
//            track_pub_.publish(track_data);
//            return;
//        }

//        ROS_INFO("POINTS");
        int32_t data[4 * 2];                        // data of 4 2D points
        for (const auto &detection : msg->detections) {
            memcpy(&data[0], &detection.pose.orientation.x, sizeof(int32_t) * 2);
            memcpy(&data[2], &detection.pose.orientation.y, sizeof(int32_t) * 2);
            memcpy(&data[4], &detection.pose.orientation.z, sizeof(int32_t) * 2);
            memcpy(&data[6], &detection.pose.orientation.w, sizeof(int32_t) * 2);
        }

        std::vector<Point2f> pic_points;
        for (int i = 0; i < 4; i++){
            Point2f point;
            point.x = float(data[2*i]);
            point.y = float(data[2*i+1]);
            pic_points.emplace_back(point);
//            ROS_INFO("x%f, y%f", point.x, point.y);
        }

        Target hit_target = pnp(pic_points);

        double params[4];
        params[3] = angular_velocity_;
        double t0 = 0;
        double t1 = delay_time_;
        int mode = 0;
        std::vector<double> hit_point = calcAimingAngleOffset(hit_target, params, t0, t1, mode);
        ROS_INFO("x%f, y%f, z%f", hit_point[0], hit_point[1], hit_point[2]);

        rm_msgs::TargetDetection detection_temp;
        detection_temp.pose.position.x = hit_point[0];
        detection_temp.pose.position.y = hit_point[1];
        detection_temp.pose.position.z = hit_point[2];

        geometry_msgs::PoseStamped pose_in;
        geometry_msgs::PoseStamped pose_out;
        pose_in.header.frame_id = msg->header.frame_id;
        pose_in.header.stamp = msg->header.stamp;
        pose_in.pose = detection_temp.pose;

        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                    "base_link", pose_in.header.frame_id, msg->header.stamp, ros::Duration(1));

            tf2::doTransform(pose_in.pose, pose_out.pose, transform);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
//    ROS_INFO_STREAM(pose_out.pose.position.x
//                    << ",y:" << pose_out.pose.position.y
//                    << ",z:" << pose_out.pose.position.z);
        detection_temp.pose = pose_out.pose;

        track_data.id = 3;
        track_data.target_pos.x = detection_temp.pose.position.x;
        track_data.target_pos.y = detection_temp.pose.position.y;
        track_data.target_pos.z = detection_temp.pose.position.z;
        track_data.target_vel.x = 0;
        track_data.target_vel.y = 0;
        track_data.target_vel.z = 0;

        track_pub_.publish(track_data);
    }

    bool WindSpeed::updateFan(Target& object, const InfoTarget& prev_target) {
        if(!init_flag_){
            prev_fan_ = object;
            delat_t_ = prev_target.stamp;
            init_flag_ = true;
            filter_.reset();
            return false;
        }else{
            last_fan_ = prev_fan_;
            prev_fan_ = object;
            return true;
        }
    }

    void WindSpeed::speedSolution(InfoTarget& prev_target) {
//        float angle = abs(linesOrientation(prev_fan_.armor_center_points, prev_fan_.r_points, last_fan_.armor_center_points, last_fan_.r_points, 1));
        float angle = getAngle();
        double delat_t = (prev_target.stamp - delat_t_).toSec();
//        ROS_INFO("prev_target.stamp:%f", prev_target.stamp.toSec());
//        ROS_INFO("delat_t_:%f", delat_t_.toSec());
//        ROS_INFO("angle:%f", angle);
//        ROS_INFO("delat_t:%f", delat_t);
        delat_t_ = prev_target.stamp;
        if(isnan((angle / delat_t)))
            return;
        double speed = angle / delat_t;
        std_msgs::Float32 origin_msg;
        std_msgs::Float32 filtered_msg;
        double origin_speed = speed;
//        if(origin_speed > 1.5)
//            origin_speed = 0.5;
//        if(origin_speed <= 0)
//            origin_speed = 0;
//            return;
//        if(abs(origin_speed - wind_speed_) > diff_threshold_)
//            origin_speed = wind_speed_;
//        wind_speed_ = origin_speed;
        if(abs(origin_speed - wind_speed_) > diff_threshold_)
            origin_speed = wind_speed_;
        wind_speed_ = origin_speed;
        filter_.input(origin_speed, prev_target.stamp);
        double filtered_speed = filter_.output();
//        ROS_INFO("origin_speed:%f", origin_speed);
//        ROS_INFO("filtered_speed:%f", filtered_speed);
        origin_msg.data = origin_speed;
        filtered_msg.data = filtered_speed;
        prev_target.speed = origin_speed;
        OriginMsg_pub_.publish(origin_msg);
        FilteredMsg_pub_.publish(filtered_msg);
    }

    float WindSpeed::getAngle() {
      cv::Point2d vec1(prev_fan_.armor_center_points.x, prev_fan_.armor_center_points.y);
      cv::Point2d vec2(last_fan_.armor_center_points.x, last_fan_.armor_center_points.y);
      auto costheta = static_cast<float>(vec1.dot(vec2) / (cv::norm(vec1) * cv::norm(vec2)));
      float angle = acos(costheta);
        return angle;
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
//        ROS_INFO("history_info:%ld", history_info_.size());
        int deque_len;
        if (!is_params_confirmed_)
            deque_len = history_deque_len_cos_;
        else
            deque_len = history_deque_len_phase_;
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

    void WindSpeed::predict() {
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
            problem.SetParameterLowerBound(params_fitting,0,0.5);
            problem.SetParameterUpperBound(params_fitting,0,1.5);
            problem.SetParameterLowerBound(params_fitting,1,0.6);
            problem.SetParameterUpperBound(params_fitting,1,2.2);
            problem.SetParameterLowerBound(params_fitting,2,-CV_PI);
            problem.SetParameterUpperBound(params_fitting,2,CV_PI);
            problem.SetParameterLowerBound(params_fitting,3,0.5);
            problem.SetParameterUpperBound(params_fitting,3,2.5);

            ceres::Solve(options, &problem, &summary);
            double params_tmp[4] = {params_fitting[0], params_fitting[1], params_fitting[2], params_fitting[3]};
            auto rmse = evalRMSE(params_tmp);
            ROS_INFO("max_rmse:%f", max_rmse_);
            if (rmse > max_rmse_)
            {
                history_info_.clear();
                ROS_INFO("rmse:%f", rmse);
                cout<<summary.BriefReport()<<endl;
                ROS_INFO("[BUFF_PREDICT]RMSE is too high, Fitting failed!");
                return;
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
            ROS_INFO("Theta RMSE: %f", new_rmse);
        }

        //f(x) = a * sin(ω * t + θ) + b
        double a = params_[0];
        double omega = params_[1];
        double theta = params_[2];
        double b = params_[3];
        ROS_INFO("Objective function is : f(x) = %f * sin[%f * t + (%f)] + %f", a, omega, theta, b);
        is_start_pred_ = false;
        is_fitting_succeeded_ = true;
    }

    std::vector<double> WindSpeed::calcAimingAngleOffset(Target& object, double params[4], double t0, double t1 , int mode)
    {
        auto a = params[0];
        auto omega = params[1];
        auto theta = params[2];
        auto b = params[3];
        double theta1;
        double theta0;
        // cout<<"t1: "<<t1<<endl;
        // cout<<"t0: "<<t0<<endl;
        //f(x) = a * sin(ω * t + θ) + b
        //对目标函数进行积分
        if (mode == 0)//适用于小符模式
        {
            theta0 = b * t0;
            theta1 = b * t1;
        }
        else
        {
            theta0 = (b * t0 - (a / omega) * cos(omega * t0 + theta));
            theta1 = (b * t1 - (a / omega) * cos(omega * t1 + theta));
        }
        // cout<<(theta1 - theta0) * 180 / CV_PI<<endl;
//        return theta1 - theta0;
        double theta_offset = theta1 - theta0;
        ROS_INFO("theta1%f, theta0%f, b%f", theta1, theta0, b);
        Eigen::Vector3d hit_point_world = {-sin(theta_offset) * fan_length_, -(cos(theta_offset) - 1) * fan_length_,0};
        Eigen::Vector3d hit_point_cam= (object.rmat * hit_point_world) + object.tvec;
        std::cout << "hit_point_world.transpose() = \n" << hit_point_world.transpose() << std::endl;
        std::cout << "hit_point_cam.transpose() = \n" << hit_point_cam.transpose() << std::endl;
        std::vector<double> hit_points;
        hit_points.emplace_back(hit_point_cam.transpose()[0]);
        hit_points.emplace_back(hit_point_cam.transpose()[1]);
        hit_points.emplace_back(hit_point_cam.transpose()[2]);

        return hit_points;
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

    void WindSpeed::poseCallback(const rm_msgs::TargetDetectionArray::ConstPtr &msg) {
        if (!is_fitting_succeeded_)
            return;
        rm_msgs::TargetDetectionArray::Ptr pose_msg;
        auto object = msg->detections[0];
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header.frame_id = msg->header.frame_id;
        pose_in.header.stamp = msg->header.stamp;
        pose_in.pose = object.pose;

        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
                    "odom", pose_in.header.frame_id, msg->header.stamp, ros::Duration(1));
            tf2::doTransform(pose_in.pose, pose_out.pose, transform);

        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
ROS_INFO("transform x:%f y:%f z:%f", pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);
        pose_msg->detections[0].pose.position.x = pose_out.pose.position.x;
        pose_msg->detections[0].pose.position.y = pose_out.pose.position.y;
        pose_msg->detections[0].pose.position.z = pose_out.pose.position.z;
        pose_targets_pub_.publish(pose_msg);
    }
}  // namespace rm_windmill_speed
