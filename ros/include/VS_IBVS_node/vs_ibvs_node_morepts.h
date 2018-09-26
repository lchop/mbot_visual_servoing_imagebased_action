//
// Created by lchopot on 06-07-2018.
//
#pragma once
/* C++ Related */
#include <iostream>
#include <array>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

/*OPENCV*/
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/* ROS Related*/
#include <ros/ros.h>
//visp_ros bridge
#include <visp_ros/vpROSGrabber.h> // Bridge between ros camera node and VISP
//msg
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
//tf
#include <tf/transform_listener.h>
//yolo
#include <darknet_ros_py/RecognizedObjectArrayStamped.h>
#include <darknet_ros_py/RecognizedObject.h>
//message filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* VISP related */
// Tracker
#include <visp3/blob/vpDot2.h>
#include <visp3/klt/vpKltOpencv.h>
// Visual Features
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
// Core, image processing, cam parameter
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRotationMatrix.h>
// GUI, to display images
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/vs/vpServoDisplay.h>
// Visual Servoing
#include <visp3/vs/vpServo.h>
//Plot
#include <visp3/gui/vpPlot.h>
#include <visp3/core/vpMatrix.h>


namespace VS_IBVS {
    using namespace message_filters::sync_policies;

    class IBVS_mbot {
        typedef ApproximateTime<sensor_msgs::Image, darknet_ros_py::RecognizedObjectArrayStamped> DepthToBBSyncPolicy;
    public:
    /* ROS Related*/
        ros::NodeHandle nh_; // Global namespace
        ros::NodeHandle pnh_; // Node namespace
        //pub and sub
        ros::Publisher pub_vel;
        ros::Publisher event_out_pub_;
        ros::Publisher pub_error;
        ros::Publisher pub_gripper;
        ros::Subscriber event_in_sub_;
        message_filters::Subscriber<darknet_ros_py::RecognizedObjectArrayStamped> object_array_sub_;
        message_filters::Subscriber<sensor_msgs::Image> depth_img_sub_;
        //msg
        std::unique_ptr<message_filters::Synchronizer<DepthToBBSyncPolicy>> sync_;
        geometry_msgs::TwistStamped vel_output; // TwistStamped vel_output to send the target_vel to the cartesian controller
        geometry_msgs::TwistStamped vel_output_tf; //if vel vector is transformed, will be stored here
        std::string name;
    public:
        //TF
        tf::TransformListener listener;
        tf::StampedTransform transform;

    public:
        /*Parameters*/
        double node_frequency;

        bool plot;
        bool save;

        float end_effector_velocities_max;
        float error_max;
        float depth_desired;

        bool Blob_tracking;
        bool Yolo_Center_ROI;

        std::string RgbImage_Topic;
        std::string CamInfo_Topic;
        std::string DepthImage_Topic;
        std::string DetectionResultsYolo_Topic;
        std::string master_uri;
        std_msgs::Float64 error;

        bool filter_class_;
        std::string filter_class_name_;

        float command_gripper = 0.25;


    public:
    /* VISP & VISP_ROS Related*/
        vpImage<unsigned char> I;
        const double cam_px;
        const double cam_py;
        vpROSGrabber g;
        vpCameraParameters cam;

        vpImagePoint cog[4];
        vpDot2 dot[4];
        vpDot2 dotbt[4];


        vpServo task;
        vpFeaturePoint p[3];
        double x_center, y_center; // store current and update value of the image point features
        double x_center_right, y_center_right; // store current and update value of the image point features
        double x_center_left, y_center_left; // store current and update value of the image point features
        vpFeaturePoint pd[3];
        vpFeatureDepth s_Z[3];
        vpFeatureDepth s_Zd[3];
        vpColVector v;
        vpColVector v_back;


        vpFeaturePoint p_blob[4];
        vpFeaturePoint pd_blob[4];
        vpFeatureDepth s_Z_blob[4];
        vpFeatureDepth s_Zd_blob[4];

    public:
        /*YOLO related*/
        darknet_ros_py::RecognizedObjectArrayStamped last_received_array_;
        vpImage<unsigned char> C; // Cropped output image

    public:
        /*Depth image related*/
        cv_bridge::CvImageConstPtr last_received_depth_;
        double depth_center_roi; // store the depth after compute


    public:
        /*FLAG*/
        bool roi_received_;//flag to check if a ROI from YOLO has been received
        bool is_running_;// flag to check if start event has been received
        bool depth_image_received_;//flag to check if depth image has been received
        bool init_done;//flag to check if the init has been done
        bool wrong_depth;//flag to check if the depth value is correct
        bool update_done; // flag to check if features have been updated.

    public:
    /* Methods*/
        //General init
        void GetCameraParameters();
        void GrabberImageFromRos();
        void InitVispTask(vpServo::vpServoType a_ServoType , vpServo::vpServoIteractionMatrixType a_InteractionType, vpServo::vpServoInversionType a_ServoInversionType);
        void DisplayImage(std::string& text_to_print_on_image);
        void CreateSubscriber();
        void destroySubscribers();
        //4Dot blob tracking
        void InitTracking_Dot_BlobTracking();
        void CreateCurrentFeatures_Dot_BlobTracking();
        void CreateDesiredFeatures_Dot_BlobTracking();
        void UpdateCurrentFeatures_Dot_Blob_Tracking();
        void Init_Dot_BlobTracking();
        //YOLO center bounding box tracking
        vpRect RoiFromYOLO(darknet_ros_py::RecognizedObjectArrayStamped rec);
        void GetROICenterDepth();
        void CreateCurrentFeatures_CenterBoundingBox();
        void CreateDesiredFeatures_CenterBoundingBox();
        void UpdateCurrentFeatures_CenterBoundingBox();
        void Init_CenterBoundingBox();
        //General
        void CreatePlot_Features_CartesianVelocities(vpPlot& plotter);
        void AddFeaturesToTask();
        void ArmEndEffectorVelocitiesLimitation(float end_effector_velocities, vpColVector& v_in_out);
        void apply_velocities(vpColVector& velocities_to_apply, std::string velocitie_frame_to_be_published);
        void apply_velocities_to_go_back(vpColVector& velocities_to_apply);
        void publish_velocities(vpColVector& velocities_to_publish);
        void publish_error();
        void publish_gripper_command(float position_gripper);
        //Callbacks
        void EventInCallBack(const std_msgs::StringConstPtr &msg);
        void syncCallback(const sensor_msgs::ImageConstPtr &depth, const darknet_ros_py::RecognizedObjectArrayStampedConstPtr &rec);
        //loop
        void loop();

    public:
        /* Default Constructor*/
        IBVS_mbot();
    };
} //end of namespace

