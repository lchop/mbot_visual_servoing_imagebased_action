//
// Created by lchopot on 28-06-2018.
//
#include <VS_IBVS_node/vs_ibvs_node_morepts.h>

#undef DEBUG
#define NAN_ALLOWED_RATIO 0.25 //if in vector you have more than 25% you dont take depth value

namespace VS_IBVS {

        IBVS_mbot::IBVS_mbot() : cam_px(800), cam_py(800), nh_(""), pnh_("~")
        {
            error.data = 1.0;
            is_running_ = false;
            depth_image_received_ = false;
            roi_received_ = false;
            init_done = false;

            pnh_.param<bool>("filter_class", filter_class_, false);
            pnh_.param<std::string>("filter_class_name", filter_class_name_, "");

            pnh_.param<double>("node_frequency", node_frequency, 10.0);

            pnh_.getParam("plot", plot);
            pnh_.getParam("save", save);

            pnh_.getParam("error_max", error_max);
            pnh_.getParam("end_effector_velocities_max", end_effector_velocities_max);
            pnh_.getParam("depth_desired", depth_desired);

            pnh_.getParam("Blob_tracking", Blob_tracking);
            pnh_.getParam("Yolo_Center_ROI", Yolo_Center_ROI);

            pnh_.getParam("MASTER_URI", master_uri);
            pnh_.getParam("CamInfo_Topic", CamInfo_Topic);
            pnh_.getParam("RgbImage_Topic", RgbImage_Topic);
            pnh_.getParam("DepthImage_Topic", DepthImage_Topic);
            pnh_.getParam("DetectionResultsYolo_Topic", DetectionResultsYolo_Topic);

            //output event
            event_out_pub_ = pnh_.advertise<std_msgs::String>("event_out", 5);
            //subscribe to events
            event_in_sub_ = pnh_.subscribe("event_in", 5, &IBVS_mbot::EventInCallBack, this);
            //publish target_vel on a topic called ibvs/target_vel
            pub_vel = nh_.advertise<geometry_msgs::TwistStamped>("ibvs/target_vel", 1);
            //publish the quadratique error of the visual servoing law
            pub_error = nh_.advertise<std_msgs::Float64>("ibvs/error",1);
//            //publish the command to close the gripper
            pub_gripper = nh_.advertise<std_msgs::Float64>("left_arm_gripper_joint_position_controller/command", 1, true);
        }

        void IBVS_mbot::GrabberImageFromRos()
        {
            g.setMasterURI(master_uri);
            g.setCameraInfoTopic(CamInfo_Topic);
            g.setImageTopic(RgbImage_Topic);
            g.setRectify(true);
            g.setImageTransport("jpeg");
            g.open(I);
            std::cout << "Image rgb received, size: " << I.getWidth() << " " << I.getHeight() << std::endl;
        }

        void IBVS_mbot::GetCameraParameters()
        {
            if (!g.getCameraInfo(cam))
                cam.initPersProjWithoutDistortion(cam_px,cam_py,I.getWidth()/2, I.getHeight()/2);
            std::cout << "cam param!" << cam << std::endl;
        }

        void IBVS_mbot::InitVispTask(vpServo::vpServoType a_ServoType , vpServo::vpServoIteractionMatrixType a_InteractionType, vpServo::vpServoInversionType a_ServoInversionType)
        {
            task.setServo(a_ServoType);
            task.setInteractionMatrixType(a_InteractionType, a_ServoInversionType);
            task.print();
        }

        void IBVS_mbot::DisplayImage(std::string& text_to_print_on_image)
        {
            vpDisplay::display(I);
            vpDisplay::displayText(I, 10, 10, text_to_print_on_image, vpColor::red);
            vpDisplay::flush(I);
        }

        void IBVS_mbot::InitTracking_Dot_BlobTracking()
        { 
            dotbt.setGraphics(true);
            dotbt.setEllipsoidShapePrecision(0.9);  // to track a blob without any constraint on the shape
            dotbt.setGrayLevelPrecision(0.9);  // to set the blob gray level bounds for Init_4Dot_BlobTrackingbinarisation
            dotbt.setEllipsoidBadPointsPercentage(0.2); // to be accept 20% of bad inner and outside points with bad gray level
            dotbt.setGrayLevelMin(0); // the gray lvl to be in the blob (0 here because of black points)
            dotbt.setMaxSizeSearchDistancePrecision(0.2);
            dotbt.initTracking(I);
            cog = dotbt.getCog();

            vpDisplay::flush(I);

        }

        void IBVS_mbot::CreateCurrentFeatures_Dot_BlobTracking()
        {
            double coef = 1./95.; // At 1 m the blob has a surface of 95 (approximatively) (used to calculate depth)

            double surface; // surface of the blob used to compute depth
            double Z; // Depth for each vpDot2 features

                vpFeatureBuilder::create(p_blob, cam, dotbt.getCog());
                surface = 1./sqrt(dotbt.m00/(cam.get_px()*cam.get_py())); // Surface of the blob estimated from the image moment m00
                // and converted in meters
                Z = coef * surface; // Calculating the depth from each current feature point
                p_blob.set_Z(Z);

                s_Z_blob.buildFrom(p_blob.get_x(), p_blob.get_y(), Z, 0);

        }

        void IBVS_mbot::CreateDesiredFeatures_Dot_BlobTracking()
        {
            vpPoint point;
            point.set_x(0);
            point.set_y(0);
            vpFeatureBuilder::create(pd_blob, point);
            

            double Zd; // Depth for each desired feature
            double depth_desired = 0.2;
            Zd = depth_desired;
            
            pd_blob.set_Z(Zd);
            
            s_Zd_blob.buildFrom(pd_blob.get_x(), pd_blob.get_y(), Zd, 0);
        }

        void IBVS_mbot::UpdateCurrentFeatures_Dot_Blob_Tracking()
        {
            double coef = 1./95.; // At 1 m the blob has a surface of 95 (approximatively) (used to calculate depth)

            double surface; // surface of the blob used to compute depth
            double Z; // Depth for each vpDot2 features
            
            dotbt.track(I); //track the 4 blob in I
            cog = dotbt.getCog(); // get center of gravity of blob
            
            vpFeatureBuilder::create(p_blob, cam, dotbt);// Update the current features
            
            surface = 1. / sqrt(dotbt.m00 / (cam.get_px() * cam.get_py()));
            Z = coef * surface;
            std::cout << "distance :" << Z<< "\n";
            p_blob.set_Z(Z);

            s_Z_blob.buildFrom(p_blob.get_x(), p_blob.get_y(), Z, log(Z/0.2)) ;
        }

        void IBVS_mbot::Init_Dot_BlobTracking()
        {
            GetCameraParameters();
            InitVispTask(vpServo::vpServoType::EYEINHAND_CAMERA, vpServo::vpServoIteractionMatrixType::DESIRED,
                         vpServo::vpServoInversionType::PSEUDO_INVERSE);

            std::string text_to_print_on_image = "Click on the 4 dots";
            if(!text_to_print_on_image.empty()) //check if string not empty
                DisplayImage(text_to_print_on_image);
            InitTracking_Dot_BlobTracking();
            CreateCurrentFeatures_Dot_BlobTracking();
            CreateDesiredFeatures_Dot_BlobTracking();
            AddFeaturesToTask();
        }

        vpRect IBVS_mbot::RoiFromYOLO(const darknet_ros_py::RecognizedObjectArrayStamped rec)
        {
            vpRect roi_area;
            for (const darknet_ros_py::RecognizedObject &object : rec.objects.objects)
            {
                if (filter_class_)
                {
                    if (object.class_name != filter_class_name_)
                        continue;
                }
                const sensor_msgs::RegionOfInterest &roi = object.bounding_box;

                // bounding box easy access
                const int bb_x_min = roi.x_offset;
                const int bb_y_min = roi.y_offset;
                const int bb_width = roi.width;
                const int bb_height = roi.height;

                roi_area.set(bb_x_min,bb_y_min,bb_width,bb_height);
            }

            return roi_area;
        }

        float getDepthBySorting(const cv::Mat& mat)
        {
            cv::Mat flat;
            cv::Mat flat_ord;
            cv::Point BottomLeft;
            cv::Point TopRight;
            BottomLeft.x = (cv::Point(mat.size()).x*3/8);
            BottomLeft.y = (cv::Point(mat.size()).y*3/8);
            TopRight.x = (cv::Point(mat.size()).x*5/8);
            TopRight.y = (cv::Point(mat.size()).y*5/8);
            // Select a quarter of the bounding box from its center (mat.size is (x,y))
            const cv::Rect half(BottomLeft, TopRight);

            // Needs copy because it is not continuous
            const cv::Mat half_mat(mat, half);
            // Check NANs
            unsigned long nan_count = 0;
            std::vector<float> half_filtered_vec;
            for (int row = 0; row < half_mat.rows; ++row) {
                for (int col = 0; col < half_mat.cols; ++col) {
                    const auto val = half_mat.at<float>(row, col);
                    if (val==0 || std::isnan(val))
                        nan_count++;
                    else
                        half_filtered_vec.push_back(val);
                }
            }

            // If number of NANs is too high, return -1 to signal no-publish
            const int num_values = half_mat.rows * half_mat.cols;

            if (nan_count > (1 - NAN_ALLOWED_RATIO) * num_values) {
                ROS_WARN_THROTTLE(5.0,
                                  "Too many NANs (%d out of %d values) in the provided bounding-boxed depth map, do not publish",
                                  nan_count, num_values);
                return -1;
            }

            if (half_filtered_vec.empty()) {
                ROS_ERROR("NAN-Filtered vector is empty!!?? Contact a developer");
                return -1;
            }

            // Sort the NAN-filtered vector
            std::sort(half_filtered_vec.begin(), half_filtered_vec.end());

            // Return the depth at the 25th percentile
            return half_filtered_vec.at(half_filtered_vec.size() / 4);
        }

        void IBVS_mbot::GetROICenterDepth()
        {
            vpRect roi_area = RoiFromYOLO(last_received_array_);
            double depth;
            const cv::Mat depmat = last_received_depth_->image;

#ifdef DEBUG
            cv::imshow("depth",depmat);
            cv::waitKey(30);
#endif
            const cv::Rect bb_rect(roi_area.getTopLeft().get_u(), roi_area.getTopLeft().get_v(), roi_area.getWidth(),
                                   roi_area.getHeight());
            cv::Mat roi_mat; //roiArea;

            try
            {
                // obtain the image ROI:
                roi_mat = cv::Mat(depmat, bb_rect);
#ifdef DEBUG
                cv::imshow("roi_mat", roi_mat);
                cv::waitKey(30);
#endif
                depth = getDepthBySorting(roi_mat)*0.001; // we get the depth value in mm from CV_32FC1 image format convert in m

                std::cout << "distance: " << depth <<std::endl;
                if (depth <=  0 || std::isnan(depth))
                {
                    std_msgs::String e_out;
                    e_out.data = "e_nans";
                    event_out_pub_.publish(e_out);
                    ROS_INFO("Depth value incorrect, keeping the correct precedent value");
                }
                else
                    depth_center_roi = depth;

            } catch (const cv::Exception &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

        }

        void IBVS_mbot::CreateCurrentFeatures_CenterBoundingBox()
        {
            vpRect roi_rect;
            vpImagePoint center;
            vpImagePoint center_left;
            vpImagePoint center_right;
            double x, y;
            double width;

            //Get ROI from YOLO and positon of the center of this ROI
            roi_rect = RoiFromYOLO(last_received_array_);
            vpDisplay::displayRectangle(I,roi_rect.getTopLeft(),roi_rect.getBottomRight(), vpColor::green);

            roi_rect.getCenter(x, y);
            width = roi_rect.getWidth();

            //Init for low-pass filter
            x_center = x;
            y_center = y;
            x_center_left = x-25;
            y_center_left = y;
            x_center_right = x+25;
            y_center_right = y;

            center.set_uv(x_center, y_center);
            center_left.set_uv(x_center_left, y_center_left);
            center_right.set_uv(x_center_right, y_center_right);

            //Get depth of the center of the ROI
            GetROICenterDepth();


            //Create current Image features (2 features, x positon and y positon)
            vpFeatureBuilder::create(p[0], cam, center_left);
            vpFeatureBuilder::create(p[1], cam, center_right);
            vpFeatureBuilder::create(p[2], cam, center);
            p[0].set_Z(depth_center_roi);
            p[1].set_Z(depth_center_roi);
            p[2].set_Z(depth_center_roi);

            //Create current Depth features ( 1 feature, z positon)
            s_Z[0].buildFrom(p[0].get_x(), p[0].get_y(), depth_center_roi, 0);
            s_Z[1].buildFrom(p[1].get_x(), p[1].get_y(), depth_center_roi, 0);
            s_Z[2].buildFrom(p[2].get_x(), p[2].get_y(), depth_center_roi, 0);

        }

        void IBVS_mbot::CreateDesiredFeatures_CenterBoundingBox()
        {
            vpImagePoint center_left;
            vpImagePoint center_right;
            vpImagePoint center;

            // Point in the center of the camera frame.
            center.set_uv(432,372);
            center_right.set_uv(center.get_u() + 25 ,372);
            center_left.set_uv(center.get_u() - 25 ,372);
            //Create desired Image features (2 features, x positon and y positon)
            vpFeatureBuilder::create(pd[0], cam, center_left);
            vpFeatureBuilder::create(pd[1], cam, center_right);
            vpFeatureBuilder::create(pd[2], cam, center);
            pd[0].set_Z(depth_center_roi);
            pd[1].set_Z(depth_center_roi);
            pd[2].set_Z(depth_center_roi);

            //Create desired Depth features ( 1 feature, z positon)
            s_Zd[0].buildFrom(p[0].get_x(), p[0].get_y(), depth_center_roi, 0);
            s_Zd[1].buildFrom(p[1].get_x(), p[1].get_y(), depth_center_roi, 0);
            s_Zd[2].buildFrom(p[2].get_x(), p[2].get_y(), depth_center_roi, 0);

        }

        void IBVS_mbot::UpdateCurrentFeatures_CenterBoundingBox()
        {
            vpRect roi_rect;
            vpImagePoint center;
            vpImagePoint center_left;
            vpImagePoint center_right;
            double x, y;
            double width;
            double alpha = 0.5;

            //Get the new ROI and the new position of the center of this ROI
            roi_rect = RoiFromYOLO(last_received_array_);
            vpDisplay::displayRectangle(I,roi_rect.getTopLeft(),roi_rect.getBottomRight(), vpColor::green);
            roi_rect.getCenter(x, y);
            width = roi_rect.getWidth();
            //low pass filter
            x_center = x_center + alpha*(x - x_center);
            y_center = y_center + alpha*(y - y_center);
            center.set_uv(x_center, y_center);

            x_center_left = x_center_left + alpha*((x-25) - x_center_left);
            y_center_left = y_center_left + alpha*(y - y_center_left);
            center_left.set_uv(x_center_left, y_center_left);

            x_center_right = x_center_right + alpha*((x+25) - x_center_right);
            y_center_right = y_center_right + alpha*(y - y_center_right);
            center_right.set_uv(x_center_right, y_center_right);

            //Update Depth
            GetROICenterDepth();

            //Update current Image features with new position in the image
            vpFeatureBuilder::create(p[0], cam, center_left);
            vpFeatureBuilder::create(p[1], cam, center_right);
            vpFeatureBuilder::create(p[2], cam, center);


            p[0].set_Z(depth_center_roi);
            p[1].set_Z(depth_center_roi);
            p[2].set_Z(depth_center_roi);


            //Update Depth feature with new depth value
            s_Z[0].buildFrom(p[0].get_x(), p[0].get_y(), depth_center_roi, log(depth_center_roi/depth_desired)) ;
            s_Z[1].buildFrom(p[1].get_x(), p[1].get_y(), depth_center_roi, log(depth_center_roi/depth_desired)) ;
            s_Z[2].buildFrom(p[2].get_x(), p[2].get_y(), depth_center_roi, log(depth_center_roi/depth_desired)) ;


        }

        void IBVS_mbot::Init_CenterBoundingBox()
        {
            GetCameraParameters();
            InitVispTask(vpServo::vpServoType::EYEINHAND_CAMERA, vpServo::vpServoIteractionMatrixType::DESIRED,
                         vpServo::vpServoInversionType::PSEUDO_INVERSE);

            CreateCurrentFeatures_CenterBoundingBox();
            CreateDesiredFeatures_CenterBoundingBox();
            AddFeaturesToTask();
        }

        void IBVS_mbot::CreatePlot_Features_CartesianVelocities(vpPlot& plotter)
        {
            plotter.setTitle(0, "Visual features error");
            plotter.setTitle(1, "Camera velocities");
            plotter.initGraph(0, 8);
            plotter.initGraph(1, 6);
            plotter.setLegend(0, 0, "x1");
            plotter.setLegend(0, 1, "y1");
            plotter.setLegend(0, 2, "x2");
            plotter.setLegend(0, 3, "y2");
            plotter.setLegend(0, 4, "x3");
            plotter.setLegend(0, 5, "y3");
            plotter.setLegend(0, 6, "x4");
            plotter.setLegend(0, 7, "y4");
            plotter.setLegend(1, 0, "v_x");
            plotter.setLegend(1, 1, "v_y");
            plotter.setLegend(1, 2, "v_z");
            plotter.setLegend(1, 3, "w_x");
            plotter.setLegend(1, 4, "w_y");
            plotter.setLegend(1, 5, "w_z");
        }

        void IBVS_mbot::AddFeaturesToTask()
        {
            if (Blob_tracking)
            {
                task.addFeature(p_blob, pd_blob); // Add the features to the visual task
                task.addFeature(s_Z_blob, s_Zd_blob); // Add depth features to the visual task
                task.print(); //Print the task fully initialized
            }
            else if (Yolo_Center_ROI)
            {
                for (int i=0; i<3;i++)
                {
                    task.addFeature(p[i], pd[i]); // Add the features to the visual task
                    task.addFeature(s_Z[i], s_Zd[i]); // Add depth features to the visual task
                    task.print(); //Print the task fully initialized
                }
            }


        }

        void IBVS_mbot::ArmEndEffectorVelocitiesLimitation(float end_effector_velocities_max, vpColVector& v_in_out)
        {
            for (int i = 0; i < 6; i++)
            {
                if (v_in_out[i] > end_effector_velocities_max) // restrict the arm speed.
                    v_in_out[i] = end_effector_velocities_max;

                else if (v_in_out[i] < -end_effector_velocities_max)
                    v_in_out[i] = -end_effector_velocities_max;
            }
        }

        void IBVS_mbot::apply_velocities(vpColVector& velocities_to_apply, std::string velocitie_frame_to_be_published)
        {
            vel_output.header.stamp = ros::Time::now();
            vel_output.header.frame_id = "left_arm_camera_link"; //the frame where the velocities are computed
            // v[0], v[1], v[2] correspond to translation velocities in m/s
            // v[3], v[4], v[5] correspond to rotation velocities in rad/s
            //The camera frame of the actual camera and the camera frame where the velocities are computed are not the samed,
            // that's why, there is inversion of line and - sign.
            vel_output.twist.linear.x = velocities_to_apply[2];
            vel_output.twist.linear.y = -velocities_to_apply[0];
            vel_output.twist.linear.z = -velocities_to_apply[1];
            vel_output.twist.angular.x = velocities_to_apply[5];
            vel_output.twist.angular.y = -velocities_to_apply[3];
            vel_output.twist.angular.z = -velocities_to_apply[4];

//            std::cout << vel_output <<std::endl;
//
//            if (velocitie_frame_to_be_published != "left_arm_camera_link") {
//                geometry_msgs::Vector3Stamped linear_in;
//                linear_in.header.stamp = ros::Time::now();
//                linear_in.header.frame_id = "left_arm_camera_link";
//                linear_in.vector = vel_output.twist.linear;
//                geometry_msgs::Vector3Stamped angular_in;
//                angular_in.header.stamp = ros::Time::now();
//                angular_in.header.frame_id = "left_arm_camera_link";
//                angular_in.vector = vel_output.twist.angular;
//                geometry_msgs::Vector3Stamped linear_out;
//                geometry_msgs::Vector3Stamped angular_out;
//                try {
//                    listener.waitForTransform(velocitie_frame_to_be_published, "left_arm_camera_link", ros::Time::now(),
//                                              ros::Duration(0.1));
//                    listener.transformVector(velocitie_frame_to_be_published, linear_in, linear_out);
//                    listener.transformVector(velocitie_frame_to_be_published, angular_in, angular_out);
//                }
//                catch (tf::TransformException ex) {
//                    ROS_ERROR("%s", ex.what());
//                    ros::Duration(1.0).sleep();
//                }
//                angular_out.vector.x = 0.0;
//                angular_out.vector.y = 0.0;
//                angular_out.vector.z = 0.0;
//
//                vel_output_tf.header.stamp = ros::Time::now();
//                vel_output_tf.header.frame_id = velocitie_frame_to_be_published;
//                vel_output_tf.twist.linear = linear_out.vector;
//                vel_output_tf.twist.angular = angular_out.vector;
//
//                vel_output = vel_output_tf;
//            }
        }

        void IBVS_mbot::publish_velocities(vpColVector& velocities_to_publish)
        {
            pub_vel.publish(vel_output);
        }

        void IBVS_mbot::publish_error()
        {

            pub_error.publish(error);
        }


        void IBVS_mbot::publish_gripper_command(float position_gripper)
        {
            std_msgs::Float64 command;
            command.data = position_gripper;
            pub_gripper.publish(command);
        }

        void IBVS_mbot::CreateSubscriber()
        {
            depth_img_sub_.subscribe(pnh_, DepthImage_Topic, 2);
            object_array_sub_.subscribe(pnh_, DetectionResultsYolo_Topic, 2);
            sync_.reset(
                    new message_filters::Synchronizer<DepthToBBSyncPolicy>(
                            DepthToBBSyncPolicy(10), depth_img_sub_, object_array_sub_
                    )
            );
            sync_->registerCallback(
                    boost::bind(&IBVS_mbot::syncCallback, this, _1, _2)
            );
        }

        void IBVS_mbot::destroySubscribers()
        {
            sync_.reset();
            depth_img_sub_.unsubscribe();
            object_array_sub_.unsubscribe();
        }

        void IBVS_mbot::syncCallback(const sensor_msgs::ImageConstPtr &depth,
                                     const darknet_ros_py::RecognizedObjectArrayStampedConstPtr &rec)
        {
            // Save ROI
            for (const auto &obj: rec->objects.objects)
            {
                if (filter_class_)
                {
                    if (obj.class_name != filter_class_name_)
                        continue;
                }
                last_received_array_= *rec;
                roi_received_ = true;
            }

            // Save depth
            try {
                last_received_depth_ = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
            }

            catch (cv_bridge::Exception &e) {
                ROS_ERROR("Could not convert from encoding '%s'.", depth->encoding.c_str());
                return;
            }
            depth_image_received_ = true;

        }

        void IBVS_mbot::EventInCallBack(const std_msgs::StringConstPtr &msg)
        {
            std_msgs::String e_out;

            if (msg->data == "e_stop")
            {
                ROS_INFO("Received event to stop - shut down all subscribers");
                e_out.data = "e_success";
                destroySubscribers();
                // flag to stop
                is_running_ = false;
            }
            else if (msg->data == "e_start")
            {
                ROS_INFO("Received event to start - created all subscribers");
                e_out.data = "e_success";
                CreateSubscriber();
                // flag to start
                is_running_ = true;
            }
            else
            {
                ROS_WARN_STREAM("Wrong event received");
                e_out.data = "e_failure";
            }

            event_out_pub_.publish(e_out);
        }

        void IBVS_mbot::loop()
        {
            double iter = 0;

            ros::Rate loop_rate(node_frequency);
            ROS_INFO("Node will run at : %lf [hz]", IBVS_mbot::node_frequency);

            vpPlot plotter(2, 250 * 2, 500, 100, 200, "Real time curves plotter");

            if (plot)
            {
                ROS_INFO("Curves will be plot");
                CreatePlot_Features_CartesianVelocities(plotter);
            }
            else
                vpDisplay::close(plotter.I);

            while (ros::ok()) {
                ROS_WARN_COND(!is_running_, "NO startting event received");
                if (is_running_) {
                    if (Yolo_Center_ROI)
                    {
                        ROS_WARN_COND(!depth_image_received_,"NO depth image received");
                        ROS_WARN_COND(!roi_received_,"NO ROI from YOLO received");

                        if (roi_received_ && depth_image_received_) {
                            Init_CenterBoundingBox();
                            init_done = true;
                        }
                    }

                    else if (Blob_tracking)
                    {
                        Init_Dot_BlobTracking();
                        init_done = true;
                    }

                        else if (!Blob_tracking && !Yolo_Center_ROI)
                    {
                        ROS_ERROR("Please select a method: Blob_tracking or Yolo_Center_ROI");
                        return;
                    }
                    if (init_done) {
                        while (error.data > error_max)
                        {
                            g.acquire(I);
                            vpDisplay::display(I);

                            if (Blob_tracking)
                                UpdateCurrentFeatures_Dot_Blob_Tracking();
                            else if (Yolo_Center_ROI)
                                UpdateCurrentFeatures_CenterBoundingBox();

                            v = task.computeControlLaw(); // Compute the control law
                            vpServoDisplay::display(task, cam, I);

                            if (plot) {
                                plotter.plot(0, iter, task.getError());
                                plotter.plot(1, iter, v);
                            }

                            error.data = task.getError().sumSquare(); // error = s^2 - sd^2

                            std::cout << "\n" << "error quadratic: " << error << "\n" << std::endl;

                            ArmEndEffectorVelocitiesLimitation(end_effector_velocities_max, v);
                            apply_velocities(v, "left_arm_camera_link");
                            publish_velocities(v);
                            publish_error();

                            iter++;

                            depth_image_received_ = false;
                            roi_received_ = false;

                            vpDisplay::flush(I);

                            if (vpDisplay::getClick(I, false))// A click in the viewer to exit
                                return;

                            loop_rate.sleep();
                        }
                        publish_gripper_command(command_gripper);
                        if (plot)
                        {
                            if (save)
                            {
                                plotter.saveData(0, "error.dat");
                                plotter.saveData(1, "vc.dat");
                            }
                            vpDisplay::getClick(plotter.I);
                        }
                    }
                }
                task.kill();
                ros::spinOnce();
            }
            ros::shutdown();
        }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "ibvs_node");

    VS_IBVS::IBVS_mbot mbot_ibvs;
    ROS_INFO("Node initialized");

    mbot_ibvs.GrabberImageFromRos();
#if defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(mbot_ibvs.I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    mbot_ibvs.loop();
}