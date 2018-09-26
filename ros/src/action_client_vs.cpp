#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <mbot_visual_servoing_imagebased/GraspObjectAction.h>
#include <std_msgs/String.h>

using namespace mbot_visual_servoing_imagebased;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const GraspObjectResultConstPtr& result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Answer: %i", result->success);
    ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const GraspObjectFeedbackConstPtr& feedback)
{
  ROS_INFO("Got feedback error %f", feedback->error_current);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "action_client_vs");
    ros::NodeHandle nh("/mbot_manipulation");

    GraspObjectGoal goal;
    actionlib::SimpleActionClient<::GraspObjectAction> action_client(nh, "final_grasping_approach_server", true);

    ROS_INFO("waiting for server: ");

    action_client.waitForServer(); //wait forever

    ROS_INFO("connected to action server");
    std::string name{"erl4.cocacola"};

    double timeout = 30.0;

    goal.class_name = name;
    goal.timeout = timeout;

    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();
}
