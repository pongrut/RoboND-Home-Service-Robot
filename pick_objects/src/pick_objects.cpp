/* http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals 
   https://roboticsbackend.com/get-set-ros-params-rospy-roscpp/
   http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef
  actionlib::SimpleActionClient <
  move_base_msgs::MoveBaseAction >
  MoveBaseClient;

#define PICKUP_GOAL_X       5.0
#define PICKUP_GOAL_Y      -6.0
#define PICKUP_GOAL_W       1.0
#define DROPOFF_GOAL_X     -5.5
#define DROPOFF_GOAL_Y     -2.0
#define DROPOFF_GOAL_W      1.0
#define PICKUP_SUCCESSED    1
#define PICKUP_FAILED       0
#define DELIVERY_SUCCESSED  1
#define DELIVERY_FAILED     0

// Set robot goal function
int setGoal (MoveBaseClient * ac, double x, double y, double w)
{
  // Create MoveBaseGoal object as goal 
  move_base_msgs::MoveBaseGoal goal;
  // Set current goal of rosparam /goal_x
  ros::param::set ("/pick_objects/goal_x", x);
  // Set current goal of rosparam /goal_y
  ros::param::set ("/pick_objects/goal_y", y);

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now ();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO ("Sending goal x:%.2f, y:%.2f, w:%.2f", x, y, w);
  ac->sendGoal (goal);
 
  // Wait an infinite time for the results
  ac->waitForResult ();

  // Check if the robot reached its goal
  if (ac->getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO ("The robot's successfully moved to x:%.2f, y:%.2f, w:%.2f", x,
		y, w);
      return 0;
    }
  else
    {
      ROS_INFO ("The robot's failed to move to x:%.2f, y:%.2f, w:%.2f", x, y,
		w);
      return 1;
    }
}


int main (int argc, char **argv)
{
  // Pickup and Delivery variables of return status from setGoal function
  int
    pickup_success,
    delivery_success;
  // Pickup and Delivery variables for checking rosparam
  int
    pickup_status,
    delivery_status;
  // Initialize the simple_navigation_goals node
  ros::init (argc, argv, "pick_objects");
  // Initial false status of rosparam /arrived_pickup
  ros::param::set ("/pick_objects/pickup_success", PICKUP_FAILED);
  // Initial false status of rosparam /arrived_pickup
  ros::param::set ("/pick_objects/delivery_success", DELIVERY_FAILED);


  //tell the action client that we want to spin a thread by default
  MoveBaseClient
  ac ("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while (!ac.waitForServer (ros::Duration (5.0)))
    {
      ROS_INFO ("Waiting for the move_base action server to come up");
    }

  // Goto pickup location
  pickup_success = setGoal (&ac, PICKUP_GOAL_X, PICKUP_GOAL_Y, PICKUP_GOAL_W);
  if (pickup_success == 0)
    {
      // Update status of rosparam /pickup_success to PICKUP_SUCCESSED
      ros::param::set ("/pick_objects/pickup_success", PICKUP_SUCCESSED);
    }
  else
    {
      // Reset status of rosparam /pickup_success to PICKUP_FAILED
      ros::param::set ("/pick_objects/pickup_success", PICKUP_FAILED);
    }
  // Test reading rosparam /pickup_success
  ros::param::get ("/pick_objects/pickup_success", pickup_status);
  ROS_INFO ("Picking status %d", pickup_status);
  // Wait 5 sec for the robot to pickup objects
  ROS_INFO ("Waiting for the robot to pickup objects");
  ros::Duration (5.0).sleep ();

  // Goto drop off location
  delivery_success =
    setGoal (&ac, DROPOFF_GOAL_X, DROPOFF_GOAL_Y, DROPOFF_GOAL_W);
  if (delivery_success == 0)
    {
      // Update status of rosparam /delivery_success to DELIVERY_SUCCESSED
      ros::param::set ("/pick_objects/delivery_success", DELIVERY_SUCCESSED);
    }
  else
    {
      // Reset status of rosparam /delivery_success to DELIVERY_FAILED
      ros::param::set ("/pick_objects/delivery_success", DELIVERY_FAILED);
    }
  // Test reading rosparam /delivery_success
  ros::param::get ("/pick_objects/delivery_success", delivery_status);
  ROS_INFO ("Delivery status %d", delivery_status);

  return 0;
}

