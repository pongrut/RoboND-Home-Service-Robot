/* http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes 
   http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams
   
*/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include  <math.h>

#define PICKUP_GOAL_X       5.0
#define PICKUP_GOAL_Y      -6.0
#define PICKUP_GOAL_Z       0.0
#define PICKUP_GOAL_W       1.0
#define DROPOFF_GOAL_X     -5.5
#define DROPOFF_GOAL_Y     -2.0
#define DROPOFF_GOAL_Z      0.0
#define DROPOFF_GOAL_W      1.0
#define PICKUP_SUCCESSED    1
#define PICKUP_FAILED       0
#define DELIVERY_SUCCESSED  1
#define DELIVERY_FAILED     0

// Function to calculate Euclidean distance
double distanceCalculate (double x1, double y1, double x2, double y2)
{
  // Distance between x1 and x2
  double x = x1 - x2;
  // Distance between y1 and y2
  double y = y1 - y2;
  double dist;
  // Calculating Euclidean distance
  dist = pow (x, 2) + pow (y, 2);
  dist = sqrt (dist);

  return dist;
}


// Listener function to track robot position & distance to destination from /amcl_pose topic
void chatterCallback (const geometry_msgs::
		 PoseWithCovarianceStamped::ConstPtr & msgAMCL)
{
  // Variables to keep previous x, y, distance and current goal
  static double prev_x = 0.0, prev_y = 0.0, prev_dist_to_goal =
    0.0, goal_x, goal_y;
  // Read x of current goal from rosparam
  ros::param::get ("/pick_objects/goal_x", goal_x);
  // Read y of current goal from rosparam
  ros::param::get ("/pick_objects/goal_y", goal_y);

  // Calculate the distance of current position and destination
  double current_dist_to_goal =
    distanceCalculate (msgAMCL->pose.pose.position.x,
		       msgAMCL->pose.pose.position.y, goal_x, goal_y);

  // Print out robot position and distance to destination every 0.1 meter
  if (std::abs (current_dist_to_goal - prev_dist_to_goal) >= 0.1
      || current_dist_to_goal < 1.0)
    {
      ROS_INFO
	("[x:%.2f, y:%.2f, w:%.2f] Robot's %.1f m. away from destination",
	 msgAMCL->pose.pose.position.x, msgAMCL->pose.pose.position.y,
	 msgAMCL->pose.pose.orientation.w, current_dist_to_goal);
      // store current position of x
      prev_x = msgAMCL->pose.pose.position.x;
      // store current position of y
      prev_y = msgAMCL->pose.pose.position.y;
      // store current distance to destination
      prev_dist_to_goal = current_dist_to_goal;

    }

}


// Add object function with passing marker address
int addObj (visualization_msgs::Marker * marker, uint32_t shape, double x,
	double y, double z, double w)
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time::now ();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker->ns = "basic_shapes";
  marker->id = 0;

  // Set the marker type.  Initially this is SPHERE, and it can set to cycles, CUBE, ARROW, and CYLINDER
  marker->type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker->action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker->pose.position.x = x;
  marker->pose.position.y = y;
  marker->pose.position.z = 0.0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = z;
  marker->pose.orientation.w = w;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker->scale.x = 0.3;
  marker->scale.y = 0.3;
  marker->scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker->color.r = 0.0f;
  marker->color.g = 1.0f;
  marker->color.b = 0.0f;
  marker->color.a = 1.0;
  // Set this marker never to auto-delete.
  marker->lifetime = ros::Duration ();

}

// Delete object function with passing marker address
int delObj (visualization_msgs::Marker * marker)
{
  marker->action = visualization_msgs::Marker::DELETE;
}


int main (int argc, char **argv)
{
  // Pickup status and Delivery status variables
  int pickup_status = 0, delivery_status = 0;

  // Initialize node add_markers
  ros::init (argc, argv, "add_markers");
  // Starting a roscpp node
  ros::NodeHandle n;
  // Set frequncy of ros loops to 10Hz
  ros::Rate r (10);
  // Create visualization_msgs::Marker as marker_pub for publishing marker
  ros::Publisher marker_pub =
    n.advertise < visualization_msgs::Marker > ("visualization_marker", 1);

  // Initial false status of rosparam /pickup_success
  ros::param::set ("/pick_objects/pickup_success", PICKUP_FAILED);
  // Initial false status of rosparam /delivery_success
  ros::param::set ("/pick_objects/delivery_success", DELIVERY_FAILED);


  // Set our initial shape type to be a sphere
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  while (ros::ok ())
    {
      // Callback function that will get called when a new message has arrived on the /amcl_pose topic
      ros::Subscriber amcl_pose_sub =
	n.subscribe ("amcl_pose", 1000, chatterCallback);
      visualization_msgs::Marker marker;

      // Publish the marker
      while (marker_pub.getNumSubscribers () < 1)
	{
	  if (!ros::ok ())
	    {
	      return 0;
	    }
	  ROS_WARN_ONCE ("Please create a subscriber to the marker");
	  sleep (1);
	}
      // Add sphere object to pick up location
      addObj (&marker, shape, PICKUP_GOAL_X, PICKUP_GOAL_Y, PICKUP_GOAL_Z,
	      PICKUP_GOAL_W);
      // Publish the adding sphere action
      marker_pub.publish (marker);
      ROS_INFO ("Dropped object!!!\n\n\n");
      ros::Duration(5.0).sleep();

      // Remove the object from the map
      delObj (&marker);
      // Publish the deleting action
      marker_pub.publish (marker);
      ROS_INFO ("Picked object!!!\n\n\n");
      ros::Duration(5.0).sleep();

      // Add sphere object to drop off location
      addObj (&marker, shape, DROPOFF_GOAL_X, DROPOFF_GOAL_Y,
		  DROPOFF_GOAL_Z, DROPOFF_GOAL_W);
      // Publish the adding action
      marker_pub.publish (marker);
      ROS_INFO ("Object's delivered!!!");
      ros::Duration(10.0).sleep();
      
      break;
      // Sleep for the time remaining to let us hit our 10Hz publish rate
      r.sleep ();

    }
  return 0;

}

