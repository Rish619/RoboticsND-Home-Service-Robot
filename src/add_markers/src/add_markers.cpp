#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>


const float dist_thres = 0.4;
const bool DEBUG = false;
const int DEBUGodom = 200;
// max tries to receive goal from goal_sub
// /target topic from pick_objects
// if fails to receive, uses hard coded goal
// after each try, sleeps for 0.5 seconds.
const int max_wait_goal = 5;
const bool use_goal_pub = false;

class AddMarkers
{
public:
  AddMarkers();
  // 0 Start, 1 Go to pickup, 2 picking up
  // 3 picked up, 4 reach to drop off, 5 dropped off
  enum MarkerStatus {
    MarkerStart = 0,
    MarkerGoPick = 1,
    MarkerPick = 2,
    MarkerCarry = 3,
    MarkerGoDrop = 4,
    MarkerDrop = 5,
    MarkerUndefined = 6 };
  enum GoalOrder {
    GoalUndefined = 0,
    GoalPickup = 1,
    GoalDrop = 2
  };
private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber goal_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber mb_sub;
  MarkerStatus status;
  // for debug purposes, requires refactor to remove
  MarkerStatus newstatus;
  GoalOrder goal_order;
  bool subscriber_exist;
  geometry_msgs::Pose goal;
  geometry_msgs::Pose odom;
  visualization_msgs::Marker marker;
  // for debug purposes
  int logc;
  int wait_goal_pub;
  ros::Time pick_time;

  bool close_enough(const geometry_msgs::Pose &pos, const geometry_msgs::Pose &target);
  void goalCallback(const geometry_msgs::Pose &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void update();
};

AddMarkers::AddMarkers() {
  logc = 0;
  status = MarkerStart;
  goal_order = GoalUndefined;
  subscriber_exist = false;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  goal_sub = n.subscribe("/target",1,&AddMarkers::goalCallback,this);
  odom_sub = n.subscribe("/odom",1,&AddMarkers::odomCallback,this);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  //marker.pose.position.x = 0.0;
  //marker.pose.position.y = 0.0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  //marker.pose.orientation.w = 0.0;


  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.3f;
  marker.color.g = 0.5f;
  marker.color.b = 0.7f;
  marker.color.a = 1.0;
}


bool AddMarkers::close_enough(const geometry_msgs::Pose &pos, const geometry_msgs::Pose &target) {
  float dx = pos.position.x - target.position.x;
  float dy = pos.position.y - target.position.y;
  float dz = pos.position.z - target.position.z;
  float dist = sqrtf(dx * dx + dy * dy + dz * dz);
  if (DEBUG && (logc % DEBUGodom == 0)) {
    ROS_INFO("d:%f, t:%f, s:%d, n:%d, g:%d, c:%d, x:%f vs %f, y:%f vs %f, z:%f vs %f s0:%d, s1:%d, s2:%d, s3:%d, s4:%d, s5:%d",
      dist, dist_thres,
      status, newstatus, goal_order,
      (dist < dist_thres),
      pos.position.x, target.position.x,
      pos.position.y, target.position.y,
      pos.position.z, target.position.z,
      status == MarkerStart,
      status == MarkerGoPick,
      status == MarkerPick,
      status == MarkerCarry,
      status == MarkerGoDrop,
      status == MarkerDrop);
  }
  return (dist < dist_thres);
}

void AddMarkers::goalCallback(const geometry_msgs::Pose &msg) {
  if (!use_goal_pub) {
    return;
  }
  if (DEBUG) ROS_INFO("Goal received");
  goal.position.x = msg.position.x;
  goal.position.y = msg.position.y;
  goal.position.z = msg.position.z;
  goal.orientation.x = msg.orientation.x;
  goal.orientation.y = msg.orientation.y;
  goal.orientation.z = msg.orientation.z;
  goal.orientation.w = msg.orientation.w;
  if (goal_order == GoalUndefined) goal_order = GoalPickup;
  else if (goal_order == GoalPickup) {
    goal_order = GoalDrop;
    ROS_INFO("Carry the object to the drop off point");
  }
  if (DEBUG) ROS_INFO("Goal received goal_order:%d, x:%f, y:%f, w:%f",
    goal_order, msg.position.x, msg.position.y, msg.orientation.w);
  if (DEBUG) ROS_INFO("Goal received x:%f vs %f, y:%f vs %f, z:%f vs %f",
    msg.position.x,
    goal.position.x,
    msg.position.y,
    goal.position.y,
    msg.orientation.z,
    goal.position.z
    );
  this->update();
}

void AddMarkers::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom.position.x = msg->pose.pose.position.x;
  odom.position.y = msg->pose.pose.position.y;
  odom.position.z = msg->pose.pose.position.z;
  odom.orientation.x = msg->pose.pose.orientation.x;
  odom.orientation.y = msg->pose.pose.orientation.y;
  odom.orientation.z = msg->pose.pose.orientation.z;
  odom.orientation.w = msg->pose.pose.orientation.w;

  bool close = close_enough(odom, goal);
  newstatus = MarkerUndefined;
  //if (status == MarkerGoPick && close) status = MarkerPick;
  //else if (status == MarkerCarry && close) status = MarkerGoDrop;
  if (status == MarkerGoPick && goal_order == GoalPickup && close) {
    newstatus = MarkerPick;
    if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
  }
  else if (status == MarkerCarry && goal_order == GoalDrop && close) {
    newstatus = MarkerGoDrop;
    if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
  }
  logc++;
  if (newstatus != MarkerUndefined) status = newstatus;
  this->update();
}

void AddMarkers::update() {
    if (!subscriber_exist) {
      if (marker_pub.getNumSubscribers() > 0) {
        subscriber_exist = true;
      }
      else {
        if (!ros::ok())
        {
          return;
        }
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
    }
    bool wait_for_goal_pub = false;
    if (goal_order == GoalUndefined && wait_goal_pub <= max_wait_goal) {
      ROS_WARN("Waiting for a goal location %d / %d", wait_goal_pub, max_wait_goal);
      wait_goal_pub++;
      wait_for_goal_pub = true;
      ros::Duration(0.5).sleep();
    }
    if (subscriber_exist && !wait_for_goal_pub) {
      if (status == MarkerPick) {
        marker.action = visualization_msgs::Marker::DELETE;
        ROS_INFO("Robot picks up the object");
        newstatus = MarkerCarry;
        if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
        status = newstatus;
        pick_time = ros::Time::now();
      }
      else if (status == MarkerStart || status == MarkerGoDrop) {
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        if (status == MarkerStart) {
          if(goal_order != GoalPickup) {
            ROS_INFO("Goal publisher doesn't work, update manually pickup goal_order:%d", goal_order);
            goal_order = GoalPickup;
            goal.position.x = 4.0;
            goal.position.y = 6.0;
            goal.orientation.w = -0.5;
          }

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x = goal.position.x;
          marker.pose.position.y = goal.position.y;
          marker.pose.orientation.w = goal.orientation.w;
          newstatus = MarkerGoPick;
          if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
          status = newstatus;
         }

         else if (status == MarkerGoDrop) {
          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x =  goal.position.x;
          marker.pose.position.y = goal.position.y;
          marker.pose.orientation.w =  goal.orientation.w;
          newstatus = MarkerDrop;
          ROS_INFO("Drop the object at the drop off point");
          if (DEBUG) ROS_INFO("Status CHANGE %d.: newstatus:%d from status:%d", logc, newstatus, status);
          status = newstatus;
         }
         else {
          ROS_WARN("ERROR on status:%d or goal_order%d", status, goal_order);
         }
      }
      else if (status == MarkerCarry && goal_order != GoalDrop){
        ros::Duration t_from_pick = ros::Time::now() - pick_time;
        if(t_from_pick >= ros::Duration(5.0)) {
          ROS_INFO("Goal publisher doesn't work, update manually drop off goal_order:%d", goal_order);
          goal_order = GoalDrop;
          goal.position.x = -4.0;
          goal.position.y = 6.0;
          goal.orientation.w = -0.5;
          ROS_INFO("Carry the object to the drop off point");
        }
      }


      marker.lifetime = ros::Duration();

      // Publish the marker



      marker_pub.publish(marker);
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  AddMarkers addMarkers;
  ros::Rate r(1);
  ros::spin();

  return 0;
}