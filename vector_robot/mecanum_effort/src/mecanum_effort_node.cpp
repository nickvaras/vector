#include <ros/ros.h>
#include <vector_msgs/Propulsion.h>
#include <geometry_msgs/WrenchStamped.h>

class MecanumEffort
{
// private:
protected:
  ros::NodeHandle nh_; 
  ros::Publisher wrench_publisher;
  ros::Subscriber propulsion_subscriber;
public:
  MecanumEffort()
{
   wrench_publisher       = nh_.advertise<geometry_msgs::WrenchStamped>("/vector/external_wrench",10);
   propulsion_subscriber  = nh_.subscribe("/vector/feedback/propulsion", 1, &MecanumEffort::callback, this);
}

void callback(const vector_msgs::Propulsion::ConstPtr& msg)
{
  float front_left  = msg->wheel_motor_current_A0pk[0];
  float front_right = msg->wheel_motor_current_A0pk[1];
  float rear_left   = msg->wheel_motor_current_A0pk[2];
  float rear_right  = msg->wheel_motor_current_A0pk[3];

  float effort_x    = -(front_left+front_right+rear_left+rear_right);
  float effort_y    = -(-front_left+front_right+rear_left-rear_right);
  float effort_z    = -(-front_left + front_right - rear_left + rear_right);
  
  geometry_msgs::WrenchStamped wrench_msg;  
  wrench_msg.header.frame_id  = "base_link";
  wrench_msg.wrench.force.x   = effort_x;
  wrench_msg.wrench.force.y   = effort_y;
  wrench_msg.wrench.torque.z  = effort_z;
  wrench_publisher.publish(wrench_msg);
}

  ~MecanumEffort(void)
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mecanum_effort");
  MecanumEffort mecanumEffort;
  ros::spin();
  return 0;
}
