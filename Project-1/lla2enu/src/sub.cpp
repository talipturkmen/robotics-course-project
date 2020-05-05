#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class lla2enu
{

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::Publisher odom_pub;
  std_msgs::String msg_out; //TODO
  string topic;
  tf::TransformBroadcaster br;

  float x_old;
  float y_old;
  float z_old;
  ros::Time last_time;

public:
  lla2enu(char *top)
  {
    topic = string(top);
    ROS_INFO("Topic: %s", top);

    string sub_topic = "/swiftnav/" + topic + "/gps_pose";
    // ROS_INFO("Listening topic: %s",sub_topic);

    sub = n.subscribe(sub_topic, 1, &lla2enu::callback, this);
    odom_pub = n.advertise<nav_msgs::Odometry>(topic + "_odom", 1);
  }

  float calculateLinearSpeed(float x1, float x2, ros::Time current_time)
  {
    if (!isnan(x1))
    {
      double dt = (current_time - last_time).toSec();
      return (x1 - x2) / dt;
    }
    else
      return 0;
  }

  bool isSignalLost(float latitude, float longitude, float h)
  {
    return latitude == 0 && longitude == 0 && h == 0;
  }

  void callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude, msg->altitude);

    // fixed values
    double a = 6378137;
    double b = 6356752.3142;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);
    float deg_to_rad = 0.0174533;

    // input data from msg
    float latitude = msg->latitude;
    float longitude = msg->longitude;
    float h = msg->altitude;

    // fixed position
    //45.621656,9.281552, 224.616617
    float latitude_init;
    float longitude_init;
    float h0;

    ros::NodeHandle n;
    if (n.hasParam("/latitude_init"))
    {
      n.getParam("/latitude_init", latitude_init);
      ROS_INFO("Latitude init got");
    }
    else
    {
      ROS_INFO("No param named 'latitude_init' Default value:45.6311926152");
    }

    if (n.hasParam("/longitude_init"))
    {
      n.getParam("/longitude_init", longitude_init);
    }
    else
    {
      ROS_INFO("No param named 'longitude_init' Default value:9.2947495255");
    }
    if (n.hasParam("/h0"))
    {
      n.getParam("/h0", h0);
    }
    else
    {
      ROS_INFO("No param named 'h0' Default value:231.506675163");
    }

    //lla to ecef
    float lamb = deg_to_rad * (latitude);
    float phi = deg_to_rad * (longitude);
    float s = sin(lamb);
    float N = a / sqrt(1 - e_sq * s * s);

    float sin_lambda = sin(lamb);
    float cos_lambda = cos(lamb);
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    float x = (h + N) * cos_lambda * cos_phi;
    float y = (h + N) * cos_lambda * sin_phi;
    float z = (h + (1 - e_sq) * N) * sin_lambda;

    ROS_INFO("ECEF position: [%f,%f, %f]", x, y, z);

    // ecef to enu

    lamb = deg_to_rad * (latitude_init);
    phi = deg_to_rad * (longitude_init);
    s = sin(lamb);
    N = a / sqrt(1 - e_sq * s * s);

    sin_lambda = sin(lamb);
    cos_lambda = cos(lamb);
    sin_phi = sin(phi);
    cos_phi = cos(phi);

    float x0 = (h0 + N) * cos_lambda * cos_phi;
    float y0 = (h0 + N) * cos_lambda * sin_phi;
    float z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

    float xd = x - x0;
    float yd = y - y0;
    float zd = z - z0;

    float xEast = -sin_phi * xd + cos_phi * yd;
    float yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    float zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    ros::Time current_time = ros::Time::now();

    if (!isSignalLost(latitude, longitude, h))
    {
      last_time = current_time;
      x_old = xEast;
      y_old = yNorth;
      z_old = zUp;
    }
    else
    {
      xEast = NAN;
      yNorth = NAN;
      zUp = NAN;
    }

    ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth, zUp);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(xEast, yNorth, zUp));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, current_time, "world", topic));

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = xEast;
    odom.pose.pose.position.y = yNorth;
    odom.pose.pose.position.z = zUp;

    odom.twist.twist.linear.x = calculateLinearSpeed(xEast, x_old, current_time);
    odom.twist.twist.linear.y = calculateLinearSpeed(yNorth, y_old, current_time);
    odom.twist.twist.linear.z = calculateLinearSpeed(zUp, z_old, current_time);
    odom.header.stamp = current_time;

    odom.header.frame_id = "world";
    odom.child_frame_id = topic;
    odom_pub.publish(odom);
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  lla2enu my_converter(argv[1]);

  ros::spin();

  return 0;
}
