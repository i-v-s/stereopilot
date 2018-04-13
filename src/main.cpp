
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <stdlib.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "wirefinder.h"


class NavSatSubscriber
{
    void callback(const sensor_msgs::NavSatFix::ConstPtr& fix) {
        lat = fix->latitude;
        lon = fix->longitude;
        if (std::isnan(hlat) || std::isnan(hlon)) {
            hlat = lat;
            hlon = lon;
        }
    }
    ros::Subscriber sub_;
public:
    double lat, lon, hlat, hlon;
    NavSatSubscriber(ros::NodeHandle &nh, const std::string& topic = "/mavros/global_position/raw/fix") : hlat(NAN), hlon(NAN) {
        sub_ = nh.subscribe<sensor_msgs::NavSatFix>(topic, 1, &NavSatSubscriber::callback, this);
    }
};


class PosePoseSubscriber
{
  public:
    PosePoseSubscriber(geometry_msgs::Pose *pose_ptr_in, const std::string& topic )
    {
        pose_ptr = pose_ptr_in;
        mav_pose_sub = nh_.subscribe<geometry_msgs::Pose>(topic, 1, &PosePoseSubscriber::MavCallback, this);
    }

  private:
    ros::NodeHandle nh_;
    geometry_msgs::Pose *pose_ptr;
    ros::Subscriber mav_pose_sub;

    void MavCallback(const geometry_msgs::Pose::ConstPtr &pose)
    {
        *pose_ptr = *pose;
    }
};


class PoseSubscriber
{
  public:
    PoseSubscriber(geometry_msgs::PoseStamped *pose_ptr_in, const std::string& topic )
    {
        pose_ptr = pose_ptr_in;
        mav_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(topic, 1, &PoseSubscriber::MavCallback, this);
    }

  private:
    ros::NodeHandle nh_;
    geometry_msgs::PoseStamped *pose_ptr;
    ros::Subscriber mav_pose_sub;

    void MavCallback(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
        *pose_ptr = *pose;
    }
};

class MavStateSubscriber
{
  public:
    MavStateSubscriber(mavros_msgs::State *state_ptr_in)
    {
        state_ptr = state_ptr_in;
        mav_state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 1, &MavStateSubscriber::MavCallback, this);
    }

  private:
    ros::NodeHandle nh_;
    mavros_msgs::State *state_ptr;
    ros::Subscriber mav_state_sub;

    void MavCallback(const mavros_msgs::State::ConstPtr &state)
    {
        *state_ptr = *state;
    }
};

class HitDisarm
{
  public:
    HitDisarm(ros::NodeHandle nh, const std::string &imuTopic = "mavros/imu/data", const std::string &armService = "mavros/cmd/arming") {
         imuSub = nh.subscribe<sensor_msgs::Imu>(imuTopic, 100, &HitDisarm::imuCallback, this);
         armSC = nh.serviceClient<mavros_msgs::CommandBool>(armService);
    }
    void arm(bool val = true) {
        armSrv.request.value = val;
        armSC.call(armSrv);
    }
    double thresh = 25;
  private:
    ros::Subscriber imuSub;
    mavros_msgs::CommandBool armSrv;
    ros::ServiceClient armSC;
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        if(msg->linear_acceleration.z > thresh) {
            armSrv.request.value = false;
            armSC.call(armSrv);
        }
    }
};

class YawController
{
    int state_;
public:
    YawController() : state_(0) {}
    int state() {

    }
    float calcAngularZ(float a) {
        if (a > 0.5) a = 0.5;
        else if (a < -0.5) a = -0.5;
        if (fabs(a) < 0.02) {
           state_ += 3;
           if (state_ > 100) state_ = 100;
        } else {
            state_ -= 5;
           if (state_ < 0) state_ = 0;
        }
        return a;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autopilot");
    ros::NodeHandle nh_;
    ros::Rate rate(30.0);

    geometry_msgs::Pose poi;
    PosePoseSubscriber poi_subscriber(&poi,"autopilot/poi");
    ROS_INFO("poi subscriber start");

    geometry_msgs::PoseStamped pose_in;
    PoseSubscriber pose_subscriber(&pose_in,"mavros/local_position/pose");
    ROS_INFO("local position subscriber start");

    mavros_msgs::State px_state;
    MavStateSubscriber mav_subscriber(&px_state);
    ROS_INFO("px state subscriber start");

    WireFinder wf;
    NavSatSubscriber gps(nh_);

    YawController yawCtl;



    // wait for connection
    while (ros::ok() && !px_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        if (!px_state.connected)
            ROS_INFO("FCU no connect");
    }
    ROS_INFO("FCU connected");

    double roll, pitch, yaw;
    double yaw_out;

    HitDisarm hd(nh_);

    std_msgs::Bool reply_wp_out;
    ros::Publisher reply_wp_pub = nh_.advertise<std_msgs::Bool>("autopilot/reply_wp", 1);

    geometry_msgs::TwistStamped vel = {};
    vel.header.frame_id = "0";
    ros::Publisher velocity_sp = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

    reply_wp_out.data = true;
    double spy = -70, spz = 13.5;
    //double spy = -53, spz = 8.5;

    double poix, poiy, poiz;
    enum {
        TAKEOFF,
        SEARCH,
        DESCENT
    } mode = TAKEOFF;
    int pt = 0;
    int seq = 0, ctr = 0, ctr1 = 0;

    while (ros::ok())
    {
        //get actual data
        tf::Quaternion q(pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        if (px_state.armed && px_state.mode == "OFFBOARD")
        { 
            double gy = (gps.lat - gps.hlat) * 1E6, gx = (gps.lon - gps.hlon) * 1E6;

            if (mode != DESCENT) {
                double spzt = (mode == TAKEOFF) ? 13.5 : spz;
                double vz = (spzt - pose_in.pose.position.z);
                if(vz > 1.5) vz = 1.5;
                else if(vz < -1.) vz = -0.5;
                vel.twist.linear.z = vz;
            }

            if (mode == TAKEOFF) {
                if (pose_in.pose.position.z < 13) {
                    vel.twist.linear.x = 0;
                    vel.twist.linear.y = 0;
                } else if (fabs(gy - spy) > 15) {
                    vel.twist.linear.y = spy - gy;
                    if (fabs(vel.twist.linear.y) > 1.5) vel.twist.linear.y /= fabs(vel.twist.linear.y) / 1.5;
                } else mode = SEARCH;
            }

            if (mode == SEARCH || mode == DESCENT) {
                if (mode == SEARCH) {
                    if (gy > spy) vel.twist.linear.y = -0.3;
                    else vel.twist.linear.y = 0.3;
                }

                vel.twist.linear.x = 0;

                if(std::isnan(wf.a_)) vel.twist.angular.z = 0;
                else if (fabs(gy - spy) < 7){
                    vel.twist.angular.z = yawCtl.calcAngularZ(wf.a_);

                    double vy = -wf.b_ * 3 - pose_in.pose.orientation.x * 0.5;
                    if (vy > 0.5) vy = 0.5;
                    else if (vy < -0.5) vy = -0.5;

                    vel.twist.linear.y = vy;
                    if (fabs(vy) < 0.1 && fabs(pose_in.pose.orientation.x) < 0.05) {
                        if (ctr > 10) {
                            mode = DESCENT;
                            /*if (ctr > 50)
                                hd.thresh = 11;*/
                        }
                        ctr++;
                        vel.twist.linear.z = -0.1;
                    } else {
                        ctr = 0;
                        vel.twist.linear.z *= 0.8;
                    }
                }
            }

            if (mode == DESCENT && std::isnan(wf.a_)) {
                ctr1++;
                if (ctr1 > 100)
                    hd.arm(false);
            } else ctr1 = 0;
        }
        else
        {
            if (mode == DESCENT) {
                ros::Duration(3).sleep();
                //spz = 7;
                pt++;
                if (pt > 1) pt = 0;
                switch (pt) {
                case 0: spy = -70; spz = 10; break;
                case 1: spy = -53; spz = 8.5; break;
                }
                mode = TAKEOFF;
                hd.arm();
            } else if (px_state.mode == "OFFBOARD") {
                hd.arm();
            }
            yaw_out = yaw;
            /*if (px_state.mode == "OFFBOARD") {
                if (ctr1 < 100) ctr1++;
                else {
                    hd.arm();
                    ctr1 = 0;
                }
            }*/
        }

        //////////////////// copter_state publish  /////////////////////////////////////////
          // x - допустим перед y - допустим право   z - up/down
        //update_pose_out(pose_out, yaw_out, q);
        //local_pos_pub.publish(pose_out);
        vel.header.stamp = ros::Time::now();
        vel.header.seq = seq++;
        velocity_sp.publish(vel);
        reply_wp_pub.publish(reply_wp_out);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
