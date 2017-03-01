#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <harp_arm/kinematic_utils.h>
#include <harp_arm/harp_arm_tools.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>



namespace util{
tf::Transform geoPose2Transform(geometry_msgs::Pose p)
{
    tf::Transform tf1;
    tf1.setOrigin( tf::Vector3(
        p.position.x, 
        p.position.y, 
        p.position.z) );
    tf::Quaternion q(
        p.orientation.x, 
        p.orientation.y, 
        p.orientation.z, 
        p.orientation.w); 
    tf1.setRotation(q);
    return tf1;
}

Eigen::Affine3d affineFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd roll, pitch, yaw;
    roll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond Qoffset;
    Qoffset = roll * pitch * yaw ;
    Eigen::Translation3d Toffset(x, y, z);

    Eigen::Affine3d A_output;
    A_output = Toffset * Qoffset;
    return A_output;
}
} // util ns


class PoseUpsampler{

public: 
    PoseUpsampler();
    void setRobotEEOffset(double x, double y, double x, double roll, double pitch, double yaw);
    void setRobotEEoffset(const Eigen::Affine3d& Arobot);
    void setPerturbationIntervals(
        std::vector<double> x_interval,
        std::vector<double> y_interval,
        std::vector<double> z_interval,
        double roll_interval,
        double pitch_interval,
        double yaw_interval);
    void setPercentofVisualizedPoses(double pct);
    void generatePoses(const geometry_msgs::Pose& Pinput, std::vector<geometery_msgs::Pose>& vPout);

private:
    void generatePerturbationSet();
    Eigen::Affine3d Arobot_;
    double pct_; // between 0 and 1
    std::vector<double> perturb_x_interval_ ; // meters
    std::vector<double> perturb_y_interval_ ; // meters
    std::vector<double> perturb_z_interval_ ; // meters
    double perturb_roll_interval_ ; // rad
    double perturb_pitch_interval_ ; // rad
    double perturb_yaw_interval_ ; // rad
    std::vector<Eigen::Affine3d> APerturbations_;

}; // class

PoseUpsampler::PoseUpsampler():
    pct_(0.1),
    perturbation_x_interval_(1,0),
    perturbation_y_interval_(1,0),
    perturbation_z_interval_(1,0),
    perturbation_roll_interval_(0),
    perturbation_pitch_interval_(0),
    perturbation_yaw_interval_(0)
{

}

void setRobotEEOffset(double x, double y, double x, double roll, double pitch, double yaw)
{
    Arobot_ = util::affineFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw)
}

void setRobotEEoffset(const Eigen::Affine3d& Arobot)
{
    Arobot_ = Arobot;
}

void setPercentofVisualizedPoses(double pct)
{ 
    pct_ = pct;
    if(pct_ > 1.0) { 
        pct_ = 1.0; 
        std::cout << "Warning: visualization percent set greater than one, saturating value\n";
    }
    if(pct_ < 0.0){
        pct_ = 0.0;
        std::cout << "Warning: visualization percent set less than 0, saturating value\n";   
    }
}


void generatePoses(const geometry_msgs::Pose& Pinput, std::vector<geometery_msgs::Pose>& vPout){
    tf::Transform input_tf;
    input_tf = geoPose2Transform(Pinput);

    Eigen::Affine3d A_input;
    tf::transformTFToEigen(input_tf, A_input);

    Eigen::AngleAxisd roll, pitch, yaw;
    roll = Eigen::AngleAxisd(req.roll, Eigen::Vector3d::UnitX());
    pitch = Eigen::AngleAxisd(req.pitch, Eigen::Vector3d::UnitY());
    yaw = Eigen::AngleAxisd(req.yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond Qoffset;
    Qoffset = roll * pitch * yaw ;
    Eigen::Translation3d Toffset(req.x, req.y, req.z);

    Eigen::Affine3d A_output;
    A_output = A_input * A_perturbation * Toffset * Qoffset;

    geometry_msgs::Pose pout;
    pout.position.x = A_output.translation()[0];
    pout.position.y = A_output.translation()[1];
    pout.position.z = A_output.translation()[2];

    Eigen::Quaterniond Qfinal(A_output.linear() );
    pout.orientation.x = Qfinal.x();
    pout.orientation.y = Qfinal.y();
    pout.orientation.z = Qfinal.z();
    pout.orientation.w = Qfinal.w();
}

int main(int argc, char *argv[]){

    ros::init(argc, argv, "pose_upsample");
    ros::NodeHandle nh;
    ros::Rate r(5);

    ros::ServiceServer service = nh.advertiseService("pose_upsample", &serviceCB);

    

}
