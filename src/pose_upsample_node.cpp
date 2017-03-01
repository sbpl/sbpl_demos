#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>



namespace util{
tf::Transform transformFromPose(geometry_msgs::Pose p)
{
    tf::Transform tf;
    tf.setOrigin( tf::Vector3(
        p.position.x, 
        p.position.y, 
        p.position.z) );
    tf::Quaternion q(
        p.orientation.x, 
        p.orientation.y, 
        p.orientation.z, 
        p.orientation.w); 
    tf.setRotation(q);
    return tf;
}

Eigen::Affine3d affineFromXYZRPY(double x, double y, double z, double roll_rad, double pitch_rad, double yaw_rad)
{
    Eigen::AngleAxisd roll, pitch, yaw;
    roll = Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());
    pitch = Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY());
    yaw = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond Qoffset;
    Qoffset = roll * pitch * yaw ;
    Eigen::Translation3d Toffset(x, y, z);

    Eigen::Affine3d A_output;
    A_output = Toffset * Qoffset;
    return A_output;
}

Eigen::Affine3d affineFromPose(const geometry_msgs::Pose &p){
    Eigen::Translation3d T(
        p.position.x,
        p.position.y,
        p.position.z);

    Eigen::Quaterniond Q(
        p.orientation.w,
        p.orientation.x,
        p.orientation.y,
        p.orientation.z);

    Eigen::Affine3d A;
    A = T * Q;
    return A;
}

geometry_msgs::Pose poseFromAffine(const Eigen::Affine3d &A){
    geometry_msgs::Pose P;
    P.position.x = A.translation()[0];
    P.position.y = A.translation()[1];
    P.position.z = A.translation()[2];

    Eigen::Quaterniond Q(A.linear());
    P.orientation.x = Q.x();
    P.orientation.y = Q.y();
    P.orientation.z = Q.z();
    P.orientation.w = Q.w();
    return P;
}
} // util ns


class PoseUpsampler{

public: 
    PoseUpsampler(bool enableTFVisualization);
    void setRobotEEOffset(double x, double y, double z, double roll, double pitch, double yaw);
    void setPerturbationIntervals(
        std::vector<double> x_interval,
        std::vector<double> y_interval,
        std::vector<double> z_interval,
        std::vector<double> roll_interval,
        std::vector<double> pitch_interval,
        std::vector<double> yaw_interval);
    void generatePoses(const geometry_msgs::Pose& Pinput, std::vector<geometry_msgs::Pose>& vPout);

private:
    void generatePerturbationSet();

    Eigen::Affine3d Arobot_;
    double pct_; // between 0 and 1
    bool robot_transform_set_;
    bool enable_TF_visualization_;
    std::vector<double> perturb_x_interval_ ; // meters
    std::vector<double> perturb_y_interval_ ; // meters
    std::vector<double> perturb_z_interval_ ; // meters
    std::vector<double> perturb_roll_interval_ ; // rad
    std::vector<double> perturb_pitch_interval_ ; // rad
    std::vector<double> perturb_yaw_interval_ ; // rad
    std::vector<Eigen::Affine3d> APerturbations_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::NodeHandle nh_;

}; // class

PoseUpsampler::PoseUpsampler(bool enableTFVisualization=true):
    nh_(),
    tf_broadcaster_(),
    pct_(0.1),
    perturb_x_interval_(1,0),
    perturb_y_interval_(1,0),
    perturb_z_interval_(1,0),
    perturb_roll_interval_(1,0),
    perturb_pitch_interval_(1,0),
    perturb_yaw_interval_(1,0),
    robot_transform_set_(false),
    enable_TF_visualization_(enableTFVisualization)
{
    std::cout << "here\n";
    generatePerturbationSet();
}

void PoseUpsampler::setRobotEEOffset(double x, double y, double z, double roll, double pitch, double yaw)
{
    Arobot_ = util::affineFromXYZRPY(x, y, z, roll, pitch, yaw);
    robot_transform_set_ = true;
}

void PoseUpsampler::generatePoses(const geometry_msgs::Pose& Pobject, std::vector<geometry_msgs::Pose>& vecPout){
    if(not robot_transform_set_){
        std::cout << "Warning: you have to set the robot end-effector offset first, not executing pose generation\n";
        return;
    }

    if(enable_TF_visualization_){
        std::string frame("object_pose");
        tf::Transform tf = util::transformFromPose(Pobject);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", frame.c_str()));
    }

    int count = 0;
    vecPout.clear();
    Eigen::Affine3d Aobject;
    Eigen::Affine3d Apertubation;
    Eigen::Affine3d Aoutput;
    Aobject = util::affineFromPose(Pobject);
    geometry_msgs::Pose Pout;

    for(auto it = APerturbations_.begin(); it != APerturbations_.end(); it++){
        Apertubation = *it;
        Aoutput = Aobject * Apertubation * Arobot_;
        Pout = util::poseFromAffine(Aoutput);
        vecPout.push_back(Pout);

        if(enable_TF_visualization_){
            std::string frame("grasp_pose_");
            frame.append(std::to_string(count));
            count++;
            tf::Transform tf = util::transformFromPose(Pout);
            tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", frame.c_str()));

        }
    }
    return;    
}

void PoseUpsampler::generatePerturbationSet()
{
    std::cout << "generating pertubation set...\n";
    APerturbations_.clear();
    Eigen::Affine3d A;
    for(auto x = perturb_x_interval_.begin(); x != perturb_x_interval_.end(); x++){
    for(auto y = perturb_y_interval_.begin(); y != perturb_y_interval_.end(); y++){
    for(auto z = perturb_z_interval_.begin(); z != perturb_z_interval_.end(); z++){
    for(auto roll = perturb_roll_interval_.begin(); roll != perturb_roll_interval_.end(); roll++){
    for(auto pitch = perturb_pitch_interval_.begin(); pitch != perturb_pitch_interval_.end(); pitch++){
    for(auto yaw = perturb_yaw_interval_.begin(); yaw != perturb_yaw_interval_.end(); yaw++){
        std::cout <<" xyzrpy" << *x << ", " << *y << ", " << *z << "\n";
        A = util::affineFromXYZRPY(*x,*y,*z,*roll,*pitch,*yaw);
        APerturbations_.push_back(A);
    }}}}}}
    std::cout << "Info: Generated " << APerturbations_.size() << " perturbation transforms\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_upsample_node");
    PoseUpsampler PU;
    PU.setRobotEEOffset(1.0,0.0,0.0, 0.0,0.0,1.57);
    ros::Duration(1.0).sleep();
    geometry_msgs::Pose p;
    p.orientation.w = 1;
    std::vector<geometry_msgs::Pose> vPoses;
    PU.generatePoses(p, vPoses);
    return 0;
}