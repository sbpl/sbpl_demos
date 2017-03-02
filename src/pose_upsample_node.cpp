#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>

#include <sbpl_demos/PoseUpsample.h>
#include <sbpl_demos/XYZRPY.h>



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
    void setRobotEEOffset(sbpl_demos::XYZRPY xyzrpy);
    void setPerturbationIntervals(
        std::vector<double> x_interval,
        std::vector<double> y_interval,
        std::vector<double> z_interval,
        std::vector<double> roll_interval,
        std::vector<double> pitch_interval,
        std::vector<double> yaw_interval);
    void generatePoses(const geometry_msgs::Pose& Pinput, std::vector<geometry_msgs::Pose>& vPout);
    void generatePoseswIKChecking(const geometry_msgs::Pose& Pinput, std::vector<geometry_msgs::Pose>& vPout);
    void run();

private:
    
    void generatePerturbationSet();
    bool checkIK(const geometry_msgs::Pose& p);
    bool serviceCB(sbpl_demos::PoseUpsample::Request &req, sbpl_demos::PoseUpsample::Response &res);


    ros::NodeHandle nh_;
    Eigen::Affine3d Arobot_;
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
    ros::ServiceClient ikService_;
    std::vector<std::string> joint_names_;
    std::string reference_frame_; // usually map
    std::string planning_group_; // usually manipulator
    ros::ServiceServer service_;

}; // class

PoseUpsampler::PoseUpsampler(bool enableTFVisualization=true):
    nh_(),
    tf_broadcaster_(),
    perturb_x_interval_(1,0),
    perturb_y_interval_(1,0),
    perturb_z_interval_(1,0),
    perturb_roll_interval_(1,0),
    perturb_pitch_interval_(1,0),
    perturb_yaw_interval_(1,0),
    robot_transform_set_(false),
    enable_TF_visualization_(enableTFVisualization),
    ikService_(nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik")),
    joint_names_({"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}),
    reference_frame_("map"),
    planning_group_("manipulator"),
    service_(nh_.advertiseService("pose_upsampling", &PoseUpsampler::serviceCB, this))
{

    generatePerturbationSet();
    std::cout << "Connecting to IK Service\n";
    if(not ikService_.waitForExistence(ros::Duration(1.0))){
        std::cout << "Warning: Could not connect to IK service\n";
    }
}

void PoseUpsampler::setRobotEEOffset(double x, double y, double z, double roll, double pitch, double yaw)
{
    Arobot_ = util::affineFromXYZRPY(x, y, z, roll, pitch, yaw);
    std::cout << " set robot ee offset to " << x << ", " << y << ", " << x << ", " << yaw << "\n";
    robot_transform_set_ = true;
}

void PoseUpsampler::setRobotEEOffset(sbpl_demos::XYZRPY xyzrpy)
{
    setRobotEEOffset(xyzrpy.x, xyzrpy.y, xyzrpy.z, xyzrpy.roll, xyzrpy.pitch, xyzrpy.yaw);
}

void PoseUpsampler::setPerturbationIntervals(
        std::vector<double> x_interval,
        std::vector<double> y_interval,
        std::vector<double> z_interval,
        std::vector<double> roll_interval,
        std::vector<double> pitch_interval,
        std::vector<double> yaw_interval)
{
    if(not x_interval.empty()    ){perturb_x_interval_ = x_interval;        } else {perturb_x_interval_= std::vector<double>(1,0);}
    if(not y_interval.empty()    ){perturb_y_interval_ = y_interval;        } else {perturb_y_interval_= std::vector<double>(1,0);}
    if(not z_interval.empty()    ){perturb_z_interval_ = z_interval;        } else {perturb_z_interval_= std::vector<double>(1,0);}
    if(not roll_interval.empty() ){perturb_roll_interval_ = roll_interval;  } else {perturb_roll_interval_= std::vector<double>(1,0);}
    if(not pitch_interval.empty()){perturb_pitch_interval_ = pitch_interval;} else {perturb_pitch_interval_= std::vector<double>(1,0);}
    if(not yaw_interval.empty()  ){perturb_yaw_interval_ = yaw_interval;    } else {perturb_yaw_interval_= std::vector<double>(1,0);}
    generatePerturbationSet();
}

void PoseUpsampler::generatePoseswIKChecking(const geometry_msgs::Pose& Pobject, std::vector<geometry_msgs::Pose>& vecPout)
{
    int count = 0;
    if(not ikService_.exists())
    {
        std::cout << "Warning: ik service not connected, returning early\n";
        return;
    }
    std::vector<geometry_msgs::Pose> allPoses;
    generatePoses(Pobject, allPoses);
    vecPout.clear();
    for(auto it = allPoses.begin(); it != allPoses.end(); it++)
    {
        if(checkIK(*it)){
            vecPout.push_back(*it); 
        }
    }
    std::cout << "Info: found " << vecPout.size() << " valid poses\n";

    for(auto pit = vecPout.begin(); pit != vecPout.end(); pit++){
        std::string frame("grasp_pose_");
        frame.append(std::to_string(count));
        count++;
        tf::Transform tf = util::transformFromPose(*pit);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), reference_frame_, frame.c_str()));
        ros::Duration(0.001).sleep();

    }


    return;
}

void PoseUpsampler::generatePoses(const geometry_msgs::Pose& Pobject, std::vector<geometry_msgs::Pose>& vecPout){
    if(not robot_transform_set_){
        std::cout << "Warning: you have to set the robot end-effector offset first, not executing pose generation\n";
        return;
    }

    if(enable_TF_visualization_){
        std::string frame("object_pose");
        tf::Transform tf = util::transformFromPose(Pobject);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), reference_frame_, frame.c_str()));
        ros::Duration(0.001).sleep();
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
            tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), reference_frame_, frame.c_str()));
            ros::Duration(0.001).sleep();

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
        A = util::affineFromXYZRPY(*x,*y,*z,*roll,*pitch,*yaw);
        APerturbations_.push_back(A);
    }}}}}}
    std::cout << "Info: Generated " << APerturbations_.size() << " perturbation transforms\n";
}

bool PoseUpsampler::checkIK(const geometry_msgs::Pose &input_pose)
{
    // build ik service messages
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    js.name = joint_names_;
    js.position = std::vector<double>(js.name.size(), 0); // set seed position to 0,0,0....
    js.velocity = std::vector<double>(js.name.size(), 0);
    
    moveit_msgs::PositionIKRequest ik_request;
    ik_request.robot_state.joint_state = js;
    ik_request.group_name = planning_group_;
    ik_request.avoid_collisions = true; // <-- if true, should not need to call validity checker
    ik_request.pose_stamped.header.frame_id = reference_frame_;
    ik_request.attempts = 10;
    ik_request.pose_stamped.pose = input_pose;
    ik_request.pose_stamped.header.stamp = ros::Time::now();

    moveit_msgs::GetPositionIK ik_srv;
    ik_srv.request.ik_request = ik_request;

    if(ikService_.call(ik_srv) ){ // compute_IK server is UP

        // Check if valid IK solution was found
        //std::cout << "Error code: " << ik_srv.response.error_code.val << "\n";
        if(ik_srv.response.error_code.val == 1){ 
            return true;
        }

    }
    return false;

}

void PoseUpsampler::run(){
    while(ros::ok()){
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
}

bool PoseUpsampler::serviceCB(sbpl_demos::PoseUpsample::Request &req, sbpl_demos::PoseUpsample::Response &res)
{
    std::cout << "Info: Received service call\n";

    enable_TF_visualization_ = req.visualize_poses_with_tf;
    planning_group_ = req.planning_group;
    reference_frame_ = req.reference_frame;
    joint_names_ = req.joint_names;
    setRobotEEOffset(req.robot_transform_to_object);
    setPerturbationIntervals(req.x_interval, req.y_interval, req.z_interval,
        req.roll_interval, req.pitch_interval, req.yaw_interval);
    std::vector<geometry_msgs::Pose> valid_poses;
    if(req.check_against_ik){
        generatePoseswIKChecking(req.object_pose, valid_poses);
    }else{
        generatePoses(req.object_pose, valid_poses);
    }
    res.valid_poses = valid_poses;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_upsample_node");
    PoseUpsampler PU;

    PU.setRobotEEOffset(1.0,0.0,0.0, 0.0,0.0,1.57);
    PU.setPerturbationIntervals(
        {0},
        {0},
        {0},
        {-3.0, -2.0, -1, 0, 1, 2, 3},
        {0},
        {-3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3});
    ros::Duration(1.0).sleep();
    geometry_msgs::Pose p;
    p.orientation.w = 1;
    std::vector<geometry_msgs::Pose> vPoses;
    PU.generatePoses(p, vPoses);
    PU.generatePoseswIKChecking(p, vPoses);
    PU.run();
    return 0;
}