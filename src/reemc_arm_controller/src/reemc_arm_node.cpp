/* reemc__arm_node.cpp
*
* Zheng Du, Oct 2018
* Lets the REEM-C arms follow the user's.
*
* Takes the TF data, which was produced by the Perception Neuron suit
* and sent to tf by perc_neuron_tf_broadcaster.
* Calculates the goal positions via IK using the Jacobian, checks for
* singular configurations and publishes them to the robot.
*
* This software may be modified and distributed under the terms
* of the GNU license.  See the LICENSE file for details.
*/

/*
 * git: github.com/zhengduu/reemc
 * second test...
 *
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <math.h>
#include <stdio.h>
#include <ctime>
#include <qt4/Qt/qvector.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Dense>

using namespace KDL;

// Looks up a tf transfrom and pushes them into an array to calculate goal transform
// Takes the tf from transOrigin to transDestination in tfListener and
// pushes them into transArray
// Returns 0 for success, -1 for failure
int pushIntoTransformArray(std::string target_frame, std::string source_frame,
                           std::vector<tf::Transform>* transArray,
                           tf::TransformListener* tfListener) {

    tf::StampedTransform transform;
    try {
      tfListener->lookupTransform(target_frame, source_frame,
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }
    transArray->push_back(tf::Transform(transform.getRotation(), transform.getOrigin()));
    return 0;
}

// Interface to publish joint state messages to robot controller
struct pubIntf{

    double jointPubCycleTime;                 // Cycle time at which joint states are published
    ros::Publisher jointStatePublisher;       // Publishes desired joint states
    sensor_msgs::JointState armJointStateMsg; // Published message with desired values for arm joints

};

// Interface to tf Data, which is published by PN tf broadcaster
struct tfPNInterface {
    // tf listener to data from external tf broadcaster
    tf::TransformListener    tfListener;
    // Broadcasts goal transforms for visualization
    tf::TransformBroadcaster tfBroadcaster;

    // PN body joints. Necessary to listen to them with tfListener
    typedef std::vector<std::string> BodyJoints;
    typedef std::vector<tf::Transform> Array;

    BodyJoints jointsLeftArm;
    BodyJoints jointsRightArm;

    // 6x1 vector with translation and rotation in euler angles of desired position
    // needed to calculate goal joint states with inverse Jacobian
    Eigen::Matrix<double, 6, 1> leftarmPose;
    Eigen::Matrix<double, 6, 1> rightarmPose;

    int getTransform(){

        Array transformArrayLeftArm;
        Array transformArrayRightArm;

        // Transforms of both arms
        // Rotation and translation will be needed for the position vector
        tf::Transform transformLeftArm;
        tf::Transform transformRightArm;

        int success = 0;  // = 0 for success of getTransform; < 0 for failure

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Fill arrays with transforms, array with joint names is provided, i = 0 equals Hips
        for (int i = 0; i < jointsLeftArm.size() - 1; i++){
            // Returns 0 for success, -1 for failure
            success += pushIntoTransformArray(jointsLeftArm.at(i),  jointsLeftArm.at(i + 1),
                                              &transformArrayLeftArm, &tfListener);
            success += pushIntoTransformArray(jointsRightArm.at(i), jointsRightArm.at(i + 1),
                                              &transformArrayRightArm, &tfListener);

            if (success != 0) {
                printf("\nNo tf transformations could be found."
                       "\nPlease check rosserial_server node , perc_neuron_tf_broadcaster "
                       "or PN publisher on Windows.");
                return -1;
            }
        }

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Goal transform of arms and the 6-dim position vector
        transformLeftArm  = transformArrayLeftArm.at(0);
        transformRightArm = transformArrayRightArm.at(0);
        for (int i = 1; i < transformArrayLeftArm.size(); i++){
            transformLeftArm  = transformLeftArm.operator *=(transformArrayLeftArm.at(i));
            transformRightArm = transformRightArm.operator*=(transformArrayRightArm.at(i));
        }
        double roll_left;
        double pitch_left;
        double yaw_left;
        double roll_right;
        double pitch_right;
        double yaw_right;

        // Get EE's rotation in euler angles
        transformLeftArm.getBasis().getRPY(roll_left, pitch_left, yaw_left);
        transformRightArm.getBasis().getRPY(roll_right, pitch_right, yaw_right);

        leftarmPose(0) = transformLeftArm.getOrigin().getX();
        leftarmPose(1) = transformLeftArm.getOrigin().getY();
        leftarmPose(2) = transformLeftArm.getOrigin().getZ();
        leftarmPose(3) = roll_left;
        leftarmPose(4) = pitch_left;
        leftarmPose(5) = yaw_left;

        rightarmPose(0) = transformRightArm.getOrigin().getX();
        rightarmPose(1) = transformRightArm.getOrigin().getY();
        rightarmPose(2) = transformRightArm.getOrigin().getZ();
        rightarmPose(3) = roll_right;
        rightarmPose(4) = pitch_right;
        rightarmPose(5) = yaw_right;

        // Broadcast transforms
        tfBroadcaster.sendTransform(tf::StampedTransform(transformLeftArm, ros::Time(0),
                                                         "/LeftShoulder", "/ShoulderToLeftArm"));
        tfBroadcaster.sendTransform(tf::StampedTransform(transformRightArm, ros::Time(0),
                                                         "/RightShoulder", "/ShoulderToRightArm"));

        transformArrayLeftArm.clear();
        transformArrayRightArm.clear();
        return 0;
    }
};

// Inverse kinematics calculation to get desired joint states
// Check constraints using the SVD
// Publish new joint values to robot
// Return 0 if successful, -1 otherwise
int calcIKJacAndPublishArmJointStates(pubIntf& pubIntf, tfPNInterface& tfPNIntf,
                                      Eigen::Matrix<double, 7, 1>& defaultPosLeftArm,
                                      Eigen::Matrix<double, 7, 1>& defaultPosRightArm,
                                      Eigen::Matrix<double, 7, 1>& lastPubPosLeftArm,
                                      Eigen::Matrix<double, 7, 1>& lastPubPosRightArm,
                                      double cycleTime){

    /* Input arguments needed for function:
     * - cycleTime -> 1/fpsPercNeuron
     * - publish interface to publish joint states
     * - tfPNInterface to get transforms for EE = goal position
     * - receive robots default position from controller (or set manually)
     * - last published position (=default position when initializing)
     */


    // Define joint/pose vectors and Jacobian (inverse) matrices
    enum{ joints = 7 };
    typedef Eigen::Matrix<double, 6, 1>           PoseVector;
    typedef Eigen::Matrix<double, joints, 1>      JointVector;
    typedef Eigen::Matrix<double, 6, joints>      Jacobian;
    typedef Eigen::Matrix<double, joints, joints> Jac_tmp;
    typedef Eigen::Matrix<double, joints, 6>      Jac_pseudoInv;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Vector with current position and orientation of EE
    // Equals robot's default position when initializing
    PoseVector X_current_left;
    PoseVector X_current_right;

    // Difference between goal position and current/last position of EE
    // Take goal position from tfPNInterface
    PoseVector X_delta_left;
    PoseVector X_delta_right;

    // Vector with current joint states
    // First iteration is using default positions of the robot
    // Get default/last published joint states from publisher
    JointVector q_current_left;
    JointVector q_current_right;

    // Vector with the desired joint states
    // Values will be published to armJointStatemsg
    JointVector q_goal_left;
    JointVector q_goal_right;

    // Jacobian matrix for robot's current joint states
    // Copy from jac_kdl
    Jacobian J_left;
    Jacobian J_right;

    // (J.transpose * J).inverse
    Jac_tmp J_tmp_left;
    Jac_tmp J_tmp_right;

    // Left pseudo-inverse of Jacobian, robot restrictions can be checked by taking
    // the SVD and looking at smallest singular value
    // J_# = (J.T * J).inv * J_T;
    // J_# * J = I (size nxn)
    Jac_pseudoInv J_inv_left;
    Jac_pseudoInv J_inv_right;
    Eigen::VectorXd singularValues_left;
    Eigen::VectorXd singularValues_right;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Construct KDL tree from robots URDF file using kdl_parser
    // Create kinematic chains from the tree
    KDL::Tree reemc_kin;
    kdl_parser::treeFromFile("/home/duzheng/ros/workspaces/duzheng_test_ws/"
                             "src/tum_ics_h1_robot/tum_ics_h1_description/robots/"
                             "reemc_full_ft_hey5.urdf", reemc_kin);
    // Set root of chains at the "shoulders" and tips at the EE
    KDL::Chain chain_left;
    KDL::Chain chain_right;
    reemc_kin.getChain("arm_left_1_link", "arm_left_7_link", chain_left);
    reemc_kin.getChain("arm_right_1_link", "arm_right_7_link", chain_right);

    // Create JntArrays and Jacobians
    // Equals q_current (last published/default joint states)
    KDL::JntArray q_kdl_left(chain_left.getNrOfJoints());
    KDL::JntArray q_kdl_right(chain_right.getNrOfJoints());
    KDL::Jacobian jac_kdl_left;
    KDL::Jacobian jac_kdl_right;
    // Create solvers to get the Jacobians based on the kinematic chains of the arms
    // and the current joint states
    ChainJntToJacSolver jac_solver_left  = ChainJntToJacSolver(chain_left);
    ChainJntToJacSolver jac_solver_right = ChainJntToJacSolver(chain_right);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // (?) for loop needed to compute joint states and publish every cycle

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Set q_current as last published position or get robot's
    // default confuguration when initializing for the first time
    // Copy q_current in q_kdl to calculate corresponding Jacobian matrix
    for (int i = 0; i < q_current_left.size(); i++){
        q_kdl_left(i)  = q_current_left(i);
        q_kdl_right(i) = q_current_right(i);
    }
    jac_solver_left .JntToJac(q_kdl_left,  jac_kdl_left);
    jac_solver_right.JntToJac(q_kdl_right, jac_kdl_right);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Get X_current and calculate X_delta = tf(armpose) - X_current

    X_delta_left  = tfPNIntf.leftarmPose  - X_current_left;
    X_delta_right = tfPNIntf.rightarmPose - X_current_right;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // KDL::Jacobian -> Eigen::Matrix; J_left == jac_kdl
    // Jacobian has m = 6 rows and n = #joints columns
    for (int n = 0; n < chain_left.getNrOfJoints(); n++){
        for (int m = 0; m < 6; m++){
            J_left(m, n)  = jac_kdl_left(m, n);
            J_right(m, n) = jac_kdl_right(m, n);
        }
    }

    // Determine the left pseudo-inverse of Jacobian matrix
    // (Moore-Penrose inverse)
    J_tmp_left  = (J_left.transpose() * J_left).inverse();
    J_tmp_right = (J_right.transpose()* J_right).inverse();
    J_inv_left  = J_tmp_left * J_left.transpose();
    J_inv_right = J_tmp_right * J_right.transpose();

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Determine SVD of Jacobians, check singular values
    // to avoid robot going into a singular configuration
    // (0 < sigma_j < epsilon_threshhold)
    Eigen::JacobiSVD<Jacobian> svd_left(J_left);
    Eigen::JacobiSVD<Jacobian> svd_right(J_right);
    singularValues_left  = svd_left.singularValues();
    singularValues_right = svd_right.singularValues();

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    /* for (all singular values)
         if (bigger than threshhold)
             ok; ee still far enough from singularity
         else
             ->Damped Jacobian method:
             J_damped = V * S_rho * U^T, with rho << 1 as damping factor
             (rho -> 0, damped pseudo-inv = pseudo-inverse)
    */

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Final joint states (after checking for singularities):
    // q_goal = q_current + Jac_pseudoInv * X_delta;

    q_goal_left = q_current_left + J_inv_left * X_delta_left;
    q_goal_left = q_current_left + J_inv_left * X_delta_left;


    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Publish joint states to robot
    // for loop to publish every cycle and interpolate with the rate
    // ros::Rate rate(1 / pubIntf.jointPubCycleTime); //rate.sleep();

    pubIntf.armJointStateMsg.header.stamp = ros::Time(0);
    pubIntf.armJointStateMsg.position = {
        q_goal_left(0),  q_goal_left(1),  q_goal_left(2),  q_goal_left(3),
        q_goal_left(4),  q_goal_left(5),  q_goal_left(6),
        q_goal_right(0), q_goal_right(1), q_goal_right(2), q_goal_right(3),
        q_goal_right(4), q_goal_right(5), q_goal_right(6)};
    pubIntf.jointStatePublisher.publish(pubIntf.armJointStateMsg);

    ROS_INFO_STREAM("REEM-C arm joint states: " <<
                    q_goal_left(0)  << " " << q_goal_left(1)  << " " <<
                    q_goal_left(2)  << " " << q_goal_left(3)  << " " <<
                    q_goal_left(4)  << " " << q_goal_left(5)  << " " <<
                    q_goal_left(6)  << " " << q_goal_right(0) << " " <<
                    q_goal_right(1) << " " << q_goal_right(2) << " " <<
                    q_goal_right(3) << " " << q_goal_right(4) << " " <<
                    q_goal_right(5) << " " << q_goal_right(6) << "\n");
    return 0;
}

int main(int argc, char** argv){
    // Declare general variables
    ros::init(argc, argv, "follow_hand_node");
    ros::NodeHandle  node;
    ros::NodeHandle& node_ref = node;
    int loopCounter = 0;
    int success;    // check for successful execution of functions

    // PN runs at 60 Hz for 19-32 connected IMUs, 120 Hz for 18 or less.
    // Modules in between run at 60 Hz, 120 Hz causes instabilities in the rate.
    // Adapt rate by changing parameters in perc_neuron_tf_broadcaster
    // and PercNeuronROSserial
    double fpsPercNeuron = 60;

    double mainCycleTime = 1 / fpsPercNeuron;

    // Initialize PN Interface
    // Interface to PN tf data created by PN tf broadcaster
    struct tfPNInterface tfPNIntf;

    // String arrays with joints in order of the skeleton hierarchy
    // Provided by PN API, needed for calculations of transforms
    tfPNIntf.jointsLeftArm   = {"LeftShoulder",  "LeftArm",  "LeftForeArm",  "LeftHand"};
    tfPNIntf.jointsRightArm  = {"RightShoulder", "RightArm", "RightForeArm", "RightHand"};

    struct tfPNInterface& tfPNIntf_ref = tfPNIntf;

    // Declare interface to REEM-C / publisher for joint states
    struct pubIntf pubIntf;
    pubIntf.jointPubCycleTime = 0.008;       // Cycle time of output joints

    // Set output topic
    // Publishing to robot simulation of REEM-C
    std::string simulationTopic = "/h1/joint_desired_cmd";
    printf("Starting the simulation and publishing on %s" , simulationTopic.c_str());
    pubIntf.jointStatePublisher =
            node.advertise<sensor_msgs::JointState>(simulationTopic, 1000);

    // String with names of REEM-C's arm joints
    std::vector<std::string> armJointNames = {
        "arm_left_1_joint",  "arm_left_2_joint",  "arm_left_3_joint",
        "arm_left_4_joint",  "arm_left_5_joint",  "arm_left_6_joint",  "arm_left_7_joint",
        "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
        "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"};

    // Initialize the joint state message for arm joints
    pubIntf.armJointStateMsg.header.stamp = ros::Time(0);
    pubIntf.armJointStateMsg.header.frame_id = "";
    pubIntf.armJointStateMsg.name.resize(14);
    pubIntf.armJointStateMsg.name = armJointNames;
    pubIntf.armJointStateMsg.velocity = {};
    pubIntf.armJointStateMsg.effort = {};

    struct pubIntf& pubIntf_ref = pubIntf;

    // Starting loop to get tf position from PN and publish it to the robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ros::Time begin = ros::Time::now();
    ros::Duration elapsedTime = begin - begin;
    ros::Rate rate(fpsPercNeuron);
    while(node.ok()){

        printf("\nYou can start moving now.");
        begin = ros::Time::now();

        // Robot moving in simulation according to tf_skeleton
        // Get transforms from base_link to PN's Left/RightFoot
        success = -1;

        while(success == -1){
            success = tfPNIntf_ref.getTransform();
        }
        // publish them to robot
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        elapsedTime = ros::Time::now() - begin;
        loopCounter++;
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
