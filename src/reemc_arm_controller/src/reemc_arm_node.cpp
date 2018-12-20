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
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
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
    bool publishToController;                 // True: publishes to robot controller
                                              // False: directly to joint states instead

};


// Define joint/pose vectors and Jacobian (inverse) matrices
enum{ joints = 7 };
typedef Eigen::Matrix<double, 6, 1>           PoseVector;
typedef Eigen::Matrix<double, joints, 1>      JointVector;

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

    // 6x1 vector with position (x,y,z) and rotation (r,p,y) in euler angles
    // Used for IK calculation with the Jacobian method
    PoseVector leftarmPose;
    PoseVector rightarmPose;

    int getTransform(){

        Array transformArrayLeftArm;
        Array transformArrayRightArm;

        // Transforms of both arms
        // Rotation and translation will be needed for the position vector
        tf::Transform transformLeftArm;
        tf::Transform transformRightArm;

        int success = 0;  // = 0 for success of getTransform; < 0 for failure

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Fill arrays with transforms, array with joint names is provided
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
        // Goal transform of arms and the 6-dimensional position vector
        transformLeftArm  = transformArrayLeftArm.at(0);
        transformRightArm = transformArrayRightArm.at(0);
        for (int i = 1; i < transformArrayLeftArm.size(); i++){
            transformLeftArm  = transformLeftArm.operator *=(transformArrayLeftArm.at(i));
            transformRightArm = transformRightArm.operator*=(transformArrayRightArm.at(i));
        }

        // Get EE's rotation in euler angles
        double roll_left; double pitch_left; double yaw_left;
        double roll_right; double pitch_right; double yaw_right;
        transformLeftArm.getBasis().getRPY(roll_left, pitch_left, yaw_left);
        transformRightArm.getBasis().getRPY(roll_right, pitch_right, yaw_right);

        leftarmPose << transformLeftArm.getOrigin().getX(),
                       transformLeftArm.getOrigin().getY(),
                       transformLeftArm.getOrigin().getZ(),
                       roll_left, pitch_left, yaw_left;

        rightarmPose << transformRightArm.getOrigin().getX(),
                        transformRightArm.getOrigin().getY(),
                        transformRightArm.getOrigin().getZ(),
                        roll_right, pitch_right, yaw_right;

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

struct ik{

    typedef Eigen::Matrix<double, 6, joints>      Jacobian;
    typedef Eigen::Matrix<double, joints, joints> Jac_tmp;
    typedef Eigen::Matrix<double, joints, 6>      Jac_pseudoInv;

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Vector with current position and orientation of EE
    // Equals robot's default position when initializing
    PoseVector X_current_left;
    PoseVector X_current_right;

    // Desired goal position of end effectors
    // Get pose vector from tfPNInterface
    PoseVector X_goal_left;
    PoseVector X_goal_right;

    // Vector with current joint states
    // First iteration is using default positions of the robot
    // Get default/last published joint states from publisher
    JointVector q_current_left;
    JointVector q_current_right;

    // Vector with the desired joint states
    // Values will be published to armJointStatemsg
    JointVector q_goal_left;
    JointVector q_goal_right;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Inverse kinematics calculation to get desired joint states
    // Check constraints using the SVD
    // Return 0 for success
    int calcIKJac(tfPNInterface& tfPNIntf, int init,
                  // For first iteration x_current and q_current have to be set
                  // with default position of the robot
                  JointVector& currentJointsLeft, JointVector& currentJointsRight){

        /* Input arguments needed for function:
         * - cycleTime -> 1/fpsPercNeuron
         * - publish interface to publish joint states
         * - tfPNInterface to get transforms for EE = goal position
         * - receive robots default position from controller (or set manually)
         * - last published position (=default position when initializing)
         */

        // Difference between goal position and current/last position of EE
        // Take goal position from tfPNInterface
        PoseVector X_delta_left; PoseVector X_delta_right;

        // Jacobian matrix for robot's current joint states
        // Copy from jac_kdl
        Jacobian J_left; Jacobian J_right;

        // (J.transpose * J).inverse
        Jac_tmp J_tmp_left; Jac_tmp J_tmp_right;

        // Left pseudo-inverse of Jacobian, robot restrictions can be checked by taking
        // the SVD and looking at smallest singular value
        // J_# = (J.T * J).inv * J_T;
        // J_# * J = I (size nxn)
        Jac_pseudoInv J_inv_left; Jac_pseudoInv J_inv_right;
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

        // Create JntArrays, frames and Jacobians
        // Equals q_current (last published/default joint states)
        KDL::JntArray q_kdl_left(chain_left.getNrOfJoints());
        KDL::JntArray q_kdl_right(chain_right.getNrOfJoints());
        KDL::Jacobian jac_kdl_left;
        KDL::Jacobian jac_kdl_right;
        // Create solvers to get the Jacobians based on the kinematic chains of the arms
        // and the current joint states
        ChainJntToJacSolver jac_solver_left  = ChainJntToJacSolver(chain_left);
        ChainJntToJacSolver jac_solver_right = ChainJntToJacSolver(chain_right);

        // Create Frame that will contain the current cartesian position
        KDL::Frame pos_left;
        KDL::Frame pos_right;
        // Create solvers to get cartesian position using FK
        ChainFkSolverPos_recursive fksolver_left =
                ChainFkSolverPos_recursive(chain_left);
        ChainFkSolverPos_recursive fksolver_right =
                ChainFkSolverPos_recursive(chain_right);

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // First start:
        // Setting values for initializing from robot's parking position
        // Set q_current as last published position or get robot's
        // default values for first iteration
        // X_goal is taken from tfPNInterface
        q_current_left = currentJointsLeft;
        q_current_right= currentJointsRight;
        X_goal_left = tfPNIntf.leftarmPose;
        X_goal_right= tfPNIntf.rightarmPose;

        // Copy q_current to q_kdl
        // Use to calculate jac_kdl and X_kdl using FK
        for (int i = 0; i < q_current_left.size(); i++){
            q_kdl_left(i)  = q_current_left(i);
            q_kdl_right(i) = q_current_right(i);
        }

        // Calculate X_current through forward kinematics using q_current/q_kdl
        fksolver_left.JntToCart(q_kdl_left, pos_left);
        fksolver_right.JntToCart(q_kdl_right, pos_right);

        // Copy cartpos to X_current
        // KDL::Frame contains (Rotation, Vector)
        double r_left; double p_left; double y_left;
        double r_right; double p_right; double y_right;

        pos_left.M.GetRPY(r_left, p_left, y_left);
        pos_right.M.GetRPY(r_right, p_right, y_right);

        X_current_left << pos_left.p.x(),
                          pos_left.p.y(),
                          pos_left.p.z(),
                          r_left, p_left, y_left;

        X_current_right << pos_right.p.x(),
                           pos_right.p.y(),
                           pos_right.p.z(),
                           r_left, p_right, y_right;

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Begin IK calculations
        // Calculate the Jacobian matrices
        jac_solver_left .JntToJac(q_kdl_left,  jac_kdl_left);
        jac_solver_right.JntToJac(q_kdl_right, jac_kdl_right);

        // Get X_current and calculate X_delta = X_goal - X_current
        X_delta_left  = X_goal_left  - X_current_left;
        X_delta_right = X_goal_right - X_current_right;

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

        // Determine SVD of Jacobians, check singular values
        // to avoid robot going into a singular configuration
        // (0 < sigma_j < epsilon_threshhold)
        Eigen::JacobiSVD<Jacobian> svd_left(J_left);
        Eigen::JacobiSVD<Jacobian> svd_right(J_right);
        singularValues_left  = svd_left.singularValues();
        singularValues_right = svd_right.singularValues();

        // Todo:
        /* Damped Pseudo Inverse method:
                 J_damped = V * S_rho * U^T, with rho << 1 as damping factor
                 (rho -> 0, damped pseudo-inv => pseudo-inverse)
        */

        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Final joint states (todo: handling EE near singularities):
        // q_goal = q_current + Jac_pseudoInv * X_delta;
        q_goal_left = q_current_left + J_inv_left * X_delta_left;
        q_goal_right = q_current_right + J_inv_right * X_delta_right;

        /*
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // Publish joint states to robot

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

        // Copy q_goal and use it as new q_current for next iteration
        q_current_left  = q_goal_left;
        q_current_right = q_goal_right;
        */
        return 0;
    }
};

// Publish new joint values to robot
int publishJointStates(pubIntf& pubIntf, double cycleTime,
                       JointVector& goalPosLeft, JointVector& goalPosRight,
                       JointVector& lastPosLeft, JointVector& lastPosRight){

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    
    // Publish joint states to REEM-C
    pubIntf.armJointStateMsg.header.stamp = ros::Time(0);
    pubIntf.armJointStateMsg.position = {
        goalPosLeft(0),  goalPosLeft(1),  goalPosLeft(2),  goalPosLeft(3),
        goalPosLeft(4),  goalPosLeft(5),  goalPosLeft(6),
        goalPosRight(0), goalPosRight(1), goalPosRight(2), goalPosRight(3),
        goalPosRight(4), goalPosRight(5), goalPosRight(6)};

        pubIntf.jointStatePublisher.publish(pubIntf.armJointStateMsg);

    ROS_INFO_STREAM("REEM-C arm joint states: " <<
                    goalPosLeft(0)  << " " << goalPosLeft(1)  << " " <<
                    goalPosLeft(2)  << " " << goalPosLeft(3)  << " " <<
                    goalPosLeft(4)  << " " << goalPosLeft(5)  << " " <<
                    goalPosLeft(6)  << " " <<
                    goalPosRight(0) << " " << goalPosRight(1) << " " <<
                    goalPosRight(2) << " " << goalPosRight(3) << " " <<
                    goalPosRight(4) << " " << goalPosRight(5) << " " <<
                    goalPosRight(6) << "\n");

    // Copy last published position to use as new current position for next iteration
    lastPosLeft = goalPosLeft;
    lastPosRight= goalPosRight;
    return 0;
}

int initializeRobot(ros::NodeHandle& node, pubIntf& pubIntf, tfPNInterface& tfPNIntf, ik& ik,
                    JointVector& defaultJointsLeft, JointVector& defaultJointsRight){
    /*
    - Set default values for initializing
    - Calculate ik with default pos
    - Publish them to the robot
    - End initialization loop
    - Start main loop
    */

    int success = -1;
    while (success == -1 && node.ok()){
        //returns 0 for success
        success = tfPNIntf.getTransform();
    }

    // Set the values for robot's parking position
    // Start IK with default values
    int init = true;
    ik.calcIKJac(tfPNIntf, init, defaultJointsLeft, defaultJointsRight);

    // Take calculated goal joint states and publish them to robot

    // Finish initialization process
    return 0;
}

int getDefaultPosition(JointVector& positionLeft, JointVector& positionRight,
                       bool publishToController){
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // In Simulation:
    if(publishToController == false) {
        // Set default parking position in simulation
        // From topic /h1/joint_states
        JointVector defaultLeft;
        JointVector defaultRight;

        defaultLeft  << 0.0, 0.0, 0.0, 1.12496, 0.0, 0.0, 0.0;
        defaultRight << 0.0, 0.0, 0.0, 1.12496, 0.0, 0.0, 0.0;

        positionLeft = defaultLeft;
        positionRight= defaultRight;
        printf("\nStarting in simulation with parking position");
    }

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Retrieve real robot's position
    else{
        std::string robotStateTopic = "/h1/joint_states";
        printf("\nGet real robot's default position from topic: "
               "%s", robotStateTopic.c_str());
        boost::shared_ptr<sensor_msgs::JointState const> defaultPosPtr;
        defaultPosPtr = ros::topic::waitForMessage<sensor_msgs::JointState>
                (robotStateTopic, ros::Duration(1));

        if(defaultPosPtr == NULL){
            printf("\nPosition not published. Start robot controller or set "
                   "publishToController to false and restart in simulation.");
            return -1;
        }
        else{
            // Pass default position over as positionLeft/Right for initializing
            std::vector<double> robotPosition;
            sensor_msgs::JointState robotPositionMsg;
            robotPositionMsg = *defaultPosPtr;
            robotPosition = robotPositionMsg.position;
            for(int i = 0; i < 7; i++){
                positionLeft(i) = robotPosition.at(i);
                positionRight(i) = robotPosition.at(i + 7);
            }


        }
    }      
}

int main(int argc, char** argv){

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Declare interface to REEM-C / publisher for joint states
    struct pubIntf pubIntf;
    struct pubIntf& pubIntf_ref = pubIntf;
    pubIntf.jointPubCycleTime = 0.008;       // Cycle time of output joints


    // String with names of REEM-C's arm joints
    std::vector<std::string> armJointNames = {
        "arm_left_1_joint",  "arm_left_2_joint",  "arm_left_3_joint", "arm_left_4_joint",
        "arm_left_5_joint",  "arm_left_6_joint",  "arm_left_7_joint",
        "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint",
        "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"};
    // Initialize the joint state message for arm joints
    pubIntf.armJointStateMsg.header.stamp = ros::Time(0);
    pubIntf.armJointStateMsg.header.frame_id = "";
    pubIntf.armJointStateMsg.name.resize(14);
    pubIntf.armJointStateMsg.name = armJointNames;
    pubIntf.armJointStateMsg.velocity = {};
    pubIntf.armJointStateMsg.effort = {};
    // Initialize PN Interface
    // Interface to PN tf data created by PN tf broadcaster
    struct tfPNInterface tfPNIntf;
    struct tfPNInterface& tfPNIntf_ref = tfPNIntf;

    // String arrays with joints in order of the skeleton hierarchy
    // Provided by PN API, needed for calculations of transforms
    tfPNIntf.jointsLeftArm   = {"LeftShoulder",  "LeftArm",  "LeftForeArm",  "LeftHand"};
    tfPNIntf.jointsRightArm  = {"RightShoulder", "RightArm", "RightForeArm", "RightHand"};

    // Set output topic
    // Publishing to robot controller
    // .......
    std::string realTopic; // = ?
    // Publishing to robot simulation of REEM-C
    std::string simTopic = "/h1/joint_desired_cmd";
    pubIntf.publishToController = true;

    node_ref.getParam("/reemc_arm_node/publishToController", pubIntf.publishToController);
    if(pubIntf.publishToController){
        printf("\nPublish data to robot controller on topic: %s.", realTopic.c_str());
        pubIntf.jointStatePublisher =
                node.advertise<sensor_msgs::JointState>(realTopic, 1000);
    }
    else{
        printf("\nConfirm to start in simulation and publish on %s. y/n",
               simTopic.c_str());
        char response;
        std::cin >> response;
        if((response == 'Y' || response == 'y')){
            printf("\nStarting the simulation and publishing on %s." , simTopic.c_str());
            pubIntf.jointStatePublisher =
                    node.advertise<sensor_msgs::JointState>(simTopic, 1000);
        }
        else{
            printf("\nTerminating node. Restart and set publishToController to true.");
            ros::shutdown();
        }
    }

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Get robot's default configuration, use it for initialization
    JointVector defaultJointsLeft;
    JointVector defaultJointsRight;
    JointVector& defaultJointsLeft_ref = defaultJointsLeft;
    JointVector& defaultJointsRight_ref = defaultJointsRight;

    success = -1;
    while(success == -1 && node.ok()){
        success = getDefaultPosition(defaultJointsLeft_ref, defaultJointsRight_ref,
                                     pubIntf.publishToController);
        ros::Duration(1).sleep();
    }

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // First initialization of robot
    // Default positions are used for IK calculation with Jacobian
    /*
     * - get tf
     * - start init function
     * - in init function: jacobian (with default positions) and then publish
     * - end init loop
     */

    struct ik  ik;
    struct ik& ik_ref = ik;

    success = -1;
    printf("\nStarting initialization from robot's default position");
    success = initializeRobot(node_ref, pubIntf_ref, tfPNIntf_ref, ik_ref,
                              defaultJointsLeft_ref, defaultJointsRight_ref);

    if(success = -1){
        printf("\nInitialization process has failed");
        ros::shutdown();
    }

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Start main event loop
    // Get tf position from PN
    // Calculate goal position with IK Jacobian
    // Publish to the robot
    ros::Time begin = ros::Time::now();
    ros::Duration elapsedTime = begin - begin;
    ros::Rate rate(fpsPercNeuron);
    while(node.ok()){

        printf("\nYou can start moving now.");
        begin = ros::Time::now();

        // Get transforms from base_link to PN's Left/RightFoot
        success = -1;

        while(success == -1){
            success = tfPNIntf_ref.getTransform();
        }        
        // Calculate goal positions using inverse Jacobian
        // ik.calcIKJac(tfPNIntf_ref....)

        // Publish desired positions to robot
        // Last position is set as new current position for next loop

        elapsedTime = ros::Time::now() - begin;

        success = publishJointStates(pubIntf_ref, mainCycleTime,
                                     ik.q_goal_left, ik.q_goal_right,
                                     ik.q_current_left, ik.q_current_right);

        loopCounter++;
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
