/* reemc_leg_node.cpp
*
* Zheng Du, Oct 2018
* Lets the REEM-C legs follow the user's legs.
*
* Takes the TF data, which was produced by the Perception Neuron suit
* and sent to tf by perc_neuron_tf_broadcaster. Calculates the goal positions
* of the REEM-C using the IK in the PAL walking_controller package
* and publishes them to the robot in the simulation.
*
* This software may be modified and distributed under the terms
* of the GNU license.  See the LICENSE file for details.
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
#include <walking_controller/ik/pair_ik_leg_analytic_kajita.h>
#include <walking_controller/reemc_definitions.h>

using namespace std;

// Looks up a tf transfrom and pushes them into an array for calculating
// the goal transform
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

    double jointPubCycleTime;              // Cycle time at which joint states are published
    ros::Publisher jointStatePublisher;    // Publishes desired joint states
    sensor_msgs::JointState jointStateMsg; // Published message with desired values

};

// Interface to tf Data, which is published by PN tf broadcaster
struct tfPNInterface {
    // PN joints. Necessary to listen to them with tfListener
    std::vector<std::string> bodyJointsLeftFoot;
    std::vector<std::string> bodyJointsRightFoot;

    // Tf listener to data from external tf broadcaster
    tf::TransformListener    tfListener;
    // Broadcasts goal transforms for visualization
    tf::TransformBroadcaster tfBroadcaster;

    // Goal transforms, needed for IK calculations
    tf::Transform transLeftFoot;
    tf::Transform transRightFoot;

    int getTransform(){

        std::vector<tf::Transform> transformArrayLeftFoot;
        std::vector<tf::Transform> transformArrayRightFoot;

        int success = 0;  // = 0 for success of getTransform; < 0 for failure
        // Get transform from Hips to Feet
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        /*
         * Set Hip Pos in IK function
         *
        // fill array with transform between base_link and world
        success += pushIntoTransformArray("/world", "/WorldPerceptionNeuron",
                                          &transformArrayBase, &tfListener);
        success += pushIntoTransformArray("/WorldPerceptionNeuron", "/Hips",
                                          &transformArrayBase, &tfListener);
        success += pushIntoTransformArray("/Hips", "/base_link_TX",
                                          &transformArrayBase, &tfListener);
        */
        // Fill arrays with transforms, array with joint names is provided, i = 0 equals Hips
        for (int i = 0; i < bodyJointsLeftFoot.size() - 1; i++){
            // Returns 0 for success, -1 for failure
            success += pushIntoTransformArray(bodyJointsLeftFoot.at(i), bodyJointsLeftFoot.at(i + 1),
                                              &transformArrayLeftFoot, &tfListener);
            success += pushIntoTransformArray(bodyJointsRightFoot.at(i), bodyJointsRightFoot.at(i + 1),
                                              &transformArrayRightFoot, &tfListener);
            if (success != 0) {
                printf("\nNo tf transformations could be found."
                       "\nPlease start rosserial_server node , perc_neuron_tf_broadcaster "
                       "or PN publisher on Windows");
                return -1;
            }
        }

        transLeftFoot  = transformArrayLeftFoot.at(0);
        transRightFoot = transformArrayRightFoot.at(0);
        for (int i = 1; i < transformArrayLeftFoot.size(); i++){
            transLeftFoot  = transLeftFoot.operator *=(transformArrayLeftFoot.at(i));
            transRightFoot = transRightFoot.operator*=(transformArrayRightFoot.at(i));
        }

        transLeftFoot.getOrigin() = transLeftFoot.getOrigin() * 0.80;
        transRightFoot.getOrigin() = transRightFoot.getOrigin() * 0.80;

        /*
         * Adjusting orientation of EEs in the IK function
         */

        // Broadcast transforms
        tfBroadcaster.sendTransform(tf::StampedTransform(transLeftFoot, ros::Time(0),
                                                         "/Hips", "/HipsToLeftFoot"));
        tfBroadcaster.sendTransform(tf::StampedTransform(transRightFoot, ros::Time(0),
                                                         "/Hips", "/HipsToRightFoot"));

        transformArrayLeftFoot.clear();
        transformArrayRightFoot.clear();
        return 0;
    }
};

namespace pal{
    // Values taken from REEM-C manual
    // Definitions copied from walking_controller tutorial from the manual
    const double hip_spacing = (0.145 / 2.0);
    const double upperleg_lenght = 0.300;
    const double lowerleg_lenght = 0.300;
    const double foot_height     = 0.110;

    const eVector3 ankle_to_foot_center_left  (0, 0, -foot_height);
    const eVector3 ankle_to_foot_center_right (0, 0, -foot_height);
    const eVector3 ankle_to_foot_center[2] = {ankle_to_foot_center_left,
                                              ankle_to_foot_center_right};

    const eMatrixHom center_to_left_matrix  =
              initMatrixHom(eMatrixRot::Identity(), eVector3(0, hip_spacing, 0));
    const eMatrixHom center_to_right_matrix =
              initMatrixHom(eMatrixRot::Identity(), eVector3(0, -hip_spacing, 0));
}

struct ik{
    std::vector<double> legIKJointStates;    // Solution vector in joint space

    int calcIKLeg(const tfPNInterface& tfPNIntf){
        // Instantiating an object IK for legs
        // Matrix from base_link to first joint, upper and lower leg length are provided
        LegsPairIKAnalyticKajita legs_kinematics(
                    pal::center_to_left_matrix, pal::center_to_right_matrix,
                    pal::upperleg_lenght, pal::lowerleg_lenght);

        /* set hip pose and EE rotation manually
        eQuaternion hipRPY (tfPNIntf.transBaseLink.getRotation().getX(),
                            tfPNIntf.transBaseLink.getRotation().getY(),
                            tfPNIntf.transBaseLink.getRotation().getZ(),
                            tfPNIntf.transBaseLink.getRotation().getW());
        eVector3 hipPos  (tfPNIntf.transBaseLink.getOrigin().getX(),
                          tfPNIntf.transBaseLink.getOrigin().getY(),
                          tfPNIntf.transBaseLink.getOrigin().getZ());

        // get postition of left foot for ik_analytics

        eQuaternion leftFootRPY (tfPNIntf.transLeftFoot.getRotation().getX(),
                                tfPNIntf.transLeftFoot.getRotation().getY(),
                                tfPNIntf.transLeftFoot.getRotation().getZ(),
                                tfPNIntf.transLeftFoot.getRotation().getW());

        eQuaternion rightFootRPY (tfPNIntf.transRightFoot.getRotation().getX(),
                                 tfPNIntf.transRightFoot.getRotation().getY(),
                                 tfPNIntf.transRightFoot.getRotation().getZ(),
                                 tfPNIntf.transRightFoot.getRotation().getW());
        */
        // CoM / Hip pose
        eVector3 hipRPY (-M_PI/2, 3*M_PI/2, 0.0);
        eVector3 hipPos (0.0, 0.0, 0.0);
        // Setting rotation for the EE so the feet are facing the front with
        // the soles pointing down to the ground
        // Set positions and orientation of EE for ik_analytics
        eVector3 leftFootRPY  (-M_PI/2, -M_PI/2, 0.0);
        eVector3 leftFootPos  (tfPNIntf.transLeftFoot.getOrigin().getX(),
                               tfPNIntf.transLeftFoot.getOrigin().getY(),
                               tfPNIntf.transLeftFoot.getOrigin().getZ());
        eVector3 rightFootRPY  (-M_PI/2, -M_PI/2, 0.0);
        eVector3 rightFootPos  (tfPNIntf.transRightFoot.getOrigin().getX(),
                                tfPNIntf.transRightFoot.getOrigin().getY(),
                                tfPNIntf.transRightFoot.getOrigin().getZ());

        // Init IK with CoM and EE poses
        legs_kinematics.setBaseFrameCoord(createMatrix(hipRPY, hipPos));
        // SetFootCoord inputs:
        // - transform matrix from world to ee of foot
        // - vector from ankle to the foot/(ee)
        // math_utils.h for def of createMatrix
        legs_kinematics.setFootCoord(pal::LEFT, createMatrix(leftFootRPY, leftFootPos),
                                     pal::ankle_to_foot_center[pal::LEFT]);
        legs_kinematics.setFootCoord(pal::RIGHT, createMatrix(rightFootRPY, rightFootPos),
                                     pal::ankle_to_foot_center[pal::RIGHT]);

        // Inverse kinematics calculation: ik_analytic (see walking_controller/IK_leg.h)
        // SetHipCoord and setFootCoord have to be called first
        // Store computed joint states in solution vector
        // Return 1 for success, -1 otherwise
        legs_kinematics.ik_analytic(pal::LEFT,  &legIKJointStates[0]);
        legs_kinematics.ik_analytic(pal::RIGHT, &legIKJointStates[6]);
        return 0;
    }
};

void printVector(std::vector<double>& vector, bool debugStream) {
  if (debugStream) {
    ROS_DEBUG("\n");
    for (auto& currentElement : vector) {
      ROS_DEBUG("%f ", currentElement * 180 / M_PI);
    }
  }
  else {
    printf("\n");
    for (auto& currentElement : vector) {
      printf("%f ", currentElement * 180 / M_PI);
    }
  }
}

int publishJointStates(pubIntf& pubIntf, double cycleTime,
                       std::vector<double>& goalPos,
                       std::vector<double>& lastPublishedPos){

    // Initialize variables
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    std::vector<double> publishPos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> deltaPos = {2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI,
                                    2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI,
                                    2 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI};

    // Round down to not publish faster thann 125 Hz
    int numberPublishes = floor(cycleTime / pubIntf.jointPubCycleTime);

    if (numberPublishes == 0){ numberPublishes == 1; }    // Publish at least once

    // DeltaPos is the difference from current to next goal position
    for (int i = 0; i < goalPos.size(); i++) {
      deltaPos.at(i) = goalPos.at(i) - lastPublishedPos.at(i);
    }
    // Interpolate and publish positions
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ros::Rate rate(1 / pubIntf.jointPubCycleTime);
    for (int step = 1; step <= numberPublishes; step++){
        // deltaposition -> publish = last + step/numberpub * deltaposition
        for (int i = 0; i < deltaPos.size(); i++){
            publishPos.at(i) = lastPublishedPos.at(i) +
                               ((float)step / (float)numberPublishes) * deltaPos.at(i);
        }
        // Send joint states to REEM-C
        pubIntf.jointStateMsg.header.stamp = ros::Time(0);
        pubIntf.jointStateMsg.position = { publishPos.at(0), publishPos.at(1),  publishPos.at(2),
                                           publishPos.at(3), publishPos.at(4),  publishPos.at(5),
                                           publishPos.at(6), publishPos.at(7),  publishPos.at(8),
                                           publishPos.at(9), publishPos.at(10), publishPos.at(11)};
        pubIntf.jointStatePublisher.publish(pubIntf.jointStateMsg);

        ROS_INFO_STREAM("REEM-C leg joint states: " <<
                        publishPos.at(0) << " " << publishPos.at(1) << " " <<
                        publishPos.at(2) << " " << publishPos.at(3) << " " <<
                        publishPos.at(4) << " " << publishPos.at(5) << " " <<
                        publishPos.at(6) << " " << publishPos.at(7) << " " <<
                        publishPos.at(8) << " " << publishPos.at(9) << " " <<
                        publishPos.at(10)<< " " << publishPos.at(11)<< "\n");

        rate.sleep();
    }
    // No feedback from robot, copying last published position to use as new current position
    std::copy_n(publishPos.begin(), publishPos.size(),
                lastPublishedPos.begin());
    return 0;
}

int initializeRobot(ros::NodeHandle& node, pubIntf& pubIntf,
                    tfPNInterface& tfPNIntf, ik& ik,
                    std::vector<double>& lastPublishedPos,
                    std::vector<double>& defaultPosition){

    printf("\nDriving robot from default position to the current position.");
    // Get tf transform from base_link of robot to PN
    int success = -1;
    while (success == -1 && node.ok()){
        printf("\nGetting transform from robot to PN.");
        success = tfPNIntf.getTransform();
    }
    // Get goal positions ik.legIKJointStates by inverse kinematics
    printf("\nGetting goal joint states through inverse kinematics");
    ik.calcIKLeg(tfPNIntf);

    // Set robot's current and goal position
    // Initialize with default position
    lastPublishedPos = defaultPosition;
    std::vector<double>  currentGoal    = defaultPosition;
    std::vector<double>& currentGoalRef = currentGoal;
    if (node.ok()){
        int success = -1;
        success = publishJointStates(pubIntf, pubIntf.jointPubCycleTime,
                                     currentGoalRef, lastPublishedPos);
    }
    else {
        printf("\nAborted initialization process. ");
        ros::shutdown();
    }
    return 0;
}

int main(int argc, char** argv){
    // Declare general variables
    ros::init(argc, argv, "reemc_leg_node");
    ros::NodeHandle  node;
    ros::NodeHandle& nodeRef = node;
    int loopCounter = 0;
    int success;                    // check for successful execution of functions

    // PN runs at 60 Hz for 19-32 connected IMUs, 120 Hz for 18 or less.
    // Modules in between run at 60 Hz, 120 Hz causes instabilities in the rate.
    // Adapt rate by changing parameters in perc_neuron_tf_broadcaster & PercNeuronROSserial
    double fpsPercNeuron = 60;

    double mainCycleTime = 1 / fpsPercNeuron;

    // Initialize PN Interface
    // Interface to PN tf data created by PN tf broadcaster
    struct tfPNInterface tfPNIntf;

    // String arrays with joints in order of the skeleton hierarchy
    // Provided by PN API, needed for calculations of transforms
    tfPNIntf.bodyJointsLeftFoot  = { "Hips", "LeftUpLeg",  "LeftLeg",  "LeftFoot" };
    tfPNIntf.bodyJointsRightFoot = { "Hips", "RightUpLeg", "RightLeg", "RightFoot" };


    // Transform from base_link to LeftFoot
    tfPNIntf.transLeftFoot = tf::Transform(tf::Quaternion(0, 0, 0, 1),
                                               tf::Vector3(0,0,0));
    tfPNIntf.tfBroadcaster.sendTransform(
                tf::StampedTransform(tfPNIntf.transLeftFoot, ros::Time(0),
                                     "/base_link_TX", "/BaseLinkToLeftFoot"));
    // Transform from base_link to RightFoot
    tfPNIntf.transRightFoot = tf::Transform(tf::Quaternion(0,0,0,1),
                                                    tf::Vector3(0,0,0));
    tfPNIntf.tfBroadcaster.sendTransform(
                tf::StampedTransform(tfPNIntf.transRightFoot, ros::Time(0),
                                     "/base_link_TX", "/BaseLinkToRightFoot"));


    struct tfPNInterface& tfPNIntfRef = tfPNIntf;

    // Declare interface to REEM-C / publisher for joint states
    struct pubIntf pubIntf;
    pubIntf.jointPubCycleTime = 0.008;       // Cycle time of output joints

    // Set output topic
    // Publishing to robot simulation of REEM-C
    std::string simulationTopic = "/h1/joint_desired_cmd";
    printf("Starting the simulation and publishing on %s" ,
           // "Confirm y or n.\n",
           simulationTopic.c_str());
    //char response;
    //cin >> response;
    //printf("\nResponse: %c", response);
    //if ((response == 'Y' || response == 'y')){
        pubIntf.jointStatePublisher = node.advertise<sensor_msgs::JointState>(
                                      simulationTopic, 1000);
    //}
    //else {
    //    printf("\nTerminating node.");
    //    ros::shutdown();
    //}
    // Initialize joint state message
    std::vector<std::string> jointNames = {
        "leg_left_1_joint",   "leg_left_2_joint",   "leg_left_3_joint",
        "leg_left_4_joint",   "leg_left_5_joint",   "leg_left_6_joint",
        "leg_right_1_joint",  "leg_right_2_joint",  "leg_right_3_joint",
        "leg_right_4_joint",  "leg_right_5_joint",  "leg_right_6_joint"};
    pubIntf.jointStateMsg.header.stamp = ros::Time(0);
    pubIntf.jointStateMsg.header.frame_id = "";
    pubIntf.jointStateMsg.name.resize(12);
    pubIntf.jointStateMsg.name = jointNames;
    pubIntf.jointStateMsg.velocity = {};
    pubIntf.jointStateMsg.effort = {};

    struct pubIntf& pubIntfRef = pubIntf;

    // Set the default parking position of the robot in simulation
    std::vector<double> defaultPosition = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double>& defaultPositionRef = defaultPosition;

    // Last published position
    std::vector<double> lastPublishedPos    = defaultPositionRef;
    std::vector<double> lastPublishedPosRef = lastPublishedPos;
    // Initialize variables for IK calculations
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    struct ik ik;
    ik.legIKJointStates = defaultPositionRef;

    struct ik& ikRef = ik;

    // Drive robot from default position to PN default position
    printf("\nInitialize robot for the first time.");
    printf("\nDefault position: ");
    printVector(defaultPositionRef, 0);
    success = -1;
    printf("\nStarting initialization\n");

    success = initializeRobot(nodeRef, pubIntfRef, tfPNIntfRef, ikRef,
                              lastPublishedPosRef, defaultPositionRef);

    if(success == -1){
        printf("\nInitialization failed. Shutdown ros\n");
        ros::shutdown();
    }

    // Starting loop to get tf position from PN and publish it to the robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    printf("\nYou can start moving now.");
    ros::Time begin = ros::Time::now();
    ros::Duration elapsedTime = begin - begin;
    ros::Rate rate(fpsPercNeuron);
    while(node.ok()){
        begin = ros::Time::now();

        // Robot moving in simulation according to tf_skeleton
        // Get transforms from base_link to PN's Left/RightFoot
        success = -1;

        while(success == -1){
            success = tfPNIntfRef.getTransform();
        }
        // Get joint positions by inverse kinematics
        // Sets ik.legIKJointStates
        ik.calcIKLeg(tfPNIntfRef);
        // Interpolate points from rate fpsPN to max 125Hz = 1 frame / 8 ms and
        // publish them to robot
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        elapsedTime = ros::Time::now() - begin;

        success = publishJointStates(pubIntfRef, mainCycleTime,
                                     ik.legIKJointStates,
                                     lastPublishedPosRef);
        loopCounter++;
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
