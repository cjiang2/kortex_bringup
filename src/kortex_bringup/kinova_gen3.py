#!/usr/bin/python3

import time
from math import radians, degrees

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from kortex_driver.srv import *
from kortex_driver.msg import *


# #####
# Limits for Gen3 Joints
# In degree

JOINT_LIMIT = {
    0: [radians(-180.0), radians(180.0)],
    1: [radians(-128.9), radians(128.9)],
    2: [radians(-180.0), radians(180.0)],
    3: [radians(-147.8), radians(147.8)],
    4: [radians(-180.0), radians(180.0)],
    5: [radians(-120.3), radians(120.3)],
    6: [radians(-180.0), radians(180.0)],
}

JOINT_NAME_TO_ID = {
    "joint_1": 0,
    "joint_2": 1,
    "joint_3": 2,
    "joint_4": 3,
    "joint_5": 4,
    "joint_6": 5,
    "joint_7": 6,
}

# #####
# Kinova Gen3 Object
# #####


class KinovaGen3(object):
    """Kinova Gen3.
    Interact with robot control.
    """
    def __init__(
        self,
        robot_name: str = "my_gen3",
        ):
        # ####################
        # Connect to Gen3 and setup publishers and subscribers
        # ####################
        try:
            # Gen3 Parameters
            # -----
            self.robot_name = rospy.get_param('~robot_name', robot_name)
            self.dof = rospy.get_param("/{}/kortex_driver/dof".format(self.robot_name))
            self.joint_names = rospy.get_param("/{}/kortex_driver/joint_names".format(self.robot_name))
            self.is_gripper_present = rospy.get_param("/{}/is_gripper_present".format(self.robot_name), True)
            self.HOME_ACTION_IDENTIFIER = 2 # The Home Action is used to home the robot. It cannot be deleted and is always ID #2
            self.JOINT_NAME_TO_ID = JOINT_NAME_TO_ID
            self.JOINT_LIMIT = JOINT_LIMIT


            # Gen3 services
            # -----
            # Action topic subscriber
            self.last_action_notif_type = None
            self.action_topic_sub = rospy.Subscriber("/{}/action_topic".format(self.robot_name), 
                                        ActionNotification, self._action_topic_cb)

            self._init_gen3_services()


            # Gen3 subscribers
            # -----
            # Store robot pose into python
            self.position = None
            self.vel = None
            self.joint_state_sub = rospy.Subscriber("/{}/joint_states".format(self.robot_name), 
                                            JointState, self._joint_state_cb)


            # Gen3 publishers
            # -----
            #self.cartesian_vel_pub = rospy.Publisher("/{}/in/cartesian_velocity".format(self.robot_name), 
            #                            TwistCommand, queue_size=10)
            
            #self.joint_vel_pub = rospy.Publisher("/{}/in/joint_velocity".format(self.robot_name), 
            #                            Base_JointSpeeds, queue_size=10)

            # Further initialization
            self._call_clear_faults()
            self._call_subscribe_to_a_robot_notification()


        except rospy.ServiceException:
            rospy.logerr("Failed to initialize Kinova Gen3, {}!".format(self.robot_name))
            rospy.signal_shutdown("Exiting...")


    def _init_gen3_services(self):
        """Initialize Gen3 services.
        """
        try:
            # Clear Faults
            clear_faults_full_name = '/{}/base/clear_faults'.format(self.robot_name)
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            # For Joint Angles
            # -----
            # Read action
            read_action_full_name = '/{}/base/read_action'.format(self.robot_name)
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            # Execute action
            execute_action_full_name = '/{}/base/execute_action'.format(self.robot_name)
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            # For Joint Velocities
            send_joint_speeds_command_full_name = '/{}/base/send_joint_speeds_command'.format(self.robot_name)
            rospy.wait_for_service(send_joint_speeds_command_full_name)
            self.send_joint_speeds_command = rospy.ServiceProxy(send_joint_speeds_command_full_name, SendJointSpeedsCommand)

            # Set Cartesian Reference frame
            set_cartesian_reference_frame_full_name = '/{}/control_config/set_cartesian_reference_frame'.format(self.robot_name)
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            # Gripper command
            send_gripper_command_full_name = '/{}/base/send_gripper_command'.format(self.robot_name)
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command_srv = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            # Activate publishing of action notification
            activate_publishing_of_action_notification_full_name = '/{}/base/activate_publishing_of_action_topic'.format(self.robot_name)
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/{}/base/get_product_configuration'.format(self.robot_name)
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/{}/base/validate_waypoint_list'.format(self.robot_name)
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

        except rospy.ServiceException:
            rospy.logerr("Failed to initialize Kinova Gen3 services!")

    def _action_topic_cb(self, notif):
        """Monitor kinova action notification.
        """
        self.last_action_notif_type = notif.action_event

    def _joint_state_cb(self, msg):
        """Store joint angles inside the class instance.
        """
        self.position = msg.position[:len(self.joint_names)].astype(np.float32)
        for i in range(len(self.position)):
            self.position[i] = np.clip(self.position[i], JOINT_LIMIT[i][0], JOINT_LIMIT[i][1])
        self.vel = np.array(msg.velocity[:len(self.joint_names)]).astype(np.float32)
        #print(self.position)

    def _call_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def _call_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    # #####
    # Full Arm Movement
    # #####

    def _wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def go_home(self):
        """Send Gen3 to default home position.
        """
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self._wait_for_action_end_or_abort()

    def send_joint_angles(
        self,
        angles: list,
        angular_duration: float = 0.0,
        MAX_ANGULAR_DURATION: float = 30.0,
        ):
        """Move Gen3 to specified joint angles.
        Args:
            angles: list, 7 DOF, in degrees.
            angular_duration: float. Control duration between AngularWaypoint 
                in a trajectory. 0 by default.
            MAX_ANGULAR_DURATION: float. To validate if angles are safe.
        """
        # NOTE: IMPORTANT!
        # Beforehand, make sure joint velocity is 0.0
        # TODO: THIS IS CAUSING SOME WEIRD BUGS
        # [ERROR] [1650665544.432314487]: Attempt to get goal status on an uninitialized ServerGoalHandle or one that has no ActionServer associated with it.
        #self.send_joint_velocities([0, 0, 0, 0, 0, 0, 0])

        # Make sure angles is a numpy array
        if isinstance(angles, list):
            angles = np.array(angles)

        # Clip the degrees by joint_limit
        for i in range(len(angles)):
            angles[i] = np.clip(angles[i], a_min=JOINT_LIMIT[i][0], a_max=JOINT_LIMIT[i][1])
            angles[i] = degrees(angles[i])

        # Initialization
        self.last_action_notif_type = None
        req = ExecuteActionRequest()

        # Angles to send the arm
        # Use AngularWaypoint, per Kinova official example
        angular_waypoint = AngularWaypoint()
        try:
            assert len(angles) == self.dof
        except:
            rospy.logerr("Invalid angles.")
            return False
        for i in range(len(angles)):
            angular_waypoint.angles.append(angles[i])
        #print(angular_waypoint)
        
        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_waypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        trajectory = WaypointList()
        waypoint = Waypoint()
        #print(waypoint)
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angular_waypoint)
        #print(waypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)
        
        # Validate before proceeding
        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        # Kinova's official validation process
        # NOTE: Kinova APIs check if physical joints reach limits
        # Critical for safe robot joint manipulation
        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        #print(error_number, res)

        if (angular_duration == MAX_ANGULAR_DURATION):
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False


        # Send the angles
        #print(trajectory)
        rospy.loginfo("Sending the joint angles...")
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self._wait_for_action_end_or_abort()


    def send_joint_velocities(
        self, 
        vels: list,
        ):
        """Set velocity for each individual joint.
        Use rad/s as inputs, then convert to deg/s to Kinova's likings.
        TODO: Might need to change max_rate to suit control frequency.
        """
        # Make sure angles is a numpy array
        if isinstance(vels, list):
            vels = np.array(vels)

        # Initialization
        req = SendJointSpeedsCommandRequest()

        # Joint Speed to send to the robot
        for i in range(len(vels)):
            vel = vels[i]
            joint_speed = JointSpeed()
            joint_speed.joint_identifier = i
            joint_speed.value = vel
            req.input.joint_speeds.append(joint_speed)

        # Send the velocity
        rospy.loginfo("Sending the joint speed...")

        try:
            self.send_joint_speeds_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendJointSpeedsCommand!")
            return False
        else:
            return True

    # #####
    # Gripper Control
    # #####
    def send_gripper_command(
        self, 
        value: float,
        ):
        try:
            assert self.is_gripper_present == True
        except:
            rospy.logerr("No gripper is present on the arm.")
            return False

        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command_srv(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def __str__(self):
        string = "Kinova Gen3\n"
        string += "  robot_name: {}\n  dof: {}".format(self.robot_name, self.dof)
        return string