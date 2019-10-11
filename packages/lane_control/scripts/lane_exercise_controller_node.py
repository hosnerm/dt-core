#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState, BoolStamped, VehiclePose
from duckietown_msgs.srv import SetFSMState
import os, imp, time
import sys

################# NOTE TO ALL TEACHING ASSISTANTS!!! ###################
# IF you need to customize the behavior of a part exercise outside     #
# the exercise itself like for example in HWExercise 1-3 (saturation)  #
# or HWExercise 1-4 (different sampling rate) then please edit this in #
# the marked regions "CUSTOMIZATION". This will ensure a "clear"       #
# structure. Just orient yourself at the examples of HWExercise 1.     #
########################################################################


class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        # self.sub_fsm_mode = rospy.Subscriber("~switch", BoolStamped, self.cbMode, queue_size=1)
        self.sub_stop_line = rospy.Subscriber("~at_stop_line", BoolStamped, self.cbStopLine, queue_size=1)

        # Services
        self.srv_fsm_mode = rospy.ServiceProxy('~fsm_mode', SetFSMState)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        duckietown_root = os.environ['DUCKIETOWN_ROOT'] #assumes they sourced environment.sh
        # Load files for HW-Exercises

        exercise_name = rospy.get_param("~exercise_name")
        exercise = exercise_name.split("-")

        self.exercise = exercise

        ex_path = "/CSII/Exercises/HWExercise" + str(exercise[0]) + "/controller-" + str(exercise[1]) + ".py"
        template_src = imp.load_source('module.name', duckietown_root + ex_path)
        self.controller_class = template_src.Controller()

        # HACK: Add listener for FSM machine in order to avoid integrating if not in autopilot mode
        veh_name = os.environ['VEHICLE_NAME']
        self.sub_fsm_mode = rospy.Subscriber("/" + str(veh_name) + "/fsm_node/mode", FSMState, self.cbMode, queue_size=1)

        # Customizations for different exercises. These are defined at top level rosparam "/"
        # self.sampling_factor = rospy.get_param("~sampling_factor")
        # self.time_delay = rospy.get_param("~time_delay")
        # self.omega_sat = rospy.get_param("~omega_sat")

        # if not isinstance(self.time_delay, int):
        #     print "Time delay must be an INT"
        #     self.time_delay = int(self.time_delay)


        # Set up variable which measures how long it took since last command
        self.last_ms = None

        # Set up operating variable
        self.operating = False

        # Setup variable for different sampling rate
        self.z_samp = 0

        # Timer for abrupt stopVeh
        self.stopTimer = None

        # Setup array for time delay
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 5:
            k_d = self.controller_class.k_d
            if k_d > 0:
                self.arr_delay = [[0, 0, 0, 0, 0, 0, 0]] # d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last
                for i in range(1, k_d):
                    self.arr_delay.append([0, 0, 0, 0, 0, 0, 0])

        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
        rospy.loginfo("\n\n\n\n\nREADY FOR EXERCISE " + exercise_name + "\n\n\n\n\n")

        # Setup subscriptions for HWExercise 3
        if int(self.exercise[0]) == 3 and str(self.exercise[1]) != "reference":
            self.sub_veh_pos = rospy.Subscriber("~veh_pos", VehiclePose, self.cbVehPose, queue_size=1)


    def stopVeh(self, nth):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

    def cbVehPose(self, pose_msg):
        if self.stopTimer is not None:
            self.stopTimer.shutdown()
        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - pose_msg.header.stamp

        # delay from taking the image until now in seconds
        t_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        # Calculate time since last command
        currentMillis = int(round(time.time() * 1000))
        if self.last_ms is not None:
            dt_last = (currentMillis - self.last_ms) / 1000.0
        else:
            dt_last = 0 # None before, let's make 0 such that it is way simpler for students

        # Return if not in autopilot
        if not self.operating:
            self.last_ms = currentMillis
            return

        # Obtain parameters for controller
        rho = pose_msg.rho.data
        theta = pose_msg.theta.data
        psi = pose_msg.psi.data


        # Obtain new v and omega from controller
        v_out, omega_out = self.controller_class.getControlOutput(rho, theta, psi, t_delay, dt_last)


        # Print out infos
        rospy.loginfo("Omega: " + str(omega_out) + "    V: " + str(v_out) + "    Delay: " + str(t_delay) + "    T: " + str(dt_last) )

        # Create message and publish
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header
        car_control_msg.v = v_out
        car_control_msg.omega = omega_out
        self.publishCmd(car_control_msg)

        # Update last timestamp
        self.last_ms = currentMillis

        self.stopTimer = rospy.Timer(rospy.Duration(1), self.stopVeh, oneshot=True)

    def cbStopLine(self, msg):
        if msg.data and self.measure_time:
            tracktime = self.getCurrentMillis() - self.start_time
            rospy.loginfo("\n\n\n\n\nTRACK TOOK " + str(tracktime/1000.0) + " seconds" + "\n\n\n\n\n")
            self.measure_time = False
            self.operating = False
            # Send stop command
            car_control_msg = Twist2DStamped()
            car_control_msg.v = 0.0
            car_control_msg.omega = 0.0
            self.publishCmd(car_control_msg)
            self.srv_fsm_mode("JOYSTICK_CONTROL")


    def cbPose(self, lane_pose_msg):
        if int(self.exercise[0]) == 3 and str(self.exercise[1]) != "reference":
            return


        self.z_samp += 1

        lane_reading = lane_pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - lane_reading.header.stamp

        # delay from taking the image until now in seconds
        t_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        # Calculate time since last command
        currentMillis = int(round(time.time() * 1000))
        if self.last_ms is not None:
            dt_last = (currentMillis - self.last_ms) / 1000.0
        else:
            dt_last = 0 # None before, let's make 0 such that it is way simpler for students

        # Return if not in autopilot
        if not self.operating:
            self.last_ms = currentMillis
            return

        # Obtaining parameters to give to controller_class
        d_est = lane_pose_msg.d
        phi_est = lane_pose_msg.phi
        d_ref = 0
        phi_ref = 0
        v_ref = 0.22



        ########## SUBEXERCISE CUSTOMIZATION BEFORE CONTROLLER ##########

        # SAMPLING RATE ADJUSTMENT IN EXERCISE 1-4
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 4:
            k_s = self.controller_class.k_s
            if k_s != 0:
                if self.z_samp % k_s != 0:
                    return
            else:
                dt_last *= k_s # HACK: supposing constant sampling rate. Approx. true for our purposes.
                self.z_samp = 0

        # TIME DELAY IN EXERCISE 1-5
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 5:
            k_d = self.controller_class.k_d
            if k_d > 0:
                self.arr_delay.append([d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last])
                d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last = self.arr_delay.pop(0)
                t_delay += k_d * dt_last # HACK: same as in ex 1-4

        ########## END SUBEXERCISE CUSTOMIZATION BEFORE CONTROLLER ##########

        # Obtain new v and omega
        v_out, omega_out = self.controller_class.getControlOutput(d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last)


        ########## SUBEXERCISE CUSTOMIZATION AFTER CONTROLLER ##########

        # SATURATION IN EXERCISE 1-3
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 3:
            omega_max = self.controller_class.u_sat
            if omega_out > omega_max:
                omega_out = omega_max
            if omega_out < -omega_max:
                omega_out = -omega_max



        ########## END SUBEXERCISE CUSTOMIZATION AFTER CONTROLLER ##########


        # Print out infos
        rospy.loginfo("Omega: " + str(omega_out) + "    V: " + str(v_out) + "    Err: " + str(d_est - d_ref))

        # Create message and publish
        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = v_out
        car_control_msg.omega = omega_out
        self.publishCmd(car_control_msg)

        # Update last timestamp
        self.last_ms = currentMillis


    # FSM
    def cbMode(self,fsm_state_msg):
        self.operating = fsm_state_msg.state == "LANE_FOLLOWING"

    def getCurrentMillis(self):
        return int(round(time.time() * 1000))


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


if __name__ == "__main__":
    rospy.init_node("lane_exercise_controller_node",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
