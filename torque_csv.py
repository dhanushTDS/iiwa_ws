import rospy
import csv
import sys
from geometry_msgs.msg import WrenchStamped
from iiwa_msgs.msg import CartesianPose
from std_msgs.msg import Bool
import matplotlib.pyplot as plt
import tf.transformations as tf

SamplingRate = 1

class RosToCsv:
    def __init__(self, wrench_topic, pose_topic, csv_file):
        self.csv_file = csv_file
        self.wrench_values = []
        self.pose_values = []
        self.times = []
        self.fx_values = []
        self.fy_values = []
        self.fz_values = []
        self.tx_values = []
        self.ty_values = []
        self.tz_values = []
        self.x_values = []
        self.y_values = []
        self.z_values = []
        self.a_values = []
        self.b_values = []
        self.c_values = []
        self.is_recording = False

        # Subscribe to the ROS topics
        rospy.init_node('ros_to_csv', anonymous=True)
        rospy.Subscriber(wrench_topic, WrenchStamped, self.wrench_callback)
        rospy.Subscriber(pose_topic, CartesianPose, self.pose_callback)
        rospy.Subscriber("/toggle_recording", Bool, self.toggle_recording_callback)

        self.rate = rospy.Rate(SamplingRate)
        with open(self.csv_file, "w+") as file:
            csvwriter = csv.writer(file)
            csvwriter.writerow(["Time Stamp", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "X", "Y", "Z", "A", "B", "C"])

    def wrench_callback(self, wrench_data):
        if not self.is_recording:
            return

        fx = wrench_data.wrench.force.x
        fy = wrench_data.wrench.force.y
        fz = wrench_data.wrench.force.z
        tx = wrench_data.wrench.torque.x
        ty = wrench_data.wrench.torque.y
        tz = wrench_data.wrench.torque.z

        self.wrench_values.append((fx, fy, fz, tx, ty, tz))

    def pose_callback(self, pose_data):
        if not self.is_recording:
            return

        x = pose_data.poseStamped.pose.position.x
        y = pose_data.poseStamped.pose.position.y
        z = pose_data.poseStamped.pose.position.z
        quaternion = (
            pose_data.poseStamped.pose.orientation.x,
            pose_data.poseStamped.pose.orientation.y,
            pose_data.poseStamped.pose.orientation.z,
            pose_data.poseStamped.pose.orientation.w
        )
        euler = tf.euler_from_quaternion(quaternion)
        a, b, c = euler[0], euler[1], euler[2]

        self.pose_values.append((x, y, z, a, b, c))

    def toggle_recording_callback(self, data):
        self.is_recording = data.data
        if self.is_recording:
            print("Recording started")
        else:
            print("Recording stopped")

    def save_data(self):
        while not rospy.is_shutdown():
            if self.is_recording:
                for wrench_value, pose_value in zip(self.wrench_values, self.pose_values):
                    time = rospy.Time.now().to_sec()
                    fx, fy, fz, tx, ty, tz = wrench_value
                    x, y, z, a, b, c = pose_value

                    with open(self.csv_file, "a") as file:
                        csvwriter = csv.writer(file)
                        csvwriter.writerow([time, fx, fy, fz, tx, ty, tz, x, y, z, a, b, c])

                self.wrench_values = []
                self.pose_values = []

            self.rate.sleep()



if __name__ == '__main__':
    wrench_topic = "/cam/iiwa_pink/state/EndEffectorWrench"
    pose_topic = "/iiwa_pink/state/CartesianPose"

    trial = "rotation_1deg_3"



    csv_file = "/home/spaceassembly/iiwa_ws/src/iiwa_package-main/iiwa_cam/src/utilities/data_collection/"+trial+"/"+trial+".csv"

    ros_to_csv = RosToCsv(wrench_topic, pose_topic, csv_file)
    ros_to_csv.save_data()


