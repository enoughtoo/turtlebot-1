from netbook_battery import netbook_battery
from kobuki_battery import kobuki_battery
import rospy
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import actionlib
from actionlib_msgs.msg import GoalStatus
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState  #base power and auto docking
from visit_spots import get_spot, get_num_spots
from go_to_specific_point_on_map import GoToPose
import sys
import random
import datetime

class OfficePatroller():

    charging_at_dock_station = False
    mBatteryQuery = netbook_battery()
    mRobotBattery = kobuki_battery()
    move_base = False  # _init_ converts this to a MoveBaseAction that is used to set the goals
    near_docking_station_x = -1.88  # x coordinate for pose approx 1 meter from docking station
    near_docking_station_y = 0.06   # y coordinate for pose approx 1 meter from docking station
    num_spots = get_num_spots()
    current_time = datetime.datetime.now()

    def __init__(self):

        # initialize
        rospy.init_node('office_patroller', anonymous=False)

        # What to do you ctrl + c
        rospy.on_shutdown(self.shutdown)

        while self.current_time.weekday() > 5 or 0 < self.current_time.time().hour < 6:
            self.current_time = datetime.datetime.now()
            if self.checkChargingStatus():
                while not self.getBatteryPercentages():
                    time.sleep(600)
                    rospy.loginfo("wait for the battery being charged.")
                self.GoCloseToTheChargingStation()
                self.patrol()
                self.DockWithChargingStation()
            else:
                self.DockWithChargingStation()

    def patrol(self):
        if not self.num_spots:
            return

        navigator = GoToPose()
        attempts = 0
        for i in xrange(self.num_spots):
            curr_spot = get_spot(i)
            name = curr_spot['filename']
            success = navigator.goto(curr_spot['position'], curr_spot['quaternion'])
            while not success and attempts < 10:
                rospy.loginfo("%i attempt failed to reach %s pose", attempts, name[:-4])
                noised_pos = curr_spot['position']
                noised_pos['x'] += random.choice(sys.float_info.epsilon, -sys.float_info.epsilon)
                noised_pos['y'] += random.choice(sys.float_info.epsilon, -sys.float_info.epsilon)
                success = navigator.goto(noised_pos, curr_spot['quaternion'])
                attempts += 1

            if not success:
                return

    def checkChargingStatus(self):
        # if it is already at dock station, no need to check again
        if self.charging_at_dock_station:
            return True

        # detects the docking status can take up to seconds
        time.sleep(15)
        percentage, status = self.mRobotBattery.getRobotBatteryInfo()
        self.charging_at_dock_station = status
        return self.charging_at_dock_station

    def getBatteryPercentages(self):
        robot_percentage, status = self.mRobotBattery.getRobotBatteryInfo()
        laptop_percentage, status = self.mBatteryQuery.getLaptopInfo()
        if robot_percentage > 80 and laptop_percentage > 80:
            return True
        else:
            return False

    def DockWithChargingStation(self):

        # before we can run auto-docking we need to be close to the docking station..
        if (not self.GoCloseToTheChargingStation()):
            return False

        # We're close to the docking station... so let's dock
        return self.WereCloseDock()

    def GoCloseToTheChargingStation(self):

        # the auto docking script works well as long as you are roughly 1 meter from the docking station.
        # So let's get close first...
        rospy.loginfo("Let's go near the docking station")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # set a Pose near the docking station
        goal.target_pose.pose = Pose(
            Point(float(self.near_docking_station_x), float(self.near_docking_station_y), float(0)),
            Quaternion(float(0), float(0), float(0.892), float(-1.5)))

        # start moving
        self.move_base.send_goal(goal)

        # allow TurtleBot up to 60 seconds to get close to
        success = self.move_base.wait_for_result(rospy.Duration(60))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose near the charging station")
            return False
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, reached the desired pose near the charging station")
                return True

    def WereCloseDock(self):

        # The following will start the AutoDockingAction which will automatically find and dock TurtleBot
        # with the docking station as long as it's near the docking station when started
        self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
        rospy.loginfo("waiting for auto_docking server")
        self._client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo(
            "Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
        self._client.send_goal(goal)

        # Give the auto docking script 180 seconds.  It can take a while if it retries.
        success = self._client.wait_for_result(rospy.Duration(180))

        if success:
            rospy.loginfo("Auto_docking succeeded")
            # The callback which detects the docking status can take up to 3 seconds to update which was causing bot
            # to try and redock (presuming it failed) even when the dock was successful.
            # Therefore hardcoding this value after success.
            self.charging_at_dock_station = True
            return True
        else:
            rospy.loginfo("Auto_docking failed")
            return False

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop our patroller")


if __name__ == '__main__':
    try:
        OfficePatroller()
    except Exception, e:
        rospy.loginfo("node terminated because of "+str(e))


    # mBatteryQuery = netbook_battery()
    # print (mBatteryQuery.getLaptopInfo())
    #
    # mRobotBattery = kobuki_battery()
    # print (mRobotBattery.getRobotBatteryInfo())