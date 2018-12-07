from netbook_battery import netbook_battery
from kobuki_battery import kobuki_battery
import rospy
import time


class OfficePatroller():

    mChargingAtDockStation = False
    mBatteryQuery = netbook_battery()
    mRobotBattery = kobuki_battery()


    def __init__(self):

        # initialize
        rospy.init_node('office_patroller', anonymous=False)

        # What to do you ctrl + c
        rospy.on_shutdown(self.shutdown)

    def checkChargingStatus(self):
        # if it is already at dock station, no need to check again
        if self.mChargingAtDockStation:
            return True

        # detects the docking status can take up to seconds
        time.sleep(15)
        percentage, status = self.mRobotBattery.getRobotBatteryInfo()
        self.mChargingAtDockStation = status
        return self.mChargingAtDockStation

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop our patroller")

if __name__ == '__main__':
    try:
        OfficePatroller()
    except:
        rospy.loginfo("node terminated.")


    # mBatteryQuery = netbook_battery()
    # print (mBatteryQuery.getLaptopInfo())
    #
    # mRobotBattery = kobuki_battery()
    # print (mRobotBattery.getRobotBatteryInfo())