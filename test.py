from netbook_battery import netbook_battery
from kobuki_battery import kobuki_battery
import rospy


if __name__ == '__main__':
    rospy.init_node("test")

    mBatteryQuery = netbook_battery()
    print (mBatteryQuery.getLaptopInfo())

    mRobotBattery = kobuki_battery()
    print (mRobotBattery.getRobotBatteryInfo())
