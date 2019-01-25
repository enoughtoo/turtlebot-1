import rospy
from geometry_msgs.msg import Twist
from math import radians

class RegistrationMoveRobot():

    def __init__(self):
        # initiliaze
        rospy.init_node('registration_move_robot', anonymous=False)

        # What to do you ctrl + c
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # 5 HZ
        r = rospy.Rate(5);

        # create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

        # let's go forward at 0.05 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.05
        # by default angular.z is 0 so setting this isn't required

        # let's turn at 45 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(45);  # 45 deg/s in radians/s

        while not rospy.is_shutdown():
            # go forward 0.4 m (8 seconds * 0.05 m / seconds)
            rospy.loginfo("Going Straight")
            for x in range(0, 40):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
            # turn 90 degrees
            rospy.loginfo("Turning")
            for x in range(0, 10):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()

            # go forward 0.3 m (6 seconds * 0.05 m / seconds)
            rospy.loginfo("Going Straight")
            for x in range(0, 30):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            # turn 45 degrees
            rospy.loginfo("Turning")
            for x in range(0, 5):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()

            # go forward 0.4 m (8 seconds * 0.05 m / seconds)
            rospy.loginfo("Going Straight")
            for x in range(0, 40):
                self.cmd_vel.publish(move_cmd)
                r.sleep()


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        RegistrationMoveRobot()
    except Exception, e:
        rospy.loginfo("node terminated because of "+str(e))