import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus


# callback functions for send_goals_fn
def active_callback():
    rospy.loginfo("Goal just went active.")

def done_callback(status, result):
    if status == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")
    else:
        rospy.loginfo("Goal failed with status code: " + str(status))

def move_robot_to_goal(x, y, z, qx, qy, qz, qw):
    try:
        rospy.init_node('move_base_client')

        # Creating a SimpleActionClient
        ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for action server to start...")
        ac.wait_for_server()

        # Create a goal to send
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, z)
        goal.target_pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        rospy.loginfo(f"Sending goal location {x,y,z} {qz,qw} to Action Server")
        # Sending goal
        ac.send_goal(goal, done_cb=done_callback, active_cb=active_callback)

        # Wait for action to complete
        ac.wait_for_result()

        # PLogging final status
        rospy.loginfo("Final Status: "+str(ac.get_goal_status_text()))

    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")


# Passing the goals to the function
move_robot_to_goal(2.0, 0.787, 0.0, 0.0, 0.0, 0.18, 1.0)
rospy.loginfo("Moving on to the next goal...")
move_robot_to_goal(-3.92393159866333, -6.811859607696533,0.0,0.0,0.0, 0.25223372436334385,0.9676663414079238)
rospy.loginfo("Moving on to the final goal...")
move_robot_to_goal( 6.50444221496582, 5.1161065101623535,0.0,0.0,0.0, 0.0,1.0)
rospy.loginfo("All Goals reached! Exiting...")
rospy.sleep(2)
rospy.signal_shutdown("Program Terminated!")
# rospy.spin()
