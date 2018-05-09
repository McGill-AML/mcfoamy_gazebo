#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Wrench, Pose, Twist
from std_srvs.srv import Trigger, TriggerResponse

from gazebo_example.controllers import PID, State
        
class ContollerNode(object):
    MAX_PUB_QUEUE = 10
  
    def __init__(self):
        self._start = [False]
        self._pid = None
        self._pose = Pose()
        self._twist = Twist()
        
    def init(self):
        self._wrench_pub = rospy.Publisher("external_wrench", 
                                           Wrench, 
                                           queue_size=self.MAX_PUB_QUEUE)
        self._pose_sub = rospy.Subscriber("pose", Pose, self._pose_callback)
        self._twist_sub = rospy.Subscriber("twist", Twist, self._twist_callback)
        self._start_service = rospy.Service('start_controller', Trigger, self._start_controller)
        self._pid = PID(5.0, 5.0, 0.0)
        
        return True
      
    def run(self):
        self._wait_for_trigger()
        
        frequency = 10
        rate = rospy.Rate(frequency) # 10hz
        
        while not rospy.is_shutdown():
            # ros::spinOnce not needed in Python
            
            self._wrench_pub.publish(self._compute_control_wrench(frequency))
            rate.sleep()
            
    def _wait_for_trigger(self):
      while not rospy.is_shutdown():
          if(self._start[0]):
              break
          rospy.loginfo_throttle(10, "Waiting for control start trigger")
          
    def _compute_control_wrench(self, frequency):
        desired_state = State(7.0, 0.0)
        measured_state = State(self._pose.position.z, self._twist.linear.z)
        force_z = self._pid.output(desired_state, measured_state, 1.0/frequency)
        
        command_wrench = Wrench()
        command_wrench.force.z = force_z
        
        return command_wrench
        
    def _pose_callback(self, data):
        self._pose = data
        
    def _twist_callback(self, data):
        self._twist = data
        
    def _start_controller(self, req):
        self._start[0] = True
        
        response = TriggerResponse()
        response.success = True
        return response

if __name__ == '__main__':
    try:
        rospy.init_node('controller_node_py', anonymous=True)
        node = ContollerNode()
        if not node.init():
            raise rospy.ROSInterruptException

        node.run()
        
    except rospy.ROSInterruptException:
        pass
