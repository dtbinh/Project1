#!/usr/bin/env python


import rospy

from vrep_common.srv import simRosStartSimulation, simRosStopSimulation

def mainLoop():
	# ---------------------------------------	
	rospy.init_node('controller', anonymous=True)	
	print("Stop Vrep :")
	rospy.wait_for_service('/vrep/simRosStopSimulation')
	try:
		endVrep = rospy.ServiceProxy('/vrep/simRosStopSimulation',simRosStopSimulation)
		resp1 = endVrep()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	rospy.sleep(2.)

	print("Launch Vrep :")
	rospy.wait_for_service('/vrep/simRosStartSimulation')
	try:
		launchVrep = rospy.ServiceProxy('/vrep/simRosStartSimulation', simRosStartSimulation)
		resp1 = launchVrep()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	rospy.sleep(3.)


# ---------------------------------------------
# ---------------------------------------------
if __name__ == '__main__':
	mainLoop()
