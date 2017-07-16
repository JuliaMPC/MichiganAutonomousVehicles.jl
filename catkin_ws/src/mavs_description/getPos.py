#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo')

import sys

import rospy
from gazebo.srv import *

def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [model_name] [relative_entity_name]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        model_name = sys.argv[1]
        relative_entity_name = sys.argv[2]
    else:
        print usage()
        sys.exit(1)
    res = gms_client(model_name,relative_entity_name)
    print "returnd x position ",res.pose.position.x
