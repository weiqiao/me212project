#!/usr/bin/python
import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad, poselist2pose

class ApriltagMeasure():
    def __init__(self, route_idx = 0):
                
        self.lr = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher('/cmdvel',WheelCmdVel,queue_size=1)
        
        self.thread = threading.Thread(target = self.measure)
        self.thread.start()
        rospy.sleep(1)

        ######
        # 443
        #1222
        #1
        #1
        ######
    ## apriltag msg handling function (Need to modify for Task 2)
    def apriltag_callback(self,data):
        # use apriltag pose detection to find where is the robot
        # print data
        for detection in data.detections:
            print 'apriltag id', detection.id
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(self.lr, poselist_tag_cam, sourceFrame = '/camera', targetFrame = '/robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(self.lr, poselist_base_tag,sourceFrame = '/apriltag', targetFrame = '/map')
            
            pubFrame(self.br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')
    
    ## route 0 (turn right early)
    def measure(self):
        target_pose2d = [0.25, 0, np.pi]
        rate = rospy.Rate(100) # 100hz
        
        while not rospy.is_shutdown() :

            # 1. get robot pose
            robot_pose3d = lookupTransform(self.lr, '/map', '/robot_base')
            
            if robot_pose3d is None:
                print '1. Tag not in view'
                rate.sleep()
                continue
            
            robot_position2d  = robot_pose3d[0:2]
            target_position2d = target_pose2d[0:2]
            
            robot_yaw    = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
            robot_pose2d = robot_position2d + [robot_yaw]
            
            pos_delta         = np.array(target_position2d) - np.array(robot_position2d)
            robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
            heading_err_cross = cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
                
            
            #print 'robot_position2d', robot_position2d
            ## print 'target_position2d', target_position2d
            ## print 'pos_delta', pos_delta
            #print 'robot_yaw', robot_yaw
            ## print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d[2])
            ## print 'heading_err_cross', heading_err_cross
                
            '''
            if arrived or (np.linalg.norm( pos_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
                print 'Case 2.1  Stop'
                wcv.desiredWV_R = 0  
                wcv.desiredWV_L = 0
                arrived = True
            elif np.linalg.norm( pos_delta ) < 0.08:
                arrived_position = True
                if diffrad(robot_yaw, target_pose2d[2]) > 0:
                    print 'Case 2.2.1  Turn right slowly'      
                    wcv.desiredWV_R = -0.05 
                    wcv.desiredWV_L = 0.05
                else:
                    print 'Case 2.2.2  Turn left slowly'
                    wcv.desiredWV_R = 0.05  
                    wcv.desiredWV_L = -0.05
            elif arrived_position or np.fabs( heading_err_cross ) < 0.2:
                print 'Case 2.3  Straight forward'  
                wcv.desiredWV_R = 0.1
                wcv.desiredWV_L = 0.1
            else:
                if heading_err_cross < 0:
                    print 'Case 2.4.1  Turn right'
                    wcv.desiredWV_R = -0.1
                    wcv.desiredWV_L = 0.1
                else:
                    print 'Case 2.4.2  Turn left'
                    wcv.desiredWV_R = 0.1
                    wcv.desiredWV_L = -0.1
                    
            self.velcmd_pub.publish(wcv)  
            '''
            rate.sleep()
            


def main():
    rospy.init_node('apriltag_navi',anonymous=True)
    april_navi = ApriltagMeasure()
    rospy.spin()

if __name__=='__main__':
    main()
