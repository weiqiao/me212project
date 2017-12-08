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

gripperOpen = 120
girpperClose = 110

class ApriltagNavigator():
    def __init__(self, route_idx = 0):
        
        self.detection_id = -1
        self.idx = 5
        
        self.lr = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher('/cmdvel',WheelCmdVel,queue_size=1)
        self.idx6cnt=0
        self.stage9 = 0
        self.bottle = 2

        
        self.thread = threading.Thread(target = self.navi_loop)
        self.thread.start()
        
        rospy.sleep(1)

    def apriltag_callback(self,data):
        # use apriltag pose detection to find where is the robot
        # print data
        for detection in data.detections:
            if (self.idx == 2 and detection.id == 4) \
                or (self.idx == 3 and (detection.id == 12 or detection.id == 14 or detection.id == 16)) \
                or (self.idx == 4 and detection.id == 3) \
                or (self.idx == 5 and detection.id == 3) \
                or (self.idx == 9 and detection.id == 4) \
                or (self.idx == 10 and detection.id == 4):   # tag id is the correct one
                self.detection_id = detection.id
                poselist_tag_cam = pose2poselist(detection.pose)
                poselist_tag_base = transformPose(self.lr, poselist_tag_cam, sourceFrame = '/camera', targetFrame = '/robot_base')
                poselist_base_tag = invPoselist(poselist_tag_base)
                poselist_base_map = transformPose(self.lr, poselist_base_tag,sourceFrame = '/apriltag', targetFrame = '/map')
                
                pubFrame(self.br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')

    
    def navi_loop(self):
        
            
        generic_target_pose2d = [0.4, 0, np.pi]
                    
        #target_pose2d = [0.25, 0, np.pi]
                
        rate = rospy.Rate(100) # 100hz
        
        wcv = WheelCmdVel()
        wcv.desiredGripperPos = gripperOpen
        arrived = False
        arrived_position = False
        self.print_id = -1
        while not rospy.is_shutdown() :
            if self.idx >= 11:
                break
            # get robot pose
            robot_pose3d = lookupTransform(self.lr, '/map', '/robot_base')
            
            target_pose2d = generic_target_pose2d
            
            # 6 go to bottle
            if self.idx == 6:
                print 'idx =', self.idx
                if self.bottle == 0:#left
                    wcv.desiredWV_R  = -0.3
                    wcv.desiredWV_L = -0.1
                    self.velcmd_pub.publish(wcv)  
                    rospy.sleep(2)
                    wcv.desiredWV_R  = 0.3
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                    rospy.sleep(1.8)
                    '''
                    wcv.desiredWV_R  = 0.05
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                    rospy.sleep(1)
                    '''
                    wcv.desiredWV_R  = 0.1
                    wcv.desiredWV_L = 0.1
                    self.velcmd_pub.publish(wcv)  
                    
                    rospy.sleep(3.8)
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                    self.idx = self.idx + 1
                elif self.bottle == 1:
                    # middle
                    wcv.desiredWV_R  = 0.05
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                    rospy.sleep(3)
                    wcv.desiredWV_R  = 0.05
                    wcv.desiredWV_L = 0.05
                    self.velcmd_pub.publish(wcv)  
                    self.idx = self.idx + 1
                    rospy.sleep(3.6)
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                elif self.bottle == 2:# right
                    wcv.desiredWV_R  = 0.05
                    wcv.desiredWV_L = 0.05
                    self.velcmd_pub.publish(wcv)  
                    self.idx = self.idx + 1
                    rospy.sleep(6)
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                # left
            #7 close gripper
            elif self.idx == 7:
                wcv.desiredGripperPos = girpperClose
                self.velcmd_pub.publish(wcv)  
                self.idx = self.idx + 1
                rospy.sleep(2)
                self.idx6cnt =0
            
            #8    
            elif self.idx == 8:
                # retreat
                print 'idx =', self.idx
                if self.bottle == 0:#left
                    if self.route_idx == 0:
                        wcv.desiredWV_R = -0.3
                        wcv.desiredWV_L = 0
                        self.velcmd_pub.publish(wcv) 
                        rospy.sleep(2)
                        self.idx = self.idx + 1
                        wcv.desiredWV_R = -0.1
                        wcv.desiredWV_L = -0.1
                        #self.velcmd_pub.publish(wcv) 
                        rospy.sleep(2)# left
                        wcv.desiredWV_R = 0
                        wcv.desiredWV_L = 0
                        self.velcmd_pub.publish(wcv) 
                    elif self.route_idx == 1:
                        wcv.desiredWV_R = -0.3
                        wcv.desiredWV_L = 0
                        self.velcmd_pub.publish(wcv) 
                        rospy.sleep(2)
                        self.idx = self.idx + 1
                        wcv.desiredWV_R = -0.1
                        wcv.desiredWV_L = -0.1
                        #self.velcmd_pub.publish(wcv) 
                        rospy.sleep(1)# left
                        wcv.desiredWV_R = 0
                        wcv.desiredWV_L = 0
                        self.velcmd_pub.publish(wcv) 
                elif self.bottle == 1:#middle
                    wcv.desiredWV_R = -0.3
                    wcv.desiredWV_L = -0.1
                    self.velcmd_pub.publish(wcv) 
                    self.idx = self.idx + 1
                    rospy.sleep(5) #middle
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv) 
                elif self.bottle == 2:#right
                    wcv.desiredWV_R = -0.3
                    wcv.desiredWV_L = -0.1
                    self.velcmd_pub.publish(wcv) 
                    self.idx = self.idx + 1
                    rospy.sleep(4)
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv) 

            elif self.idx == 9:
                # turn right
                print 'idx =', self.idx
                if self.bottle == 0:#left
                    if self.route_idx == 0:
                        wcv.desiredWV_R = 0
                        wcv.desiredWV_L = 0.2
                        self.velcmd_pub.publish(wcv) 
                        self.idx = self.idx + 1
                        rospy.sleep(3) # left
                        wcv.desiredWV_R = 0
                        wcv.desiredWV_L = 0
                        self.velcmd_pub.publish(wcv) 
                    elif self.route_idx == 1:
                        wcv.desiredWV_R = 0
                        wcv.desiredWV_L = 0.2
                        self.velcmd_pub.publish(wcv) 
                        rospy.sleep(3) # left
                        if self.detection_id != 4:
                            wcv.desiredWV_R = 0
                            wcv.desiredWV_L = 0.05
                            self.velcmd_pub.publish(wcv) 
                        else:
                            self.idx = self.idx + 1
                elif self.bottle == 1:#middle
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0.2
                    self.velcmd_pub.publish(wcv) 
                    self.idx = self.idx + 1
                    rospy.sleep(5.5) # middle
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv) 
                elif self.bottle == 2:#right
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0.2
                    self.velcmd_pub.publish(wcv) 
                    self.idx = self.idx + 1
                    rospy.sleep(5)
                    wcv.desiredWV_R = 0
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv) 
            # start at 5
            elif (self.idx == 5 and self.detection_id == 3) or (self.idx == 10 and self.detection_id == 4):
                if robot_pose3d is None:
                    print_id = 0
                    if print_id != self.print_id:
                        self.print_id = print_id
                        print '1. Tag not in view, Stop'
                    wcv.desiredWV_R = 0  # right, left
                    wcv.desiredWV_L = 0
                    self.velcmd_pub.publish(wcv)  
                    rate.sleep()
                    continue
                
                robot_position2d  = robot_pose3d[0:2]
                target_position2d = target_pose2d[0:2]
                
                robot_yaw    = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
                robot_pose2d = robot_position2d + [robot_yaw]
                
                # 2. navigation policy
                # 2.1 if       in the target, stop
                # 2.2 else if  close to target position, turn to the target orientation
                # 2.3 else if  in the correct heading, go straight to the target position,
                # 2.4 else     turn in the direction of the target position
                
                pos_delta         = np.array(target_position2d) - np.array(robot_position2d)
                robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
                heading_err_cross = cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
                
                # print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
                # print 'pos_delta', pos_delta
                # print 'robot_yaw', robot_yaw
                # print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d[2])
                # print 'heading_err_cross', heading_err_cross
            
                if arrived or (np.linalg.norm( pos_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
                    print_id = 21
                    if print_id != self.print_id:
                        self.print_id = print_id
                        print 'Case 2.1  Stop'
                    wcv.desiredWV_R = 0  
                    wcv.desiredWV_L = 0
                    arrived = True
                elif np.linalg.norm( pos_delta ) < 0.08:
                    arrived_position = True
                    if diffrad(robot_yaw, target_pose2d[2]) > 0:
                        print_id = 221
                        if print_id != self.print_id:
                            self.print_id = print_id
                            print 'Case 2.2.1  Turn right slowly'      
                        wcv.desiredWV_R = -0.05 
                        wcv.desiredWV_L = 0.05
                    else:
                        print_id = 222
                        if print_id != self.print_id:
                            self.print_id = print_id
                            print 'Case 2.2.2  Turn left slowly'
                        wcv.desiredWV_R = 0.05  
                        wcv.desiredWV_L = -0.05
                elif arrived_position or np.fabs( heading_err_cross ) < 0.2:
                    print_id = 23
                    if print_id != self.print_id:
                        self.print_id = print_id
                        print 'Case 2.3  Straight forward'  
                    wcv.desiredWV_R = 0.1
                    wcv.desiredWV_L = 0.1
                else:
                    if heading_err_cross < 0:
                        print_id = 241
                        if print_id != self.print_id:
                            self.print_id = print_id
                            print 'Case 2.4.1  Turn right'
                        wcv.desiredWV_R = -0.1
                        wcv.desiredWV_L = 0.1
                    else:
                        print_id = 242
                        if print_id != self.print_id:
                            self.print_id = print_id
                            print 'Case 2.4.2  Turn left'
                        wcv.desiredWV_R = 0.1
                        wcv.desiredWV_L = -0.1
                        
                self.velcmd_pub.publish(wcv)  
                
                if arrived:
                    print 'old idx', self.idx, 'new idx', self.idx + 1
                    self.idx = self.idx + 1
                    arrived = False
                    arrived_position = False
                    rospy.sleep(1)                    
            rate.sleep()
            
def main():
    rospy.init_node('apriltag_navi',anonymous=True)
    april_navi = ApriltagNavigator()
    rospy.spin()

if __name__=='__main__':
    main()
