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
gripperClose = 110

class ApriltagNavigator():
    def __init__(self, route_idx = 0):
        
        self.detection_id = -1
        self.idx = 5
        
        self.lr = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        
        self.velcmd_pub = rospy.Publisher('/cmdvel',WheelCmdVel,queue_size=1)
        wcv = WheelCmdVel()
        wcv.desiredGripperPos = gripperOpen
        self.velcmd_pub.publish(wcv)  

        self.idx6cnt= 0
        self.stage9 = 0
        
        self.thread = threading.Thread(target = self.navi_loop)
        self.thread.start()
        
        rospy.sleep(1)

    def apriltag_callback(self,data):
        # use apriltag pose detection to find where is the robot
        # print data
        for detection in data.detections:
            if self.idx == 6 and detection.id == 12:
                
                self.idx = self.idx + 1
            
            if (self.idx == 5 and detection.id == 4) and (self.idx == 8 and detection.id == 7):   # tag id is the correct one
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
            if self.idx >= 10:
                break
            # get robot pose
            robot_pose3d = lookupTransform(self.lr, '/map', '/robot_base')
            
            target_pose2d = generic_target_pose2d
            
            
            #7 close gripper
            if self.idx == 7:
                rospy.sleep(1)

                wcv.desiredGripperPos = gripperClose
                self.velcmd_pub.publish(wcv)  
                self.idx = self.idx + 1
            
            #8    
            elif self.idx == 8:
                # retreat
                print 'idx =', self.idx
                wcv.desiredWV_R = -0.1
                wcv.desiredWV_L = -0.1
                self.velcmd_pub.publish(wcv) 
                rospy.sleep(2)
                wcv.desiredWV_R = -0.3
                wcv.desiredWV_L = -0.1
                self.velcmd_pub.publish(wcv) 
                rospy.sleep(2.5) 
                wcv.desiredWV_R = 0
                wcv.desiredWV_L = 0.1
                self.velcmd_pub.publish(wcv) 
                rospy.sleep(4) 
                wcv.desiredWV_R = 0
                wcv.desiredWV_L = 0
                self.velcmd_pub.publish(wcv) 
                self.idx = self.idx + 1

            elif self.idx == 9:
                # go straight
                wcv.desiredWV_R = 0.1
                wcv.desiredWV_L = 0.1
                self.velcmd_pub.publish(wcv) 
                self.idx = self.idx + 1
                rospy.sleep(18)
                wcv.desiredWV_R = 0
                wcv.desiredWV_L = 0
                self.velcmd_pub.publish(wcv) 


            elif (self.idx == 5 and self.detection_id == 4):
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
                        wcv.desiredWV_R = -0.05
                        wcv.desiredWV_L = 0.05
                    else:
                        print_id = 242
                        if print_id != self.print_id:
                            self.print_id = print_id
                            print 'Case 2.4.2  Turn left'
                        wcv.desiredWV_R = 0.05
                        wcv.desiredWV_L = -0.05
                        
                self.velcmd_pub.publish(wcv)  
                
                if arrived:
                    print 'old idx', self.idx, 'new idx', self.idx + 1
                    self.idx = self.idx + 1
                    arrived = False
                    arrived_position = False
                    rospy.sleep(3)

            rate.sleep()
            
            
    # Task 3 callback
    def rosRGBDCallBack(self, rgb_data, depth_data):
        if self.idx != 6:
            return
        
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
        except CvBridgeError as e:
            print(e)
        
        if self.idx == 6:
            contours_red, mask_red = self.HSVObjectDetection_route_detection(cv_image, toPrint = False)
            
            red_detected = False
            
                
            for cnt in contours_red:
                xp,yp,w,h = cv2.boundingRect(cnt)
                centerx, centery = xp+w/2, yp+h/2
                
                # Get depth value from depth image, need to make sure the value is in the range 0.5-2 meter (add constraint about (xp and yp) or (w and h))
                if not math.isnan(cv_depthimage2[int(yp)][int(xp)]) and  cv_depthimage2[int(yp)][int(xp)] <= 0.5 \
                    and w > 100 and h > 100:
                    red_detected = True
                    break
                    # zc = cv_depthimage2[int(yp)][int(xp)]
                    # print 'zc', zc
                else:
                    continue
            print 'red_obstacle_detected', red_detected
            if red_detected:
                self.idx = self.idx + 1
           
            rospy.sleep(1)

            
    # object detection code
    def HSVObjectDetection_route_detection(self, cv_image, toPrint = True):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # define range of red color in HSV
        lower_red = np.array([170,50,50])
        upper_red = np.array([180,255,255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_red, upper_red)   ##
        mask_eroded         = cv2.erode(mask, None, iterations = 3)  ##
        mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 10)  ##

        if toPrint:
            print 'hsv', hsv_image[240][320] # the center point hsv
            
        #self.showImageInCVWindow(cv_image, mask_eroded_green, mask_eroded_dilated_green)
        image_red,contours_red,hierarchy_red = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return contours_red, mask_eroded_dilated

    def showImageInCVWindow(self, cv_image, mask_erode_image, mask_image):
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image, cv_image, mask = mask_image)
        
        # Draw a cross at the center of the image
        cv2.line(cv_image, (320, 235), (320, 245), (255,0,0))
        cv2.line(cv_image, (325, 240), (315, 240), (255,0,0))
        
        # Show the images
        
        print 'open image'
        #cv2.imshow('OpenCV_Original', cv_image)
        #cv2.imshow('OpenCV_Mask_Erode', mask_erode_image)
        #cv2.imshow('OpenCV_Mask_Dilate', mask_image)
        cv2.imshow('OpenCV_View', res)
        cv2.waitKey(3)
        

    def getXYZ(self, xp, yp, zc, fx,fy,cx,cy):
        ## 
        xn = (xp - cx) / fx
        yn = (yp - cy) / fy
        xc = xn * zc
        yc = yn * zc
        return (xc,yc,zc)

        

    # Create a pyramid using 4 triangles
    def showPyramid(self, xp, yp, zc, w, h):
        # X1-X4 are the 4 corner points of the base of the pyramid
        X1 = self.getXYZ(xp-w/2, yp-h/2, zc, self.fx, self.fy, self.cx, self.cy)
        X2 = self.getXYZ(xp-w/2, yp+h/2, zc, self.fx, self.fy, self.cx, self.cy)
        X3 = self.getXYZ(xp+w/2, yp+h/2, zc, self.fx, self.fy, self.cx, self.cy)
        X4 = self.getXYZ(xp+w/2, yp-h/2, zc, self.fx, self.fy, self.cx, self.cy)
        self.vis_pub.publish(self.createTriangleListMarker(1, [X1, X2, X3, X4], rgba = [1,0,0,1], frame_id = '/camera'))

    # Create a list of Triangle markers for visualization
    def createTriangleListMarker(self, marker_id, points, rgba, frame_id = '/camera'):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.TRIANGLE_LIST
        marker.scale = Vector3(1,1,1)
        marker.id = marker_id
        
        n = len(points)
        
        if rgba is not None:
            marker.color = ColorRGBA(*rgba)
            
        o = Point(0,0,0)
        for i in xrange(n):
            p = Point(*points[i])
            marker.points.append(p)
            p = Point(*points[(i+1)%4])
            marker.points.append(p)
            marker.points.append(o)
            
        marker.pose = self.poselist2pose([0,0,0,0,0,0,1])
        return marker

    def poselist2pose(self, poselist):
        return Pose(Point(*poselist[0:3]), Quaternion(*poselist[3:7]))
            
def main():
    rospy.init_node('apriltag_navi',anonymous=True)
    april_navi = ApriltagNavigator()
    
    # Task 3: Use Kinect depth data
    #    Subscribe to both RGB and Depth images with a Synchronizer
    '''
    image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
    ts.registerCallback(april_navi.rosRGBDCallBack)
    '''
    rospy.spin()

if __name__=='__main__':
    main()
