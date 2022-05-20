#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32MultiArray

class Skeleton():
    def __init__(self):
        rospy.Subscriber("/kinect2/body_tracking_data", MarkerArray, self.callbackMarkerArray)  # raw kinect joints
        rospy.Subscriber("/ergonomics/rula", Float32MultiArray, self.callbackFloat32MultiArray) # calculated RULA scores
        self.pubSkeleton = rospy.Publisher('/ergonomics/skeleton', MarkerArray, queue_size=1)   # skeleton visualization

        #https://docs.microsoft.com/en-us/azure/kinect-dk/body-joints
        self.joints = {"PELVIS": 0, "SPINE_NAVAL": 1, "SPINE_CHEST": 2, "NECK": 3, "CLAVICLE_LEFT": 4,
                        "SHOULDER_LEFT": 5, "ELBOW_LEFT": 6, "WRIST_LEFT": 7, "HAND_LEFT": 8,
                        "HANDTIP_LEFT": 9, "THUMB_LEFT": 10, "CLAVICLE_RIGHT": 11, "SHOULDER_RIGHT": 12,
                        "ELBOW_RIGHT": 13, "WRIST_RIGHT": 14, "HAND_RIGHT": 15, "HANDTIP_RIGHT": 16,
                        "THUMB_RIGHT": 17, "HIP_LEFT": 18, "KNEE_LEFT": 19, "ANKLE_LEFT": 20,
                        "FOOT_LEFT": 21, "HIP_RIGHT": 22, "KNEE_RIGHT": 23, "ANKLE_RIGHT": 24, 
                        "FOOT_RIGHT": 25, "HEAD": 26, "NOSE": 27, "EYE_LEFT": 28,
                        "EAR_LEFT": 29, "EYE_RIGHT": 30, "EAR_RIGHT": 31}
        
        self.head = Marker()
        self.neck = Marker()
        self.shoulder_left = Marker()
        self.shoulder_right = Marker()
        self.elbow_right = Marker()
        self.elbow_left = Marker()
        self.wrist_right = Marker()
        self.wrist_left = Marker()
        self.hand_left = Marker()
        self.hand_right = Marker()
        self.spine_naval = Marker()
        self.knee_right = Marker()
        self.knee_left = Marker()
        self.ankle_left = Marker()
        self.ankle_right = Marker()
        self.leg_left = Marker()
        self.leg_right = Marker()
        self.hip_left = Marker()
        self.hip_right = Marker()
        self.pelvis = Marker()

        self.header = Header()
        self.header.frame_id = "kinect2_depth_camera_link"
        self.rula = [0,0,0,0,0,0,0,0,0,0,0,0]


    def colorToConfidence(self, color):
        if color.r == 1.0:
            confidence = 0.0
        elif color.b == 1.0:
            confidence = 0.1
        else:
            confidence = 1.0
        return confidence

    def scoreToColor(self,score=0.0):
        # 0 (red) - 151 (green) Hue
        # https://en.wikipedia.org/wiki/ISO_3864#Colours
        riskColor = {"safe": (35/255.0, 127/255.0, 82/255.0),
                  "low": (249/255.0, 168/255.0, 0),
                  "medium": (208/255.0, 93/255.0, 41/255.0),
                  "high": (155/255.0, 36/255.0, 35/255.0)}
        
        if (score <= 0.35):
            return riskColor["safe"]
        elif (score <= 0.6):
            return riskColor["low"]
        elif (score <= 0.85):
            return riskColor["medium"]
        else:
            return riskColor["high"]


    def createListMsg(self, joint1, joint2, ns, color=(35/255.0, 127/255.0, 82/255.0)):
        """ Create a LINE_LIST from two joints. Returns Marker-msg """
        listMsg = Marker()
        listMsg.header = self.header
        listMsg.header.stamp.secs = rospy.get_rostime().secs
        listMsg.header.stamp.nsecs = rospy.get_rostime().nsecs
        listMsg.ns = ns
        listMsg.action = 0
        listMsg.type = 5
        listMsg.scale.x = 0.03
        listMsg.pose.orientation.w = 1.0
        listMsg.lifetime.nsecs = 250000000

        # color = REBA Score
        listMsg.color.r = color[0]
        listMsg.color.g = color[1]
        listMsg.color.b = color[2]
        K = min(self.colorToConfidence(joint1.color),self.colorToConfidence(joint2.color))  # transparency = min confidence value
        if K < 1:
            transparency = 0.25
        else:
            transparency = 1.0
        listMsg.color.a = transparency

        listMsg.points = [joint1.pose.position, joint2.pose.position]

        return listMsg


    def createTextMsg(self, text, ns, color=(1,1,1), y=0, scale=0.15):
        """ Create a TEXT_VIEW_FACING. Returns Marker-msg (PLACEHOLDER)"""
        listMsg = Marker()
        listMsg.header = self.header
        listMsg.header.stamp.secs = rospy.get_rostime().secs
        listMsg.header.stamp.nsecs = rospy.get_rostime().nsecs
        listMsg.ns = ns
        listMsg.action = 0
        listMsg.type = 9
        listMsg.scale.z = scale
        listMsg.pose.orientation.w = 1.0
        listMsg.lifetime.nsecs = 250000000

        # color = REBA Score
        listMsg.color.r = color[0]
        listMsg.color.g = color[1]
        listMsg.color.b = color[2]
        listMsg.color.a = 1.0

        listMsg.pose.position.x = 1.5
        listMsg.pose.position.y = y
        
        listMsg.text = text

        return listMsg


    def callbackMarkerArray(self, data):
        """get all relevant joints"""
        if (data.markers):
            self.head = next((x for x in data.markers if x.id % 100 == self.joints["HEAD"]), None)
            self.neck = next((x for x in data.markers if x.id % 100 == self.joints["NECK"]), None)
            self.spine_naval = next((x for x in data.markers if x.id % 100 == self.joints["SPINE_NAVAL"]), None)
            self.pelvis = next((x for x in data.markers if x.id % 100 == self.joints["PELVIS"]), None)

            
            self.shoulder_left = next((x for x in data.markers if x.id % 100 == self.joints["SHOULDER_LEFT"]), None)
            self.shoulder_right = next((x for x in data.markers if x.id % 100 == self.joints["SHOULDER_RIGHT"]), None)
            
            self.elbow_right = next((x for x in data.markers if x.id % 100 == self.joints["ELBOW_RIGHT"]), None)
            self.elbow_left = next((x for x in data.markers if x.id % 100 == self.joints["ELBOW_LEFT"]), None)

            self.wrist_left = next((x for x in data.markers if x.id % 100 == self.joints["WRIST_LEFT"]), None)
            self.wrist_right = next((x for x in data.markers if x.id % 100 == self.joints["WRIST_RIGHT"]), None)

            self.hand_right = next((x for x in data.markers if x.id % 100 == self.joints["HAND_RIGHT"]), None)
            self.hand_left = next((x for x in data.markers if x.id % 100 == self.joints["HAND_LEFT"]), None)
             
            self.knee_right = next((x for x in data.markers if x.id % 100 == self.joints["KNEE_RIGHT"]), None)
            self.knee_left = next((x for x in data.markers if x.id % 100 == self.joints["KNEE_LEFT"]), None)

            self.ankle_right = next((x for x in data.markers if x.id % 100 == self.joints["ANKLE_RIGHT"]), None)
            self.ankle_left = next((x for x in data.markers if x.id % 100 == self.joints["ANKLE_LEFT"]), None)

            self.hip_left = next((x for x in data.markers if x.id % 100 == self.joints["HIP_LEFT"]), None)
            self.hip_right = next((x for x in data.markers if x.id % 100 == self.joints["HIP_RIGHT"]), None)
            

            self.header = self.head.header


    def callbackFloat32MultiArray(self,data):
        #print(data.data)
        self.rula = data.data


    def publishSkeleton(self):
        msg = MarkerArray()
        # A) Neck, Trunk, Legs
        neck_list = self.createListMsg(self.head, self.neck, "neck", color=self.scoreToColor(self.rula[4]/4.0))

        trunk_list = self.createListMsg(self.neck, self.spine_naval, "trunk", color=self.scoreToColor(self.rula[5]/6.0))
        trunk_list2 = self.createListMsg(self.spine_naval, self.pelvis, "trunk2", color=self.scoreToColor(self.rula[5]/6.0))
        trunk_list3 = self.createListMsg(self.hip_right, self.pelvis, "trunk3", color=self.scoreToColor(self.rula[5]/6.0))
        trunk_list4 = self.createListMsg(self.hip_left, self.pelvis, "trunk4", color=self.scoreToColor(self.rula[5]/6.0))



        upper_leg_left_list = self.createListMsg(self.hip_left, self.knee_left, "upper_leg_left", color=self.scoreToColor(self.rula[6]/4.0))
        upper_leg_right_list = self.createListMsg(self.hip_right, self.knee_right, "upper_leg_right", color=self.scoreToColor(self.rula[7]/4.0))

        lower_leg_left_list  = self.createListMsg(self.knee_left, self.ankle_left, "lower_leg_left", color=self.scoreToColor(self.rula[6]/4.0))
        lower_leg_right_list  = self.createListMsg(self.knee_right, self.ankle_right, "lower_leg_right", color=self.scoreToColor(self.rula[7]/4.0))

        # B) Arm and Wrist
        upper_arm_left_list = self.createListMsg(self.shoulder_left, self.elbow_left, "upper_arm_left", color=self.scoreToColor(self.rula[0]/6.0))
        upper_arm_right_list = self.createListMsg(self.shoulder_right, self.elbow_right, "upper_arm_right", color=self.scoreToColor(self.rula[1]/6.0))

        shoulder_left_list = self.createListMsg(self.neck, self.shoulder_left, "shoulder_left", color=self.scoreToColor(self.rula[0]/6.0))
        shoulder_right_list = self.createListMsg(self.neck, self.shoulder_right, "shoulder_right", color=self.scoreToColor(self.rula[1]/6.0))

        
        lower_arm_right_list = self.createListMsg(self.elbow_right, self.wrist_right, "lower_arm_right", color=self.scoreToColor(self.rula[3]/4.0))
        lower_arm_left_list = self.createListMsg(self.elbow_left, self.wrist_left, "lower_arm_left", color=self.scoreToColor(self.rula[2]/4.0))

        wrist_left_list = self.createListMsg(self.wrist_left, self.hand_left, "wrist_left", color=self.scoreToColor(self.rula[8]/4.0))
        wrist_right_list = self.createListMsg(self.wrist_right, self.hand_right, "wrist_right", color=self.scoreToColor(self.rula[9]/4.0))

        msg = [neck_list,shoulder_left_list, shoulder_right_list, upper_arm_left_list, upper_arm_right_list, lower_arm_left_list, lower_arm_right_list, wrist_left_list, wrist_right_list, trunk_list, trunk_list2, trunk_list3, trunk_list4, upper_leg_left_list, upper_leg_right_list, lower_leg_left_list, lower_leg_right_list]

        # Add TEXT
        msg.append(self.createTextMsg("Neck Score: "+str(int(max(1,self.rula[4]))), "neck_score", y=-1, color=self.scoreToColor(int(self.rula[4])/6.0)))
        msg.append(self.createTextMsg("Trunk Score: "+str(int(max(1,self.rula[5]))), "trunk_score", y=-0.8, color=self.scoreToColor(int(self.rula[5])/6.0)))
        msg.append(self.createTextMsg("Upper Arm Score: "+str(int(max(self.rula[0], self.rula[1]))), "upper_arm_score", color=self.scoreToColor(int(max(self.rula[0], self.rula[1]))/6.0), y=-0.6))
        msg.append(self.createTextMsg("Lower Arm Score: "+str(int(max(self.rula[2], self.rula[3]))), "lower_arm_score", color=self.scoreToColor(int(max(self.rula[2], self.rula[3]))/4.0), y=-0.4))
        msg.append(self.createTextMsg("Wrist Score: "+str(int(max(self.rula[8], self.rula[9]))), "wrist_score", color=self.scoreToColor(int(max(self.rula[8], self.rula[9]))/4.0), y=-0.2))
        msg.append(self.createTextMsg("Leg Score: "+str(int(max(self.rula[6], 1))), "leg_score", color=self.scoreToColor(int(max(self.rula[6], self.rula[7]))/4.0), y=-0.0))

        msg.append(self.createTextMsg("RULA Score: "+str(int(self.rula[10])), "rula_score", color=self.scoreToColor(self.rula[10]/7.0), y=+0.3, scale=0.25))
        msg.append(self.createTextMsg("Confidence: "+str(round(self.rula[11],2)), "rula_conf", color=self.scoreToColor(1.0-self.rula[11]**2), y=+0.45, scale=0.1))
            
        # 4 RULA action levels
        if(self.rula[10] <= 2):
            msg.append(self.createTextMsg("Acceptable posture", "rula_action_level", color=self.scoreToColor(0), y=+0.9, scale=0.2))
        elif(self.rula[10] <= 4):
            msg.append(self.createTextMsg("Posture changes may be required", "rula_action_level", color=self.scoreToColor(0.4), y=+0.9, scale=0.2))
        elif(self.rula[10] <= 6):
            msg.append(self.createTextMsg("Posture changes are required soon", "rula_action_level", color=self.scoreToColor(0.7), y=+0.9, scale=0.2))
        else:
            msg.append(self.createTextMsg("Posture changes are required immediately", "rula_action_level", color=self.scoreToColor(1), y=+0.9, scale=0.2))

        self.pubSkeleton.publish(msg)


def start():
    rospy.init_node('Skeleton', anonymous=True)
    node = Skeleton()
    r = rospy.Rate(10) # 10hz?
    while not rospy.is_shutdown():
        node.publishSkeleton()
        r.sleep()


if __name__ == '__main__':
    start()