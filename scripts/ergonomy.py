#!/usr/bin/env python3

from datetime import datetime
import os
import rospy
import rospkg
import numpy as np
from math import floor, sqrt
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# defines the way confidence is calculated
# 0: default behaviour. average used through whole process
# 1: minimum used for all subscores
# 2: simple average. average confidence of all joints multiplied with rula score
conf_variant = 1

# save data to csv file 
# 0: no (default)
# 1: yes
save_data = 0

def angle_relative(a, b, c, d, adjust):
    """return angle between two vectors"""
    ba = a.position - b.position
    dc = c.position - d.position

    cosine_angle = np.dot(ba, dc) / (np.linalg.norm(ba) * np.linalg.norm(dc))
    angle = np.arccos(cosine_angle)

    if (adjust == 1):
        angle_adjusted = abs(np.degrees(angle) - 180)
        return Angle(int(round(angle_adjusted)), calc_conf([a.confidence, b.confidence, c.confidence, d.confidence]))
    else:
        return Angle(int(round(np.degrees(angle))), calc_conf([a.confidence, b.confidence, c.confidence, d.confidence]))

def projection(a, v):
    """return projection of point a onto horizontal plane"""
    v_norm = v / np.linalg.norm(v)
    dist = a.position[0]*v_norm[0]+a.position[1]*v_norm[1]+a.position[2]*v_norm[2]
    proj = a.position  - dist*v_norm
    return Joint(proj, a.confidence)

def angle_vertical(a, b, v, adjust):
    """return angle of vector in relation to vertical vector"""    
    ba = a.position - b.position

    cosine_angle = np.dot(ba, v) / (np.linalg.norm(ba) * np.linalg.norm(v))
    angle = np.arccos(cosine_angle)

    if (adjust == 1):
        angle_adjusted = abs(np.degrees(angle) - 180)
        return Angle(int(round(angle_adjusted)), calc_conf([a.confidence, b.confidence, 1]))
    else:
        return Angle(int(round(np.degrees(angle))), calc_conf([a.confidence, b.confidence, 1]))

def calc_distance(a, b):
    """calculate distance between two joints"""
    return Distance(np.linalg.norm(a.position-b.position), calc_conf([a.confidence, b.confidence]))

def get_joint(a):
    """return necessary information of a joint"""
    return Joint(np.array([a.pose.position.x, a.pose.position.y, a.pose.position.z]), get_conf(a))

def calc_conf(a):
    """calculate confidence value"""
    if (conf_variant == 1):
        return calc_conf_min(a)
    else:
        return calc_conf_avg(a)

def calc_conf_avg(a):
    """return average confidence value"""
    return sum(a) / len(a)

def calc_conf_min(a):
    """return lowest confidence value"""
    return min(a)

def get_conf(a):
    """return confidence value"""
    if (a.color.r == 1.0 and a.color.g == 0.0 and a.color.b == 0.0):    #none/out of range
        return 0.1
    elif (a.color.r == 0.0 and a.color.g == 0.0 and a.color.b == 1.0):  #low
        return 0.25
    elif (a.color.r == 0.0 and a.color.g == 1.0 and a.color.b == 0.0):  #medium
        return 1.0
    else:
        return 0

def midpoint(l, r):
    """return midpoint between l and r"""
    pos = np.array([(l.position[0]+r.position[0])/2, (l.position[1]+r.position[1])/2, (l.position[2]+r.position[2])/2])
    conf = calc_conf([l.confidence, r.confidence])
    return Joint(pos, conf)

def table_b(neck, legs, trunk):
    """return score B based on neck, trunk and leg scores"""
    table = [
        [[1,2,3,5,7,8], [3,3,3,5,7,8]],
        [[2,2,3,5,7,8], [3,3,4,6,7,8]],
        [[3,4,4,6,7,8], [4,5,5,7,8,8]],
        [[5,5,5,7,8,8], [5,5,6,7,8,9]],
        [[6,6,6,7,8,9], [6,7,7,7,8,9]],
        [[7,7,7,8,8,9], [7,7,7,8,8,9]]
    ]
    return table[int(round(trunk))-1][int(round(legs))-1][int(round(neck))-1]

def table_a(lower_arm, wrist, upper_arm):
    """return score A based on lower arm, wrist and upper arm scores"""
    table = [
        [[1,2,2,3], [2,2,3,3], [2,3,3,4]],
        [[2,3,3,4], [3,3,3,4], [3,4,4,5]],
        [[3,4,4,5], [3,4,4,5], [4,4,4,5]],
        [[4,4,4,5], [4,4,4,5], [4,4,5,6]],
        [[5,5,5,6], [5,6,6,7], [6,6,7,7]],
        [[7,7,7,8], [8,8,8,9], [9,9,9,9]]
    ]
    return table[int(round(upper_arm))-1][int(round(lower_arm))-1][int(round(wrist))-1]

def table_c(a, b):
    """return RULA Score based on the scores from table A and B"""
    a_lim = min(a, 8)
    b_lim = min(b, 7)
    table = [
        [1,2,3,3,4,5,5],
        [2,2,3,4,4,5,5],
        [3,3,3,4,4,5,6],
        [3,3,3,4,5,6,6],
        [4,4,4,5,6,7,7],
        [4,4,5,6,6,7,7],
        [5,5,6,6,7,7,7],
        [5,5,6,7,7,7,7]
    ]
    return table[a_lim-1][b_lim-1]

def neck_score(angle, angle_alt):
    """return score based on trunk position"""
    if (140 < angle_alt.angle):     #neck in extension
        return Score(4*angle_alt.confidence, angle_alt.confidence, 4, -angle.angle)
    elif (0 <= angle.angle < 10):
        return Score(1*angle.confidence, angle.confidence, 1, angle.angle)
    elif (10 <= angle.angle < 20):
        return Score(2*angle.confidence, angle.confidence, 2, angle.angle)
    elif (20 <= angle.angle):
        return Score(3*angle.confidence, angle.confidence, 3, angle.angle)

def neck_twist_score(angle):
    """is neck twisted?"""
    if ((angle < 75) or (105 < angle)):
        return 1
    else:
        return 0

def neck_bend_score(angle):
    """is neck side bending"""
    if ((angle <= 82) or (98 <= angle)):
        return 1
    else:
        return 0

def neck_evaluation(spine_chest, neck, nose, ear_left, ear_right, shoulder_left, shoulder_right):
    """return neck score"""
    head = midpoint(get_joint(ear_left), get_joint(ear_right))
    neck_pos = neck_score(angle_relative(get_joint(spine_chest), get_joint(neck), head, get_joint(neck), 1), 
                            angle_relative(get_joint(spine_chest), get_joint(neck), get_joint(nose), get_joint(neck), 0))
    neck_twist_angle = angle_relative(get_joint(nose), get_joint(neck), get_joint(shoulder_left), get_joint(shoulder_right), 0)
    neck_twist = Score(neck_twist_score(neck_twist_angle.angle)*neck_twist_angle.confidence, neck_twist_angle.confidence, neck_twist_score(neck_twist_angle.angle), neck_twist_angle.angle)
    neck_bend_angle = angle_relative(get_joint(shoulder_left), get_joint(shoulder_right), head, get_joint(neck), 0)
    neck_bend = Score(neck_bend_score(neck_bend_angle.angle)*neck_bend_angle.confidence, neck_bend_angle.confidence, neck_bend_score(neck_bend_angle.angle), neck_bend_angle.angle)
    return Score(neck_bend.score + neck_twist.score + neck_pos.score, calc_conf([neck_bend.confidence, neck_twist.confidence, neck_pos.confidence]), neck_bend.base_score + neck_twist.base_score + neck_pos.base_score, neck_pos.angle)

def trunk_score(angle):
    """return score based on trunk position"""
    if (angle <= 7):
        return 1
    elif (7 < angle <= 20):
        return 2
    elif (20 < angle <= 60):
        return 3
    elif (60 < angle):
        return 4

def trunk_twist_score(angle):
    """is trunk twisted?"""
    if (angle > 15):
        return 1
    else:
        return 0

def trunk_bend_score(angle):
    """is trunk side bending?"""
    if (87 < angle < 93):
        return 0
    else:
        return 1

def trunk_evaluation(hip_left, hip_right, shoulder_left, shoulder_right, pelvis, spine_chest, neck, v):
    """return trunk score"""
    trunk_angle = angle_vertical(get_joint(spine_chest), get_joint(pelvis), v, 0)
    trunk = Score(trunk_score(trunk_angle.angle)*trunk_angle.confidence, trunk_angle.confidence, trunk_score(trunk_angle.angle), trunk_angle.angle)
    trunk_twist_angle = angle_relative(projection(get_joint(hip_left), v), projection(get_joint(hip_right), v), 
                    projection(get_joint(shoulder_left), v), projection(get_joint(shoulder_right), v), 0)
    trunk_twist = Score(trunk_twist_score(trunk_twist_angle.angle)*trunk_twist_angle.confidence, trunk_twist_angle.confidence, trunk_twist_score(trunk_twist_angle.angle), trunk_twist_angle.angle)
    trunk_bend_angle = angle_relative(get_joint(hip_left), get_joint(hip_right), get_joint(neck), get_joint(pelvis), 0)
    trunk_bend = Score(trunk_bend_score(trunk_bend_angle.angle)*trunk_bend_angle.confidence, trunk_bend_angle.confidence, trunk_bend_score(trunk_bend_angle.angle), trunk_bend_angle.angle)
    return Score(trunk.score+trunk_bend.score+trunk_twist.score, calc_conf([trunk.confidence, trunk_bend.confidence, trunk_twist.confidence]), trunk.base_score + trunk_bend.base_score + trunk_twist.base_score, trunk.angle)

def leg_score(angle_l, angle_r):
    """standing on one leg or two"""
    dif = abs(angle_l - angle_r)
    if (dif > 15):
        return 2
    else:
        return 1

def leg_evaluation(hip_left, hip_right, knee_left, knee_right, ankle_left, ankle_right):
    """return leg score"""
    leg_left = angle_relative(get_joint(hip_left), get_joint(knee_left), get_joint(ankle_left), get_joint(knee_left), 1)
    leg_right = angle_relative(get_joint(hip_right), get_joint(knee_right), get_joint(ankle_right), get_joint(knee_right), 1)
    legs_conf = calc_conf([leg_left.confidence, leg_right.confidence])
    score = leg_score(leg_left.angle, leg_right.angle)
    return Score(score*legs_conf, legs_conf, score, min(leg_left.angle, leg_right.angle))

def upper_arm_score(angle):
    """return score based on upper arm position"""
    if (angle < 20):
        return 1
    elif (20 <= angle < 45):
        return 2
    elif (45 <= angle < 90):
        return 3
    elif (90 <= angle):
        return 4

def shoulder_score(angle):
    """is shoulder raised?"""
    if (angle <= 105):
        return 1
    else:
        return 0

def abduction_score(angle):
    """is arm abducted?"""
    if (angle < 125):
        return 0
    else:
        return 1

def upper_arm_evaluation(elbow_left, elbow_right, shoulder_left, shoulder_right, head, neck, v):
    """return upper arm score"""
    upper_arm_l_angle = angle_vertical(get_joint(elbow_left), get_joint(shoulder_left), v, 1)
    upper_arm_l = Score(upper_arm_score(upper_arm_l_angle.angle)*upper_arm_l_angle.confidence, upper_arm_l_angle.confidence, upper_arm_score(upper_arm_l_angle.angle), upper_arm_l_angle.angle)
    upper_arm_r_angle = angle_vertical(get_joint(elbow_right), get_joint(shoulder_right), v, 1)
    upper_arm_r = Score(upper_arm_score(upper_arm_r_angle.angle)*upper_arm_r_angle.confidence, upper_arm_r_angle.confidence, upper_arm_score(upper_arm_r_angle.angle), upper_arm_r_angle.angle)
    shoulder_l_angle = angle_relative(get_joint(head), get_joint(neck), get_joint(shoulder_left), get_joint(neck), 0)
    shoulder_l = Score(shoulder_score(shoulder_l_angle.angle)*shoulder_l_angle.confidence, shoulder_l_angle.confidence, shoulder_score(shoulder_l_angle.angle), shoulder_l_angle.angle)
    shoulder_r_angle = angle_relative(get_joint(head), get_joint(neck), get_joint(shoulder_right), get_joint(neck), 0)
    shoulder_r = Score(shoulder_score(shoulder_r_angle.angle)*shoulder_r_angle.confidence, shoulder_r_angle.confidence, shoulder_score(shoulder_r_angle.angle), shoulder_r_angle.angle)
    abduction_l_angle = angle_relative(get_joint(shoulder_right), get_joint(shoulder_left), get_joint(elbow_left), get_joint(shoulder_left), 0)
    abduction_l = Score(abduction_score(abduction_l_angle.angle)*abduction_l_angle.confidence, abduction_l_angle.confidence, abduction_score(abduction_l_angle.angle), abduction_l_angle.angle)
    abduction_r_angle = angle_relative(get_joint(shoulder_left), get_joint(shoulder_right), get_joint(elbow_right), get_joint(shoulder_right), 0)
    abduction_r = Score(abduction_score(abduction_r_angle.angle)*abduction_r_angle.confidence, abduction_r_angle.confidence, abduction_score(abduction_r_angle.angle), abduction_r_angle.angle)
    score_l = Score(upper_arm_l.score + shoulder_l.score + abduction_l.score, calc_conf([upper_arm_l.confidence, shoulder_l.confidence, abduction_l.confidence]), upper_arm_l.base_score + shoulder_l.base_score + abduction_l.base_score, upper_arm_l.angle)
    score_r = Score(upper_arm_r.score + shoulder_r.score + abduction_r.score, calc_conf([upper_arm_r.confidence, shoulder_r.confidence, abduction_r.confidence]), upper_arm_r.base_score + shoulder_r.base_score + abduction_r.base_score, upper_arm_r.angle)
    return [score_l, score_r]

def lower_arm_score(angle):
    """return score based on lower arm position"""
    if ((angle < 60) or (100 <= angle)):
        return 2
    elif (60 <= angle < 100):
        return 1

def midline(same_side, other_side):
    """check if lower arm is working across midline of the body"""
    conf = calc_conf([other_side.confidence, same_side.confidence])
    if (other_side.distance < same_side.distance):
        return Score(1*conf, conf, 1, None)
    else:
        return Score(0*conf, conf, 0, None)

def outward(angle):
    """check if lower arm is working out to the side of the body"""
    if (100 < angle):
        return 1
    else:
        return 0

def midline_score(shoulder_left, shoulder_right, elbow_left, elbow_right, wrist_left, wrist_right):
    """lower arm working across the midline of the body or out to the side"""
    out_left_angle = angle_relative(get_joint(shoulder_right), get_joint(shoulder_left), get_joint(wrist_left), get_joint(elbow_left), 0)
    out_left = Score(outward(out_left_angle.angle)*out_left_angle.confidence, out_left_angle.confidence, outward(out_left_angle.angle), out_left_angle.angle)
    out_right_angle = angle_relative(get_joint(shoulder_left), get_joint(shoulder_right), get_joint(wrist_right), get_joint(elbow_right), 0)
    out_right = Score(outward(out_right_angle.angle)*out_right_angle.confidence, out_right_angle.confidence, outward(out_right_angle.angle), out_right_angle.angle)
    in_left = midline(calc_distance(get_joint(wrist_left), get_joint(shoulder_left)), calc_distance(get_joint(wrist_left), get_joint(shoulder_right)))
    in_right = midline(calc_distance(get_joint(wrist_right), get_joint(shoulder_right)), calc_distance(get_joint(wrist_right), get_joint(shoulder_right)))
    left = Score(max(out_left.score, in_left.score), calc_conf([out_left.confidence, in_left.confidence]), max(out_left.base_score, in_left.base_score), None)
    right = Score(max(out_right.score, in_right.score), calc_conf([out_right.confidence, in_right.confidence]), max(out_right.base_score, in_right.base_score), None)
    return [left, right]

def lower_arm_evaluation(elbow_left, elbow_right, shoulder_left, shoulder_right, wrist_left, wrist_right):
    """return lower arm score"""
    lower_arm_l_angle = angle_relative(get_joint(shoulder_left), get_joint(elbow_left), get_joint(wrist_left), get_joint(elbow_left), 1)
    lower_arm_l = Score(lower_arm_score(lower_arm_l_angle.angle)*lower_arm_l_angle.confidence, lower_arm_l_angle.confidence, lower_arm_score(lower_arm_l_angle.angle), lower_arm_l_angle.angle)
    lower_arm_r_angle = angle_relative(get_joint(shoulder_right), get_joint(elbow_right), get_joint(wrist_right), get_joint(elbow_right), 1)
    lower_arm_r = Score(lower_arm_score(lower_arm_r_angle.angle)*lower_arm_r_angle.confidence, lower_arm_r_angle.confidence, lower_arm_score(lower_arm_r_angle.angle), lower_arm_r_angle.angle)
    midline = midline_score(shoulder_left, shoulder_right, elbow_left, elbow_right, wrist_left, wrist_right)
    score_l = Score((lower_arm_l.score + midline[0].score), calc_conf([lower_arm_l.confidence, midline[0].confidence]), lower_arm_l.base_score + midline[0].base_score, lower_arm_l.angle)
    score_r = Score((lower_arm_r.score + midline[1].score), calc_conf([lower_arm_r.confidence, midline[1].confidence]), lower_arm_r.base_score + midline[1].base_score, lower_arm_r.angle)
    return [score_l, score_r]

def wrist_score(angle):
    """return score based on wrist position"""
    #tracking is not reliable enough so always returns 1
    if (0 <= angle < 60):
        return 1
    elif (60 <= angle):
        return 1

def wrist_evaluation(elbow_left, elbow_right, wrist_left, wrist_right, hand_left, hand_right):
    """return wrist score"""
    wrist_l_angle = angle_relative(get_joint(elbow_left), get_joint(wrist_left), get_joint(hand_left), get_joint(wrist_left), 1)
    wrist_l = Score(wrist_score(wrist_l_angle.angle)*wrist_l_angle.confidence, wrist_l_angle.confidence, wrist_score(wrist_l_angle.angle), wrist_l_angle.angle)
    wrist_r_angle = angle_relative(get_joint(elbow_right), get_joint(wrist_right), get_joint(hand_right), get_joint(wrist_right), 1)
    wrist_r = Score(wrist_score(wrist_r_angle.angle)*wrist_r_angle.confidence, wrist_r_angle.confidence, wrist_score(wrist_r_angle.angle), wrist_r_angle.angle)
    return [wrist_l, wrist_r]

class Joint():
    def __init__(self, position, confidence):
        self.position = position
        self.confidence = confidence

class Score():
    def __init__(self, score, confidence, base_score, angle):
        self.score = score
        self.confidence = confidence
        self.base_score = base_score
        self.angle = angle

class Angle():
    def __init__(self, angle, confidence):
        self.angle = angle
        self.confidence = confidence

class Distance():
    def __init__(self, distance, confidence):
        self.distance = distance
        self.confidence = confidence

class Ergonomy():
    def __init__(self):
        rospack = rospkg.RosPack()
        imu_vector = rospack.get_path('ergonomic_assessment') + '/imu/vertical_vector.txt'
        self.vertical_vector = np.loadtxt(imu_vector)
        if (save_data == 1):
            header = np.array(["time", "rula_score", "rula_base_score", "rula_confidence",
                                        "neck_score", "neck_base_score", "neck_confidence", "neck_angle",
                                        "trunk_score", "trunk_base_score", "trunk_confidence", "trunk_angle",
                                        "legs_score", "legs_base_score", "legs_confidence", "legs_angle",
                                        "upper_arm_left_score", "upper_arm_left_base_score", "upper_arm_left_confidence","upper_arm_left_angle",
                                        "upper_arm_right_score", "upper_arm_right_base_score", "upper_arm_right_confidence", "upper_arm_right_angle",
                                        "lower_arm_left_score", "lower_arm_left_base_score", "lower_arm_left_confidence", "lower_arm_left_angle",
                                        "lower_arm_right_score", "lower_arm_right_base_score", "lower_arm_right_confidence", "lower_arm_right_angle",
                                        "wrist_left_score", "wrist_left_base_score", "wrist_left_confidence", "wrist_left_angle",
                                        "wrist_right_score", "wrist_right_base_score", "wrist_right_confidence", "wrist_right_angle"])
            
            path = rospack.get_path('ergonomic_assessment') + "/data/"      #default file path
            if not os.path.exists(path):     #creates target folder if it doesn't exists
                os.makedirs(path)
            
            #path = "/custom/file/path/"        #alternatively define a custom file path

            self.filename = path + "rula_data_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
            with open(self.filename, 'w') as f:
                    np.savetxt(f, [header], fmt='%s', delimiter=' ')
        
        self.neck = Score(0, 0, 0, 0)
        self.trunk = Score(0, 0, 0, 0)
        self.legs = Score(0, 0, 0, 0)
        self.upper_arm_left = Score(0, 0, 0, 0)
        self.upper_arm_right = Score(0, 0, 0, 0)
        self.lower_arm_left = Score(0, 0, 0, 0)
        self.lower_arm_right = Score(0, 0, 0, 0)
        self.wrist_left = Score(0, 0, 0, 0)
        self.wrist_right = Score(0, 0, 0, 0)
        self.score_a = Score(0, 0, 0, 0)
        self.score_b = Score(0, 0, 0, 0)
        self.rula_score = Score(0, 0, 0, 0)
        rospy.Subscriber("/kinect2/body_tracking_data", MarkerArray, self.callback)
        self.pub = rospy.Publisher('/ergonomics/rula', Float32MultiArray, queue_size=1)
        self.pub_joint_angles = rospy.Publisher('/ergonomics/joint_angles', Float32MultiArray, queue_size=1)

    def createMultiArray(self,data):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "RULA Scores"
        dim.stride = 1
        dim.size = len(data)
        msg.layout.dim.append(dim)
        msg.data = data
        return msg       

    def callback(self, data):
        if (data.markers):
            #get all necessary joints
            closest_body = {"id": 0, "distance": 999999}

            # if multiple people are in the camera image, look for the body with the closest distance
            for marker in data.markers:
                body_id = floor(marker.id / 100)
                joint_index = marker.id % 100  # https://docs.microsoft.com/de-de/azure/kinect-dk/body-joints

                if (joint_index == 2):  # SPINE_CHEST
                    distance = sqrt(marker.pose.position.x**2 + marker.pose.position.y**2 + marker.pose.position.z**2)  # euclidian distance to the camera
                    if (distance < closest_body["distance"]):
                        closest_body["id"] = body_id
                        closest_body["distance"] = distance

            hip_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 18), None)
            hip_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 22), None)
            shoulder_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 5), None)
            shoulder_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 12), None)
            pelvis = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 0), None)
            spine_chest = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 2), None)
            neck = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 3), None)
            head = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 26), None)
            nose = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 27), None)
            ear_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 29), None)
            ear_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 31), None)
            knee_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 19), None)
            knee_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 23), None)
            ankle_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 20), None)
            ankle_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 24), None)
            elbow_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 6), None)
            elbow_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 13), None)
            wrist_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 7), None)
            wrist_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 14), None)
            hand_left = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 8), None)
            hand_right = next((x for x in data.markers if floor(x.id/100) == closest_body["id"] and x.id % 100 == 15), None)

            #calculate individual scores
            self.neck = neck_evaluation(spine_chest, neck, nose, ear_left, ear_right, shoulder_left, shoulder_right)
            self.trunk = trunk_evaluation(hip_left, hip_right, shoulder_left, shoulder_right, pelvis, spine_chest, neck, self.vertical_vector)
            self.legs = leg_evaluation(hip_left, hip_right, knee_left, knee_right, ankle_left, ankle_right)
            upper_arms = upper_arm_evaluation(elbow_left, elbow_right, shoulder_left, shoulder_right, head, neck, self.vertical_vector)
            self.upper_arm_left = upper_arms[0]
            self.upper_arm_right = upper_arms[1]
            lower_arms = lower_arm_evaluation(elbow_left, elbow_right, shoulder_left, shoulder_right, wrist_left, wrist_right)
            self.lower_arm_left = lower_arms[0]
            self.lower_arm_right = lower_arms[1]
            wrists = wrist_evaluation(elbow_left, elbow_right, wrist_left, wrist_right, hand_left, hand_right)
            self.wrist_left = wrists[0]
            self.wrist_right = wrists[1]
            
            #put scores through tables to get the final RULA score
            a = table_a(max(self.lower_arm_left.score, self.lower_arm_right.score, 1), max(self.wrist_left.score, self.wrist_right.score, 1), max(self.upper_arm_left.score, self.upper_arm_right.score, 1))
            a_base = table_a(max(self.lower_arm_left.base_score, self.lower_arm_right.base_score, 1), max(self.wrist_left.base_score, self.wrist_right.base_score, 1), max(self.upper_arm_left.base_score, self.upper_arm_right.base_score, 1))
            a_conf = calc_conf_avg([self.lower_arm_left.confidence, self.lower_arm_right.confidence, self.upper_arm_left.confidence, self.upper_arm_right.confidence])      #confidence of both wrists not included since they always return 1
            self.score_a = Score(a, a_conf, a_base, None)
            b = table_b(max(self.neck.score, 1), max(self.legs.score, 1), max(self.trunk.score, 1))
            b_base = table_b(max(self.neck.base_score, 1), max(self.legs.base_score, 1), max(self.trunk.base_score, 1))
            b_conf = calc_conf_avg([self.neck.confidence, self.legs.confidence, self.trunk.confidence])
            self.score_b = Score(b, b_conf, b_base, None)
            self.rula_score = Score(table_c(self.score_a.score, self.score_b.score), calc_conf_avg([self.score_a.confidence, self.score_b.confidence]), table_c(self.score_a.base_score, self.score_b.base_score), None)

            if (conf_variant == 2):     #confidence of both wrists not included since they always return 1
                conf_simple_avg = calc_conf_avg([get_conf(hip_left), get_conf(hip_right), get_conf(shoulder_left), get_conf(shoulder_right), get_conf(pelvis), 
                                            get_conf(spine_chest), get_conf(neck), get_conf(head), get_conf(nose), get_conf(ear_left), get_conf(ear_right),
                                            get_conf(knee_left), get_conf(knee_right), get_conf(ankle_left), get_conf(ankle_right), get_conf(elbow_left), get_conf(elbow_right), 
                                            get_conf(hand_left), get_conf(hand_right)])
                self.rula_score.score = int(round(self.rula_score.base_score * conf_simple_avg))
                self.rula_score.confidence = conf_simple_avg

            print("Score: {} Base: {} Confidence: {}".format(self.rula_score.score, self.rula_score.base_score, self.rula_score.confidence))

            if (save_data == 1):
                time = pelvis.header.stamp.secs + pelvis.header.stamp.nsecs * 10**-9
                rula = np.array([time, self.rula_score.score, self.rula_score.base_score, self.rula_score.confidence,
                                        self.neck.score, self.neck.base_score, self.neck.confidence, self.neck.angle,
                                        self.trunk.score, self.trunk.base_score, self.trunk.confidence, self.trunk.angle,
                                        self.legs.score, self.legs.base_score, self.legs.confidence, self.legs.angle,
                                        self.upper_arm_left.score, self.upper_arm_left.base_score, self.upper_arm_left.confidence, self.upper_arm_left.angle,
                                        self.upper_arm_right.score, self.upper_arm_right.base_score, self.upper_arm_right.confidence, self.upper_arm_right.angle,
                                        self.lower_arm_left.score, self.lower_arm_left.base_score, self.lower_arm_left.confidence, self.lower_arm_left.angle,
                                        self.lower_arm_right.score, self.lower_arm_right.base_score, self.lower_arm_right.confidence, self.lower_arm_right.angle,
                                        self.wrist_left.score, self.wrist_left.base_score, self.wrist_left.confidence, self.wrist_left.angle,
                                        self.wrist_right.score, self.wrist_right.base_score, self.wrist_right.confidence, self.wrist_right.angle])

                with open(self.filename, 'a') as f:
                    np.savetxt(f, [rula], fmt='%1.8f', delimiter=' ')

            score_msg = self.createMultiArray([self.upper_arm_left.score, self.upper_arm_right.score, self.lower_arm_left.score, self.lower_arm_right.score, self.neck.score, self.trunk.score, self.legs.score, self.legs.score, self.wrist_left.score, self.wrist_right.score, self.rula_score.score, self.rula_score.confidence])
            joint_angle_msg = self.createMultiArray([self.upper_arm_left.angle, self.upper_arm_right.angle, self.lower_arm_left.angle, self.lower_arm_right.angle, self.neck.angle, self.trunk.angle, self.legs.angle, self.wrist_left.angle, self.wrist_right.angle])

            self.pub.publish(score_msg)
            self.pub_joint_angles.publish(joint_angle_msg)

def ergonomy_start():
    rospy.init_node('Ergonomy', anonymous=True)
    process = Ergonomy()
    rospy.spin()

if __name__ == '__main__':
    ergonomy_start()
