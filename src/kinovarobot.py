"""
  @File: kinovarobot.py
  @Brief: Control program of j6n2s300, Kinova Robot, percent command to test gripper goals, cartesian position to test cartesian goals for arm
 
  @Author: Benxiaogu
  @Github: https://github.com/Benxiaogu
  @CSDN: https://blog.csdn.net/weixin_51995147?type=blog
 
  @Date: 2024-12-19
"""


import roslib; roslib.load_manifest('kinova_demo')
import rospy
import numpy as np
import math
import time

from detector import Detector

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg


class KinovaRobot:
    """
        kinova 机械臂控制， 此程序中采用笛卡尔坐标进行控制机械臂
    """
    def __init__(self,kinova_robotType) -> None:
        rospy.init_node(kinova_robotType)
        rospy.loginfo("init_node successfully")
        self.kinova_robotType = kinova_robotType
        # self.kinova_robotType = 'j2n6s300'
        self.prefix = self.kinova_robotType + "_"
        robot_category = kinova_robotType[0]
        robot_category_version = int(kinova_robotType[1])
        wrist_type = kinova_robotType[2]
        self.arm_joint_number = int(kinova_robotType[3])
        self.finger_number = int(kinova_robotType[5])
        self.finger_maxDist = 18.9/2/1000
        self.finger_maxTurn = 6800
        self.currentFingerPosition = [0.0, 0.0, 0.0]
        # self.currentCartesianCommand = [0.21258243918418884, -0.25638914108276367, 0.50766521692276, 1.648742437362671, 1.1138312816619873, 0.50766521692276] # default home in unit mq
        self.currentCartesianCommand = [0.2104809731245041, -0.25873029232025146, 0.5095799565315247, 1.6373136043548584, 1.1021580696105957, 0.5095799565315247]
        # self.homePositionMdeg = [0.2104809731245041, -0.25873029232025146, 0.5095799565315247, 81.040, 83.972, 11.606]
        self.init_pose = [0.36157482862472534, 0.10528099536895752, 0.5518933534622192, 81.040, 83.972, 11.606]
        self.getcurrentCartesianCommand()
        self.action_address_arm = '/' + self.prefix + 'driver/pose_action/tool_pose'
        self.client_arm = actionlib.SimpleActionClient(self.action_address_arm, kinova_msgs.msg.ArmPoseAction)
        self.client_arm.wait_for_server()
        self.goal_arm = kinova_msgs.msg.ArmPoseGoal()
        self.goal_arm.pose.header = std_msgs.msg.Header(frame_id=(self.prefix + 'link_base'))
        rospy.loginfo("arm service connect successfully")
        # self.arm_run(pose_target=self.homePositionMdeg)

        self.getCurrentFingerPosition()
        self.action_address_finger = '/' + self.prefix + 'driver/fingers_action/finger_positions'
        self.client_finger = actionlib.SimpleActionClient(self.action_address_finger, kinova_msgs.msg.SetFingersPositionAction)
        self.client_finger.wait_for_server()
        self.goal_finger = kinova_msgs.msg.SetFingersPositionGoal()
        rospy.loginfo("finger service connect successfully")
        self.finger_run(finger_target=[5,5,5])

        # self.observe = [0.3975765824317932, -0.09243141114711761, 0.223219232559204, 57.052, 89.704, 36.111]
        # self.observe_up = [0.3975765824317932, -0.09243141114711761, 0.443219232559204, 57.052, 89.704, 36.111]

        self.arm_run(pose_target=self.init_pose)
        # self.arm_run(pose_target=self.observe)


# translation: 
#   x: 1.3770057834549245
#   y: -0.2412003234934955
#   z: 1.1389934196882465
# rotation: 
#   x: -0.6623319419347857
#   y: -0.49721831394149835
#   z: 0.23229646282231145
#   w: 0.5100281368065014

        # qw = 0.9253157381720597
        # qx = 0.06627723386580903
        # qy = 0.3725719868848615
        # qz = 0.024253402871750047
        # tx = 0.3736087834130508
        # ty = -0.50307384193553
        # tz = 0.5675200940448776
        # qw= 0.45429706054764263
        # qx= -0.535903911049248
        # qy= 0.5277709785169646
        # qz= -0.47736670719179514
        # tx =-0.1502858702037763
        # ty =-0.021521096750382464
        # tz =1.0976953673920185
        # R = self.quaternion_to_rotation_matrix(qw,qx,qy,qz)
        # self.kinectA2kinova_matrix = np.eye(4)  # Initialize a 4x4 identity matrix
        # self.kinectA2kinova_matrix[:3, :3] = R  # Set rotation
        # self.kinectA2kinova_matrix[:3, 3] = [tx, ty, tz]
        # self.kinectA2kinova_matrix = np.array([[ 0.12635357 , 0.60785586 ,-0.78392986 , 1.50248239],
        #                                         [ 0.99040387, -0.12190675,  0.06510701, -0.03760981],
        #                                         [-0.05599067, -0.78463367, -0.61742615,  0.96636926],
        #                                         [ 0.        ,  0.        ,  0.        ,  1.        ]])
        
        # self.kinectA2kinova_matrix = np.array([[0.3976246, 0.42169168, -0.8149054, 1.37700578],
        #                                         [0.89560261,  0.0147095 ,  0.44461174, -0.24120032],
        #                                         [0.19947593, -0.90661996, -0.37181931,  1.13899342],
        #                                         [0.        ,  0.        ,  0.        ,  1.        ]])
        
        # self.kinectA2kinova_matrix = np.array([[ 0.0680977  , 0.57993979 ,-0.8118082  , 1.31925447],
        #                                         [ 0.99319388,  0.03765974,  0.11021642, -0.00671641],
        #                                         [ 0.09449137, -0.81378842, -0.57342811,  0.88751502],
        #                                         [ 0.        ,  0.        ,  0.        ,  1.        ],])

        self.kinectA2kinova_matrix = np.array([[-0.09811982, -0.50830614,  0.85556845, -0.11490607],
                                                [-0.98824356, -0.05152143, -0.14394517, -0.00222445],
                                                [ 0.11724832, -0.85963388, -0.49727499,  1.07570852],
                                                [ 0. ,         0.  ,        0.   ,       1.        ]])

        # self.kinectB2kinova_matrix = np.array([[0]])

        self.realsense2kinova_matrix = np.array([
                                                [-0.81552114, -0.02233797, -0.57829602,  1.37482036],
                                                [ 0.06292209, -0.99675352, -0.05023179, -0.04230057],
                                                [-0.57529651, -0.07735268,  0.81427912,  0.84682352],
                                                [ 0.        ,  0.        ,  0.        ,  1.        ]
                                                ])


    def quaternion_to_rotation_matrix(self, qw, qx, qy, qz):
        """Convert quaternion to a 3x3 rotation matrix."""
        r00 = 1 - 2 * (qy**2 + qz**2)
        r01 = 2 * (qx * qy - qz * qw)
        r02 = 2 * (qx * qz + qy * qw)
        r10 = 2 * (qx * qy + qz * qw)
        r11 = 1 - 2 * (qx**2 + qz**2)
        r12 = 2 * (qy * qz - qx * qw)
        r20 = 2 * (qx * qz - qy * qw)
        r21 = 2 * (qy * qz + qx * qw)
        r22 = 1 - 2 * (qx**2 + qy**2)
        return np.array([[r00, r01, r02],
                        [r10, r11, r12],
                        [r20, r21, r22]])



    def arm_run(self,unit='mdeg',pose_target=None,relative=False):
        pose_mq, pose_mdeg, pose_mrad = self.unitParser_arm(unit,pose_target,relative)
        try:
            self.poses = [float(n) for n in pose_mq]
            self.cartesian_pose_client(self.poses[:3], self.poses[3:])
            print('Cartesian pose sent!')
        except rospy.ROSInterruptException:
            print("program interrupted before completion")

    def finger_run(self,unit='percent',finger_target=None,relative=False):
        finger_turn, finger_meter, finger_percent = self.unitParser_finger(unit, finger_target, relative)
        try:
            if self.finger_number == 0:
                print('Finger number is 0, check with "-h" to see how to use this node.')
                self.positions = []  # Get rid of static analysis warning that doesn't see the exit()
                exit()
            else:
                positions_temp1 = [max(0.0, n) for n in finger_turn]
                positions_temp2 = [min(n, self.finger_maxTurn) for n in positions_temp1]
                self.positions = [float(n) for n in positions_temp2]

            print('Sending finger position ...')
            result = self.gripper_client(self.positions)
            print('Finger position sent!')

        except rospy.ROSInterruptException:
            print('program interrupted before completion')

    def cartesian_pose_client(self, position, orientation):
        """Send a cartesian goal to the action server."""
        self.goal_arm.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        self.goal_arm.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        self.client_arm.send_goal(self.goal_arm)

        if self.client_arm.wait_for_result(rospy.Duration(10.0)):
            return self.client_arm.get_result()
        else:
            self.client_arm.cancel_all_goals()
            print('the cartesian action timed-out')
            return None
        
    def gripper_client(self, finger_positions):
        """Send a gripper goal to the action server."""
        self.goal_finger.fingers.finger1 = float(finger_positions[0])
        self.goal_finger.fingers.finger2 = float(finger_positions[1])
        # The MICO arm has only two fingers, but the same action definition is used
        if len(finger_positions) < 3:
            self.goal_finger.fingers.finger3 = 0.0
        else:
            self.goal_finger.fingers.finger3 = float(finger_positions[2])
        self.client_finger.send_goal(self.goal_finger)

        # 如果等待结果超时，会取消所有未完成的目标
        if self.client_finger.wait_for_result(rospy.Duration(5.0)):
            return self.client_finger.get_result()
        else:
            self.client_finger.cancel_all_goals()
            rospy.logwarn('the gripper action timed-out')
            return None
        
    def unitParser_arm(self, unit_, pose_value_, relative_):
        """ Argument unit """
        position_ = pose_value_[:3]
        orientation_ = pose_value_[3:]
        print(f"position_:{position_}, orientation_:{orientation_}")

        for i in range(0,3):
            if relative_:
                position_[i] = pose_value_[i] + self.currentCartesianCommand[i]
            else:
                position_[i] = pose_value_[i]

        # print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

        if unit_ == 'mq':
            # 四元数
            if relative_:
                orientation_XYZ = self.Quaternion2EulerXYZ(orientation_)
                orientation_xyz_list = [orientation_XYZ[i] + self.currentCartesianCommand[3+i] for i in range(0,3)]
                orientation_q = self.EulerXYZ2Quaternion(orientation_xyz_list)
            else:
                orientation_q = orientation_

            orientation_rad = self.Quaternion2EulerXYZ(orientation_q)
            orientation_deg = list(map(math.degrees, orientation_rad))

        elif unit_ == 'mdeg':
            # 角度欧拉角
            if relative_:
                orientation_deg_list = list(map(math.degrees, self.currentCartesianCommand[3:]))
                orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
            else:
                orientation_deg = orientation_

            orientation_rad = list(map(math.radians, orientation_deg))
            orientation_q = self.EulerXYZ2Quaternion(orientation_rad)

        elif unit_ == 'mrad':
            # 弧度欧拉角
            if relative_:
                orientation_rad_list =  self.currentCartesianCommand[3:]
                orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
            else:
                orientation_rad = orientation_

            orientation_deg = list(map(math.degrees, orientation_rad))
            orientation_q = self.EulerXYZ2Quaternion(orientation_rad)

        else:
            raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

        pose_mq_ = position_ + orientation_q
        pose_mdeg_ = position_ + orientation_deg
        pose_mrad_ = position_ + orientation_rad

        # print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

        return pose_mq_, pose_mdeg_, pose_mrad_
        
    def QuaternionNorm(self, Q_raw):
        qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
        qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
        qx_ = qx_temp/qnorm
        qy_ = qy_temp/qnorm
        qz_ = qz_temp/qnorm
        qw_ = qw_temp/qnorm
        Q_normed_ = [qx_, qy_, qz_, qw_]
        return Q_normed_


    def Quaternion2EulerXYZ(self, Q_raw):
        Q_normed = self.QuaternionNorm(Q_raw)
        qx_ = Q_normed[0]
        qy_ = Q_normed[1]
        qz_ = Q_normed[2]
        qw_ = Q_normed[3]

        tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
        ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
        tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
        EulerXYZ_ = [tx_,ty_,tz_]
        return EulerXYZ_


    def EulerXYZ2Quaternion(self, EulerXYZ_):
        print("EulerXYZ_:",EulerXYZ_)
        tx_, ty_, tz_ = EulerXYZ_[0:3]
        sx = math.sin(0.5 * tx_)
        cx = math.cos(0.5 * tx_)
        sy = math.sin(0.5 * ty_)
        cy = math.cos(0.5 * ty_)
        sz = math.sin(0.5 * tz_)
        cz = math.cos(0.5 * tz_)

        qx_ = sx * cy * cz + cx * sy * sz
        qy_ = -sx * cy * sz + cx * sy * cz
        qz_ = sx * sy * cz + cx * cy * sz
        qw_ = -sx * sy * sz + cx * cy * cz

        Q_ = [qx_, qy_, qz_, qw_]
        return Q_
    


    def unitParser_finger(self, unit_, finger_value_, relative_):
        """ Argument unit """
        # 根据用户指定的单位将目标值转换为内部表示
        # transform between units
        if unit_ == 'turn':
            # get absolute value
            if relative_:
                finger_turn_absolute_ = [finger_value_[i] + self.currentFingerPosition[i] for i in range(0, len(finger_value_))]
            else:
                finger_turn_absolute_ = finger_value_

            finger_turn_ = finger_turn_absolute_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]

        elif unit_ == 'mm':
            # get absolute value
            finger_turn_command = [x/1000 * self.finger_maxTurn / self.finger_maxDist for x in finger_value_]
            if relative_:
                finger_turn_absolute_ = [finger_turn_command[i] + self.currentFingerPosition[i] for i in range(0, len(finger_value_))]
            else:
                finger_turn_absolute_ = finger_turn_command

            finger_turn_ = finger_turn_absolute_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        elif unit_ == 'percent':
            # get absolute value
            finger_turn_command = [x/100.0 * self.finger_maxTurn for x in finger_value_]
            if relative_:
                finger_turn_absolute_ = [finger_turn_command[i] + self.currentFingerPosition[i] for i in
                                        range(0, len(finger_value_))]
            else:
                finger_turn_absolute_ = finger_turn_command

            finger_turn_ = finger_turn_absolute_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        else:
            raise Exception("Finger value have to be in turn, mm or percent")

        return finger_turn_, finger_meter_, finger_percent_

    def getcurrentCartesianCommand(self):
        # wait to get current position
        topic_address = '/' + self.prefix + 'driver/out/cartesian_command'
        rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand)
        rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
        print('position listener obtained message for Cartesian pose. ')

    def setcurrentCartesianCommand(self, feedback):
        currentCartesianCommand_str_list = str(feedback).split("\n")

        for index in range(0,len(currentCartesianCommand_str_list)):
            temp_str=currentCartesianCommand_str_list[index].split(": ")
            self.currentCartesianCommand[index] = float(temp_str[1])
        # the following directly reading only read once and didn't update the value.
        # self.currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z] 
        # print('currentCartesianCommand in setcurrentCartesianCommand is: ', self.currentCartesianCommand)


    def getCurrentFingerPosition(self):
        # wait to get current position
        # 获取当前夹爪手指的位置
        topic_address = '/' + self.prefix + 'driver/out/finger_position'
        rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, self.setCurrentFingerPosition) # 将接收到的手指位置通过setCurrentFingerPosition存入全局变量
        rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
        print('obtained current finger position ')

    def setCurrentFingerPosition(self,feedback):
        self.currentFingerPosition[0] = feedback.finger1
        self.currentFingerPosition[1] = feedback.finger2
        self.currentFingerPosition[2] = feedback.finger3

    def transform(self, position):
        """
            将目标位置由相机坐标系转换到kinova机器人base坐标系
        """
        # translation = self.kinectA2kinova_matrix[:3, 3]
        # print("translation: ",translation)
        # rotation = self.kinectA2kinova_matrix[:3, :3]
        # print("rotation: ",rotation)
        # pos_camera = np.array([position[0],position[1],position[2]])
        # position_end_effector = np.dot(rotation,pos_camera) + translation
        # print("position_end_effector: ",position_end_effector)
        pos_camera = np.array([position[0],position[1],position[2],1.0])
        position_end_effector = self.kinectA2kinova_matrix.dot(pos_camera)
        # position_end_effector = self.realsense2kinova_matrix.dot(pos_camera)

        position_end_effector = position_end_effector[:3]
        print("position_end_effector: ",position_end_effector)
        

        return position_end_effector
    
    def image_to_arm(self,camera_x,camera_y,camera_z):
        """
            眼在手上，相机坐标系转末端执行器坐标系
        """
        # qingdao 
        x= 0.020816677729270594
        y= 0.06788506740074149
        z= -0.10464690917725247
        translation = np.array([[x],[y],[z]])
        q0= 0.5112811368530545
        q1= -0.4841087597350034
        q2= -0.49895024240575236
        q3= -0.5052513863903056
# 
        R = np.array([[-0.00846062,0.99974337,-0.02101445],
                        [-0.03355864,0.02071949,0.99922196],
                        [ 0.99940094,0.00915925,0.03337473]])



        R_1 = np.array([[1-2*q2*q2-2*q3*q3, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2],  
                            [2*q1*q2+2*q0*q3, 1-2*q1*q1-2*q3*q3, 2*q2*q3-2*q0*q1],  
                            [2*q1*q3-2*q0*q2, 2*q2*q2+2*q0*q1, 1-2*q1*q1-2*q2*q2]])
        # R = np.array([[4.56550802e-04 , 9.98906302e-01 , 4.67545876e-02 ], 
        #              [-1.74852234e-02 ,-4.67554189e-02 , 9.98753322e-01 ],
        #              [9.99847018e-01 ,-3.61532781e-04 , 1.74874461e-02 ]]) # 手眼标定转换矩阵
        P_camera = np.array([[camera_x], [camera_y], [camera_z]])
        P_robot = np.dot(R, P_camera) + translation 
        # print("R",R,"R_1",R_1)
        # print(P_robot)
        return P_robot
    

    def verboseParser(self, verbose=False):
        """ Argument verbose """
        position_ = self.poses[:3]
        orientation_q = self.poses[3:]
        if verbose:
            orientation_rad = self.Quaternion2EulerXYZ(orientation_q)
            orientation_deg = list(map(math.degrees, orientation_rad))
            print('Cartesian position is: {}'.format(position_))
            print('Cartesian orientation in Quaternion is: ')
            print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]))
            print('Cartesian orientation in Euler-XYZ(radian) is: ')
            print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]))
            print('Cartesian orientation in Euler-XYZ(degree) is: ')
            print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]))

            finger_turn_ = self.positions
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
            print('Finger values in turn are: ')
            print(', '.join('finger{:1.0f} {:4.0f}'.format(k[0] + 1, k[1]) for k in enumerate(finger_turn_)))
            print('Finger values in mm are: ')
            print(', '.join('finger{:1.0f} {:2.1f}'.format(k[0]+1, k[1]*1000) for k in enumerate(finger_meter_)))
            print('Finger values in percentage are: ')
            print(', '.join('finger{:1.1f} {:3.1f}%'.format(k[0]+1, k[1]) for k in enumerate(finger_percent_)))
        return position_, orientation_deg



if __name__ == "__main__":
    kinova = KinovaRobot("j2n6s300")
    rospy.timer.sleep(3)
    detector = Detector()
    result = detector.detect(camera='k4a',pattern='find',target='bottle',depth=True,range=0.8)
    for name, xyz in result[0].items():
        print(f"xyz: {xyz}")
        # xyz[2]
        xyz_arm = kinova.transform(xyz)
        y_error = xyz_arm[1] * 0.1
        pose_target_1 = [xyz_arm[0]-0.15,xyz_arm[1]+0.06+abs(y_error)*(xyz_arm[1]<0),xyz_arm[2]+0.05+y_error*0.4]
        pose_target_2 = [xyz_arm[0]-0.06,xyz_arm[1]+0.06+abs(y_error)*(xyz_arm[1]<0),xyz_arm[2]+0.05+y_error*0.4]
        pose_target_3 = [xyz_arm[0]-0.15,xyz_arm[1]+0.06+abs(y_error)*(xyz_arm[1]<0),xyz_arm[2]+0.25+y_error*0.4]
        # 81.040, ty 83.972, tz 11.606
        pose_target_1.extend([81.040, 83.972, 11.606])
        print("pose_target_1: ",pose_target_1)
        kinova.arm_run(pose_target=pose_target_1)
        # time.sleep(2)
        pose_target_2.extend([81.040, 83.972, 11.606])
        print("pose_target_2: ",pose_target_2)
        kinova.arm_run(pose_target=pose_target_2)
        time.sleep(1)
        pose_target_3.extend([81.040, 83.972, 11.606])
        print("pose_target_3: ",pose_target_3)
        kinova.finger_run(finger_target=[95,95,95])
        kinova.arm_run(pose_target=pose_target_3)
        kinova.arm_run(pose_target=kinova.init_pose)


















    # for name, xyz in result[0].items():
    #     print("xyz: ",xyz)
    #     xyz_target = kinova.image_to_arm(xyz[0],xyz[2],xyz[1])
    #     # xyz_target = kinova.image_to_arm(xyz[0],xyz[1],xyz[2])
    #     print("xyz_target: ",xyz_target)
    #     euler = [57.052, 89.704, 36.111]
    #     pose_target = [xyz_target[0],xyz_target[1],xyz_target[2]]
    #     pose_target.extend(euler)
    #     pose_, ori_deg = kinova.verboseParser(verbose=True)
    #     pose_target = [xyz_target[0]+pose_[0]-0.08, xyz_target[1]+pose_[1]-0.13,xyz_target[2]+pose_[2]+0.12,euler[0],euler[1],euler[2]]
    #     kinova.arm_run(pose_target=pose_target,relative=False)
    #     time.sleep(2)
    #     kinova.finger_run(finger_target=[95,95,95])
    #     time.sleep(2)
    #     kinova.arm_run(pose_target=kinova.observe_up)





    # # name,xyz = result[0].items()
    #     print("xyz: ",xyz)
    #     xyz_arm = kinova.transform([xyz[0],xyz[1],xyz[2]])

    #     print("xyz_arm: ",type(xyz_arm))
    #     print(xyz_arm.shape)
    #     print(xyz_arm[0],xyz_arm[1],xyz_arm[2])

    #     pose_target_1 = [xyz_arm[0]+0.05,xyz_arm[1]+0.06,xyz_arm[2]]
    #     pose_target_2 = [xyz_arm[0]+0.05,xyz_arm[1]+0.06,xyz_arm[2]-0.2]
    #     # 81.040, ty 83.972, tz 11.606
    #     pose_target_1.extend([81.040, 83.972, 11.606])
    #     print("pose_target_1: ",pose_target_1)
    #     # kinova.arm_run(pose_target=pose_target_1)
    #     # time.sleep(2)
    #     pose_target_2.extend([81.040, 83.972, 11.606])
    #     print("pose_target_2: ",pose_target_2)
        # kinova.arm_run(pose_target=pose_target_2)
        # time.sleep(2)
        # kinova.finger_run(finger_target=[95,95,95])


# 0.48333248496055603, -0.06454424560070038, 0.12640617787837982

    # pose_value = [0, 0, 0, 0, 0, 0]
    # pose_value = [0.21106165647506714, -0.2587187886238098, 0.5096041560173035, 93.819, 63.324, 8.105]
    # pose_value = [0.21106165647506714, -0.2587187886238098, 0.5096041560173035, 0, 0, 0]
    # kinova.arm_run(pose_target=pose_value, relative=True)







    
    
    