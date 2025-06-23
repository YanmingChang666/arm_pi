#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/pi/ArmPi/')
import time
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from ArmIK.InverseKinematics import *
from ArmIK.Transform import getAngle
from mpl_toolkits.mplot3d import Axes3D
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse

#机械臂根据逆运动学算出的角度进行移动
ik = IK('arm')
#设置连杆长度
l1 = ik.l1 + 0.75    # 第一段连杆长度
l4 = ik.l4 - 0.15    # 第四段连杆长度
# 将调整后的连杆长度应用到逆运动学求解器
ik.setLinkLength(L1=l1, L4=l4)

class ArmIK:
    """机械臂逆运动学控制类，用于处理机械臂运动控制和逆运动学求解"""

    # 定义3-6号舵机的脉宽范围和角度范围
    # 格式：(最小脉宽, 最大脉宽, 最小角度, 最大角度)    
    servo3Range = (0, 1000, 0, 240) #脉宽， 角度
    servo4Range = (0, 1000, 0, 240)
    servo5Range = (0, 1000, 0, 240)
    servo6Range = (0, 1000, 0, 240)

    def __init__(self):
        """初始化机械臂控制类，设置舵机参数"""
        self.setServoRange()    # 调用方法设置舵机范围和参数

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        """
        设置舵机参数，适配不同型号的舵机
        
        参数:
            servo3_Range: 3号舵机的脉宽和角度范围
            servo4_Range: 4号舵机的脉宽和角度范围
            servo5_Range: 5号舵机的脉宽和角度范围
            servo6_Range: 6号舵机的脉宽和角度范围
        """
        # 保存各舵机的范围参数
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        # 计算各舵机的脉宽-角度转换系数
        # 公式：(最大脉宽-最小脉宽)/(最大角度-最小角度)
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        """
        将逆运动学计算出的关节角度转换为舵机所需的脉宽值
        
        参数:
            theta3: 3号关节角度
            theta4: 4号关节角度
            theta5: 5号关节角度
            theta6: 6号关节角度
            
        返回:
            包含各舵机脉宽值的字典，若脉宽超出范围则返回False
        """
        # 计算3号舵机脉宽
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        # 检查脉宽是否超出有效范围（+60是安全阈值）
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0] + 60:
            # 记录日志（假设logger已定义）
            logger.info('servo3(%s)超出范围(%s, %s)', servo3, self.servo3Range[0] + 60, self.servo3Range[1])
            return False

        # 计算4号舵机脉宽
        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        # 检查脉宽是否超出有效范围
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            logger.info('servo4(%s)超出范围(%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False

        # 计算5号舵机脉宽（注意角度转换公式与其他舵机不同）
        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 - (90.0 - theta5) * self.servo5Param))
        # 检查脉宽是否超出有效范围
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0])/2 + 90*self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0])/2 - 90*self.servo5Param):
            logger.info('servo5(%s)超出范围(%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False
        
        # 计算6号舵机脉宽（根据角度范围分段计算）
        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param))
        # 检查脉宽是否超出有效范围
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            logger.info('servo6(%s)超出范围(%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def servosMove(self, servos, movetime=None):
        """
        控制3-6号舵机移动到指定位置
        
        参数:
            servos: 包含四个舵机目标脉宽的元组 (servo3, servo4, servo5, servo6)
            movetime: 移动时间(ms)，若未指定则自动计算
            
        返回:
            实际使用的移动时间
        """
        # 短暂延时（确保舵机控制稳定）
        time.sleep(0.02)
         # 如果未指定移动时间，自动计算
        if movetime is None:
            max_d = 0
            # 计算当前脉宽与目标脉宽的最大差值
            for i in  range(0, 4):
                d = abs(getBusServoPulse(i + 3) - servos[i])
                if d > max_d:
                    max_d = d
            # 移动时间 = 最大差值 * 4 (经验系数)
            movetime = int(max_d*4)
        # 控制四个舵机移动到目标位置
        setBusServoPulse(3, servos[0], movetime)
        setBusServoPulse(4, servos[1], movetime)
        setBusServoPulse(5, servos[2], movetime)
        setBusServoPulse(6, servos[3], movetime)

        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da = 1):
        """
        在指定俯仰角范围内寻找机械臂到达目标坐标的解
        
        参数:
            coordinate_data: 目标坐标(x, y, z)，单位cm
            alpha1: 俯仰角范围起始值
            alpha2: 俯仰角范围结束值
            da: 俯仰角遍历步长，默认为1度
            
        返回:
            若找到解，返回(舵机脉宽字典, 俯仰角)；若无解返回False
        """
        #给定坐标coordinate_data和俯仰角的范围alpha1，alpha2, 自动在范围内寻找到的合适的解
        #如果无解返回False,否则返回对应舵机角度,俯仰角
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        #da为俯仰角遍历时每次增加的角度
        x, y, z = coordinate_data   # 解包目标坐标
        # 确保遍历方向正确
        if alpha1 >= alpha2:
            da = -da
        # 遍历俯仰角范围，寻找可行解
        for alpha in np.arange(alpha1, alpha2, da):#遍历求解
            # 使用逆运动学求解器计算关节角度
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                # 提取计算出的关节角度
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                # 将关节角度转换为舵机脉宽
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha    # 返回可行解

        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        """
        在指定俯仰角附近寻找机械臂到达目标坐标的解并控制机械臂移动
        
        参数:
            coordinate_data: 目标坐标(x, y, z)，单位cm
            alpha: 目标俯仰角
            alpha1: 俯仰角搜索范围下限
            alpha2: 俯仰角搜索范围上限
            movetime: 舵机移动时间(ms)，若未指定则自动计算
            
        返回:
            若成功，返回(舵机脉宽字典, 实际俯仰角, 移动时间)；若失败返回False
        """
        #给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置
        #如果无解返回False,否则返回舵机角度、俯仰角、运行时间
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        #alpha为给定俯仰角
        #alpha1和alpha2为俯仰角的取值范围
        #movetime为舵机转动时间，单位ms, 如果不给出时间，则自动计算
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]

        movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)

        return servos, alpha, movetime
    '''
    #for test
    def drawMoveRange2D(self, x_min, x_max, dx, y_min, y_max, dy, z, a_min, a_max, da):
        # 测试可到达点, 以2d图形式展现，z固定
        #测试可到达点, 以3d图形式展现，如果点过多，3d图会比较难旋转
        try:
            for y in np.arange(y_min, y_max, dy):
                for x in np.arange(x_min, x_max, dx):
                    result = self.setPitchRange((x, y, z), a_min, a_max, da)
                    if result:
                        plt.scatter(x, y, s=np.pi, c='r')

            plt.xlabel('X Label')
            plt.ylabel('Y Label')

            plt.show()
        except Exception as e:
            print(e)
            pass

    def drawMoveRange3D(self, x_min, x_max, dx, y_min, y_max, dy, z_min, z_max, dz, a_min, a_max, da):
        #测试可到达点, 以3d图形式展现，如果点过多，3d图会比较难旋转
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        try:
            for z in np.arange(z_min, z_max, dz):
                for y in np.arange(y_min, y_max, dy):
                    for x in np.arange(x_min, x_max, dx):
                        result = self.setPitchRange((x, y, z), a_min, a_max, da)
                        if result:
                            ax.scatter(x, y, z, s=np.pi, c='r')

            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')

            plt.show()
        except Exception as e:
            print(e)
            pass
    '''

if __name__ == "__main__":
    AK = ArmIK()
    setBusServoPulse(1, 200, 500)
    setBusServoPulse(2, 500, 500)
    #AK.setPitchRangeMoving((0, 10, 10), -30, -90, 0, 2000)
    #time.sleep(2)
    print(AK.setPitchRangeMoving((-4.8, 15, 1.5), 0, -90, 0, 2000))
    #AK.drawMoveRange2D(-10, 10, 0.2, 10, 30, 0.2, 2.5, -90, 90, 1)
