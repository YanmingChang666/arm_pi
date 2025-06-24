#!/usr/bin/env python3
import os
import sys
sys.path.append('/home/pi/ArmPi/HiwonderSDK/')
import time
import RPi.GPIO as GPIO
from BusServoCmd import *
from smbus2 import SMBus, i2c_msg
from rpi_ws281x import PixelStrip
from rpi_ws281x import Color as PixelColor

#幻尔科技raspberrypi扩展板sdk#
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# I2C寄存器地址定义
__ADC_BAT_ADDR = 0      # 电池电压检测寄存器地址
__SERVO_ADDR   = 21     # PWM舵机角度控制基址
__MOTOR_ADDR   = 31     # 电机控制基址
__SERVO_ADDR_CMD  = 40  # 舵机命令控制基址

# 设备状态变量
__motor_speed = [0, 0, 0, 0]        # 四个电机的速度值
__servo_angle = [0, 0, 0, 0, 0, 0]  # 六个PWM舵机的角度值
__servo_pulse = [0, 0, 0, 0, 0, 0]  # 六个PWM舵机的脉宽值
__i2c = 1                           # I2C总线编号
__i2c_addr = 0x7A                   # 扩展板的I2C地址

# 初始化GPIO设置
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# RGB灯条配置
__RGB_COUNT = 2         # RGB灯珠数量
__RGB_PIN = 12          # RGB灯条控制引脚
__RGB_FREQ_HZ = 800000  # PWM频率
__RGB_DMA = 10          # DMA通道
__RGB_BRIGHTNESS = 120  # 亮度(0-255)
__RGB_CHANNEL = 0       # PWM通道
__RGB_INVERT = False    # 是否反相
RGB = PixelStrip(__RGB_COUNT, __RGB_PIN, __RGB_FREQ_HZ, __RGB_DMA, __RGB_INVERT, __RGB_BRIGHTNESS, __RGB_CHANNEL)
RGB.begin()
# 初始化RGB灯为关闭状态
for i in range(RGB.numPixels()):
    RGB.setPixelColor(i, PixelColor(0,0,0))
    RGB.show()

def setMotor(index, speed):
    """
    设置电机速度
    
    参数:
        index: 电机编号(1-4)
        speed: 电机速度(-100到100)，负值表示反转
    
    返回:
        实际设置的速度值
    """
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d"%index)
    index = index - 1       # 转换为0-3的索引
    speed = 100 if speed > 100 else speed   # 限制最大速度
    speed = -100 if speed < -100 else speed # 限制最小速度
    speed = -speed          # 反转方向(根据硬件特性可能需要)
    reg = __MOTOR_ADDR + index  # 计算寄存器地址
    # 通过I2C发送速度值
    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, [reg, speed.to_bytes(1, 'little', signed=True)[0]])
        bus.i2c_rdwr(msg)
        __motor_speed[index] = speed    # 更新状态变量
    return __motor_speed[index]
    
def getMotor(index):
    """
    获取电机当前速度
    
    参数:
        index: 电机编号(1-4)
    
    返回:
        电机当前速度值
    """
    if index < 1 or index > 4:
        raise AttributeError("Invalid motor num: %d"%index)
    index = index - 1
    return __motor_speed[index]

def setPWMServoAngle(index, angle):
    """
    设置PWM舵机角度(直接控制)
    
    参数:
        index: 舵机编号(1-6)
        angle: 舵机角度(0-180度)
    
    返回:
        实际设置的角度值
    """
    if servo_id < 1 or servo_id > 6:    # 修正参数名错误: servo_id -> index
        raise AttributeError("Invalid Servo ID: %d"%servo_id)
    index = servo_id - 1    # 转换为0-5的索引

    angle = 180 if angle > 180 else angle   # 限制最大角度
    angle = 0 if angle < 0 else angle       # 限制最小角度

    reg = __SERVO_ADDR + index  # 计算寄存器地址

    # 通过I2C发送角度值
    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, [reg, angle])
        bus.i2c_rdwr(msg)
        __servo_angle[index] = angle     # 更新角度状态
        __servo_pulse[index] = int(((200 * angle) / 9) + 500)   # 计算对应的脉宽值

    return __servo_angle[index]

def setPWMServoPulse(servo_id, pulse = 1500, use_time = 1000):
    """
    设置PWM舵机脉宽(带时间控制)
    
    参数:
        servo_id: 舵机编号(1-6)
        pulse: 脉宽值(500-2500)，对应舵机角度范围
        use_time: 运动时间(0-30000ms)
    
    返回:
        实际设置的脉宽值
    """
    if servo_id< 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d" %servo_id)
    index = servo_id - 1    # 转换为0-5的索引

    # 限制脉宽和时间范围
    pulse = 500 if pulse < 500 else pulse
    pulse = 2500 if pulse > 2500 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    # 构建I2C消息
    buf = [__SERVO_ADDR_CMD, 1] + list(use_time.to_bytes(2, 'little')) + [servo_id,] + list(pulse.to_bytes(2, 'little'))
    # 通过I2C发送命令
    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, buf)
        bus.i2c_rdwr(msg)
        __servo_pulse[index] = pulse    # 更新脉宽状态
        __servo_angle[index] = int((pulse - 500) * 0.09)    # 计算对应的角度值

    return __servo_pulse[index]

def getPWMServoAngle(servo_id):
    if servo_id < 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d"%servo_id)
    index = servo_id - 1
    return __servo_pulse[index]

def getPWMServoPulse(index):
    if servo_id < 1 or servo_id > 6:
        raise AttributeError("Invalid Servo ID: %d"%servo_id)
    index = servo_id - 1
    return __servo_pulse[index]
    
def getBattery():
    ret = 0
    with SMBus(__i2c) as bus:
        msg = i2c_msg.write(__i2c_addr, [__ADC_BAT_ADDR,])
        bus.i2c_rdwr(msg)
        read = i2c_msg.read(__i2c_addr, 2)
        bus.i2c_rdwr(read)
        ret = int.from_bytes(bytes(list(read)), 'little')
    return ret

def setBuzzer(new_state):
    GPIO.setup(31, GPIO.OUT)
    GPIO.output(31, new_state)

def setBusServoID(oldid, newid):
    """
    配置舵机id号, 出厂默认为1
    :param oldid: 原来的id， 出厂默认为1
    :param newid: 新的id
    """
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_ID_WRITE, newid)

def getBusServoID(id=None):
    """
    读取串口舵机id
    :param id: 默认为空
    :return: 返回舵机id
    """
    
    while True:
        if id is None:  # 总线上只能有一个舵机
            serial_servo_read_cmd(0xfe, LOBOT_SERVO_ID_READ)
        else:
            serial_servo_read_cmd(id, LOBOT_SERVO_ID_READ)
        # 获取内容
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ID_READ)
        if msg is not None:
            return msg

def setBusServoPulse(id, pulse, use_time):
    """
    驱动串口舵机转到指定位置
    :param id: 要驱动的舵机id
    :pulse: 位置
    :use_time: 转动需要的时间
    """

    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_TIME_WRITE, pulse, use_time)

def stopBusServo(id=None):
    '''
    停止舵机运行
    :param id:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_MOVE_STOP)

def setBusServoDeviation(id, d=0):
    """
    调整偏差
    :param id: 舵机id
    :param d:  偏差
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_ADJUST, d)

def saveBusServoDeviation(id):
    """
    配置偏差，掉电保护
    :param id: 舵机id
    """
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_WRITE)

time_out = 50
def getBusServoDeviation(id):
    '''
    读取偏差值
    :param id: 舵机号
    :return:
    '''
    # 发送读取偏差指令
    count = 0
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_OFFSET_READ)
        # 获取
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_OFFSET_READ)
        count += 1
        if msg is not None:
            return msg
        if count > time_out:
            return None

def setBusServoAngleLimit(id, low, high):
    '''
    设置舵机转动范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_WRITE, low, high)

def getBusServoAngleLimit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_ANGLE_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_ANGLE_LIMIT_READ)
        if msg is not None:
            count = 0
            return msg

def setBusServoVinLimit(id, low, high):
    '''
    设置舵机电压范围
    :param id:
    :param low:
    :param high:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_VIN_LIMIT_WRITE, low, high)

def getBusServoVinLimit(id):
    '''
    读取舵机转动范围
    :param id:
    :return: 返回元祖 0： 低位  1： 高位
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_LIMIT_READ)
        if msg is not None:
            return msg

def setBusServoMaxTemp(id, m_temp):
    '''
    设置舵机最高温度报警
    :param id:
    :param m_temp:
    :return:
    '''
    serial_serro_wirte_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE, m_temp)

def getBusServoTempLimit(id):
    '''
    读取舵机温度报警范围
    :param id:
    :return:
    '''
    
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
        if msg is not None:
            return msg

def getBusServoPulse(id):
    '''
    读取舵机当前位置
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_POS_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_POS_READ)
        if msg is not None:
            return msg

def getBusServoTemp(id):
    '''
    读取舵机温度
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_TEMP_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_TEMP_READ)
        if msg is not None:
            return msg

def getBusServoVin(id):
    '''
    读取舵机电压
    :param id:
    :return:
    '''
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_VIN_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_VIN_READ)
        if msg is not None:
            return msg

def restBusServoPulse(oldid):
    # 舵机清零偏差和P值中位（500）
    serial_servo_set_deviation(oldid, 0)    # 清零偏差
    time.sleep(0.1)
    serial_serro_wirte_cmd(oldid, LOBOT_SERVO_MOVE_TIME_WRITE, 500, 100)    # 中位

##掉电
def unloadBusServo(id):
    serial_serro_wirte_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE, 0)

##读取是否掉电
def getBusServoLoadStatus(id):
    while True:
        serial_servo_read_cmd(id, LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        msg = serial_servo_get_rmsg(LOBOT_SERVO_LOAD_OR_UNLOAD_READ)
        if msg is not None:
            return msg

setBuzzer(0)
