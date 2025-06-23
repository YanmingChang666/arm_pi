import os, sys, time
import RPi.GPIO as GPIO

# 定义按键连接的GPIO引脚号
key1_pin = 13  # 重置WiFi按键连接的引脚
key2_pin = 23  # 关机按键连接的引脚

def reset_wifi():
    """重置WiFi配置并重启WiFi服务"""
    # 删除Hiwonder目录下的所有WiFi配置文件，> /dev/null 2>&1用于静默执行
    os.system("rm /etc/Hiwonder/* -rf > /dev/null 2>&1")
    # 重启自定义的WiFi服务，使其重新配置网络连接
    os.system("systemctl restart hw_wifi.service > /dev/null 2>&1")

if __name__ == "__main__":
    # 初始化GPIO设置
    GPIO.setmode(GPIO.BCM)  # 使用BCM编号模式
    # 将两个按键引脚设置为输入模式，并启用上拉电阻(默认高电平)
    GPIO.setup(key1_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(key2_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    # 按键状态和计数器初始化
    key1_pressed = False  # 记录key1是否被按下
    key2_pressed = False  # 记录key2是否被按下
    count = 0  # 长按计时计数器
    
    # 主循环，持续检测按键状态
    while True:
        # 检测key1(WiFi重置按键)是否被按下(低电平)
        if GPIO.input(key1_pin) == GPIO.LOW:
            time.sleep(0.05)     # 消抖延时
            # 再次确认按键是否确实被按下
            if GPIO.input(key1_pin) == GPIO.LOW:
                if key1_pressed == False:   # 首次按下
                    key1_pressed = True
                    count = 0   # 重置计数器
                else:   # 持续按下状态
                    count += 1  # 计数器递增
                    if count == 60: # 约3秒(60*0.05=3秒)长按
                        count = 0
                        key1_pressed = False
                        reset_wifi()# 执行WiFi重置
            else:   # 抖动导致的短暂低电平
                count = 0
                key1_pressed = False
                continue     # 跳过本次循环剩余部分

        # 检测key2(关机按键)是否被按下(低电平)        
        elif GPIO.input(key2_pin) == GPIO.LOW:
            time.sleep(0.05)    # 消抖延时

            # 再次确认按键是否确实被按下
            if GPIO.input(key2_pin) == GPIO.LOW:
                if key2_pressed == False:    # 首次按下
                    key2_pressed = True 
                    count = 0   # 重置计数器
                else:   # 持续按下状态
                    count += 1
                    if count == 60: # 约3秒长按
                        count = 0
                        key2_pressed = False
                        os.system('sudo halt')  # 执行关机命令
            else:   # 抖动导致的短暂低电平
                count = 0
                key2_pressed = False    
                continue    # 跳过本次循环剩余部分
        else:   # 两个按键都未被按下
            count = 0   # 重置计数器
            key1_pressed = False
            key2_pressed = False
            time.sleep(0.05)     # 短暂延时，减少CPU占用
        
