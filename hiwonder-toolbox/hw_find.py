import os
import sys
import getopt
import socket

def get_cpu_serial_number():
    """获取树莓派CPU序列号，用于设备唯一标识"""
    # 打开CPU信息文件
    f_cpu_info = open("/proc/cpuinfo")
    # 逐行读取文件内容
    for i in f_cpu_info.readlines():
        # 查找包含"Serial"的行（此行包含CPU序列号）
        if i.find('Serial',0, len(i)) == 0:
            # 提取序列号并格式化：去除换行符→反转字符串→截取前16位→转为大写
            serial_num = i.replace('\n', '')[::-1][0:16].upper()
    return serial_num

if __name__ == "__main__":
    host = '0.0.0.0'    # 服务器监听地址（所有可用接口）
    port = 9027         # 服务器监听端口
    robot_type = "SPIDER"           # 默认机器人类型
    sn = get_cpu_serial_number()    # 获取设备唯一序列号
    
    # 解析命令行参数
    try:
        opts, argsa = getopt.getopt(sys.argv[1:], "ht:a:p:", [])
    except getopt.GetoptError:
        print('test.py -t <robot type>')
        print('example: test.py -t SPIDER')
        sys.exit(2) # 参数格式错误时退出程序
    
    # 处理解析后的命令行参数    
    for opt, arg in opts:
        if opt == '-h': # 显示帮助信息
            print('test.py -t <robot type>')
            print('example: test.py -t SPIDER')
            sys.exit()
        elif opt == '-t':   # 设置机器人类型
            robot_type = arg
        elif opt == '-a':   # 设置监听地址
            host = arg
        elif opt == '-p':   # 设置监听端口（需转换为整数）
            port = int(arg)
        else:
            print(opt)
            print("unknow argument" + "\"" + str(opt) + "\"")
    # 配置UDP服务器
    addr = (host, port)
    udpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    # 创建UDP套接字
    udpServer.bind(addr)    # 绑定地址和端口
    
    # 格式化设备序列号为32位（不足补零）
    sn = (get_cpu_serial_number() + "00000000000000000000000000")[:32]
    # 服务器主循环
    while True:
        # 接收客户端数据（最大1024字节）
        data, addr = udpServer.recvfrom(1024)
        # 将接收到的字节数据解码为字符串
        msg = str(data, encoding = 'utf-8')
        print(msg)   # 打印接收到的消息
        # 响应设备发现请求
        if msg == "LOBOT_NET_DISCOVER":
            udpServer.sendto(bytes(robot_type +":" + sn + "\n", encoding='utf-8'),addr)
   
