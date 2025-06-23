import os, sys
import socketserver
import json

class TCPHandler(socketserver.BaseRequestHandler):
    """TCP请求处理类，负责处理客户端连接和数据"""

    def handle(self):
        """处理客户端请求的主函数"""
        self.request.settimeout(2)  # 设置超时时间为2秒
        count = 0                   # 计数器（未实际使用）
        data = None                 # 存储接收数据的变量
        
        # 循环接收数据直到完成或超时
        while True:
            try:
                if data is None:
                    data = self.request.recv(1024)  # 首次接收数据
                else:
                    data += self.request.recv(1024) # 追加接收后续数据
                if not data:    # 无数据接收时的处理
                    pass
                else:
                    # 将接收到的字节数据转换为UTF-8字符串
                    msg = str(data, encoding='utf-8')
                    try:
                        # 尝试将字符串解析为JSON对象
                        key_dict = json.loads(msg)
                    except:
                        # 解析失败则继续接收数据
                        continue
                    print(key_dict)  # 打印解析后的JSON对象
                    
                    # 处理WiFi设置请求
                    if 'setwifi' in key_dict:
                        key_dict = key_dict['setwifi']  # 获取WiFi设置子字典
                        # 检查是否包含SSID和密码字段
                        if 'ssid' in key_dict and 'passwd' in key_dict:
                            # 构建WiFi配置文件内容
                            buf = "HW_WIFI_MODE = 2\n"  # 设置WiFi模式为STA
                            buf += "HW_WIFI_STA_SSID = \"" + key_dict['ssid'] + '\"\n' # 设置SSID
                            buf += "HW_WIFI_STA_PASSWORD = \"" + key_dict['passwd'] + '\"\n'# 设置密码
                            
                            # 将配置写入文件
                            with open("/etc/Hiwonder/hiwonder_wifi_conf.py", "w") as fp:
                                fp.write(buf)
                            print(buf)  # 打印配置内容
                            # 重启WiFi服务使配置生效
                            os.system("systemctl restart hw_wifi.service")
                    break   # 处理完成后跳出循环
            except:
                # 发生异常（如超时）时跳出循环
                break
        self.request.close()    # 关闭客户端连接

class PhoneServer(socketserver.TCPServer):
    """自定义TCP服务器类"""
    timeout = 2 # 服务器超时时间
    daemon_threads = True       # 服务器超时时间
    allow_reuse_address = True  # 允许地址重用（避免端口占用问题）

    def __init__(self, server_address, RequestHandlerClass):
        """初始化服务器"""
        socketserver.TCPServer.__init__(self, server_address, RequestHandlerClass)

    def handle_timeout(self):
        """处理超时事件"""
        print('Timeout')

if __name__ == "__main__":
    if not os.path.exists("/etc/Hiwonder"):
        os.system("mkdir /etc/Hiwonder")
    server = PhoneServer(('0.0.0.0', 9026), TCPHandler)
    server.serve_forever()
