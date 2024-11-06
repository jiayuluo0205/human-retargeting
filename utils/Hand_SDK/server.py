import socket
import time

# 创建 TCP/IP 套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定到本地端口 5555
server_address = ('0.0.0.0', 5555)
print('Starting up on {} port {}'.format(*server_address))
sock.bind(server_address)

# 开始监听连接
sock.listen(1)

print('Waiting for a connection...')

# 接受连接
connection, client_address = sock.accept()

try:
    print('Connection from', client_address)

    # 持续接收数据
    while True:
        data = ""
        data = connection.recv(65536)
        if data:
            # 解码并打印收到的数据
            print(f"Received: {data.decode('utf-8')}")
            time.sleep(0.1)

finally:
    # 关闭连接
    connection.close()
