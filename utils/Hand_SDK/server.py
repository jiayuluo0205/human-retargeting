import socket
import time
import numpy as np

# 创建 TCP/IP 套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 绑定到本地端口 5555
server_address = ('127.0.0.1', 5555)
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
        data = connection.recv(6553600)
        if data:
            # 解码并打印收到的数据
<<<<<<< Updated upstream
            print(f"Received: {data.decode('utf-8')}")
            data = data.decode('utf-8')
            print(type(data))
            time.sleep(0.1)
=======
            data_str = data.decode('utf-8')
            # print(f"Received: {data_str}")

            data_list = [x for x in data_str.split(',') if x.strip() != '']
            data_float = list(map(float, data_list))
            data_array = np.array(data_float).reshape(-1, 4)

            print(f"Received: {data_array}")
>>>>>>> Stashed changes

finally:
    # 关闭连接
    connection.close()
    print("connection close")
