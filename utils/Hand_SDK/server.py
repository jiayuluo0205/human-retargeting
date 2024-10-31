import socket
import time

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
        # 轮询接收数据，设置接收超时为 1 秒
        connection.settimeout(1.0)
        try:
            data = connection.recv(1024)
            if data:
                # 解码并打印收到的数据
                print(f"Received: {data.decode('utf-8')}")
            else:
                # 如果接收到空数据，继续轮询
                continue
        except socket.timeout:
            # 超时没有收到数据，继续轮询
            print("No data received, polling again...")
            time.sleep(0.5)  # 稍作延时，避免过于频繁轮询

finally:
    # 关闭连接
    connection.close()