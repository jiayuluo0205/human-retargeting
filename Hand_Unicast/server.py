import socket

def start_server(host='127.0.0.1', port=5555, target_host='172.25.105.244', target_port=6666):
    # 创建本地服务器 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")

    while True:
        # 接受客户端连接（从本地接收数据）
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address} established.")

        # 接收数据
        data = b""
        try:
            while True:
                # 接收 1024 字节的数据
                chunk = client_socket.recv(1024)
                if not chunk:
                    break
                data += chunk
        except ConnectionResetError:
            print("Connection closed by the local client.")

        # 打印接收到的数据
        print("Received data:")
        print(data.decode('utf-8'))

        # 将接收到的数据转发给目标电脑
        send_to_target(data, target_host, target_port)

        # 关闭本地客户端连接
        client_socket.close()
        print(f"Connection from {client_address} closed.\n")

def send_to_target(data, target_host, target_port):
    # 创建用于连接目标电脑的 socket 对象
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as target_socket:
        try:
            target_socket.connect((target_host, target_port))
            print(f"Connected to target at {target_host}:{target_port}")

            # 发送数据到目标电脑
            target_socket.sendall(data)
            print("Data sent to target.")
        except ConnectionRefusedError:
            print("Could not connect to the target computer.")

if __name__ == "__main__":
    # 设置 `target_host` 为目标电脑的 IP 地址，`target_port` 为目标电脑的端口
    start_server(target_host='172.25.105.244', target_port=6666)