import socket

def start_target_server(host='0.0.0.0', port=6666):
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Target server listening on {host}:{port}")

    while True:
        # 接受来自本地服务器的连接
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
            print("Connection closed by the sender.")

        # 打印接收到的数据
        print("Received data from sender:")
        print(data.decode('utf-8'))

        # 关闭客户端连接
        client_socket.close()
        print(f"Connection from {client_address} closed.\n")

if __name__ == "__main__":
    start_target_server(host='0.0.0.0', port=6666)
