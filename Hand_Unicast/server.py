import socket

def start_server(host='127.0.0.1', port=5555):
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")

    while True:
        # 接受客户端连接
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address} established.")

        # 接收数据
        try:
            while True:
                # 接收 1024 字节的数据
                chunk = client_socket.recv(1024)
                if not chunk:
                    break
                # 每次接收数据就打印
                print("Received data chunk:")
                print(chunk.decode('utf-8'))
        except ConnectionResetError:
            print("Connection closed by the client.")

        # 关闭客户端连接
        client_socket.close()
        print(f"Connection from {client_address} closed.\n")

if __name__ == "__main__":
    start_server()