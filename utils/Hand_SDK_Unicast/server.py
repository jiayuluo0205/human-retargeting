import socket
import time

def start_server(host='0.0.0.0', port=5556):
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
                chunk = client_socket.recv(65536)
                if not chunk:
                    break
                # 每次接收数据就打印
                print("Received data chunk:")
                print(chunk.decode('utf-8'))
                time.sleep(0.1)
        except ConnectionResetError:
            print("Connection closed by the client.")

        # 关闭客户端连接
        client_socket.close()
        print(f"Connection from {client_address} closed.\n")

if __name__ == "__main__":
    start_server()