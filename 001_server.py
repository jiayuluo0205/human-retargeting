import socket
import time
import re

def start_server(host='0.0.0.0', port=5555):
    # 创建 socket 对象
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    print(f"Server listening on {host}:{port}")

    try:
        while True:
            # 接受客户端连接
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address} established.")
            
            try:
                while True:
                    # 从客户端套接字接收数据
                    data = ""  # 清空data确保每次为新的数据包
                    chunk = client_socket.recv(65536)
                    if not chunk:
                        break
                    data += chunk.decode("utf-8")  # 解码并累积接收到的数据
                    
                    # 定义数组用于存储左右手的数据
                    left_hand_data = []
                    right_hand_data = []
                
                    # 匹配左手和右手的所有传感器值
                    left_hand_matches = re.findall(r"L\d+:\s(-?\d+\.\d+)", data)
                    right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)

                    # 将匹配的结果转换为浮点数并存储到数组中
                    left_hand_data = [float(value) for value in left_hand_matches[:28]]
                    right_hand_data = [float(value) for value in right_hand_matches[:28]]
                    
                    # 输出接收的数据
                    print("\n--- New Data Received ---")
                    print("Left Hand Data:", left_hand_data)
                    print("Right Hand Data:", right_hand_data)
                    print("\n--------------------------\n")
                    time.sleep(0.2)
            
            except ConnectionResetError:
                print("Connection closed by the client.")
            
            # 关闭客户端连接
            client_socket.close()
            print(f"Connection from {client_address} closed.\n")
    
    except KeyboardInterrupt:
        print("Server shutting down.")
    finally:
        # 关闭服务器套接字
        server_socket.close()

if __name__ == "__main__":
    start_server()
