import zmq
import sys
sys.path.append('utils')
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import socket
import re

counter = 0

def main():
    # server (glove)
    host, port = '0.0.0.0', 5559
    glove_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    glove_server_socket.bind((host, port))
    glove_server_socket.listen(1)
    print(f"Server listening on {host}:{port}")
    glove_server_socket.settimeout(10.0)
    glove_client_socket, client_address = glove_server_socket.accept()
    print(f"Connection from {client_address} established.")

    while True:
        while True:
            data = ""
            chunk = glove_client_socket.recv(65536)
            data += chunk.decode("utf-8")

            right_hand_matches = re.findall(r"R\d+:\s(-?\d+\.\d+)", data)
            # print(right_hand_matches)
            # print("***", len(right_hand_matches))
            right_hand_data = [float(value) for value in right_hand_matches[-28:]]
            if len(right_hand_data) == 28:
                break
        right_hand_data = np.array(right_hand_data).reshape((-1, 4))
        print(type(right_hand_data))
        print(right_hand_data.shape)

if __name__ == "__main__":
    main()