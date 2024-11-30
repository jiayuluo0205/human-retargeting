import sys, os, cv2
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
from loguru import logger as lgr
from scipy.spatial.transform import Rotation as R

def rvec_to_rmatrix(rvec):
    # Ensure rvec is a numpy array and has the correct shape
    rvec = np.array(rvec).reshape(3)
    
    # Convert the rotation vector to a rotation matrix using SciPy
    rotation_matrix = R.from_rotvec(rvec).as_matrix()
    
    return rotation_matrix
if __name__ == "__main__":
    K_path = 'xarm6_interface/calib/K.npy'
    d_path = 'xarm6_interface/calib/d.npy'
    save_dir = 'xarm6_interface/utils/box_data'

    # Chessboard configuration
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    board = cv2.aruco.CharucoBoard((16, 16), 0.02, 0.015, aruco_dict)

    K=np.load(K_path)
    d=np.load(d_path)

    d=d[:, :8]

    input_files = glob(os.path.join(save_dir, 'color_*.png'))
    input_files.sort()

    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(board, detectorParams=parameters)

    # try to detect the chessboard in the image, then get the pose of the object(or the chessboard, they are fixed linked)
    valid_frame_idx = []
    valid_frame_object_X = []
    for file_idx, i in enumerate(input_files):
        lgr.info(f"Processing {i}")
        frame = cv2.imread(i)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        try:
            c_corners, c_ids, corners, ids = detector.detectBoard(cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY))
            objPoints, imgPoints = board.matchImagePoints(c_corners, c_ids)
            ret, p_rvec, p_tvec = cv2.solvePnP(objPoints, imgPoints, K, d)

            
            if p_rvec is None or p_tvec is None:
                lgr.warning(f"Failed to get pose for {i}")
                continue
            if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
                lgr.warning(f"Failed to get pose for {i}")
                continue
            cv2.drawFrameAxes(frame,
                            K,
                            d,
                            p_rvec,
                            p_tvec,
                            0.1)
            lgr.info(f"frame {file_idx}: {p_rvec}, {p_tvec}")
            valid_frame_idx.append(file_idx)
            X_object = np.eye(4)
            X_object[:3, :3] = rvec_to_rmatrix(p_rvec)
            X_object[:3, 3] = p_tvec.flatten()
            valid_frame_object_X.append(X_object)
        except cv2.error:
            continue

    # now, save these matrices and the valid frame idx
    np.save(os.path.join(save_dir, 'valid_frame_idx.npy'), np.array(valid_frame_idx))
    np.save(os.path.join(save_dir, 'valid_frame_object_X.npy'), np.array(valid_frame_object_X))
    