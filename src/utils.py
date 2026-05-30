import numpy as np
import os


def read_calibration_file(calib_file_path):
    if not os.path.exists(calib_file_path):
        raise FileNotFoundError(f"Calibration file not found: {calib_file_path}")

    with open(calib_file_path, 'r') as f:
        lines = f.readlines()

    return lines


def extract_intrinsic_matrix(calib_lines, camera_id='P0'):
    for line in calib_lines:
        if line.startswith(camera_id):
            values = line.strip().split()[1:]
            values = [float(val) for val in values]

            if len(values) != 12:
                raise ValueError(f"Invalid calibration format for {camera_id}")

            P = np.array(values).reshape(3, 4)
            K = P[:3, :3]

            return K

    print(f"[WARN] Intrinsic matrix not found for camera ID: {camera_id}")
    return None


def main():
    calib_file_path = "../data/data_odometry_gray/dataset/sequences/00/calib.txt"

    try:
        calib_lines = read_calibration_file(calib_file_path)
        intrinsic_matrix = extract_intrinsic_matrix(calib_lines, camera_id='P0')

        if intrinsic_matrix is not None:
            print("Intrinsic Matrix (K):")
            print(intrinsic_matrix)

    except Exception as e:
        print("[ERROR]", e)


if __name__ == "__main__":
    main()