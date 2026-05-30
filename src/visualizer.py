import cv2
import numpy as np
import imageio

class Visualizer:
    def __init__(self):
        self.frames = []

    def draw(self, img, poses, pts):
        canvas = np.zeros((600, 1200, 3), dtype=np.uint8)

        # LEFT: Frame view
        frame_small = cv2.resize(img, (400, 300))
        canvas[0:300, 0:400] = frame_small

        # CENTER: Trajectory 
        traj = np.zeros((300, 400, 3), dtype=np.uint8)

        if len(poses) > 0:
            traj_pts = np.array([[p[0,3], p[2,3]] for p in poses])

            xs = traj_pts[:, 0]
            zs = traj_pts[:, 1]

            min_x, max_x = xs.min(), xs.max()
            min_z, max_z = zs.min(), zs.max()

            range_x = max(max_x - min_x, 1e-5)
            range_z = max(max_z - min_z, 1e-5)

            for x, z in traj_pts:
                x_plot = int((x - min_x) / range_x * 380 + 10)
                z_plot = int((z - min_z) / range_z * 280 + 10)

                # flip Z for better top-view orientation
                z_plot = 300 - z_plot

                cv2.circle(traj, (x_plot, z_plot), 2, (0,255,0), -1)

        canvas[0:300, 400:800] = traj

        # RIGHT: Point cloud 
        pc = np.zeros((300, 400, 3), dtype=np.uint8)

        if len(pts) > 0:
            pts_np = np.array(pts)

            xs = pts_np[:, 0]
            zs = pts_np[:, 2]

            min_x, max_x = xs.min(), xs.max()
            min_z, max_z = zs.min(), zs.max()

            range_x = max(max_x - min_x, 1e-5)
            range_z = max(max_z - min_z, 1e-5)

            for x, z in zip(xs, zs):
                x_plot = int((x - min_x) / range_x * 380 + 10)
                z_plot = int((z - min_z) / range_z * 280 + 10)

                # flip Z axis
                z_plot = 300 - z_plot

                cv2.circle(pc, (x_plot, z_plot), 1, (0,0,255), -1)

        canvas[0:300, 800:1200] = pc

        # Store for GIF
        self.frames.append(canvas.copy())

        cv2.putText(canvas, "Frame + Matches", (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.putText(canvas, "Trajectory (Top View)", (420, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.putText(canvas, "3D Map (Top View)", (820, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        cv2.imshow("SLAM Visualization", canvas)
        cv2.waitKey(1)

    def save_gif(self, path="slam.gif"):
        if len(self.frames) == 0:
            print("[WARN] No frames to save for GIF")
            return

        imageio.mimsave(path, self.frames, fps=10)
        print(f"[INFO] GIF saved: {path}")