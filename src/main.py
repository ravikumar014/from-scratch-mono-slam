import cv2
import numpy as np
import open3d as o3d 

from extractor import Frame, denormalize, match_frames, add_ones
from pointmap import Map, Point
from visualizer import Visualizer
from live_3d import Live3D

# Camera intrinsics
W, H = 1920 // 2, 1080 // 2
F = W

K = np.array([[F, 0, W // 2],
              [0, F, H // 2],
              [0, 0, 1]])

# Initialize system
mapp = Map()
viz = Visualizer()
live3d = Live3D()


class Live3D:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="3D Map", width=800, height=600)

        self.pcd = o3d.geometry.PointCloud()
        self.line_set = o3d.geometry.LineSet()

        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(self.line_set)

        self.vis.get_render_option().point_size = 2

    def update(self, points, poses):
        if len(points) == 0:
            return
        
        if len(points) > 10:
            self.vis.reset_view_point(True)

        pts = np.array(points) * 5.0

        # update point cloud
        self.pcd.points = o3d.utility.Vector3dVector(pts[:, :3])

        # update trajectory
        traj = np.array([p[:3, 3] for p in poses])

        if len(traj) > 1:
            lines = [[i, i+1] for i in range(len(traj)-1)]

            self.line_set.points = o3d.utility.Vector3dVector(traj)
            self.line_set.lines = o3d.utility.Vector2iVector(lines)
            self.line_set.colors = o3d.utility.Vector3dVector(
                [[0, 1, 0] for _ in lines]
            )

        self.vis.update_geometry(self.pcd)
        self.vis.update_geometry(self.line_set)

        self.vis.poll_events()
        self.vis.update_renderer()

# Triangulation
def triangulate(pose1, pose2, pts1, pts2):
    ret = np.zeros((pts1.shape[0], 4))

    pose1 = np.linalg.inv(pose1)
    pose2 = np.linalg.inv(pose2)

    for i, p in enumerate(zip(add_ones(pts1), add_ones(pts2))):
        A = np.zeros((4, 4))
        A[0] = p[0][0] * pose1[2] - pose1[0]
        A[1] = p[0][1] * pose1[2] - pose1[1]
        A[2] = p[1][0] * pose2[2] - pose2[0]
        A[3] = p[1][1] * pose2[2] - pose2[1]

        _, _, vt = np.linalg.svd(A)
        ret[i] = vt[3]

    return ret

# Reprojection error
def reprojection_error(pose, pt3d, pt2d, K):
    proj = pose[:3, :] @ pt3d
    proj = K @ proj
    proj /= proj[2]
    return np.linalg.norm(proj[:2] - pt2d)

# Frame Processing
def process_frame(img):
    global live3d

    img = cv2.resize(img, (W, H))
    frame = Frame(mapp, img, K)

    if frame.id == 0:
        return

    f1 = mapp.frames[-1]
    f2 = mapp.frames[-2]

    idx1, idx2, Rt = match_frames(f1, f2)

    if len(idx1) < 20:
        print("[WARN] Weak frame skipped")
        return

    # Pose stabilization
    t = Rt[:3, 3]
    scale = np.linalg.norm(t)

    if scale < 0.001:
        return

    if scale > 5:
        t = t / scale * 2

    Rt[:3, 3] = t

    new_pose = Rt @ f2.pose

    alpha = 0.7

    f1.pose = np.eye(4)

    f1.pose[:3, :3] = (
        alpha * f2.pose[:3, :3] +
        (1 - alpha) * new_pose[:3, :3]
    )

    U, _, Vt = np.linalg.svd(f1.pose[:3, :3])
    f1.pose[:3, :3] = U @ Vt

    f1.pose[:3, 3] = (
        alpha * f2.pose[:3, 3] +
        (1 - alpha) * new_pose[:3, 3]
    )

    f1.pose[:3, :3] = new_pose[:3, :3]

    # Parallax filtering
    parallax = np.linalg.norm(f1.pts[idx1] - f2.pts[idx2], axis=1)
    good = parallax > 0.003
    print("Parallax mean:", np.mean(parallax))

    idx1 = idx1[good]
    idx2 = idx2[good]

    if len(idx1) < 10:
        return

    # Triangulation
    pts4d = triangulate(f1.pose, f2.pose,
                        f1.pts[idx1], f2.pts[idx2])

    pts4d /= pts4d[:, 3:]
    pts4d = pts4d.astype(np.float64)

    valid = (
        (pts4d[:, 2] > 0.05) &
        (pts4d[:, 2] < 300)
    )

    pts4d = pts4d[valid]
    idx1 = idx1[valid]
    idx2 = idx2[valid]

    if len(pts4d) == 0:
        return

    # Add map points
    MAX_POINTS = 8000
    added = 0

    for i, p in enumerate(pts4d):
        if added >= MAX_POINTS:
            break

        err1 = reprojection_error(f1.pose, p, f1.pts[idx1[i]], K)
        err2 = reprojection_error(f2.pose, p, f2.pts[idx2[i]], K)

        if err1 > 100 or err2 > 100:
            continue

        pt = Point(mapp, p)
        pt.add_observation(f1, idx1[i])
        pt.add_observation(f2, idx2[i])

        added += 1

    if len(mapp.points) > 20000:
        mapp.points = mapp.points[-000:]

    # Draw matches
    for pt1, pt2 in zip(f1.pts[idx1], f2.pts[idx2]):
        u1, v1 = denormalize(K, pt1)
        u2, v2 = denormalize(K, pt2)

        cv2.circle(img, (u1, v1), 2, (77, 243, 255))
        cv2.line(img, (u1, v1), (u2, v2), (255, 0, 0))
        cv2.circle(img, (u2, v2), 2, (204, 77, 255))

    # Visualization
    poses = [f.pose for f in mapp.frames]
    pts = [p.pt[:3] for p in mapp.points]
    viz.draw(img, poses, pts)

    live3d.update(pts, poses)

    print(f"[INFO] Matches: {len(idx1)} | Added: {added} | Map: {len(mapp.points)}")

# Main Loop
if __name__ == "__main__":
    cap = cv2.VideoCapture("videos/test_nyc.mp4")

    try:
        while cap.isOpened():
            ret, frame = cap.read()

            print("\n#################  [NEW FRAME]  #################\n")

            if ret:
                process_frame(frame)
            else:
                break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        print("[INFO] Saving GIF...")
        viz.save_gif("slam.gif")
        print("[INFO] Saved slam.gif")