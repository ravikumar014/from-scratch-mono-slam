import open3d as o3d
import numpy as np

class Live3D:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="3D SLAM", width=800, height=600)

        self.pcd = o3d.geometry.PointCloud()
        self.traj = o3d.geometry.LineSet()

        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(self.traj)

        self.initialized = False

    def update(self, points, poses):
        if len(points) < 10:
            return

        # POINT CLOUD
        pts = np.array(points)

        # scale for visibility
        pts = pts * 5.0

        self.pcd.points = o3d.utility.Vector3dVector(pts)
        self.pcd.paint_uniform_color([1, 0, 0])  # red

        # TRAJECTORY
        traj_pts = np.array([p[:3, 3] for p in poses])

        if len(traj_pts) > 1:
            lines = [[i, i+1] for i in range(len(traj_pts)-1)]
        else:
            lines = []

        self.traj.points = o3d.utility.Vector3dVector(traj_pts)
        self.traj.lines = o3d.utility.Vector2iVector(lines)
        self.traj.colors = o3d.utility.Vector3dVector([[0,1,0] for _ in lines])  # green

        # RENDER
        if not self.initialized:
            self.vis.reset_view_point(True)
            self.initialized = True

        self.vis.update_geometry(self.pcd)
        self.vis.update_geometry(self.traj)

        self.vis.poll_events()
        self.vis.update_renderer()