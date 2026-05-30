import numpy as np
import cv2
from multiprocessing import Process, Queue

try:
    import pangolin
    import OpenGL.GL as gl
    USE_PANGOLIN = True
except ImportError:
    print("[INFO] Pangolin not found. Using fallback viewer.")
    USE_PANGOLIN = False


class Map(object):
    def __init__(self):
        self.frames = []
        self.points = []
        self.state = None
        self.q = None
        self.q_image = None

    def create_viewer(self):
        self.q = Queue()
        self.q_image = Queue()

        if USE_PANGOLIN:
            p = Process(target=self.viewer_thread, args=(self.q,))
            p.daemon = True
            p.start()
            self.viewer_process = p
        else:
            print("[INFO] Using fallback viewer in main thread")
            self.use_fallback = True

    def viewer_thread(self, q):
        self.viewer_init(1280, 720)
        while True:
            self.viewer_refresh(q)

    def viewer_init(self, w, h):
        pangolin.CreateWindowAndBind('Main', w, h)
        gl.glEnable(gl.GL_DEPTH_TEST)

        self.scam = pangolin.OpenGlRenderState(
            pangolin.ProjectionMatrix(w, h, 420, 420, w//2, h//2, 0.2, 10000),
            pangolin.ModelViewLookAt(0, -10, -8, 0, 0, 0, 0, -1, 0)
        )

        self.handler = pangolin.Handler3D(self.scam)
        self.dcam = pangolin.CreateDisplay()
        self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -w/h)
        self.dcam.SetHandler(self.handler)

        width, height = 480, 270
        self.dimg = pangolin.Display('image')
        self.dimg.SetBounds(0, height / 768., 0.0, width / 1024., 1024 / 768.)
        self.dimg.SetLock(pangolin.Lock.LockLeft, pangolin.Lock.LockTop)

        self.texture = pangolin.GlTexture(width, height, gl.GL_RGB, False, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
        self.image = np.ones((height, width, 3), 'uint8')

    def viewer_refresh(self, q):
        width, height = 480, 270

        if self.state is None or not q.empty():
            self.state = q.get()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        self.dcam.Activate(self.scam)

        gl.glLineWidth(1)
        gl.glColor3f(0.0, 1.0, 0.0)
        pangolin.DrawCameras(self.state[0])

        gl.glPointSize(2)
        gl.glColor3f(1.0, 0.0, 0.0)
        pangolin.DrawPoints(self.state[1])

        if not self.q_image.empty():
            self.image = self.q_image.get()
            self.image = cv2.resize(self.image, (width, height))

        self.texture.Upload(self.image, gl.GL_RGB, gl.GL_UNSIGNED_BYTE)
        self.dimg.Activate()
        self.texture.RenderToViewport()

        pangolin.FinishFrame()


    def viewer_thread_fallback(self, q):
        while True:
            if self.state is None or not q.empty():
                self.state = q.get()

            if self.state is None:
                continue

            poses, pts = self.state

            # Simple 2D top-view projection
            canvas = np.zeros((600, 600, 3), dtype=np.uint8)

            # Draw points
            for p in pts:
                x, z = int(p[0]*10 + 300), int(p[2]*10 + 300)
                cv2.circle(canvas, (x, z), 1, (0, 0, 255), -1)

            # Draw camera trajectory
            for pose in poses:
                x, z = int(pose[0, 3]*10 + 300), int(pose[2, 3]*10 + 300)
                cv2.circle(canvas, (x, z), 2, (0, 255, 0), -1)

            cv2.imshow("Top-View Map (Fallback)", canvas)
            cv2.waitKey(1)


    def display(self):
        poses, pts = [], []

        for f in self.frames:
            poses.append(f.pose)

        for p in self.points:
            pts.append(p.pt)

        poses = np.array(poses)
        pts = np.array(pts)

        if USE_PANGOLIN:
            if self.q is not None:
                self.q.put((poses, pts))
        else:
            self.display_fallback(poses, pts)

    def display_image(self, ip_image):
        if self.q_image is not None:
            self.q_image.put(ip_image)

    def display_fallback(self, poses, pts):
        canvas = np.zeros((600, 600, 3), dtype=np.uint8)

        # draw points
        for p in pts:
            x, z = int(p[0]*10 + 300), int(p[2]*10 + 300)
            if 0 <= x < 600 and 0 <= z < 600:
                cv2.circle(canvas, (x, z), 1, (0, 0, 255), -1)

        # draw trajectory
        for pose in poses:
            x, z = int(pose[0, 3]*10 + 300), int(pose[2, 3]*10 + 300)
            if 0 <= x < 600 and 0 <= z < 600:
                cv2.circle(canvas, (x, z), 2, (0, 255, 0), -1)

        cv2.imshow("Top-View Map", canvas)
        cv2.waitKey(1)


class Point(object):
    def __init__(self, mapp, loc):
        self.frames = []
        self.pt = loc
        self.idxs = []

        self.id = len(mapp.points)
        mapp.points.append(self)

    def add_observation(self, frame, idx):
        self.frames.append(frame)
        self.idxs.append(idx)