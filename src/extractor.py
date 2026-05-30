import cv2
import numpy as np
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform


def add_ones(x):
    return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)


IRt = np.eye(4)


def extractPose(F):
    W = np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]])

    U, d, Vt = np.linalg.svd(F)

    if np.linalg.det(U) < 0:
        U *= -1
    if np.linalg.det(Vt) < 0:
        Vt *= -1

    R = U @ W @ Vt
    if np.sum(R.diagonal()) < 0:
        R = U @ W.T @ Vt

    t = U[:, 2]

    ret = np.eye(4)
    ret[:3, :3] = R
    ret[:3, 3] = t

    return ret


def extract(img):
    orb = cv2.ORB_create()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    pts = cv2.goodFeaturesToTrack(
        gray,
        20000,           
        qualityLevel=0.003,  
        minDistance=5   
    )

    if pts is None:
        return np.array([]), None

    pts = pts.reshape(-1, 2)

    h, w = gray.shape

    filtered_pts = []
    right_pts = []

    for x, y in pts:
        if y < 0.4 * h:
            continue

        filtered_pts.append([x, y])

        if x >= w // 2:
            right_pts.append([x, y])

    MIN_RIGHT = 300

    if len(right_pts) < MIN_RIGHT:
        for x, y in pts:
            if x >= w // 2 and y >= 0.3 * h:
                right_pts.append([x, y])
            if len(right_pts) >= MIN_RIGHT:
                break

        filtered_pts.extend(right_pts)

    pts = np.array(filtered_pts, dtype=np.float32)

    if len(pts) == 0:
        return np.array([]), None

    # ORB descriptors
    kps = [cv2.KeyPoint(x, y, 20) for x, y in pts]
    kps, des = orb.compute(gray, kps)

    if des is None or len(kps) == 0:
        return np.array([]), None

    return np.array([kp.pt for kp in kps]), des


def normalize(Kinv, pts):
    return (Kinv @ add_ones(pts).T).T[:, 0:2]


def denormalize(K, pt):
    ret = K @ [pt[0], pt[1], 1.0]
    ret /= ret[2]
    return int(round(ret[0])), int(round(ret[1]))


def match_frames(f1, f2):
    if f1.des is None or f2.des is None:
        return np.array([]), np.array([]), IRt

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(f1.des, f2.des, k=2)

    ret = []
    idx1, idx2 = [], []

    # Lowe's ratio test 
    for m, n in matches:
        if m.distance < 0.85 * n.distance:
            p1 = f1.pts[m.queryIdx]
            p2 = f2.pts[m.trainIdx]

            idx1.append(m.queryIdx)
            idx2.append(m.trainIdx)
            ret.append((p1, p2))

    if len(ret) < 8:
        return np.array([]), np.array([]), IRt

    ret = np.array(ret)
    idx1 = np.array(idx1)
    idx2 = np.array(idx2)

    # Spatial balancing 
    pts1_px = np.array([denormalize(f1.K, p) for p in ret[:, 0]])

    w = f1.K[0, 2] * 2  # image width approx

    left_mask = pts1_px[:, 0] < w / 2
    right_mask = pts1_px[:, 0] >= w / 2

    # randomly subsample left
    if np.sum(right_mask) > 0:
        keep_mask = right_mask.copy()

        # allow some left, but not dominate
        left_indices = np.where(left_mask)[0]
        np.random.shuffle(left_indices)
        keep_left = left_indices[:len(right_mask) * 2]  # allow 2x left

        keep_mask[keep_left] = True

        ret = ret[keep_mask]
        idx1 = idx1[keep_mask]
        idx2 = idx2[keep_mask]

    # Essential Matrix
    E, mask = cv2.findEssentialMat(
        ret[:, 0], ret[:, 1],
        cameraMatrix=f1.K,
        method=cv2.RANSAC,
        prob=0.999,
        threshold=5.0
    )

    if E is None:
        return np.array([]), np.array([]), IRt

    pts1 = ret[:, 0]
    pts2 = ret[:, 1]

    _, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, f1.K)

    Rt = np.eye(4)
    Rt[:3, :3] = R
    Rt[:3, 3] = t[:, 0]

    # Apply inlier mask
    inliers = mask_pose.ravel().astype(bool)

    if len(inliers) != len(idx1):
        return np.array([]), np.array([]), IRt

    idx1 = idx1[inliers]
    idx2 = idx2[inliers]

    # Final safety
    if len(idx1) < 8:
        return np.array([]), np.array([]), IRt

    return idx1, idx2, Rt

class Frame(object):
    def __init__(self, mapp, img, K):
        self.K = K
        self.Kinv = np.linalg.inv(self.K)
        self.pose = IRt.copy()

        self.id = len(mapp.frames)
        mapp.frames.append(self)

        pts, self.des = extract(img)

        if self.des is not None and len(pts) > 0:
            self.pts = normalize(self.Kinv, pts)
        else:
            self.pts = np.array([])