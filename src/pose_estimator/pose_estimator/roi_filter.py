"""ROI 提纯：配准前剔除支撑面（地面 / 沙面）点。

为什么需要
----------
``pose_estimation_node._scene_metrics`` 把 coverage 定义为
「ROI 场景点中，落在变换后模型表面 ``scene_coverage_dist_thresh`` 内的比例」，
分母是 ROI 的**全部**点。而 ROI 只是 YOLO bbox 按比例外扩后的直裁，
框内任何不属于模型的点都会被算进分母：

  * 水泥地：平整地面照样算分母；
  * 沙子：密集、不规则、紧贴物体，稀释更严重。

支撑面不是目标，却按点数参与了验收，于是「配准其实对、验收不过是分」这种
假阴性会随背景变脏而变多。先把支撑面剔掉再送配准，coverage 反映的就只剩
「物体自己贴合得怎么样」。

判据（三条同时满足才剔除）
--------------------------
1. ``inlier_ratio >= min_inlier_ratio``：确实存在一个主导平面；
2. ``1 - inlier_ratio >= min_keep_ratio``：剔除后还剩得下东西，
   否则说明被拟合出来的「平面」就是物体本身；
3. 平面比其余点更远离相机（相机光学系 +z 是镜头看出去的方向）：
   俯视抓取时支撑面在物体后面。这条用来挡住「把物体朝上的那个面
   当成支撑面删掉」的错误。

纯 numpy 实现，不依赖 ROS / Open3D，可以离线单测。
"""

import numpy as np

# 少于这个点数，三点采样和平面拟合都没有统计意义
_MIN_POINTS = 30


def _fit_plane_least_squares(points: np.ndarray) -> np.ndarray:
    """最小二乘平面 a*x + b*y + c*z + d = 0，返回单位法向 + d。"""
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    normal = vt[-1]
    norm = np.linalg.norm(normal)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0, -center[2]], dtype=np.float64)
    normal = normal / norm
    d = -float(normal @ center)
    return np.array([normal[0], normal[1], normal[2], d], dtype=np.float64)


def fit_dominant_plane(points: np.ndarray, dist_thresh: float = 0.005,
                       max_iter: int = 200, seed: int = 0):
    """RANSAC 找出 ROI 内点数最多的平面。

    返回 ``(coef, inlier_mask, inlier_ratio)``；点太少时 ``coef`` 为 None。
    """
    pts = np.asarray(points, dtype=np.float64)
    n = len(pts)
    if n < _MIN_POINTS:
        return None, np.zeros(n, dtype=bool), 0.0

    rng = np.random.default_rng(seed)
    triples = rng.integers(0, n, size=(int(max_iter), 3))

    p0, p1, p2 = pts[triples[:, 0]], pts[triples[:, 1]], pts[triples[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    lengths = np.linalg.norm(normals, axis=1)
    usable = lengths > 1e-9
    if not usable.any():
        return None, np.zeros(n, dtype=bool), 0.0
    normals[usable] /= lengths[usable, None]
    offsets = -np.einsum('ij,ij->i', normals, p0)

    # (n, iters) 的所有点到所有候选平面的距离，一次算完
    dist = np.abs(pts @ normals.T + offsets[None, :])
    inliers = (dist < float(dist_thresh)) & usable[None, :]
    best = int(np.argmax(inliers.sum(axis=0)))

    coef = np.array([normals[best][0], normals[best][1], normals[best][2],
                     offsets[best]], dtype=np.float64)
    mask = inliers[:, best].copy()

    # 两轮最小二乘重拟合，把 RANSAC 的 3 点解拉回到整个内点集上
    for _ in range(2):
        if int(mask.sum()) < 3:
            break
        coef = _fit_plane_least_squares(pts[mask])
        mask = np.abs(pts @ coef[:3] + coef[3]) < float(dist_thresh)

    ratio = float(mask.mean()) if n else 0.0
    return coef, mask, ratio


def grow_mask(points, mask, margin: float = 0.015):
    """把掩码按距离膨胀，保住贴着前景的物体边缘 / 深度噪声点。

    只按连通域裁剪会把物块边缘的散点，以及被背景判据拒掉的贴边点一并删掉，
    而这些点本身是模型能解释的 —— 删掉反而压低 coverage。
    """
    pts = np.asarray(points, dtype=np.float64)
    mask = np.asarray(mask, dtype=bool)
    if not mask.any() or margin <= 0.0 or len(pts) == 0:
        return mask
    seed = pts[mask]
    try:
        from scipy.spatial import cKDTree
        dist, _ = cKDTree(seed).query(pts, k=1)
    except Exception:  # 无 scipy 时退回分块暴力计算
        dist = np.empty(len(pts), dtype=np.float64)
        for start in range(0, len(pts), 512):
            block = pts[start:start + 512]
            d = np.linalg.norm(block[:, None, :] - seed[None, :, :], axis=2)
            dist[start:start + 512] = d.min(axis=1)
    return mask | (dist <= float(margin))


def remove_support_plane(points, *, dist_thresh: float = 0.005,
                         min_inlier_ratio: float = 0.20,
                         min_keep_ratio: float = 0.25,
                         require_farther: bool = True,
                         max_iter: int = 200, seed: int = 0):
    """剔除 ROI 里的支撑面点。

    参数都是"多保守"的旋钮，默认值对应 5 mm 平面厚度、至少 20% 点共面、
    最多删掉 75% 的点。返回 ``(kept_points, info)``，``info['applied']``
    说明这一帧到底改没改点云，``info['reason']`` 给出原因，方便在 RViz /
    日志里判断是不是被误触发。
    """
    pts = np.asarray(points, dtype=np.float64)
    n = len(pts)
    info = {
        'applied': False,
        'reason': 'too_few_points',
        'inlier_ratio': 0.0,
        'removed': 0,
        'kept': n,
        'z_inlier': float('nan'),
        'z_other': float('nan'),
        'stage': 1,
    }
    if n < _MIN_POINTS:
        return np.asarray(points, dtype=np.float32), info

    coef, mask, ratio = fit_dominant_plane(pts, dist_thresh, max_iter, seed)
    info['inlier_ratio'] = float(ratio)
    if coef is None:
        info['reason'] = 'no_plane'
        return np.asarray(points, dtype=np.float32), info

    if ratio < float(min_inlier_ratio):
        info['reason'] = f'plane_weak({ratio:.2f})'
        return np.asarray(points, dtype=np.float32), info

    if (1.0 - ratio) < float(min_keep_ratio):
        info['reason'] = 'plane_is_object'
        return np.asarray(points, dtype=np.float32), info

    if require_farther:
        z_inlier = float(np.median(pts[mask, 2]))
        z_other = float(np.median(pts[~mask, 2]))
        info['z_inlier'] = z_inlier
        info['z_other'] = z_other
        if z_inlier <= z_other:
            # 主导平面比其余点更靠近相机 —— 多半是物体朝上的那个面。
            # 但不能就此放弃：支撑面往往还在后面，只是点数没它多
            # （放宽 bbox 后物体顶面常常占 55%，沙子只占 45%）。
            # 先把这张脸排除，再在剩下的点里找一次支撑面。
            rest = pts[~mask]
            coef2, mask_rest, ratio2 = fit_dominant_plane(rest, dist_thresh, max_iter, seed)
            info['stage1_ratio'] = float(ratio)
            info['stage2_ratio'] = float(ratio2)
            if coef2 is not None and ratio2 >= float(min_inlier_ratio):
                z2_inlier = float(np.median(rest[mask_rest, 2]))
                z2_other = float(np.median(rest[~mask_rest, 2])) if (~mask_rest).any() else float('nan')
                # 支撑面与物体朝上的面平行，且比它更远。拿这两条判第二张平面，
                # 侧脸（与顶面垂直）就不会被误当成支撑面删掉。
                parallel = abs(float(np.dot(coef[:3], coef2[:3]))) > 0.9
                info['stage2_z_inlier'] = z2_inlier
                info['stage2_z_other'] = z2_other
                info['stage2_parallel'] = parallel
                if parallel and z2_inlier > z_inlier:
                    far_idx = np.flatnonzero(~mask)[mask_rest]
                    if (n - len(far_idx)) / float(n) < float(min_keep_ratio):
                        # 把第二张平面也删掉之后剩不下东西：那是物体自己
                        info['reason'] = 'plane_is_object'
                        return np.asarray(points, dtype=np.float32), info
                    info['stage'] = 2
                    kept = pts[np.setdiff1d(np.arange(n), far_idx, assume_unique=True)]
                    info.update(
                        applied=True,
                        reason='removed_support_plane_stage2',
                        removed=int(len(far_idx)),
                        kept=int(len(kept)),
                        inlier_ratio=float(ratio2),
                        z_inlier=z2_inlier,
                        z_other=z2_other,
                    )
                    return kept.astype(np.float32), info
            # 剩下的点里也没有更远的平面 —— 认为整块就是物体自己
            info['reason'] = 'plane_in_front'
            return np.asarray(points, dtype=np.float32), info

    kept = pts[~mask]
    info.update(
        applied=True,
        reason='removed_support_plane',
        removed=int(mask.sum()),
        kept=int(len(kept)),
    )
    return kept.astype(np.float32), info
