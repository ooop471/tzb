"""ROI 提纯的离线单测：用合成点云复现「沙子 / 水泥地把 coverage 稀释」这一现象。"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_estimator.roi_filter import fit_dominant_plane, remove_support_plane


def _grid_plane(z, size=0.16, step=0.002, noise=0.0, seed=1):
    """相机前方 z 处的水平面片（光学系 +z 朝远离相机方向）。"""
    lin = np.arange(-size / 2, size / 2, step)
    xx, yy = np.meshgrid(lin, lin)
    pts = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, z)], axis=1)
    if noise > 0:
        rng = np.random.default_rng(seed)
        pts = pts + rng.normal(0.0, noise, pts.shape)
    return pts


def _cube_surface(center, edge=0.05, step=0.002):
    """立方体表面点（六面都采），代表模型对应的真实物体。"""
    half = edge / 2.0
    lin = np.arange(-half, half, step)
    a, b = np.meshgrid(lin, lin)
    pts = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            face = np.zeros((a.size, 3))
            other = [i for i in range(3) if i != axis]
            face[:, other[0]] = a.ravel()
            face[:, other[1]] = b.ravel()
            face[:, axis] = sign * half
            pts.append(face)
    return np.concatenate(pts, axis=0) + np.asarray(center, dtype=float)


def _scene(plane_z=0.66, plane_noise=0.002, size=0.16, box_gap=0.003):
    """支撑面 + 贴着它放的 5 cm 方块；返回 (点云, 平面掩码, 方块掩码)。"""
    plane = _grid_plane(z=plane_z, size=size, noise=plane_noise)
    # 方块底面离地 box_gap，顶面朝向相机
    center_z = plane_z - box_gap - 0.025
    cube = _cube_surface(center=(0.0, 0.0, center_z))
    pts = np.concatenate([plane, cube], axis=0).astype(np.float32)
    is_plane = np.zeros(len(pts), dtype=bool)
    is_plane[:len(plane)] = True
    return pts, is_plane, ~is_plane


class SupportPlaneRemovalTest(unittest.TestCase):
    def test_rough_sand_like_plane_is_removed_and_block_survives(self):
        pts, is_plane, _ = _scene(plane_noise=0.002)

        kept, info = remove_support_plane(pts)

        self.assertTrue(info['applied'], info)
        self.assertGreater(info['inlier_ratio'], 0.4)

        # 剔除后剩下的点，绝大多数必须落在方块上（沙子不能还占着分母）
        from scipy.spatial import cKDTree
        cube = pts[~is_plane].astype(float)
        d, _ = cKDTree(cube).query(kept.astype(float), k=1)
        self.assertGreater(float(np.mean(d < 0.01)), 0.9)

    def test_plane_points_are_the_ones_flagged_not_the_block(self):
        pts, is_plane, is_cube = _scene(plane_noise=0.002)

        _, mask, _ = fit_dominant_plane(pts)

        self.assertGreater(float(mask[is_plane].mean()), 0.9)      # 沙子被判为支撑面
        self.assertLess(float(mask[is_cube].mean()), 0.25)         # 方块基本没被误判

    def test_smooth_floor_is_removed_too(self):
        pts, is_plane, _ = _scene(plane_noise=0.0002)

        _, mask, ratio = fit_dominant_plane(pts)

        self.assertGreater(ratio, 0.5)
        self.assertGreater(float(mask[is_plane].mean()), 0.99)

    def test_does_not_delete_the_object_face_facing_the_camera(self):
        # 只有一个方块、没有支撑面：任何"平面"都是物体自己的面，必须不动
        cube = _cube_surface(center=(0.0, 0.0, 0.635)).astype(np.float32)

        kept, info = remove_support_plane(cube)

        self.assertFalse(info['applied'])
        self.assertEqual(len(kept), len(cube))

    def test_plane_in_front_of_object_is_not_treated_as_support(self):
        # 平面比物体更靠近相机（比如前方挡板）：不是支撑面，不能删
        front = _grid_plane(z=0.50, size=0.20, noise=0.0005)
        cube = _cube_surface(center=(0.0, 0.0, 0.70)).astype(np.float32)
        pts = np.concatenate([front, cube], axis=0).astype(np.float32)

        _, info = remove_support_plane(pts)

        self.assertFalse(info['applied'], info)
        self.assertEqual(info['reason'], 'plane_in_front')

    def test_scattered_noise_is_left_alone(self):
        rng = np.random.default_rng(7)
        pts = (0.635 + rng.normal(0.0, 0.02, (1200, 3))).astype(np.float32)

        _, info = remove_support_plane(pts)

        self.assertFalse(info['applied'])


class TwoStagePlaneTest(unittest.TestCase):
    """放宽 bbox 后物体顶面点数可能多于支撑面，支撑面要能被二次搜出来。"""

    def test_support_behind_the_object_face_is_still_removed(self):
        face = _grid_plane(z=0.610, size=0.05, noise=0.0005)     # 物体朝上的面（更近）
        support = _grid_plane(z=0.660, size=0.04, noise=0.002)   # 支撑面（更远）
        pts = np.concatenate([face, support], axis=0).astype(np.float32)

        kept, info = remove_support_plane(pts)

        self.assertTrue(info['applied'], info)
        self.assertEqual(info['stage'], 2)
        self.assertGreaterEqual(info['removed'], 0.9 * len(support))
        self.assertLessEqual(len(kept), 1.1 * len(face))

    def test_perpendicular_face_is_not_mistaken_for_support(self):
        top = _grid_plane(z=0.610, size=0.05, noise=0.0005)
        # 侧脸：法向沿 x，与顶面垂直；它在第二级里点数占满剩余，
        # 但因为不平行，不能被当成支撑面删掉。
        lin = np.arange(-0.02, 0.02, 0.002)
        yy, zz = np.meshgrid(lin, lin)
        side = np.column_stack([np.full(yy.size, 0.025), yy.ravel(), (zz + 0.635).ravel()])
        pts = np.concatenate([top, side], axis=0).astype(np.float32)

        kept, info = remove_support_plane(pts)

        self.assertFalse(info['applied'], info)
        self.assertEqual(len(kept), len(pts))


class SandGraspPipelineTest(unittest.TestCase):
    """沙地场景：先剔支撑面，前景提取才能把物块从沙子里摘出来。"""

    def test_plane_removal_then_foreground_purge_yields_the_block(self):
        from pose_estimator.foreground_observations import extract_foreground

        pts, is_plane, is_cube = _scene(plane_noise=0.002)
        cube = pts[is_cube].astype(float)

        # 不做剔除时：整片沙子会把 ROI 连成主导背景，前景提取得不到物块
        raw = extract_foreground(pts, 366.0, 366.0)
        self.assertIsNone(raw.center, raw.reason)

        cleaned, info = remove_support_plane(pts)
        self.assertTrue(info['applied'], info)

        result = extract_foreground(cleaned, 366.0, 366.0)
        self.assertEqual(result.reason, 'accepted')
        self.assertIsNotNone(result.mask)

        purged = cleaned[result.mask]
        # 提纯后剩下来的应当几乎就是物块自己
        from scipy.spatial import cKDTree
        d, _ = cKDTree(cube).query(purged.astype(float), k=1)
        self.assertGreater(float(np.mean(d < 0.01)), 0.9)
        self.assertGreater(len(purged), 0.5 * len(cube))


if __name__ == '__main__':
    unittest.main()
