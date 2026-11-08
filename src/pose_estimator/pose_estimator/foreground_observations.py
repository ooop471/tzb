"""Camera-frame, gravity-independent qualification of coarse block observations.

A depth ROI is not an object mask. Never carve a large connected plane into
block-sized pieces: reject dominant oversized components before selecting a
compact foreground component. Centers describe observed surfaces, not inferred
hidden geometry; no half-block ray offset is added.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


@dataclass(frozen=True)
class ForegroundResult:
    center: object
    reason: str
    points: int = 0
    required: int = 0
    extent: tuple = ()


def extract_foreground(points, fx, fy, block_size=0.05,
                       support_fraction=0.08, absolute_floor=12):
    """Return a compact, sufficiently sampled non-planar observed component.

    Connectivity grows with one angular sample at range, capped at 12 mm.
    Two principal extents must be >=20 mm, largest <=120 mm: a 50 mm
    block's visible faces may be partial, tilted, and include edge noise.
    At least 2 mm RMS thickness rejects plane-only support/background, at
    the explicit cost of rejecting a perfectly single-face block view.
    Expected samples = fx*fy*(block_size/z)^2; retain >=8% with a 12-point
    floor. This is a support test, independent of registration's 250 points.
    """
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        return ForegroundResult(None, 'invalid_shape')
    if not (np.isfinite([fx, fy, block_size, support_fraction]).all()
            and fx > 0 and fy > 0 and block_size > 0
            and 0 < support_fraction <= 1 and absolute_floor >= 3):
        return ForegroundResult(None, 'invalid_parameters')
    xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0.001)]
    if len(xyz) < absolute_floor:
        return ForegroundResult(None, 'insufficient_points', len(xyz), absolute_floor)

    voxel = min(0.012, max(0.004, 1.5 * float(np.median(xyz[:, 2])) / min(fx, fy)))
    cells, inverse = np.unique(np.floor(xyz / voxel).astype(np.int64),
                               axis=0, return_inverse=True)
    pairs = cKDTree(cells).query_pairs(2.5, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs), dtype=np.uint8),
                        (pairs[:, 0], pairs[:, 1])), shape=(len(cells), len(cells)))
    count, labels = connected_components(graph, directed=False)
    labels = labels[inverse]
    sizes = np.bincount(labels, minlength=count)
    candidates = []
    rejected = 'no_supported_component'
    for label in np.argsort(sizes)[::-1]:
        if sizes[label] < absolute_floor:
            break
        component = xyz[labels == label]
        # A detector ROI must actually be supported by this object. Do not
        # scavenge a tiny compact distractor from a mostly unrelated large box.
        if len(component) < 0.05 * len(xyz):
            rejected = 'roi_occupancy'
            continue
        center = np.mean(component, axis=0)
        required = max(absolute_floor, int(np.ceil(
            support_fraction * fx * fy * (block_size / center[2]) ** 2)))
        if len(component) < required:
            rejected = 'range_support'
            continue
        # Rotation-invariant metric shape, not camera/gravity-axis bounds.
        eigenvalues, axes = np.linalg.eigh(np.cov(component.T))
        principal = (component - center) @ axes
        extent = np.quantile(principal, .98, axis=0) - np.quantile(principal, .02, axis=0)
        ordered = np.sort(extent)
        oversized = ordered[-1] > 0.12
        if oversized and len(component) >= 0.5 * len(xyz):
            return ForegroundResult(None, 'dominant_background', len(component),
                                    required, tuple(float(v) for v in ordered))
        if oversized or ordered[-2] < 0.02:
            rejected = 'metric_extent'
            continue
        if np.sqrt(max(0., eigenvalues[0])) < 0.002:
            rejected = 'planar_background'
            continue
        candidates.append(ForegroundResult(center, 'accepted', len(component),
                                           required, tuple(float(v) for v in ordered)))
    if not candidates:
        return ForegroundResult(None, rejected)
    # Nearest supported compact object; tiny near-depth speckles fail support/extent.
    return min(candidates, key=lambda result: float(result.center[2]))
