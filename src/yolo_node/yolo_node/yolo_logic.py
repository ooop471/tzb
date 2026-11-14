"""Pure (ROS-free, ultralytics-free) helpers for the YOLO detector node.

Kept in a separate module so it can be unit-tested without a ROS graph / model.
Installed next to yolo_detector.py (lib/yolo_node/), so ``import yolo_logic`` works both
when the script is executed by ``ros2 launch`` and from the pytest in ``src/yolo_node/test``.
"""
import re

# ----------------------------------------------------------------------------- states
LOADING_MODEL = 'LOADING_MODEL'
MODEL_FAILED = 'MODEL_FAILED'
WAITING_IMAGE = 'WAITING_IMAGE'
IMAGE_STALE = 'IMAGE_STALE'
INFERENCE_STALLED = 'INFERENCE_STALLED'
CONVERT_ERROR = 'CONVERT_ERROR'
INFERENCE_ERROR = 'INFERENCE_ERROR'
NO_DETECTION = 'NO_DETECTION'
CLASS_FILTERED = 'CLASS_FILTERED'
BELOW_GATE = 'BELOW_GATE'
PUBLISHING = 'PUBLISHING'

ALL_STATES = (LOADING_MODEL, MODEL_FAILED, WAITING_IMAGE, IMAGE_STALE, INFERENCE_STALLED,
              CONVERT_ERROR, INFERENCE_ERROR, NO_DETECTION, CLASS_FILTERED, BELOW_GATE, PUBLISHING)

IMAGE_STALE_SEC = 2.0        # no image for > 2 s after having had images
INFERENCE_STALL_SEC = 3.0    # newest image is > 3 s newer than the last completed inference

_SEP_RE = re.compile(r'[_\-\s]+')


def normalize_class_name(name) -> str:
    """lowercase, '_' and '-' act as a space, whitespace collapsed and stripped.

    'blue_block' == 'Blue-Block' == '  blue   block ' == 'blue block'.
    """
    if name is None:
        return ''
    return _SEP_RE.sub(' ', str(name).strip().lower()).strip()


def names_as_dict(names) -> dict:
    """ultralytics ``model.names`` may be a dict {id: name} or a list; return {int: str}."""
    if names is None:
        return {}
    if isinstance(names, dict):
        return {int(k): str(v) for k, v in names.items()}
    return {i: str(v) for i, v in enumerate(names)}


def resolve_target_class(target_class, names):
    """Match ``target_class`` against model names with normalisation.

    Returns (valid, matched_ids, matched_names). An empty / non-matching target is invalid.
    """
    want = normalize_class_name(target_class)
    d = names_as_dict(names)
    if not want:
        return False, [], []
    ids = sorted(i for i, n in d.items() if normalize_class_name(n) == want)
    return bool(ids), ids, [d[i] for i in ids]


def is_target_class(class_name, target_class_norm: str) -> bool:
    """``target_class_norm`` must already be normalised (see normalize_class_name)."""
    return bool(target_class_norm) and normalize_class_name(class_name) == target_class_norm


def target_stats(detections, target_class_norm: str):
    """detections: iterable of (class_name, confidence).

    Returns (raw_count, target_count, max_target_conf) -- max over TARGET-class boxes only.
    """
    raw = 0
    tgt = 0
    mx = 0.0
    for cname, conf in detections:
        raw += 1
        if is_target_class(cname, target_class_norm):
            tgt += 1
            if conf > mx:
                mx = float(conf)
    return raw, tgt, mx


def normalize_class_list(classes) -> list:
    """Normalise class names, drop empties/duplicates, preserve first-seen order."""
    out = []
    seen = set()
    for name in classes or []:
        norm = normalize_class_name(name)
        if norm and norm not in seen:
            out.append(norm)
            seen.add(norm)
    return out


def resolve_class_list(classes, names):
    """Resolve an ordered class list against model names.

    Returns ``(matched_norm, missing_norm, {norm: [ids]})``; ``matched_norm`` keeps the
    requested priority order and only contains classes the model actually has.
    """
    d = names_as_dict(names)
    id_by_norm = {}
    for i, n in d.items():
        id_by_norm.setdefault(normalize_class_name(n), []).append(i)
    matched, missing = [], []
    for c in normalize_class_list(classes):
        if c in id_by_norm:
            matched.append(c)
        else:
            missing.append(c)
    return matched, missing, id_by_norm


def priority_class(detections, priority_norm, ignored_norm=()):
    """Return ``(class_norm, [det, ...])`` for the first class in *priority* with detections.

    Detections with an empty or ignored class are dropped. ``priority_norm`` and
    ``ignored_norm`` must already be normalised (see :func:`normalize_class_list`).
    """
    ignored = set(ignored_norm or ())
    buckets = {}
    for det in detections or []:
        cname = normalize_class_name(getattr(det, 'class_name', None))
        if not cname or cname in ignored:
            continue
        buckets.setdefault(cname, []).append(det)
    for cname in priority_norm or ():
        if cname in buckets:
            return cname, buckets[cname]
    return None, []


def classify_state(*, model_failed=False, model_ready=False, images_received=0,
                   last_image_age_s=None, inference_lag_s=0.0, last_outcome='none',
                   raw_detections_last=0, target_detections_last=0, published_last=False,
                   image_stale_sec=IMAGE_STALE_SEC, inference_stall_sec=INFERENCE_STALL_SEC) -> str:
    """Map a counters snapshot to exactly one state. Priority is the order below.

    last_outcome: 'none' (no inference attempt finished yet), 'ok', 'convert_error', 'inference_error'.
    inference_lag_s: age of the oldest unfinished work = now - (receive time of the frame waiting in the
                     slot, or pickup time of the frame the worker is processing); 0 when the worker is idle.
    published_last: the gate passed for the last completed inference.
    """
    if model_failed:
        return MODEL_FAILED
    if not model_ready:
        return LOADING_MODEL
    if images_received <= 0:
        return WAITING_IMAGE
    if last_image_age_s is not None and last_image_age_s > image_stale_sec:
        return IMAGE_STALE
    if inference_lag_s > inference_stall_sec:
        return INFERENCE_STALLED
    if last_outcome == 'convert_error':
        return CONVERT_ERROR
    if last_outcome == 'inference_error':
        return INFERENCE_ERROR
    if last_outcome == 'none':
        return WAITING_IMAGE          # image(s) received, first result not yet available
    if raw_detections_last <= 0:
        return NO_DETECTION
    if target_detections_last <= 0:
        return CLASS_FILTERED
    return PUBLISHING if published_last else BELOW_GATE


# ----------------------------------------------------------------------------- telemetry opt-out
def disable_openvino_telemetry(load_backend=None):
    """Process-local opt-out of OpenVINO's GA4 telemetry fork (see yolo_detector.py).

    Replaces ``GA4Backend.send`` with a no-op.  Returns ``(applied, detail)``; never raises and never
    pretends: if the private module / class / attribute is missing, ``applied`` is False and ``detail`` says why.
    ``load_backend`` returns the module that should contain ``GA4Backend`` (injectable for tests).
    """
    try:
        if load_backend is None:
            import importlib
            load_backend = lambda: importlib.import_module('openvino_telemetry.backend.backend_ga4')
        mod = load_backend()
    except Exception as exc:
        return False, f'cannot import openvino_telemetry GA4 backend: {type(exc).__name__}: {exc}'
    cls = getattr(mod, 'GA4Backend', None)
    if cls is None:
        return False, 'openvino_telemetry backend_ga4 has no class GA4Backend (library layout changed)'
    if not callable(getattr(cls, 'send', None)):
        return False, 'GA4Backend has no callable attribute send (library layout changed)'
    try:
        cls.send = lambda self, message: None
    except Exception as exc:
        return False, f'cannot patch GA4Backend.send: {type(exc).__name__}: {exc}'
    return True, 'GA4Backend.send replaced by a no-op'


def pick_target_index(boxes, mode='confidence'):
    """Choose one detection among several target-class detections.

    boxes: list of (confidence, x, y, w, h). Returns the index to keep, or None if empty.
    mode 'confidence' (default): highest confidence. Rationale (real-robot 2026-11-06): at the observe
    pose the gripper is sometimes detected as a large low-confidence 'blue block' (224x87 px, conf
    0.11-0.22) next to the real block (30x38 px, conf ~0.6); the legacy 'area_left_right' rule (two
    largest boxes, then the right-most) kept the gripper and dropped the block.
    """
    if not boxes:
        return None
    if mode == 'confidence':
        return max(range(len(boxes)), key=lambda i: boxes[i][0])
    raise ValueError('unknown selection mode %r' % mode)
