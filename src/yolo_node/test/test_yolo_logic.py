"""Unit tests for yolo_logic (no ROS graph, no model, no ultralytics).

Run:  cd src/yolo_node && python3 -m pytest -q test
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'yolo_node'))

import pytest  # noqa: E402

import yolo_logic as L  # noqa: E402


# ------------------------------------------------------------------ normalisation
@pytest.mark.parametrize('raw,expected', [
    ('blue block', 'blue block'),
    ('blue_block', 'blue block'),
    ('Blue-Block', 'blue block'),
    ('  BLUE   _ block ', 'blue block'),
    ('blue__block', 'blue block'),
    ('white_block', 'white block'),
    ('', ''),
    (None, ''),
    ('   ', ''),
    ('_-_', ''),
    (123, '123'),
])
def test_normalize(raw, expected):
    assert L.normalize_class_name(raw) == expected


def test_normalize_is_idempotent():
    for s in ('A_b-C  d', 'x', ''):
        assert L.normalize_class_name(L.normalize_class_name(s)) == L.normalize_class_name(s)


# ------------------------------------------------------------------ target resolution
DEFAULT_MODEL_NAMES = {0: 'blue block'}                       # blue_block_320_openvino_model
OLD_MODEL_NAMES = {0: 'blue_block', 1: 'blue block'}          # best_openvino_model/blue_block


def test_resolve_default_model_exact():
    assert L.resolve_target_class('blue block', DEFAULT_MODEL_NAMES) == (True, [0], ['blue block'])


def test_resolve_underscore_matches_space_model():
    # the old node default 'blue_block' used to silently match nothing on the 320 model
    assert L.resolve_target_class('blue_block', DEFAULT_MODEL_NAMES)[0] is True


def test_resolve_old_model_matches_both_ids():
    valid, ids, names = L.resolve_target_class('blue block', OLD_MODEL_NAMES)
    assert valid and ids == [0, 1] and names == ['blue_block', 'blue block']


def test_resolve_mismatch_is_invalid():
    assert L.resolve_target_class('white_block', DEFAULT_MODEL_NAMES) == (False, [], [])


@pytest.mark.parametrize('target', ['', None, '   ', '_'])
def test_resolve_empty_target_invalid(target):
    assert L.resolve_target_class(target, DEFAULT_MODEL_NAMES)[0] is False


def test_resolve_names_as_list_and_none():
    assert L.resolve_target_class('blue block', ['blue block'])[0] is True
    assert L.resolve_target_class('blue block', None)[0] is False
    assert L.resolve_target_class('blue block', {})[0] is False


def test_names_as_dict_coerces_keys():
    assert L.names_as_dict({'0': 'a', 1: 'b'}) == {0: 'a', 1: 'b'}


# ------------------------------------------------------------------ target-only confidence
def test_target_stats_ignores_other_classes():
    dets = [('person', 0.95), ('blue block', 0.30), ('blue_block', 0.45)]
    raw, tgt, mx = L.target_stats(dets, L.normalize_class_name('blue block'))
    assert (raw, tgt) == (3, 2) and mx == pytest.approx(0.45)


def test_target_stats_only_other_class_gives_zero_target():
    raw, tgt, mx = L.target_stats([('person', 0.99)], 'blue block')
    assert (raw, tgt, mx) == (1, 0, 0.0)


def test_target_stats_empty_and_empty_target():
    assert L.target_stats([], 'blue block') == (0, 0, 0.0)
    assert L.target_stats([('blue block', 0.9)], '') == (1, 0, 0.0)


def test_old_behaviour_would_have_gated_on_all_classes():
    dets = [('person', 0.95)]
    old_max = max(c for _, c in dets)
    _, _, new_max = L.target_stats(dets, 'blue block')
    assert old_max > 0.1 >= new_max       # old gate would open, new one stays closed


# ------------------------------------------------------------------ state classification
OK = dict(model_ready=True, images_received=10, last_image_age_s=0.1, inference_lag_s=0.1,
          last_outcome='ok')


def S(**kw):
    d = dict(OK)
    d.update(kw)
    return L.classify_state(**d)


def test_loading_and_failed():
    assert L.classify_state() == L.LOADING_MODEL
    assert L.classify_state(model_ready=False, images_received=5) == L.LOADING_MODEL
    assert L.classify_state(model_failed=True, model_ready=True) == L.MODEL_FAILED
    assert L.classify_state(model_failed=True) == L.MODEL_FAILED


def test_waiting_image():
    assert L.classify_state(model_ready=True, images_received=0) == L.WAITING_IMAGE
    # image arrived but no inference finished yet
    assert S(last_outcome='none') == L.WAITING_IMAGE


def test_image_stale_boundary():
    assert S(last_image_age_s=2.0) != L.IMAGE_STALE
    assert S(last_image_age_s=2.01) == L.IMAGE_STALE
    # only after having had images
    assert L.classify_state(model_ready=True, images_received=0, last_image_age_s=None) == L.WAITING_IMAGE


def test_inference_lag_means_oldest_unfinished_work():
    # idle worker (lag 0) with an old last inference must not be reported as stalled
    assert S(inference_lag_s=0.0, last_image_age_s=1.0, raw_detections_last=1,
             target_detections_last=1, published_last=True) == L.PUBLISHING


def test_inference_stalled_boundary():
    assert S(inference_lag_s=3.0, raw_detections_last=1, target_detections_last=1,
             published_last=True) == L.PUBLISHING
    assert S(inference_lag_s=3.01) == L.INFERENCE_STALLED


def test_stale_beats_stalled_and_errors():
    assert S(last_image_age_s=5.0, inference_lag_s=9.0, last_outcome='inference_error') == L.IMAGE_STALE
    assert S(inference_lag_s=9.0, last_outcome='convert_error') == L.INFERENCE_STALLED


def test_errors():
    assert S(last_outcome='convert_error') == L.CONVERT_ERROR
    assert S(last_outcome='inference_error') == L.INFERENCE_ERROR


def test_detection_states():
    assert S(raw_detections_last=0) == L.NO_DETECTION
    assert S(raw_detections_last=3, target_detections_last=0) == L.CLASS_FILTERED
    assert S(raw_detections_last=2, target_detections_last=1, published_last=False) == L.BELOW_GATE
    assert S(raw_detections_last=2, target_detections_last=1, published_last=True) == L.PUBLISHING


def test_every_state_is_reachable_and_known():
    seen = {
        L.classify_state(), L.classify_state(model_failed=True),
        L.classify_state(model_ready=True), S(last_image_age_s=9), S(inference_lag_s=9),
        S(last_outcome='convert_error'), S(last_outcome='inference_error'),
        S(), S(raw_detections_last=1), S(raw_detections_last=1, target_detections_last=1),
        S(raw_detections_last=1, target_detections_last=1, published_last=True),
    }
    assert seen == set(L.ALL_STATES)


def test_custom_thresholds():
    assert S(last_image_age_s=1.0, image_stale_sec=0.5) == L.IMAGE_STALE
    assert S(inference_lag_s=1.0, inference_stall_sec=0.5) == L.INFERENCE_STALLED


# ------------------------------------------------------------------ telemetry patch (both branches)
class _FakeMod:
    class GA4Backend:
        def send(self, message):
            return 'sent'


def test_telemetry_patch_applied_branch():
    mod = _FakeMod()
    ok, detail = L.disable_openvino_telemetry(lambda: mod)
    assert ok is True and 'no-op' in detail
    assert mod.GA4Backend().send({'a': 1}) is None          # network call suppressed


def test_telemetry_patch_missing_class():
    class Mod:
        pass
    ok, detail = L.disable_openvino_telemetry(lambda: Mod)
    assert ok is False and 'GA4Backend' in detail


def test_telemetry_patch_missing_send():
    class Mod:
        class GA4Backend:
            pass
    ok, detail = L.disable_openvino_telemetry(lambda: Mod)
    assert ok is False and 'send' in detail


def test_telemetry_patch_import_error_is_reported_not_raised():
    def boom():
        raise ImportError('no module')
    ok, detail = L.disable_openvino_telemetry(boom)
    assert ok is False and 'ImportError' in detail


def test_telemetry_patch_setattr_failure_is_reported():
    class Frozen(type):
        def __setattr__(cls, k, v):
            raise TypeError('frozen')

    class Mod:
        class GA4Backend(metaclass=Frozen):
            def send(self, message):
                return 'sent'
    ok, detail = L.disable_openvino_telemetry(lambda: Mod)
    assert ok is False and 'TypeError' in detail


def test_pick_target_prefers_confidence_over_large_gripper_box():
    # real observe-pose case: gripper box (large, right, low conf) + real block (small, conf 0.6)
    boxes = [(0.60, 326, 292, 30, 38), (0.18, 284, 391, 224, 87)]
    assert L.pick_target_index(boxes) == 0


def test_pick_target_empty_and_bad_mode():
    assert L.pick_target_index([]) is None
    import pytest
    with pytest.raises(ValueError):
        L.pick_target_index([(0.5, 0, 0, 1, 1)], 'nope')
