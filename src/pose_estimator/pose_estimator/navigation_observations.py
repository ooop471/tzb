"""Match delayed detector output to its capture-time depth cloud."""
from collections import deque
import math


class NavigationCloudHistory:
    def __init__(self, history_sec=0.75, max_sync_sec=0.15):
        self.history_sec = history_sec
        self.max_sync_sec = max_sync_sec
        self.clouds = deque(maxlen=64)

    def add(self, cloud, stamp):
        if not math.isfinite(stamp) or stamp <= 0:
            return
        if self.clouds and stamp < self.clouds[-1][0]:
            self.clouds.clear()
        self.clouds.append((stamp, cloud))
        while self.clouds and stamp - self.clouds[0][0] > self.history_sec:
            self.clouds.popleft()

    def match(self, detection_stamp):
        if not self.clouds or not math.isfinite(detection_stamp) or detection_stamp <= 0:
            return None
        stamp, cloud = min(self.clouds, key=lambda item: abs(item[0] - detection_stamp))
        return cloud if abs(stamp - detection_stamp) <= self.max_sync_sec else None
