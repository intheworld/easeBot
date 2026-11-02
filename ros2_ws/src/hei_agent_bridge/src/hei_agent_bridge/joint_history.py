"""Bounded timestamp interpolation; never substitutes latest joints for an image pose."""
from collections import deque
import math
import time


class JointHistory:
    def __init__(self):self.samples=deque(maxlen=1024)

    def append(self,stamp,positions,received):
        if stamp<=0 or len(positions)!=6 or not all(math.isfinite(v) for v in [stamp,*positions]):return
        if self.samples and stamp<self.samples[-1][0]-1:self.samples.clear()  # ROS clock reset.
        if self.samples and stamp<=self.samples[-1][0]:return
        self.samples.append((stamp,list(positions),received))
        while self.samples and received-self.samples[0][2]>5:self.samples.popleft()

    def window(self,duration):
        now=time.monotonic()
        samples=list(self.samples)
        while len(samples)>2 and samples[1][2]<=now-duration:samples.pop(0)
        if (len(samples)<3 or now-samples[-1][2]>.05 or
                now-samples[0][2]<duration):return None
        span=[max(v[1][i] for v in samples)-min(v[1][i] for v in samples) for i in range(6)]
        return dict(sample_count=len(samples),duration_sec=samples[-1][2]-samples[0][2],joint_span_rad=span)

    def at(self,stamp,*,max_skew_sec,max_span_rad):
        if not math.isfinite(stamp) or not self.samples or time.monotonic()-self.samples[-1][2]>1:return None
        for before,after in zip(self.samples,list(self.samples)[1:]):
            if before[0]<=stamp<=after[0]:
                skew=max(stamp-before[0],after[0]-stamp)
                if skew>max_skew_sec:return None
                span=max(abs(a-b) for a,b in zip(before[1],after[1]))
                if span>max_span_rad:return None
                fraction=(stamp-before[0])/(after[0]-before[0])
                return dict(method='ros_stamp_interpolation',right=[a+fraction*(b-a) for a,b in zip(before[1],after[1])],
                            image_stamp_sec=stamp,before_stamp_sec=before[0],after_stamp_sec=after[0],
                            max_skew_sec=skew,joint_span_rad=span)
        return None
