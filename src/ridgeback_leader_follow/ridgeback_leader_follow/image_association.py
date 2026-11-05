"""ByteTrack image identity cues for the selected CV Kalman fusion runtime."""
from types import SimpleNamespace
import numpy as np


class ImageAssociation:
    def __init__(self, threshold=.1):
        from ultralytics.trackers.byte_tracker import BYTETracker
        self.tracker=BYTETracker(SimpleNamespace(track_high_thresh=threshold,
            track_low_thresh=threshold/2,new_track_thresh=threshold,track_buffer=30,
            match_thresh=.8,fuse_score=threshold >= .25))

    def update(self, boxes, image_shape):
        from ultralytics.engine.results import Boxes
        boxes=np.asarray(boxes,dtype=np.float32).reshape(-1,5)
        data=np.column_stack((boxes,np.zeros(len(boxes),dtype=np.float32)))
        rows=self.tracker.update(Boxes(data,orig_shape=image_shape))
        output=[]
        for row in rows:
            index=int(row[7])
            if 0 <= index < len(boxes):output.append((int(row[4]),boxes[index]))
        return output
