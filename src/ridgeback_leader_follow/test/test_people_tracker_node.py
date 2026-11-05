"""Regression for the asynchronous RGB-D/TF backlog seen in bag playback."""
import struct
import pytest
from types import SimpleNamespace
from sensor_msgs.msg import Image,CameraInfo
from rclpy.time import Time
from ridgeback_leader_follow.people_tracker_node import PeopleTrackerNode
from ridgeback_leader_follow.benchmarking.dataset import stamp


@pytest.mark.parametrize("previous", [1.9, -1.])
def test_tick_uses_newest_complete_bundle_when_latest_rgb_is_unpaired(previous):
    def image(at,encoding,data):
        msg=Image();msg.header.stamp=Time(nanoseconds=round(at*1e9)).to_msg()
        msg.header.frame_id='camera';msg.width=msg.height=1
        msg.encoding=encoding;msg.step=len(data);msg.data=data
        return msg
    older=image(1.95,'rgb8',b'\0\0\0');newer=image(2.,'rgb8',b'\0\0\0')
    depth=image(1.95,'32FC1',struct.pack('<f',1.))
    calls=[];processed=[];published=[]
    def prepare(rgb):
        calls.append(stamp(rgb))
        if stamp(rgb) > 1.99:raise ValueError('Depth has not arrived for latest RGB')
        return depth,CameraInfo(),None,[],[]
    node=SimpleNamespace(appearance_encoder=None,queues={'rgb':[older,newer]},last_rgb=previous,last_output=previous,
        output_frame='odom',observation_timeout=.15,get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=2)),
        prepare_inputs=prepare,get_logger=lambda:SimpleNamespace(warning=lambda *a,**k:None),
        fusion=SimpleNamespace(stamp=1.9,update=lambda at,measurements:processed.append(at) or []),
        detector=SimpleNamespace(detect=lambda pixels:[]),image_association=SimpleNamespace(update=lambda boxes,shape:[]),lidars={},
        publisher=SimpleNamespace(publish=published.append))
    node.lidar_measurements=lambda scans,matrices:[]
    node.publish_measurements=lambda at,measurements:PeopleTrackerNode.publish_measurements(node,at,measurements)
    PeopleTrackerNode.tick(node)
    assert calls == pytest.approx([2.,1.95])
    assert processed == pytest.approx([1.95])
    assert len(published) == 1
    assert stamp(published[0]) == pytest.approx(1.95)


def test_lidar_only_uses_observed_scan_when_rgb_bundle_is_unavailable():
    def scan(at):
        return SimpleNamespace(header=SimpleNamespace(stamp=Time(nanoseconds=round(at*1e9)).to_msg(),frame_id='laser'))
    published=[]
    node=SimpleNamespace(queues={'front':[scan(1.9),scan(2.)]},last_output=1.8,
        latest=lambda key,at:scan(at),
        transform=lambda frame,at:None,
        lidar_measurements=lambda scans,matrices:['observed scan measurements'],
        publish_measurements=lambda at,measurements:published.append((at,measurements)))
    assert PeopleTrackerNode.lidar_only(node)
    assert published==[(2.,['observed scan measurements'])]
    node.last_output=2.
    assert not PeopleTrackerNode.lidar_only(node)
    assert len(published)==1


def test_empty_heartbeat_waits_for_configured_observation_timeout():
    published=[]
    rgb=SimpleNamespace(header=SimpleNamespace(stamp=Time(seconds=2).to_msg()))
    node=SimpleNamespace(queues={'rgb':[rgb]},last_rgb=2.,last_output=2.,
        output_frame='odom',observation_timeout=.25,appearance_encoder=None,
        get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=2_200_000_000)),
        get_logger=lambda:SimpleNamespace(warning=lambda *a,**k:None),
        publisher=SimpleNamespace(publish=published.append))
    PeopleTrackerNode.tick(node)
    assert published==[]
    node.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=2_300_000_000))
    PeopleTrackerNode.tick(node)
    assert len(published)==1 and not published[0].people
