import numpy as np

from ridgeback_leader_follow.camera_people import parse_tracks


def test_rows_map_to_tracks_and_bad_indices_are_dropped():
    features = np.eye(2, dtype=np.float32)
    rows = np.array([[1, 2, 3, 4, 9, 0.8, 0, 1], [0, 0, 1, 1, 5, 0.5, 0, 7]], np.float32)
    tracks = parse_tracks(rows, features)
    assert len(tracks) == 1
    assert (tracks[0].track_id, round(tracks[0].score, 3)) == (9, 0.8)
    np.testing.assert_array_equal(tracks[0].feature, features[1])
    np.testing.assert_array_equal(tracks[0].bbox, [1, 2, 3, 4])
    assert parse_tracks(np.zeros((1, 7), np.float32), features) == []
