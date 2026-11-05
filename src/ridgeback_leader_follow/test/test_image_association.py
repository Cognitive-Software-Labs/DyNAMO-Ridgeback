import pytest


def test_low_confidence_person_keeps_image_identity():
    pytest.importorskip('lap')
    pytest.importorskip('ultralytics')
    from ridgeback_leader_follow.image_association import ImageAssociation
    tracker=ImageAssociation(.1)
    observed=[]
    for _ in range(4):
        observed.append([identity for identity,box in tracker.update([[0,0,10,20,.15]],(480,640))])
    assert observed == [[1],[1],[1],[1]]
