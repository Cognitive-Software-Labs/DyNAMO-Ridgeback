import pytest
from ridgeback_leader_follow.benchmarking.image_metrics import tracking_metrics


def test_official_metrics_penalize_identity_swaps():
    pytest.importorskip('trackeval')
    box=[0,0,10,20]
    perfect=tracking_metrics([([('a',box)],[(1,box)]) for _ in range(8)])
    swapped=tracking_metrics([([('a',box)],[(1 if i < 4 else 2,box)]) for i in range(8)])
    assert perfect['HOTA'] == pytest.approx(1.)
    assert perfect['IDF1'] == pytest.approx(1.)
    assert swapped['IDF1'] < perfect['IDF1']
    assert swapped['HOTA'] < perfect['HOTA']
