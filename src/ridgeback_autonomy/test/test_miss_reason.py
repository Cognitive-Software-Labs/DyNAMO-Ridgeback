from __future__ import annotations

from ridgeback_autonomy.common.miss_reason import MissReason, reason_name


def test_sentinels() -> None:
    assert MissReason.OK == 0
    assert MissReason.UNSET == 255


def test_codes_are_unique() -> None:
    values = [reason.value for reason in MissReason]
    assert len(values) == len(set(values))


def test_reason_name_tolerates_none_and_unknown() -> None:
    assert reason_name(0) == 'OK'
    assert reason_name(255) == 'UNSET'
    assert reason_name(None) == 'UNSET'
    assert reason_name(MissReason.SCAN_INVALID) == 'SCAN_INVALID'
    assert reason_name(200) == 'UNKNOWN_200'


def test_observation_totals_is_the_shared_definition() -> None:
    # Both the CSV summary and the markdown report read totals through this,
    # so they cannot disagree about how many boxes an estimator saw.
    from ridgeback_autonomy.common.miss_reason import MissReason, observation_totals

    total, ok = observation_totals({
        int(MissReason.OK): 30,
        int(MissReason.SCAN_INVALID): 8,
        int(MissReason.UNSET): 2,
    })

    assert (total, ok) == (40, 30)
    assert observation_totals({}) == (0, 0)
    assert observation_totals(None) == (0, 0)
