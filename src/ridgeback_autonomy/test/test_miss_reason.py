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
