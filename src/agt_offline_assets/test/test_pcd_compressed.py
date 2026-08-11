import pytest

from agt_offline_assets.contracts import AssetContractError
from agt_offline_assets.pcd_io import _lzf_decompress


def test_lzf_overlapping_back_reference_matches_pcl_semantics():
    # Literal "abc", followed by a 6-byte back-reference to the same 3-byte
    # sequence. Overlapping copy must expand abc -> abcabc -> abcabcabc.
    compressed = bytes([2]) + b"abc" + bytes([4 << 5, 2])
    assert _lzf_decompress(compressed, 9) == b"abcabcabc"


def test_lzf_invalid_reference_fails_closed():
    with pytest.raises(AssetContractError) as raised:
        _lzf_decompress(bytes([1 << 5, 0]), 3)
    assert raised.value.code == "pcd_lzf_invalid"
