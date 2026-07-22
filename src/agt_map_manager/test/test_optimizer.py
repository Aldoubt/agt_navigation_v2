from agt_map_manager.optimizer import NOT_IMPLEMENTED, reject_optimization


def test_optimizer_reservation_never_fabricates_success():
    success, error_code, message = reject_optimization("noop")
    assert not success
    assert error_code == NOT_IMPLEMENTED
    assert "not implemented" in message
