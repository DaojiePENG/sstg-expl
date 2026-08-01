from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

from sstg_policy_ros.readiness import LifecycleActiveGate


class FakeFuture:
    def __init__(self, *, done=False, response=None, error=None):
        self.is_done = done
        self.response = response
        self.error = error
        self.cancelled = False

    def done(self):
        return self.is_done

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, futures=(), *, ready=True):
        self.ready = ready
        self.futures = list(futures)
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.futures.pop(0)


def _state_response(state_id, label):
    response = GetState.Response()
    response.current_state.id = state_id
    response.current_state.label = label
    return response


def test_gate_fails_closed_until_service_and_active_response_are_ready():
    active = FakeFuture(
        response=_state_response(State.PRIMARY_STATE_ACTIVE, "active")
    )
    client = FakeClient([active], ready=False)
    gate = LifecycleActiveGate(client, "/bt_navigator/get_state")

    unavailable = gate.poll()
    assert not unavailable.ready
    assert "unavailable" in unavailable.detail
    assert client.requests == []

    client.ready = True
    pending = gate.poll()
    assert not pending.ready
    assert gate.query_pending
    assert len(client.requests) == 1

    assert not gate.poll().ready
    active.is_done = True
    ready = gate.poll()
    assert ready.ready
    assert not gate.query_pending


def test_gate_requeries_after_inactive_state_instead_of_caching_it():
    inactive = FakeFuture(
        done=True,
        response=_state_response(State.PRIMARY_STATE_INACTIVE, "inactive"),
    )
    active = FakeFuture(
        done=True,
        response=_state_response(State.PRIMARY_STATE_ACTIVE, "active"),
    )
    client = FakeClient([inactive, active])
    gate = LifecycleActiveGate(client, "/bt_navigator/get_state")

    assert not gate.poll().ready
    observed_inactive = gate.poll()
    assert not observed_inactive.ready
    assert "inactive" in observed_inactive.detail

    assert not gate.poll().ready
    assert gate.poll().ready
    assert len(client.requests) == 2


def test_gate_fails_closed_on_query_error_and_can_retry():
    failed = FakeFuture(done=True, error=RuntimeError("transport lost"))
    active = FakeFuture(
        done=True,
        response=_state_response(State.PRIMARY_STATE_ACTIVE, "active"),
    )
    client = FakeClient([failed, active])
    gate = LifecycleActiveGate(client, "/bt_navigator/get_state")

    assert not gate.poll().ready
    failure = gate.poll()
    assert not failure.ready
    assert "transport lost" in failure.detail

    assert not gate.poll().ready
    assert gate.poll().ready


def test_gate_reset_cancels_and_discards_pending_query():
    pending = FakeFuture()
    client = FakeClient([pending])
    gate = LifecycleActiveGate(client, "/bt_navigator/get_state")

    assert not gate.poll().ready
    gate.reset()

    assert pending.cancelled
    assert not gate.query_pending
