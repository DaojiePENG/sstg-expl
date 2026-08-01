"""Fail-closed readiness checks for lifecycle-managed Nav2 servers."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


@dataclass(frozen=True)
class ReadinessResult:
    """One non-blocking lifecycle readiness observation."""

    ready: bool
    detail: str


class LifecycleActiveGate:
    """Require a fresh ACTIVE response before allowing one goal dispatch.

    ``poll`` never blocks the executor.  A first call starts a GetState request;
    a later call consumes its response.  The successful response is deliberately
    not cached, so every new goal dispatch must be preceded by a fresh lifecycle
    observation.
    """

    def __init__(self, client: Any, service_name: str) -> None:
        if not service_name:
            raise ValueError("lifecycle state service name must be non-empty")
        self._client = client
        self._service_name = service_name
        self._future: Optional[Any] = None

    @property
    def query_pending(self) -> bool:
        return self._future is not None

    def reset(self) -> None:
        """Discard a pending observation when action readiness is lost."""
        future = self._future
        self._future = None
        if future is None or future.done():
            return
        try:
            future.cancel()
        except Exception:
            # Reset is best-effort during shutdown or graph churn.  Dropping the
            # future is still fail-closed because no goal can use its response.
            pass

    def poll(self) -> ReadinessResult:
        """Advance the asynchronous state query and report readiness."""
        if self._future is None:
            if not self._client.service_is_ready():
                return ReadinessResult(
                    False,
                    f"lifecycle service {self._service_name!r} is unavailable",
                )
            try:
                self._future = self._client.call_async(GetState.Request())
            except Exception as error:
                return ReadinessResult(
                    False,
                    f"lifecycle query on {self._service_name!r} failed: {error}",
                )
            return ReadinessResult(
                False,
                f"waiting for lifecycle state from {self._service_name!r}",
            )

        if not self._future.done():
            return ReadinessResult(
                False,
                f"waiting for lifecycle state from {self._service_name!r}",
            )

        future = self._future
        self._future = None
        try:
            response = future.result()
        except Exception as error:
            return ReadinessResult(
                False,
                f"lifecycle query on {self._service_name!r} failed: {error}",
            )
        if response is None:
            return ReadinessResult(
                False,
                f"lifecycle query on {self._service_name!r} returned no response",
            )

        state = response.current_state
        state_id = int(state.id)
        state_label = str(state.label) or f"state_id={state_id}"
        if state_id != State.PRIMARY_STATE_ACTIVE:
            return ReadinessResult(
                False,
                f"Nav2 lifecycle state is {state_label} ({state_id}), not active",
            )
        return ReadinessResult(True, "Nav2 lifecycle state is active")
