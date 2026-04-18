from __future__ import annotations

import threading
import time

import uvicorn


class ManagedUvicornServer:
    def __init__(self, app, host: str, port: int, *, log_level: str = "info") -> None:
        self.error: Exception | None = None
        config = uvicorn.Config(app, host=host, port=port, log_level=log_level, access_log=False)
        self.server = uvicorn.Server(config)
        self.server.install_signal_handlers = lambda: None
        self.thread = threading.Thread(target=self._run, name=f"uvicorn-{port}", daemon=True)

    def start(self, timeout_sec: float = 5.0) -> None:
        self.thread.start()
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.server.started:
                return
            if self.error is not None:
                raise self.error
            if not self.thread.is_alive():
                raise RuntimeError("Server exited before startup completed")
            time.sleep(0.05)
        raise RuntimeError("Timed out waiting for the server to start")

    def stop(self) -> None:
        self.server.should_exit = True
        self.thread.join(timeout=2.0)

    def is_alive(self) -> bool:
        return self.thread.is_alive()

    def _run(self) -> None:
        try:
            self.server.run()
        except Exception as exc:  # pragma: no cover - thread failure path
            self.error = exc
