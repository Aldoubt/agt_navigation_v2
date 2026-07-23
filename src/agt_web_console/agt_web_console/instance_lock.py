"""Single-instance guard for a Web console runtime directory."""

import fcntl
import os
from pathlib import Path


class WebConsoleInstanceLock:
    """Keep one Web process per configured runtime directory."""

    def __init__(self, runtime_dir: str | Path) -> None:
        self.path = Path(runtime_dir).expanduser().resolve() / "logs" / "web_console.lock"
        self._stream = None

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            stream.close()
            raise RuntimeError(
                f"Web 控制台已经在运行：{self.path}；请只保留一个 web_console.py 实例"
            ) from error
        stream.seek(0)
        stream.truncate()
        stream.write(f"{os.getpid()}\n")
        stream.flush()
        self._stream = stream

    def release(self) -> None:
        stream = self._stream
        self._stream = None
        if stream is None:
            return
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        finally:
            stream.close()
