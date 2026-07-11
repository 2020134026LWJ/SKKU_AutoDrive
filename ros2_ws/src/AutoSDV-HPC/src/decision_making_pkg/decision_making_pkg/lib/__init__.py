"""원본은 py3.10 `.pyc`를 marshal로 직접 로드했다 — 경로를 디렉토리 조각으로 조립하는 방식이라
이 워크스페이스에선 존재하지도 않는 경로(`~/Desktop/src/...`)를 가리켰고, py3.12에선 magic
number도 안 맞는다. `.pyc`는 디컴파일해 `.py`로 복원했으므로 평범하게 임포트한다.
(원본 pyc = `*.cpython-310.pyc.bak`, 원본 로더 = `__init___bak.py`)
"""

from . import decision_making_func_lib

__all__ = ["decision_making_func_lib"]
