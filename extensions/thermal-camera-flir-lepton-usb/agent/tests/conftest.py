"""Pytest configuration: put ``src/`` and the test directory on ``sys.path``.

The agent half is intentionally not pre-installed during unit testing.
Adding ``src`` to the path keeps the test invocation simple and means
``python -m pytest -q`` works without a separate ``pip install -e .``
step on every fresh checkout.

The test directory itself goes on the path so the suite's synthetic
backend (``mock_backend``) imports by bare name from any invocation
directory. It stays out of ``src`` so no synthetic temperature source is
reachable from the plugin's production import path.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TESTS = Path(__file__).resolve().parent
for entry in (str(SRC), str(TESTS)):
    if entry not in sys.path:
        sys.path.insert(0, entry)
