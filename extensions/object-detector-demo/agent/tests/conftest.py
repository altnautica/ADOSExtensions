"""Pytest configuration: put ``src/`` on ``sys.path`` for in-tree imports.

The agent half is intentionally not pre-installed during unit testing.
Adding ``src`` to the path keeps the test invocation simple and means
``python -m pytest -q`` works without a separate ``pip install -e .``
step on every fresh checkout. The ``ados`` SDK is resolved from whatever
environment runs pytest (the agent venv during local development).
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
