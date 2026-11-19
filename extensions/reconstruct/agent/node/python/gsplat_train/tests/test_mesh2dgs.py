"""The 2DGS mesh pass's surface terms and their schedule."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import mesh2dgs  # noqa: E402


def test_surfel_terms_vanish_on_a_consistent_surface():
    h, w = 8, 8
    n = torch.zeros(h, w, 3)
    n[..., 2] = -1.0
    alpha = torch.ones(h, w)
    normal_err, dist = mesh2dgs.surfel_terms(n, n.clone()[None], alpha, torch.zeros(h, w))
    assert float(normal_err) == pytest.approx(0.0) and float(dist) == 0.0
    tilted = n.clone()
    tilted[..., 0], tilted[..., 2] = 1.0, 0.0  # depth normals 90 degrees off the rendered ones
    tilted[:, 4:] = n[:, 4:]  # ... on half the view
    normal_err, dist = mesh2dgs.surfel_terms(n, tilted, alpha, torch.full((h, w), 0.2))
    assert float(normal_err) == pytest.approx(0.5) and float(dist) == pytest.approx(0.2)


def test_surface_terms_start_after_the_colours_settle():
    assert mesh2dgs.surfel_weights(0) == (0.0, 0.0)
    assert mesh2dgs.surfel_weights(2999) == (0.0, 0.0)
    assert mesh2dgs.surfel_weights(3000) == (0.0, 0.01)
    assert mesh2dgs.surfel_weights(7000) == (0.05, 0.01)
