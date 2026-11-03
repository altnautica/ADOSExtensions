"""Smart re-acquire matching rules."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from follow_me.reacquire import (
    Embedding,
    best_candidate,
    blend_template,
    embedding_of,
    similarity,
)

_COV = ((0.25, 0.0), (0.0, 0.25))


@dataclass
class _Det:
    bbox: Any
    class_label: str = "person"
    confidence: float = 0.9
    attributes: dict[str, Any] | None = field(default=None)


def _det(vec: list[float] | None, *, label: str = "person", model: str = "osnet", x: float = 0.0) -> _Det:
    attrs = None if vec is None else {"embedding": vec, "embedding_model": model}
    return _Det(bbox=x, class_label=label, attributes=attrs)


def _pick(dets: list[_Det], template: Embedding | None, gate_d: dict[float, float]) -> Any:
    return best_candidate(
        dets,
        template=template,
        min_similarity=0.75,
        measure=lambda bbox: (float(bbox), 0.0, _COV),
        gate=lambda n, e, cov: gate_d.get(n, 0.0),
    )


def test_embedding_is_normalised_and_model_tagged() -> None:
    emb = embedding_of(_det([3.0, 4.0]))
    assert emb is not None and emb.model == "osnet"
    assert abs(emb.vec[0] - 0.6) < 1e-12 and abs(emb.vec[1] - 0.8) < 1e-12
    assert embedding_of(_det(None)) is None
    assert embedding_of(_det([0.0, 0.0])) is None


def test_embeddings_from_different_models_are_not_compared() -> None:
    a = Embedding("osnet", (1.0, 0.0))
    assert similarity(a, Embedding("histogram", (1.0, 0.0))) is None
    assert similarity(a, Embedding("osnet", (1.0, 0.0, 0.0))) is None
    assert similarity(a, Embedding("osnet", (0.0, 1.0))) == 0.0


def test_template_moves_slowly_and_resets_on_model_change() -> None:
    old = Embedding("osnet", (1.0, 0.0))
    mixed = blend_template(old, Embedding("osnet", (0.0, 1.0)))
    assert mixed.vec[0] > 0.9 and mixed.vec[1] > 0.0
    other = Embedding("histogram", (0.0, 1.0))
    assert blend_template(old, other) is other


def test_best_candidate_needs_appearance_and_position() -> None:
    template = Embedding("osnet", (1.0, 0.0, 0.0))
    same = _det([1.0, 0.0, 0.0], x=1.0)
    near_same = _det([0.9, 0.1, 0.0], x=2.0)
    different = _det([0.0, 1.0, 0.0], x=3.0)
    car = _det([1.0, 0.0, 0.0], label="car", x=4.0)
    far = _det([1.0, 0.0, 0.0], x=5.0)
    pick = _pick([near_same, different, car, far, same], template, {5.0: 4.0})
    assert pick is not None and pick.detection is same
    assert _pick([different, car, far], template, {5.0: 4.0}) is None
    # No template (no embeddings ever seen): nothing can be re-acquired.
    assert _pick([same], None, {}) is None
