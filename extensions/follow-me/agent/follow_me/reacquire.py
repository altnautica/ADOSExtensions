"""Smart re-acquire: the coast state and the candidate match.

When ``reacquire_mode`` is ``smart`` and the vision engine's lock on the
subject is lost, the follow loop does not give up at once. For
``reacquire_window_s`` it flies toward the predicted standoff (at reduced
feed-forward) and checks each new detection batch for a ``person`` that both

* looks like the subject: cosine similarity of its re-identification
  embedding to the stored template >= ``reacquire_min_similarity``, and
* is where the subject should be: Mahalanobis distance from the filter's
  prediction <= 3, measurement covariance included.

The best such candidate is designated with the vision engine — an explicit,
announced ``follow.reacquired`` designate, never a silent re-lock. When the
window expires with no match the follow is given up (``follow.lost``).

Without embeddings on the detection stream (no re-id model and no histogram
fallback in the engine) no candidate can pass the appearance test, so smart
mode coasts and then gives up exactly like strict mode, just later.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any

# The Mahalanobis gate: a 3-sigma ellipse around the prediction.
MAX_MAHALANOBIS = 3.0
# How much a fresh embedding of the locked subject moves the template. Slow,
# so a few frames of partial occlusion do not drag it toward the occluder.
TEMPLATE_ALPHA = 0.1
# The cosine lead the best in-gate candidate needs over the runner-up. Two
# people closer than this look alike, and neither is re-locked.
MIN_MARGIN = 0.08
# A fresh embedding moves the template only from a sighting at least this
# confident, with less than this much of the subject hidden, and with no
# other person's box overlapping the subject's by more than OVERLAP_IOU.
SOLID_LOCK_MIN_CONF = 0.6
MAX_OCCLUSION_HINT = 0.3
OVERLAP_IOU = 0.1

PERSON = "person"

Cov2 = tuple[tuple[float, float], tuple[float, float]]


@dataclass
class Coast:
    """One smart re-acquire window."""

    lost_track_id: int | None
    started_s: float
    deadline_s: float
    # Whether a re-acquire designate was accepted by the engine, and the
    # track it returned (the engine may omit the id).
    designated: bool = False
    designated_track: int | None = None
    # Receive time of the last batch already examined, so a batch is matched
    # once.
    last_batch_s: float = -math.inf

    def remaining_s(self, now: float) -> float:
        return max(0.0, self.deadline_s - now)


@dataclass(frozen=True)
class Embedding:
    """An L2-normalised appearance embedding and the model that made it.
    Embeddings from different models are never compared."""

    model: str
    vec: tuple[float, ...]


def _normalised(vec: Iterable[float]) -> tuple[float, ...] | None:
    v = tuple(float(x) for x in vec)
    n = math.sqrt(sum(x * x for x in v))
    if not v or n <= 1e-9 or not math.isfinite(n):
        return None
    return tuple(x / n for x in v)


def embedding_of(detection: Any) -> Embedding | None:
    """The embedding the engine attached to a detection
    (``attributes.embedding`` + ``attributes.embedding_model``), or ``None``."""
    attrs = getattr(detection, "attributes", None)
    if not isinstance(attrs, dict):
        return None
    raw = attrs.get("embedding")
    model = attrs.get("embedding_model")
    if not isinstance(raw, (list, tuple)) or not isinstance(model, str):
        return None
    try:
        vec = _normalised(raw)
    except (TypeError, ValueError):
        return None
    if vec is None:
        return None
    return Embedding(model=model, vec=vec)


def similarity(a: Embedding, b: Embedding) -> float | None:
    """Cosine similarity, or ``None`` when the embeddings are not comparable."""
    if a.model != b.model or len(a.vec) != len(b.vec):
        return None
    return sum(x * y for x, y in zip(a.vec, b.vec))


def blend_template(old: Embedding | None, new: Embedding) -> Embedding:
    """Move the subject template toward a fresh embedding of the locked
    subject. A model change replaces the template outright."""
    if old is None or old.model != new.model or len(old.vec) != len(new.vec):
        return new
    mixed = _normalised(
        (1.0 - TEMPLATE_ALPHA) * a + TEMPLATE_ALPHA * b
        for a, b in zip(old.vec, new.vec)
    )
    return old if mixed is None else Embedding(model=old.model, vec=mixed)


def template_may_update(
    *, lock_state: str, confidence: float, occlusion_hint: float | None, others_near: int
) -> bool:
    """Blend a fresh embedding into the subject template only while the lock is
    solid: locked, confident, and no other person's box overlaps the subject's."""
    if lock_state != "locked" or confidence < SOLID_LOCK_MIN_CONF or others_near > 0:
        return False
    return occlusion_hint is None or occlusion_hint < MAX_OCCLUSION_HINT


def box_iou(a: Any, b: Any) -> float:
    """Intersection over union of two ``x, y, width, height`` boxes."""
    ix = max(0.0, min(a.x + a.width, b.x + b.width) - max(a.x, b.x))
    iy = max(0.0, min(a.y + a.height, b.y + b.height) - max(a.y, b.y))
    inter = ix * iy
    union = a.width * a.height + b.width * b.height - inter
    return inter / union if union > 0.0 else 0.0



@dataclass(frozen=True)
class Candidate:
    """A detection that passed both the appearance and the position gate."""

    detection: Any
    similarity: float
    mahalanobis: float
    north_m: float
    east_m: float
    covariance: Cov2


def best_candidate(
    detections: Iterable[Any],
    *,
    template: Embedding | None,
    min_similarity: float,
    measure: Callable[[Any], tuple[float, float, Cov2] | None],
    gate: Callable[[float, float, Cov2], float | None],
    min_margin: float = MIN_MARGIN,
) -> Candidate | None:
    """The most similar ``person`` detection inside both gates, or ``None``.

    ``measure(bbox)`` resolves a box to an absolute north/east position and
    its covariance; ``gate(n, e, cov)`` is the Mahalanobis distance from the
    prediction. When the runner-up inside both gates is within ``min_margin``
    cosine of the best, the match is ambiguous and there is none.
    """
    if template is None:
        return None
    found: list[Candidate] = []
    for det in detections:
        if getattr(det, "class_label", None) != PERSON:
            continue
        bbox = getattr(det, "bbox", None)
        if bbox is None:
            continue
        emb = embedding_of(det)
        if emb is None:
            continue
        sim = similarity(template, emb)
        if sim is None or sim < min_similarity:
            continue
        fix = measure(bbox)
        if fix is None:
            continue
        north, east, cov = fix
        d = gate(north, east, cov)
        if d is None or d > MAX_MAHALANOBIS:
            continue
        found.append(Candidate(det, sim, d, north, east, cov))
    if not found:
        return None
    found.sort(key=lambda c: c.similarity, reverse=True)
    if len(found) >= 2 and found[0].similarity - found[1].similarity < min_margin:
        return None
    return found[0]


__all__ = [
    "MAX_MAHALANOBIS",
    "MIN_MARGIN",
    "OVERLAP_IOU",
    "SOLID_LOCK_MIN_CONF",
    "Candidate",
    "Coast",
    "Embedding",
    "best_candidate",
    "blend_template",
    "box_iou",
    "embedding_of",
    "similarity",
    "template_may_update",
]
