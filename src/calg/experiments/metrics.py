from __future__ import annotations

from dataclasses import dataclass, asdict
import numpy as np

from calg.solver.detector import DetectionResult


@dataclass
class CaseMetrics:
    case: str
    expected_gap: float | None
    min_gap: float
    abs_error: float | None
    contacts: int
    candidate_pairs: int
    graph_passed: int
    fast_path_ratio: float
    graph_contacts: int
    fallback_contacts: int
    curved_contacts: int = 0
    interval_contacts: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


def compute_case_metrics(case_name: str, expected_gap: float | None, result: DetectionResult) -> CaseMetrics:
    min_gap = result.min_gap
    if np.isinf(min_gap):
        min_gap = float("nan")
    abs_error = None if expected_gap is None or np.isnan(min_gap) else abs(min_gap - expected_gap)
    st = result.stats
    return CaseMetrics(
        case=case_name,
        expected_gap=expected_gap,
        min_gap=min_gap,
        abs_error=abs_error,
        contacts=st.contacts,
        candidate_pairs=st.candidate_pairs,
        graph_passed=st.graph_passed,
        fast_path_ratio=st.fast_path_ratio,
        graph_contacts=st.graph_contacts,
        fallback_contacts=st.fallback_contacts,
        curved_contacts=getattr(st, "curved_contacts", 0),
        interval_contacts=getattr(st, "interval_contacts", 0),
    )
