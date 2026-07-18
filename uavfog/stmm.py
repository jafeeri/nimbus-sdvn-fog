"""STMM control-plane equations and SVM-based OC selection.

Implements, faithfully to the published paper:

  Eq.(1)  control duration D of an LC as seen by the MC,
  Eq.(5)  link visibility time VT (with the visc/vismax fog ratio),
  Eq.(6)  OP  = max over candidate paths of PTP,
  Eq.(7)  PTP = min over links of VT,
  Alg. 1  OC selection: LCs are labelled by their hello-message counts
          (highest count -> OC) and an RBF-kernel SVM is trained on those
          counts, then used online to score LCs each reselection interval.

Unlike the earlier attempt, the SVM here is trained on hello counters that the
simulator actually accumulates (vehicles beacon at the CAM rate and every LC
in range counts them), not on a synthetic dataset with hand-crafted labels.
"""

from __future__ import annotations

import numpy as np

from . import paper_model
from .config import SimConfig


def duration_D(cfg: SimConfig, d_mc_m: float, hello_count: float,
               packet_bits: float, hop_count: int) -> float:
    """[STMM] Eq.(1) - delegates to paper_model.control_duration_D_seconds.

    hop_count is the actual number of LC-to-LC relay hops from the OC to the
    MC gateway (computed from the controller topology), not a fixed constant.
    """
    return paper_model.control_duration_D_seconds(
        distance_mc_to_lc_m=d_mc_m,
        balancing_constant_alpha=cfg.alpha_balance,
        packet_size_bits=packet_bits,
        backhaul_link_capacity_bps=cfg.backhaul_capacity_bps,
        hello_window_t_s=cfg.hello_window_s,
        number_of_hello_messages=hello_count,
        controller_hop_count=max(1, hop_count))


def visibility_time(cfg: SimConfig, r_max_m: float, d_ij_m: float,
                    vi: float, vj: float, thi: float, thj: float) -> float:
    """[STMM] Eq.(5) - delegates to paper_model.visibility_time_VT_seconds."""
    return paper_model.visibility_time_VT_seconds(
        maximum_communication_range_m=r_max_m,
        inter_vehicle_distance_m=d_ij_m,
        speed_node_i_ms=vi,
        speed_node_j_ms=vj,
        angle_node_i_toward_destination_rad=thi,
        angle_node_j_toward_destination_rad=thj,
        current_visibility_m=cfg.visibility_m,
        maximum_visibility_m=cfg.vis_max_m)


def path_ptp(vts: list[float]) -> float:
    """[STMM] Eq.(7) - delegates to paper_model.path_time_PTP_seconds."""
    return paper_model.path_time_PTP_seconds(vts)


def optimal_path_op(ptps: list[float]) -> float:
    """[STMM] Eq.(6) - delegates to paper_model.optimal_path_OP_seconds."""
    return paper_model.optimal_path_OP_seconds(ptps)


class OCSelector:
    """SVM-based Optimal Controller selection ([STMM] Algorithm 1).

    During warmup the simulator feeds (hello_count_vector, argmax label)
    samples; we then train an RBF SVM and afterwards select the OC as the LC
    with the highest decision-function score. With very few LCs this is an
    easy task (and the paper's own labelling makes argmax the ground truth) -
    we also track the SVM's agreement with argmax as a sanity metric.
    """

    def __init__(self):
        self._X: list[list[float]] = []
        self._y: list[int] = []
        self._model = None
        self.agreement_checks = 0
        self.agreement_hits = 0

    @staticmethod
    def _features(hello_counts: list[float]) -> np.ndarray:
        """Per-LC features from hello counts only: (count scaled to the
        window maximum, count scaled to the window total). This is feature
        scaling of the same quantity Algorithm 1 classifies on."""
        h = np.asarray(hello_counts, dtype=float)
        mx = max(1.0, float(h.max()))
        tot = max(1.0, float(h.sum()))
        return np.stack([h / mx, h / tot], axis=1)

    def add_window(self, hello_counts: list[float]):
        if not hello_counts:
            return
        best = int(np.argmax(hello_counts))
        X = self._features(hello_counts)
        for i in range(len(hello_counts)):
            self._X.append(list(X[i]))
            self._y.append(1 if i == best else -1)

    def train(self):
        if len(set(self._y)) < 2 or len(self._X) < 4:
            self._model = None
            return
        from sklearn.svm import SVC
        self._model = SVC(kernel="rbf", C=2.0, gamma="scale")
        self._model.fit(np.asarray(self._X), np.asarray(self._y))

    def select(self, hello_counts: list[float]) -> int:
        if not hello_counts:
            return 0
        truth = int(np.argmax(hello_counts))
        if self._model is None:
            return truth
        scores = self._model.decision_function(self._features(hello_counts))
        choice = int(np.argmax(scores))
        self.agreement_checks += 1
        if choice == truth:
            self.agreement_hits += 1
        return choice

    @property
    def agreement(self) -> float:
        if self.agreement_checks == 0:
            return float("nan")
        return self.agreement_hits / self.agreement_checks
