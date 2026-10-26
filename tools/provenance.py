#!/usr/bin/env python3
"""A measurement carries where it came from, and two of them refuse to be compared
unless that matches.

WHY THIS EXISTS. Seven comparisons were published wrong in one week, and not one of
them was a wrong number. Every single one was a right number compared against a
number from somewhere else:

  1. P(X,VX) "is 0.000"            a description, not a measurement
  2. var(QZ) "identical"           true for +-30 by symmetry, false for 10 vs 30
  3. "7 m vs 23-46 m"              MEDIANS quoted against an RMSE target
  4. "sigma doubles at 362 s"      adaptively scaled R read as filter covariance
  5. P_angvel = 0.2417             a 3-axis TRACE labelled as one variance
  6. "185.195 vs 9.193, 20x"       two pipelines, two configs, two scoring points
  7. "GNSS 9.046 vs FC 78.139"     fix-times GNSS against full-trajectory FusionCore
  8. "bit-identical on all five"   RMSE equal to 3 dp read as the STATE being equal

A raw-numbers rule does not catch any of these, because the numbers were raw and
correct. What was missing was the tag saying what each one measured.

USAGE

    from provenance import M
    a = M(9.046,  pipeline="replay", config="nclt_shipped",
          points="fix_times", coverage=1.00, name="GNSS 2012-06-15 RMSE")
    b = M(78.139, pipeline="replay", config="nclt_shipped",
          points="full_traj", coverage=1.00, name="FC 2012-06-15 RMSE")
    a.ratio(b)        # raises: scoring points differ, fix_times vs full_traj

Four fields, all required, because every one of the first seven failures above is a
mismatch in exactly one of them:

    pipeline   node | replay            which code path produced it
    config     a short name             which settings were in force
    points     full_traj | fix_times | gaps | window
    coverage   0.0 to 1.0               fraction of the sequence actually covered

EQUALITY IS ITS OWN CLAIM, which is failure 8 and the reason `same_as` exists. On
2026-10-19 the yaw reset was reported as changing nothing, "bit-identical trajectories
on all five sequences". What had been measured was position RMSE agreeing to three
decimals. Four of the five were in fact byte-identical and the fifth was not: on
2012-08-20 the state diverged by 0.798 m in position and 0.934 in a quaternion
component while the RMSE landed on 46.671 either way. A metric tying is weaker than a
state matching, and the stronger word was used.

So `same_as` makes the strength explicit and refuses to let a metric comparison be
reported as state equality:

    a.same_as(b, how="metric", decimals=3)   # the RMSEs agree to 3 dp
    a.same_as(b, how="bytes")                # the trajectories are byte-identical
                                             # and only a byte comparison may say so

Coverage exists because of failure 7's cause: a log exported from a benchmark run
that had been stopped partway through covered 66% of its sequence and was used as if
it were whole. Anything below 0.99 refuses to be compared at all.
"""
from dataclasses import dataclass


class ProvenanceError(AssertionError):
    """Two measurements were compared whose provenance does not match."""


@dataclass(frozen=True)
class M:
    value: float
    pipeline: str
    config: str
    points: str
    coverage: float = 1.0
    name: str = ""

    _POINTS = ("full_traj", "fix_times", "gaps", "window")
    _PIPELINES = ("node", "replay", "sim")

    def __post_init__(self):
        if self.points not in self._POINTS:
            raise ProvenanceError(
                f"{self.name}: points={self.points!r} is not one of {self._POINTS}")
        if self.pipeline not in self._PIPELINES:
            raise ProvenanceError(
                f"{self.name}: pipeline={self.pipeline!r} is not one of {self._PIPELINES}")
        if not 0.0 <= self.coverage <= 1.0:
            raise ProvenanceError(f"{self.name}: coverage={self.coverage} is not a fraction")

    def same_as(self, other, how, decimals=3, bytes_equal=None):
        """Is this the same as `other`, and say in WHICH sense.

        how="metric"  the two VALUES agree to `decimals`. Says nothing about the
                      state behind them: two different trajectories routinely score
                      the same RMSE.
        how="bytes"   the two artefacts are byte-identical. Only a real byte or
                      element-wise comparison may claim this, so the caller has to
                      pass its result in as `bytes_equal`; this class never sees the
                      trajectories and must not be allowed to infer it.

        Returns a sentence suitable for pasting into a commit message, so the claim
        and its strength travel together.
        """
        if how not in ("metric", "bytes"):
            raise ProvenanceError(f"how={how!r} must be 'metric' or 'bytes'")
        self.check_comparable(other)
        if how == "bytes":
            if bytes_equal is None:
                raise ProvenanceError(
                    "how='bytes' needs bytes_equal from an actual byte comparison: "
                    "equal metrics do NOT establish equal state, which is how "
                    "'bit-identical on all five' got published on 2026-10-18")
            return ("byte-identical" if bytes_equal else
                    "DIFFERENT in state (metrics may still agree)")
        same = round(self.value, decimals) == round(other.value, decimals)
        return (f"identical to {decimals} dp in this metric only "
                f"({self.value:.{decimals}f}); state not compared" if same else
                f"differs in this metric ({self.value:.{decimals}f} vs "
                f"{other.value:.{decimals}f})")

    def tag(self):
        return f"{self.pipeline}/{self.config}/{self.points}/{self.coverage:.0%}"

    def check_comparable(self, other):
        """Raise unless these two may legitimately be compared."""
        why = []
        if self.pipeline != other.pipeline:
            why.append(f"pipeline {self.pipeline} vs {other.pipeline}")
        if self.config != other.config:
            why.append(f"config {self.config} vs {other.config}")
        if self.points != other.points:
            why.append(f"scoring points {self.points} vs {other.points}")
        for m in (self, other):
            if m.coverage < 0.99:
                why.append(f"{m.name or 'a value'} covers only {m.coverage:.0%} of its sequence")
        if why:
            raise ProvenanceError(
                f"refusing to compare {self.name or self.value} [{self.tag()}] "
                f"with {other.name or other.value} [{other.tag()}]: " + "; ".join(why))

    def ratio(self, other):
        self.check_comparable(other)
        return self.value / other.value

    def delta_pct(self, other):
        self.check_comparable(other)
        return 100.0 * (self.value - other.value) / other.value

    def __str__(self):
        return f"{self.value:.3f} [{self.tag()}]"


def table(rows, headers):
    """Print rows of M, with every column's provenance in the header, or refuse."""
    tags = {m.tag() for row in rows for m in row if isinstance(m, M)}
    if len(tags) > 1:
        raise ProvenanceError(
            "a table may not mix provenance; found " + ", ".join(sorted(tags)))
    print(f"  provenance: {tags.pop() if tags else 'none'}")
    print("  " + "  ".join(f"{h:>12}" for h in headers))
    for row in rows:
        cells = [f"{c.value:12.3f}" if isinstance(c, M) else f"{str(c):>12}" for c in row]
        print("  " + "  ".join(cells))
