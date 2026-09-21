"""0.1.6 confirmation rule for non-responses (RC9 / external review, point 4)."""
import math

from crtk_conformance.liveness import attributable, loss_upper_bound, required_confirmations, timeout_interval_from_trials


def test_loss_upper_bounds():
    assert math.isclose(loss_upper_bound(12, 0), 1 - 0.05 ** (1 / 12), rel_tol=1e-12)
    assert abs(loss_upper_bound(12, 0) - 0.2209) < 1e-4
    assert abs(loss_upper_bound(30, 0) - 0.0950) < 1e-4
    assert loss_upper_bound(30, 3) > loss_upper_bound(30, 1) > loss_upper_bound(30, 0)


def test_required_confirmations():
    assert required_confirmations(30, 0)[0] == 2
    assert required_confirmations(30, 1)[0] == 3
    assert required_confirmations(12, 0)[0] == 4          # 0.2209^3 > 0.01 -> 4
    assert required_confirmations(30, 5)[0] is None       # too much loss: unattributable
    assert required_confirmations(12, 7)[0] is None       # the archived 50 % drop case


def _t(cls, gap, **kw):
    d = {"class": cls, "gap_s": gap, "last_stream_latency_s": 0.002}
    d.update(kw)
    return d


def test_attributable_rule_016():
    rej = _t("rejected", 0.3)
    assert attributable(rej, 0) and not attributable(rej, 0, "0.1.6")
    rej_c = _t("rejected", 0.3, confirmation={"confirmed": True})
    assert attributable(rej_c, 0, "0.1.6") and attributable(rej_c, 3, "0.1.6") and not attributable(rej_c, 3)
    assert attributable(_t("faulted", 0.3, state_observed_at_s=0.4), 0, "0.1.6")
    assert attributable(_t("drifted", 0.3), 0, "0.1.6")


def test_unconfirmed_rejection_contributes_no_bound():
    trials = [_t("held", 0.2), _t("rejected", 0.25), _t("rejected", 0.4, confirmation={"confirmed": True})]
    iv5 = timeout_interval_from_trials(trials, L=0.005, G=0.01, fp=0.01, hold_tol=1e-3, stop_class="rejected")
    iv6 = timeout_interval_from_trials(trials, L=0.005, G=0.01, fp=0.01, hold_tol=1e-3, stop_class="rejected", rule="0.1.6")
    assert math.isclose(iv5["interval_high_s"], 0.255) and math.isclose(iv6["interval_high_s"], 0.405)
