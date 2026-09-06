"""Cocotb testbench for the drone detector and its bit-exact software twin.

  make                         # fast FRAME_LOG2=8 equivalence test
  FRAME_LOG2=16 make           # full tape-out frame length

The fast build shortens the frame only; every other parameter, and the weight
file, is what tapes out. The golden model is reconfigured to match, so this is
a real equivalence check rather than a smoke test.
"""

import os
import re

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

import numpy as np  # noqa: E402
import drone_model  # noqa: E402

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
FRAME_LOG2 = int(os.environ.get("FRAME_LOG2", "8"))
NHID, HACC_W, HSHIFT, FEAT_OFF = 4, 6, 1, 6
NPHASE = 2
GATES = os.environ.get("GATES", "") == "yes"
# 40 frames at FRAME_LOG2=8 is 13 M clocks (~20 s). At the tape-out frame
# length every frame is 256x longer, so default to 8 frames there and to the
# minimum the checks accept (5) on the gate-level netlist; NFRAMES= overrides.
NFRAMES_RUN = int(os.environ.get("NFRAMES",
                                 "40" if FRAME_LOG2 <= 10 else ("5" if GATES else "8")))


def test_cfg():
    # Matches the fixed drone_2 RTL; only frame_log2 is shortened in fast tests.
    return drone_model.HWConfig(frame_log2=FRAME_LOG2, nstage=9, nband=5, tap0=4,
                                state_w=10, mant=1, feat_w=4, nframe=16,
                                nphase=NPHASE, score_w=10)


# ---------------------------------------------------------------------------
# Parse the generated weight header so RTL and model share one source of truth
# ---------------------------------------------------------------------------
def load_weights():
    """Parse the generated header so RTL and model share one source of truth."""
    hdr = "drone_weights.svh"
    with open(os.path.join(SRC, hdr)) as f:
        txt = f.read()
    def const(name):
        m = re.search(name + r"\s*=\s*(\d+)'h([0-9a-fA-F]+)", txt)
        return int(m.group(2), 16), int(m.group(1))
    cfg = test_cfg()
    H, NF, NB = NHID, cfg.nframe, cfg.nband
    v, _ = const("WW_ROW")
    W1 = np.zeros((H, NF, NB), dtype=np.int64)
    for h in range(H):
        for f in range(NF):
            row = (v >> (2 * NB * (h * NF + f))) & ((1 << (2 * NB)) - 1)
            for b in range(NB):
                c = (row >> (2 * b)) & 0b11
                W1[h, f, b] = 1 if c == 0b01 else (-1 if c == 0b11 else 0)
    hv, _ = const("WW_HBIAS")
    HB = []
    for h in range(H):
        u = (hv >> (HACC_W * h)) & ((1 << HACC_W) - 1)
        HB.append(u - (1 << HACC_W) if u >> (HACC_W - 1) else u)
    wv, _ = const("WW_W2")
    W2 = []
    for h in range(H):
        c = (wv >> (2 * h)) & 0b11
        W2.append(1 if c == 0b01 else (-1 if c == 0b11 else 0))
    tv, tw = const("WW_THRESH_PK")
    thr = tv - (1 << tw) if tv >> (tw - 1) else tv
    return W1, np.array(HB), np.array(W2), thr


# ---------------------------------------------------------------------------
# Stimulus
# ---------------------------------------------------------------------------
def golden_frames(bits, cfg, n_frames):
    """Golden features for a bit sequence the testbench drives.

    The RTL latches ui_in[0] mid-way through each mic period and consumes it
    on the *next* period's cascade, so the chip sees one extra sample of
    latency: its reset value, then the driven stream. Modelling that is the
    difference between a bit-exact comparison and a confusing near-miss.
    """
    return drone_model.frontend_bits([0] + list(bits), cfg, n_frames=n_frames)


def make_pdm(n_ticks, cfg, seed=3):
    """A deterministic drone-like multitone stimulus with a smooth envelope."""
    rng = np.random.default_rng(seed)
    t = np.arange(wwdata_len := 16000) / 16000.0
    sig = np.zeros(wwdata_len, dtype=np.float32)
    for f, a in [(220, .6), (700, .5), (1500, .35), (3000, .2)]:
        sig += a * np.sin(2 * np.pi * f * t + rng.uniform(0, 6.28))
    env = np.clip(np.sin(np.pi * t / t[-1]) ** 2, 0, 1)
    sig = (sig * env).astype(np.float32)
    sig /= max(abs(sig).max(), 1e-6)
    sig *= 0.7
    bits = []
    for y in drone_model.pdm_encode_batch(sig[None, :], n_ticks, cfg):
        bits.append(1 if y[0] > 0 else 0)
    return bits


PDM_DIV = drone_model.PDM_DIV   # clocks per mic tick (32)
S_CLASS = 2                     # FSM state that runs the template


def read_state(dut):
    """FSM state: the RTL register, or the debug pins on the gate-level netlist."""
    if GATES:
        return (int(dut.uio_out.value) >> 5) & 0b11
    return int(dut.user_project.st.value)


def read_fmax(dut, nband):
    """Per-band frame maxima at S_CLASS entry.

    On the netlist only band 0 is observable (uo_out[7:4]); the ring is back in
    band order at that moment, so uo_out[7:4] is fmax[0].
    """
    if GATES:
        return [(int(dut.uo_out.value) >> 4) & 0xF]
    return [int(dut.user_project.fmax[i].value) for i in range(nband)]


class Bench:
    def __init__(self, dut, cfg):
        self.dut, self.cfg = dut, cfg

    async def reset(self):
        self.dut.ena.value = 1
        self.dut.ui_in.value = 64 << 1       # neutral threshold trim
        self.dut.uio_in.value = 0
        self.dut.rst_n.value = 0
        await ClockCycles(self.dut.clk, 8)
        self.dut.rst_n.value = 1
        await ClockCycles(self.dut.clk, 2)

    def set_bit(self, b, trim=64):
        self.dut.ui_in.value = (b & 1) | ((trim & 0x7F) << 1)


@cocotb.test()
async def test_reset(dut):
    """Reset clears the pipeline and the mic clock is running."""
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    b = Bench(dut, test_cfg())
    await b.reset()
    assert (int(dut.uo_out.value) >> 1) & 0b111 == 0, "no detection may be asserted"
    edges = 0
    prev = int(dut.uo_out.value) & 1
    for _ in range(4 * PDM_DIV):
        await ClockCycles(dut.clk, 1)
        cur = int(dut.uo_out.value) & 1
        edges += cur != prev
        prev = cur
    assert edges >= 6, f"mic clock not toggling ({edges} edges)"
    dut._log.info(f"mic clock: {edges} edges in {4*PDM_DIV} clk (expect ~8)")


@cocotb.test()
async def test_frontend_bit_exact(dut):
    """Every frame's five band features must equal the golden model exactly."""
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    b = Bench(dut, cfg)
    await b.reset()

    n_ticks = NFRAMES_RUN << cfg.frame_log2
    bits = make_pdm(n_ticks, cfg)
    golden = golden_frames(bits, cfg, NFRAMES_RUN)

    # Capture on the RTL's own frame boundary (entry to S_CLASS, before
    # S_ROLL clears fmax) rather than guessing the cycle offset.
    got, prev_st = [], 0
    for tick_i in range(n_ticks):
        if len(got) >= len(golden):
            break
        b.set_bit(bits[tick_i])
        for _ in range(PDM_DIV):
            await RisingEdge(dut.clk)
            st = read_state(dut)
            if st == S_CLASS and prev_st != S_CLASS:
                got.append(read_fmax(dut, cfg.nband))
            prev_st = st

    n = min(len(got), len(golden))
    assert n >= 4, f"only captured {n} frames"
    golden = [list(g[:len(got[0])]) for g in golden]   # GL: band 0 only
    bad = [(i, got[i], golden[i]) for i in range(n) if got[i] != golden[i]]
    for i, g, e in bad[:5]:
        dut._log.error(f"frame {i}: RTL {g} golden {e}")
    assert not bad, f"{len(bad)}/{n} frames mismatched"
    dut._log.info(f"{n} frames bit-exact; example {got[min(3, n-1)]}")


@cocotb.test()
async def test_detector_matches_model(dut):
    """LED trace must match the golden Detector fed the golden features.

    Independent of test_frontend_bit_exact: the model is driven from
    drone_model.frontend_bits, not from whatever the RTL computed.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    b = Bench(dut, cfg)
    await b.reset()

    W1, HB, W2, thr = load_weights()
    trim = 62  # effective threshold 6: the useful high-recall operating point
    det = drone_model.Detector(W1, HB, W2, thr + ((trim - 64) << 2), cfg,
                        hacc_w=HACC_W, hshift=HSHIFT, feat_off=FEAT_OFF,
                        refractory_frames=16)

    n_ticks = NFRAMES_RUN << cfg.frame_log2
    bits = make_pdm(n_ticks, cfg, seed=5)
    golden = golden_frames(bits, cfg, NFRAMES_RUN)

    mism, frames, prev_st, pending = 0, 0, 0, False
    for tick_i in range(n_ticks):
        b.set_bit(bits[tick_i], trim)
        for _ in range(PDM_DIV):
            await RisingEdge(dut.clk)
            st = read_state(dut)
            if st == S_CLASS and prev_st != S_CLASS and frames < len(golden):
                det.push_frame(golden[frames])
                frames += 1
                pending = True
            elif pending and st == 0:
                rtl = (int(dut.uo_out.value) >> 1) & 1
                exp = 1 if det.hold > 0 else 0
                if rtl != exp:
                    mism += 1
                    if mism <= 3:
                        dut._log.error(f"frame {frames}: LED rtl={rtl} model={exp}")
                pending = False
            prev_st = st
    assert mism == 0, f"{mism}/{frames} frames disagreed on the LED"
    dut._log.info(f"{frames} frames: LED matches the model, "
                  f"{len(det.fired)} window(s) fired")
