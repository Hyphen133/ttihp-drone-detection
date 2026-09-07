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
# LED hold, in frames -- must track HOLD_FRAMES in the RTL.
# test_hold_duration asserts the RTL agrees, so a change there fails here
# instead of silently desyncing the golden Detector, whose refractory_frames is
# this same number.
HOLD_FRAMES = 2
# `hold` is loaded in S_CLASS and decremented in the S_ROLL of that same frame,
# so the LED covers HOLD_FRAMES-1 whole frames.
HOLD_FRAMES_VISIBLE = HOLD_FRAMES - 1
# Frame period of the build that tapes out, independent of the shortened
# FRAME_LOG2 the fast run uses.
TAPEOUT_FRAME_MS = (1 << 16) / (drone_model.PDM_HZ / 1000.0)
# The fast RTL build only: behavioural checks that do not need the tape-out
# frame length, where a frame costs ~2 s, or the netlist, where it costs
# ~170 s. Keeping them out of those passes is what stops the suite growing.
FAST_ONLY = GATES or FRAME_LOG2 > 10


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


async def next_frame(dut, limit):
    """Advance to the next frame boundary -- the RTL's own entry into S_CLASS.

    Returns False if none arrived within `limit` clocks, so a stalled FSM fails
    the calling test instead of hanging the simulation.
    """
    prev = read_state(dut)
    for _ in range(limit):
        await RisingEdge(dut.clk)
        st = read_state(dut)
        if st == S_CLASS and prev != S_CLASS:
            return True
        prev = st
    return False


def led(dut):
    return (int(dut.uo_out.value) >> 3) & 1


async def capture_frames(dut, b, bits, n_frames):
    """Per-frame band maxima, sampled on the RTL's own frame boundary.

    Entry to S_CLASS is before S_ROLL clears fmax, which is why the boundary is
    read from the design rather than guessed from a cycle offset.
    """
    got, prev_st = [], 0
    for tick_i in range(len(bits)):
        if len(got) >= n_frames:
            break
        b.set_bit(bits[tick_i])
        for _ in range(PDM_DIV):
            await RisingEdge(dut.clk)
            st = read_state(dut)
            if st == S_CLASS and prev_st != S_CLASS:
                got.append(read_fmax(dut, b.cfg.nband))
            prev_st = st
    return got


async def check_bit_exact(dut, b, bits, n_frames, min_frames):
    """Assert every captured frame equals the golden front end, exactly."""
    golden = golden_frames(bits, b.cfg, n_frames)
    got = await capture_frames(dut, b, bits, len(golden))
    n = min(len(got), len(golden))
    assert n >= min_frames, f"only captured {n} frames"
    golden = [list(g[:len(got[0])]) for g in golden]   # GL: band 0 only
    bad = [(i, got[i], golden[i]) for i in range(n) if got[i] != golden[i]]
    for i, g, e in bad[:5]:
        dut._log.error(f"frame {i}: RTL {g} golden {e}")
    assert not bad, f"{len(bad)}/{n} frames mismatched"
    dut._log.info(f"{n} frames bit-exact; example {got[min(3, n-1)]}")


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

    bits = make_pdm(NFRAMES_RUN << cfg.frame_log2, cfg)
    await check_bit_exact(dut, b, bits, NFRAMES_RUN, min_frames=4)


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
                        refractory_frames=HOLD_FRAMES)

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


@cocotb.test()
async def test_mic_clock_period(dut):
    """uo_out[0] divides clk by exactly 2^PDM_DIV_LOG2.

    Everything downstream is quoted in milliseconds off this divider -- the
    frame length, and so the LED hold -- but test_reset only checks that it
    toggles. A divider off by one would leave every timing claim in the docs
    wrong while all three equivalence tests still passed, because the golden
    model counts mic ticks, not nanoseconds.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    b = Bench(dut, test_cfg())
    await b.reset()

    # Two full periods, measured between rising edges of the mic clock.
    periods, prev, since = [], int(dut.uo_out.value) & 1, 0
    for _ in range(6 * PDM_DIV):
        await ClockCycles(dut.clk, 1)
        since += 1
        cur = int(dut.uo_out.value) & 1
        if cur and not prev:
            periods.append(since)
            since = 0
        prev = cur
    assert len(periods) >= 3, f"only {len(periods)} mic edges seen"
    measured = periods[1:]            # drop the first, timed from reset
    assert all(p == PDM_DIV for p in measured), \
        f"mic period {measured}, expected {PDM_DIV} clk"
    dut._log.info(f"mic period {PDM_DIV} clk = {drone_model.PDM_HZ/1000:.1f} kHz; "
                  f"tape-out frame {TAPEOUT_FRAME_MS:.2f} ms")

@cocotb.test(skip=FAST_ONLY)
async def test_hold_duration(dut):
    """The LED stays up for exactly HOLD_FRAMES-1 frames after a fire.

    This is the only check on how long the output lasts. The equivalence tests
    cannot cover it: a fire needs a full NFRAME window, which none of the short
    runs reach, so they all compare a permanently-low LED. `hold` is therefore
    loaded here exactly as the S_CLASS fire path loads it, on a real frame
    boundary, and the release is counted in the design's own frames.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    b = Bench(dut, cfg)
    await b.reset()

    assert int(dut.user_project.HOLD_FRAMES.value) == HOLD_FRAMES, \
        (f"RTL HOLD_FRAMES={int(dut.user_project.HOLD_FRAMES.value)}, test "
         f"expects {HOLD_FRAMES} -- update both, and refractory_frames with them")

    # Park the threshold at its ceiling first. The measurement needs the one
    # hold it loads to run to zero undisturbed, and at the default trim=1 the
    # wake word fires roughly every other window (10 in 40 frames), which
    # reloads `hold` mid-count and reads as a hold that never ends.
    b.set_bit(0, trim=127)
    frame_clks = (1 << cfg.frame_log2) * PDM_DIV
    assert await next_frame(dut, 3 * frame_clks), "no frame boundary"
    dut.user_project.hold.value = HOLD_FRAMES
    await RisingEdge(dut.clk)
    assert led(dut), "LED did not follow a loaded hold"

    frames = 0
    while frames <= HOLD_FRAMES + 1:
        assert await next_frame(dut, 3 * frame_clks), "frame boundary stopped"
        if not led(dut):
            break
        frames += 1
    assert frames == HOLD_FRAMES_VISIBLE, \
        f"LED held {frames} frames, expected {HOLD_FRAMES_VISIBLE}"
    dut._log.info(f"LED holds {frames} frame(s) = "
                  f"{frames * TAPEOUT_FRAME_MS:.1f} ms at the tape-out frame length")

@cocotb.test(skip=GATES)
async def test_reset_clears_led(dut):
    """Reset drops a held LED instead of leaving it lit.

    test_reset only looks at the output from a cold start, where `hold` is
    already zero, so it would pass on a reset that missed the hold counter --
    and a stuck LED is the one failure a user of the board would see.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    b = Bench(dut, test_cfg())
    await b.reset()

    dut.user_project.hold.value = HOLD_FRAMES
    await RisingEdge(dut.clk)
    assert led(dut), "LED did not follow a loaded hold"

    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 2)
    assert not led(dut), "LED still lit after reset"
    assert (int(dut.uo_out.value) >> 1) & 0b111 == 0, "a detection survived reset"
    dut._log.info("reset clears the hold counter")

@cocotb.test(skip=FAST_ONLY)
async def test_dc_input_bit_exact(dut):
    """A stuck mic is still bit-exact against the golden front end.

    make_pdm() never produces a long run of one symbol, so the cascade is only
    ever exercised near the middle of its range. A dead or shorted mic drives
    it to a rail and is where a saturation or sign-extension bug in the
    STATE_W-bit state would show up -- and it is a real board failure, not a
    hypothetical input.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()

    for name, bit in (("stuck low", 0), ("stuck high", 1)):
        b = Bench(dut, cfg)
        await b.reset()
        n_frames = min(NFRAMES_RUN, 6)
        bits = [bit] * (n_frames << cfg.frame_log2)
        dut._log.info(f"--- mic {name} ---")
        await check_bit_exact(dut, b, bits, n_frames, min_frames=3)

@cocotb.test(skip=FAST_ONLY)
async def test_trim_raises_threshold(dut):
    """Turning the trim up must not produce more detections.

    ui_in[7:1] are the board's DIP switches and the only runtime control the
    chip has; the threshold is thr + ((trim-64) << 2). Nothing tested them.
    The property is monotonicity rather than an absolute count, because the
    count depends on the stimulus -- but a sign error or a mis-slice of ui_in
    would invert it, and that is a knob the user turns the wrong way forever.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    n_frames = min(NFRAMES_RUN, 24)
    bits = make_pdm(n_frames << cfg.frame_log2, cfg, seed=5)

    fires = {}
    for trim in (1, 64, 127):
        b = Bench(dut, cfg)
        await b.reset()
        n, prev = 0, 0
        for tick_i in range(len(bits)):
            b.set_bit(bits[tick_i], trim)
            for _ in range(PDM_DIV):
                await RisingEdge(dut.clk)
                cur = led(dut)
                n += cur and not prev
                prev = cur
        fires[trim] = n
    dut._log.info(f"detections by trim: {fires}")
    assert fires[1] >= fires[64] >= fires[127], \
        f"trim is not monotonic: {fires}"
    assert fires[1] > 0, "no detection at the lowest trim -- stimulus too weak to test"

@cocotb.test(skip=FAST_ONLY)
async def test_reset_mid_frame_recovers(dut):
    """Reset part-way through a frame must leave the chip as good as cold.

    Every other test resets before the first mic tick, so a reset that missed
    a mid-frame register -- fmax, the accumulator ring, the tick divider --
    would never show. Here the design is run into the middle of a frame, reset
    there, and then held to the same bit-exact standard as a cold start.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    b = Bench(dut, cfg)
    await b.reset()

    # Run 1.5 frames so the reset lands with fmax and cnt part-way populated.
    stale = make_pdm(2 << cfg.frame_log2, cfg, seed=11)
    await capture_frames(dut, b, stale, 1)
    await ClockCycles(dut.clk, (1 << (cfg.frame_log2 - 1)) * PDM_DIV)
    assert int(dut.user_project.cnt.value) != 0, "not mid-frame; the test proves nothing"

    await b.reset()
    assert int(dut.user_project.cnt.value) == 0, "cnt survived reset"
    assert not led(dut), "LED survived reset"
    assert all(int(dut.user_project.fmax[i].value) == 0 for i in range(cfg.nband)), \
        "a frame maximum survived reset"

    n_frames = min(NFRAMES_RUN, 6)
    bits = make_pdm(n_frames << cfg.frame_log2, cfg)
    await check_bit_exact(dut, b, bits, n_frames, min_frames=3)

@cocotb.test(skip=GATES)
async def test_threshold_trim_arithmetic(dut):
    """thresh == WW_THRESH_PK + ((trim - 64) << 2) across the whole trim range.

    Added because toggle coverage showed it was missing: the suite only ever
    drove four trim values, so 11 bits of `trim` and `thresh` never changed
    state (scripts/coverage.sh). test_trim_raises_threshold covers the
    behaviour but is too slow to sweep -- it needs whole frames per point to
    count detections -- whereas the arithmetic is combinational and can be
    read straight off the pins. The values below toggle every bit of ui_in[7:1]
    in both directions, and include both ends where the sign of trim8 flips.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    b = Bench(dut, test_cfg())
    await b.reset()

    _, _, _, thr = load_weights()
    span = 1 << 10                                   # SCORE_W, for wraparound
    seen = set()
    for trim in (0, 127, 0x55, 0x2A, 1, 126, 64, 63, 65, 32, 96):
        b.set_bit(0, trim)
        await ClockCycles(dut.clk, 2)
        got = int(dut.user_project.thresh.value)
        if got >= span // 2:
            got -= span                              # SCORE_W-bit signed
        exp = thr + ((trim - 64) << 2)
        assert got == exp, f"trim={trim}: thresh={got}, expected {exp}"
        seen.add(trim)
    dut._log.info(f"threshold correct at {len(seen)} trim points, "
                  f"thr={thr} range [{thr - 256}, {thr + 252}]")

@cocotb.test(skip=FAST_ONLY)
async def test_band_dynamic_range(dut):
    """A quiet-to-loud ramp, bit-exact, spanning the log encoder's range.

    Also added from coverage: make_pdm() holds one amplitude, so the priority
    encoder's exponent (`bexp`), the band magnitude (`bmag`) and the signed
    requantised hidden value (`hval_s`) only ever moved through part of their
    range. A ramp walks the encoder from its floor to its ceiling, which is
    where an off-by-one in the mantissa shift would sit -- and it is what a
    real approach sounds like, so it is not a synthetic case.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    b = Bench(dut, cfg)
    await b.reset()

    n_frames = min(NFRAMES_RUN, 8)
    n_ticks = n_frames << cfg.frame_log2
    # Same tone set as make_pdm, but with amplitude swept over four decades of
    # level rather than held at 0.7 of full scale.
    rng = np.random.default_rng(17)
    t = np.arange(16000) / 16000.0
    sig = np.zeros(16000, dtype=np.float32)
    for f, a in [(220, .6), (700, .5), (1500, .35), (3000, .2)]:
        sig += a * np.sin(2 * np.pi * f * t + rng.uniform(0, 6.28))
    sig /= max(abs(sig).max(), 1e-6)
    sig = (sig * np.logspace(-3, 0, 16000).astype(np.float32) * 0.95).astype(np.float32)
    bits = [1 if y[0] > 0 else 0
            for y in drone_model.pdm_encode_batch(sig[None, :], n_ticks, cfg)]

    got = await capture_frames(dut, b, bits, n_frames)
    golden = golden_frames(bits, cfg, n_frames)
    n = min(len(got), len(golden))
    assert n >= 3, f"only captured {n} frames"
    golden = [list(g[:len(got[0])]) for g in golden]
    bad = [(i, got[i], golden[i]) for i in range(n) if got[i] != golden[i]]
    for i, g, e in bad[:5]:
        dut._log.error(f"frame {i}: RTL {g} golden {e}")
    assert not bad, f"{len(bad)}/{n} frames mismatched on the ramp"

    lo = min(min(f) for f in got[:n])
    hi = max(max(f) for f in got[:n])
    dut._log.info(f"{n} frames bit-exact on the ramp; feature levels {lo}..{hi}")
    assert hi - lo >= 4, f"ramp only spanned levels {lo}..{hi}; not exercising the encoder"


@cocotb.test(skip=FAST_ONLY)
async def test_unused_inputs_ignored(dut):
    """uio_in and ena change nothing: outputs identical, features bit-exact.

    The TinyTapeout wrapper wires all eight uio pins and ena into every
    project. This design uses none of them -- uio_oe is 8'hFF, so the uio pins
    are outputs, and ena is ignored -- and nothing checked it. Every other test
    parks uio_in at 0 and ena at 1, so toggle coverage had all 17 of their
    points at zero (scripts/coverage.sh): a refactor that read uio_in[0] as a
    second data input, or gated the FSM on ena, would have passed the suite.

    Same stimulus twice. The reference run parks the pins; the second walks a
    counter across uio_in, so every bit moves both ways, and drops ena for 16
    of every 32 mic ticks. The second run must be bit-exact against the golden
    model, which knows nothing of either pin, and its uo_out/uio_out/uio_oe
    trace must equal the reference clock for clock, debug pins included.
    """
    cocotb.start_soon(Clock(dut.clk, 20, units="ns").start())
    cfg = test_cfg()
    n_frames = min(NFRAMES_RUN, 4)
    bits = make_pdm(n_frames << cfg.frame_log2, cfg, seed=7)

    async def run(wiggle):
        b = Bench(dut, cfg)
        await b.reset()
        trace, frames, prev_st = [], [], 0
        for tick_i, bit in enumerate(bits):
            b.set_bit(bit)
            if wiggle:
                dut.uio_in.value = tick_i & 0xFF
                dut.ena.value = (tick_i >> 4) & 1
            for _ in range(PDM_DIV):
                await RisingEdge(dut.clk)
                trace.append((int(dut.uo_out.value), int(dut.uio_out.value),
                              int(dut.uio_oe.value)))
                st = read_state(dut)
                if st == S_CLASS and prev_st != S_CLASS:
                    frames.append(read_fmax(dut, cfg.nband))
                prev_st = st
        return trace, frames

    ref_trace, ref_frames = await run(wiggle=False)
    got_trace, got_frames = await run(wiggle=True)

    golden = [list(g) for g in golden_frames(bits, cfg, n_frames)]
    n = min(len(got_frames), len(golden))
    assert n >= 3, f"only captured {n} frames"
    bad = [i for i in range(n) if got_frames[i] != golden[i]]
    for i in bad[:5]:
        dut._log.error(f"frame {i}: RTL {got_frames[i]} golden {golden[i]}")
    assert not bad, f"{len(bad)}/{n} frames not bit-exact with uio_in/ena moving"

    assert len(got_trace) == len(ref_trace), "runs are different lengths"
    diff = [i for i, (r, g) in enumerate(zip(ref_trace, got_trace)) if r != g]
    if diff:
        i = diff[0]
        dut._log.error(f"clk {i}: parked {ref_trace[i]} vs moving {got_trace[i]}")
    assert not diff, f"{len(diff)}/{len(ref_trace)} clocks differ once uio_in/ena move"
    dut._log.info(f"{n} frames bit-exact and {len(ref_trace)} clocks identical "
                  f"with uio_in walking and ena toggling")
