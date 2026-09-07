# Coverage

Measured, not asserted. `./scripts/coverage.sh` builds `test/cov_tb.sv` with
`verilator --binary --coverage` and reports line, branch, expression and toggle
coverage of the RTL.

Read the caveat first: verilator is not in the cocotb image and cocotb's
verilator backend needs it at build time, so this cannot instrument `test.py`
itself. `cov_tb.sv` drives the same *classes* of stimulus the suite uses -- a
chirp across every band, an amplitude ramp, both mic rails, Nyquist
alternation, near-silence, the trim range, and a mid-frame reset. It answers
"is any logic unexercised", which is the question that points at a missing
test. It is not a coverage figure for the cocotb suite.

## Where it stands

| metric | |
|---|---|
| line | 93.2 % (55/59) |
| branch | 88.5 % (69/78) |
| expression | 84.5 % (71/84) |
| toggle, DUT only | 90.7 % (907/1000 bit-transitions) |

Nothing here is a gap a test would close. Every uncovered point is either
switched off by a parameter or structurally unreachable, and the accounting
below is exact -- it sums to the measured figure rather than approximately to
it, which is the only way to tell "explained" from "hand-waved".

## The four uncovered lines

| what | why |
|---|---|
| `if (MANT == 0)` / `wide = 9'(bexp)` | `MANT=1` |
| `favg` declaration and its two shift lines | `AVG_N=0`, so the frame-mean path is dead |
| `uio_in` | unused, tied into `_unused` |
| `default: st <= S_IDLE;` | `st` is 2 bits and all four encodings are named states, so `default` cannot be reached |

The FSM default is the only one that is dead rather than switched off. It stays
as defensive code; a waiver is cheaper than the argument for removing a safe
default.

## The 93 untoggled bit-transitions

| cause | points |
|---|---|
| `AVG_N=0` dead path (`favg`, `avg_nx`) | 32 |
| `uio_in`, `ena` -- unused inputs | 17 |
| `x_in` is `±IN_AMP` and nothing else | 15 |
| `uio_oe` constant `8'hFF` at `DEBUG_PINS=1` | 8 |
| `hval_s` bits 4..6 -- `hval` is 0..15 sign-extended into `OSUM_W=7` | 6 |
| `trim[1:0]` -- `<<< 2` forces them to zero | 4 |
| `bmag` top 2 bits -- unreachable at `IN_AMP = 1 << (STATE_W-3)` = 128 | 4 |
| `thresh` -- range -242..+266 does not span 10 bits | 3 |
| `bexp` bit 4 -- maximum value is `BAND_W-1` = 10, which needs 4 bits | 2 |
| `hb`, `w2c` -- weight constants, fixed for this build | 2 |
| **total** | **93** |

`x_in` deserves a note, because 15 of 20 looks alarming and is not: the input
is one PDM bit, so `x_in` takes exactly two values, `+128` and `-128`. Only the
bits that differ between them can toggle. No stimulus can improve that short of
a wider input, which the chip does not have.

## What the measurement actually changed

Two of the eleven tests came out of this rather than out of guesswork:

* **`test_threshold_trim_arithmetic`.** `thresh` had 7 untoggled points because
  the suite only ever drove four trim values. `test_trim_raises_threshold`
  covers the *behaviour* but cannot sweep -- it needs whole frames per point to
  count detections. The arithmetic is combinational, so the new test reads
  `thresh` straight off the pins across 11 trim values chosen to toggle every
  bit of `ui_in[7:1]` in both directions, and checks it against
  `WW_THRESH_PK + ((trim-64) << 2)`. 7 -> 3 untoggled, the remainder being the
  range limit above. Cost: 0.01 s.
* **`test_band_dynamic_range`.** `make_pdm()` holds one amplitude, so the log
  encoder never walked its exponent range. The new test sweeps amplitude over
  three decades and holds the result to the same bit-exact standard, then
  asserts the observed feature levels span at least 4 steps (measured 0..5).
  This one did *not* move the toggle count -- `bexp` and `hval_s` turned out to
  be structurally capped, per the table -- but it is the test that establishes
  as much, and a real approach is a ramp rather than a plateau.

An earlier reading of the same data called `bmag`, `bexp` and `hval_s` real
gaps. That was wrong, and the arithmetic above is why: their untoggled counts
match the sign-extension and range limits exactly. Only `thresh` was a test
gap.

## Coverage is not the same as the checks biting

Coverage says the logic ran. It says nothing about whether anything would have
noticed a wrong answer. `scripts/assert_mutations.sh` is the other half: it
reintroduces seven real bugs and requires the named assertion to fire for each.
It has already earned its place by finding that `A_HOLD_STEP` permitted a hold
counter stuck on forever -- 100 % line coverage over that code would not have
hinted at it. See [hold_width.md](hold_width.md).

## Running it

```bash
./scripts/coverage.sh
KEEP=1 ./scripts/coverage.sh           # annotated source into artifacts/coverage/
```

The script fails if any `WW_ASSERT` check trips during the coverage run -- a
stimulus that reaches an invariant-breaking state matters more than the number.
