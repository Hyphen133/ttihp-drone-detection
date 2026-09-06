## How it works

This project is a self-contained acoustic drone detector. A one-bit PDM
microphone stream enters on `ui[0]`; an active-high detection signal appears on
`uo[1]`, `uo[2]`, and `uo[3]`. The complete model is fixed in the logic, with
no processor, firmware, RAM, or external weight storage.

The multiplier-free integer pipeline is:

1. Divide the 50 MHz input clock by 32 and output the resulting 1.5625 MHz PDM
   microphone clock on `uo[0]`.
2. Feed microphone samples through a nine-stage dyadic one-pole cascade. One
   shared subtract/shift/add datapath services the rotating state ring.
3. Difference adjacent stages to obtain five octave bands, then encode their
   magnitudes as four-bit logarithmic features.
4. Keep each band's maximum over a 41.9 ms frame. Sixteen frames form a 671 ms
   classifier window.
5. Accumulate four 16×5 ternary templates in signed six-bit saturating
   accumulators. Two staggered windows reduce boundary sensitivity.
6. Requantize the hidden activations, apply the ternary output layer, compare
   against the trimmed threshold, and hold a detection for sixteen frames.

The `drone_2` weights reached 98.92% test AUC on the DADS evaluation split.
Reference hardening of the same logic completed in one IHP sg13g2 1×1 tile at
94.60% final core utilization with clean DRC, LVS, antenna, setup, and hold
checks.

![Reference drone_2 layout](drone_2_layout.png)

## How to test

1. Connect a compatible PDM microphone's data pin to `ui[0]` and clock pin to
   `uo[0]`, together with the appropriate board power and ground.
2. Apply the specified 50 MHz system clock and release reset.
3. Start with threshold trim `ui[7:1] = 7'd63`, which produces an effective
   threshold of 10. Detection appears on `uo[3:1]`.
4. For higher recall, use trim 62 (effective threshold 6). For fewer false
   triggers, increase the trim toward or above the neutral value 64.
5. Check `uo[7:4]` for a changing band-level value to confirm that microphone
   data is reaching the front end.

Each trim increment changes the decision threshold by four score units. Dataset
operating points were 65.8% recall / 0.7% negative clips firing at threshold 10
and 95.2% / 4.1% at threshold 6. These clip-level figures do not predict alarms
per hour in a real deployment.

## External hardware

- One PDM MEMS microphone compatible with the TinyTapeout board's I/O voltage.
- Two signal connections: microphone data to `ui[0]` and microphone clock from
  `uo[0]`.
- DIP switches or another seven-bit source on `ui[7:1]` for threshold trim.
- An LED or logic input on any of `uo[1]`, `uo[2]`, or `uo[3]`.

All `uio` pins are debug outputs: `uio[3:0]` is the frame index, `uio[4]` is
detection, `uio[6:5]` is the internal FSM state, and `uio[7]` is the microphone
tick.

## Limitations

The detector was evaluated on DADS clips, not long-duration recordings at a
deployed site. Detection range is unknown. Rotorcraft, engines, lawn equipment,
and other sustained low-frequency sounds are plausible confusers. Treat the
output as a sensor indication that requires deployment-specific validation,
not as an authenticated identification of an aircraft.
