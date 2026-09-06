// SPDX-FileCopyrightText: 2026
// SPDX-License-Identifier: Apache-2.0
//
// tt_um_hyphen133_drone_detection -- self-contained acoustic drone detector
// for a TinyTapeout IHP 1x1 tile. No host, memory, or weights to load.
//
//   PDM microphone (1 bit)  ->  [ this chip ]  ->  LED
//
// Signal chain, integer throughout, and with no multiplier anywhere:
//
//   1. Generate the mic clock, CLK / PDM_DIV, and sample one bit per period.
//   2. Dyadic 1-pole cascade. stage[b] += (in - stage[b]) >>> K_SHIFT, with
//      stage b clocked once every 2^b mic ticks -- decimate-by-two per octave,
//      so one shared shift-and-add serves the whole filterbank.
//   3. band[b] = stage[b-1] - stage[b], an octave band-pass.
//   4. feat = log2|band| from a priority encoder plus MANT mantissa bits.
//      The log is free: it is the encoder's output, not a computation.
//   5. Per-band maximum over a frame of 2^FRAME_LOG2 ticks.
//   6. NHID hidden units, each an NFRAME x NBAND ternary template accumulated
//      frame by frame, then clamp(acc >> HSHIFT, 0, 15). A purely linear
//      template caps out near 84 % AUC on these features; one hidden layer
//      reaches the required capacity. NPHASE staggered copies mean detection
//      does not depend on alignment with a window boundary.
//   7. Output layer: one ternary weight per hidden unit; over threshold ->
//      latch the LED for HOLD_FRAMES.
//
// The fixed drone_2 template weights come from drone_weights.svh. Ternary
// weights cost nothing here: a zero drops that term from the adder tree.

`default_nettype none

module tt_um_hyphen133_drone_detection #(
    parameter PDM_DIV_LOG2 = 5,     // mic clock = clk / 2^PDM_DIV_LOG2
    parameter NSTAGE       = 9,    // cascade depth
    parameter K_SHIFT      = 2,     // 1-pole coefficient
    parameter STATE_W      = 10,    // signed cascade state
    // drone_2 geometry: five octave bands and a 671 ms integration window.
    // This exact configuration completed the IHP sg13g2 1x1 flow at 94.60%
    // final core utilization with clean DRC, LVS, antenna, and timing checks.
    parameter TAP0         = 4,     // stages 3..8, keeping the lowest band
    parameter NBAND        = 5,
    parameter FRAME_LOG2   = 16,    // 65_536 mic ticks = 41.9 ms
    parameter NFRAME       = 16,    // 671 ms of integration under one window
    parameter MANT         = 1,     // mantissa bits in the log -> 3 dB steps
    parameter FEAT_W       = 4,
    // How many bands also keep a frame *mean* beside their frame maximum, out
    // of NBAND, counted from the deepest tap -- i.e. the AVG_N lowest-frequency
    // bands. 0 is what has always shipped: the maximum alone.
    //
    // The mean is what the maximum throws away, and on the eight detectors in
    // docs/task_optimization.md it is worth up to +7 test AUC, more than every
    // cascade change measured combined. It is also where the area goes, one
    // AVG_W accumulator per averaged band, so AVG_N is a parameter rather than
    // a flag: the mean of the three lowest bands captures most of the gain of
    // averaging all six, for half the accumulators.
    //
    // An exact mean would need a per-band divisor: band b ticks 2^(FRAME_LOG2-b)
    // times per frame, 8192 at band 3 against 256 at band 8. So sample instead
    // of averaging everything -- take 2^AVG_SHIFT samples per band per frame,
    // the same count for every band, and the divide is a constant >> AVG_SHIFT.
    //
    // The sampling instant is the same for every band, which is what makes
    // this nearly free: band b is due when cnt[b-1:0] is zero, and "every
    // 2^(FRAME_LOG2-b-AVG_SHIFT)-th tick of band b" reduces to
    // cnt[FRAME_LOG2-AVG_SHIFT-1:0] == 0 for all of them. One AND over counter
    // bits that already exist, shared across the ring.
    //
    // A leaky integrator was tried first and is not good enough: it averages
    // over ~2^K ticks rather than over the frame, and at the low bands that is
    // a small fraction of one frame. It recovers about a third of the gain.
    parameter AVG_N        = 0,
    parameter AVG_SHIFT    = 6,     // log2 of the samples per band per frame
    parameter NPHASE       = 2,     // staggered windows, hop = NFRAME/NPHASE
    parameter NHID         = 4,     // hidden units; 1 == the old linear template
    parameter HACC_W       = 6,     // saturating hidden accumulator
    parameter HSHIFT       = 1,
    parameter FEAT_OFF     = 6,     // constant subtracted from each band
                                    // feature before the adder tree; keeps
                                    // the accumulator and bias small     // hidden requantise: clamp(acc>>HSHIFT,0,15)
    parameter SCORE_W      = 10,
    parameter HOLD_FRAMES  = 16,
    parameter DEBUG_PINS   = 1      // 0: uo_out[7:4] and uio_out driven low
) (
    input  wire [7:0] ui_in,    // [0] PDM data in, [7:1] threshold trim
    output wire [7:0] uo_out,   // [0] PDM clock out, [3:1] detections, [7:4] debug
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // Fixed drone_2 weights. Keeping one header makes the submitted build
  // independent of command-line defines used by the original multi-model repo.
  `include "drone_weights.svh"

  localparam IN_AMP   = 1 << (STATE_W - 3);
  localparam FEAT_MAX = (1 << FEAT_W) - 1;
  // Features the template reads per frame: every band's maximum, then the
  // AVG_N means. AVG_W is one accumulator: 2^AVG_SHIFT samples of at most
  // FEAT_MAX cannot overflow FEAT_W+AVG_SHIFT bits. NAVG is 1 rather than 0
  // when AVG_N=0 because a zero-length array is not portable; the ring is
  // unread in that case and synthesis removes it.
  localparam NFEAT    = NBAND + AVG_N;
  localparam AVG_W    = FEAT_W + AVG_SHIFT;
  localparam NAVG     = (AVG_N > 0) ? AVG_N : 1;
  // The averaged bands are the last AVG_N the tap loop visits, so the ring
  // rotates only during those steps and is back in order for the classifier.
  localparam AVG_STG0 = TAP0 + NBAND - AVG_N;
  localparam FIDX_W   = $clog2(NFRAME);
  localparam HOP      = NFRAME / NPHASE;
  localparam CNT_W    = FRAME_LOG2 + FIDX_W;
  localparam STG_W    = $clog2(NSTAGE + 1);
  localparam NSLOT    = NPHASE * NHID;
  // NPHASE and NHID are powers of two, so the classifier step index splits
  // into {phase, hidden unit} by bit slicing instead of a divider.
  localparam PH_W     = (NPHASE <= 1) ? 1 : $clog2(NPHASE);
  localparam HD_W     = (NHID   <= 1) ? 1 : $clog2(NHID);
  localparam SLOT_W   = PH_W + HD_W;
  localparam BAND_W   = STATE_W + 1;
  localparam MANT_W   = (MANT < 1) ? 1 : MANT;   // avoid a [-1:0] vector

  // ---------------------------------------------------------------------
  // Microphone clock and input sampling
  // ---------------------------------------------------------------------
  logic [PDM_DIV_LOG2-1:0] div;
  logic                    pdm_bit;
  wire                     tick = (div == {{(PDM_DIV_LOG2-1){1'b0}}, 1'b1});

  // Async reset throughout: sg13g2 has no reset-less flop, so a synchronous
  // reset costs a tie-high cell on every RESET_B pin plus a reset mux in
  // front of every D input (~3 000 um^2 here). TinyTapeout deasserts rst_n
  // synchronously to clk, so the async form is safe.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      div     <= '0;
      pdm_bit <= 1'b0;
    end else begin
      div <= div + 1'b1;
      // Sample mid-way through the high phase of the emitted mic clock.
      if (div == {2'b11, {(PDM_DIV_LOG2-2){1'b0}}}) pdm_bit <= ui_in[0];
    end
  end

  assign uo_out[0] = div[PDM_DIV_LOG2-1];   // mic clock

  // ---------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------
  // Cascade states live in a ROTATING shift register, not an addressed array.
  // Every mic tick rotates all NSTAGE words past a single compute slot, so the
  // NSTAGE:1 read mux and the 1:NSTAGE write demux both disappear; the price
  // is that the cascade always takes NSTAGE clocks instead of only the due ones.
  logic signed [STATE_W-1:0] ring [NSTAGE];
  // Per-band frame maxima, also a ROTATING ring: the tap stages visit the
  // bands in order 0..NBAND-1 on every tick, so the band being updated is
  // always fmax[0] and the new value goes to the tail; after the NBAND tap
  // steps the ring is back in band order for the classifier's parallel read.
  logic        [FEAT_W-1:0]  fmax  [NBAND];
  // Per-band frame-mean accumulator, rotated in lockstep with fmax so the band
  // under update is always at the head, and cleared with it at the frame
  // boundary. It holds the sum of 2^AVG_SHIFT samples, so the mean is the top
  // FEAT_W bits.
  logic        [AVG_W-1:0]   favg  [NAVG];
  // Hidden accumulators, one per (phase, unit), also kept as a ROTATING ring:
  // S_CLASS visits the NSLOT slots in a fixed order every frame, so the
  // current slot's accumulator is always hacc[0] and the result goes to the
  // tail. No NSLOT:1 read mux, no 1:NSLOT write demux.
  logic signed [HACC_W-1:0]  hacc  [NSLOT];
  // Output-layer sum: NHID terms of at most 15 each, so it needs only
  // OSUM_W bits; the compare against the SCORE_W-bit threshold sign-extends.
  localparam OSUM_W = $clog2(NHID * 15 + 1) + 1;
  logic signed [OSUM_W-1:0]  osum;
  logic        [FIDX_W:0]    hold;

  logic [CNT_W-1:0]  cnt;          // mic-tick counter: framing + decimation
  logic [STG_W-1:0]  stg;          // cascade step
  logic [SLOT_W-1:0] slot;         // classifier step = {phase, word}
  logic [1:0]        st;
  localparam [1:0] S_IDLE = 2'd0, S_CASC = 2'd1, S_CLASS = 2'd2, S_ROLL = 2'd3;

  wire [FIDX_W-1:0] frame_idx = cnt[CNT_W-1:FRAME_LOG2];
  wire              frame_end = &cnt[FRAME_LOG2-1:0];

  // ---------------------------------------------------------------------
  // Cascade datapath -- one shared subtract-shift-add
  // ---------------------------------------------------------------------
  // No size casts here on purpose: yosys reads `-STATE_W'(IN_AMP)` as a cast
  // with a negated width and produces +IN_AMP, i.e. a rectified microphone.
  // iverilog and the Python model give -IN_AMP. Caught by gate-level sim.
  localparam signed [STATE_W-1:0] X_POS = IN_AMP;
  localparam signed [STATE_W-1:0] X_NEG = -IN_AMP;
  wire signed [STATE_W-1:0] x_in = pdm_bit ? X_POS : X_NEG;
  // The previous stage's output is whatever the last rotation wrote to the
  // ring tail: casc_nx if that stage was due, else its unchanged state. When
  // it was not due, this stage is not due either and casc_in is dead, so no
  // separate prev_v register is needed.
  wire signed [STATE_W-1:0] casc_in = (stg == 0) ? x_in : ring[NSTAGE-1];
  wire signed [STATE_W-1:0] casc_st = ring[0];              // head of the ring
  wire signed [STATE_W-1:0] casc_nx = casc_st + ((casc_in - casc_st) >>> K_SHIFT);

  // Stage b is idle unless the low b bits of the tick counter are zero.
  // Once a stage is idle every deeper stage is too, so the ring still rotates
  // but stops updating.
  wire casc_due = ((cnt & ((1 << stg) - 1)) == 0);
  wire casc_last = (stg == STG_W'(NSTAGE - 1));

  wire is_tap  = (stg >= STG_W'(TAP0)) && (stg < STG_W'(TAP0 + NBAND));

  // Explicit sign extension everywhere a signed value is widened: yosys
  // zero-extends `N'(signed_expr)` where the LRM (and iverilog, and the
  // Python model) sign-extend. Found by simulating the netlist.
  wire signed [BAND_W-1:0] casc_in_w = {casc_in[STATE_W-1], casc_in};
  wire signed [BAND_W-1:0] casc_nx_w = {casc_nx[STATE_W-1], casc_nx};
  wire signed [BAND_W-1:0] band = casc_in_w - casc_nx_w;
  wire        [BAND_W-2:0] bmag = band[BAND_W-1] ? (~band[BAND_W-2:0] + 1'b1)
                                                 : band[BAND_W-2:0];

  // Priority encoder: exp = index of the most significant set bit, +1.
  logic [4:0] bexp;
  always_comb begin
    bexp = 5'd0;
    for (int i = 0; i < BAND_W-1; i++) if (bmag[i]) bexp = 5'(i + 1);
  end

  // log2 with MANT mantissa bits below the leading one.
  logic [FEAT_W-1:0] feat;
  always_comb begin
    logic [4:0] sh;
    logic [MANT_W-1:0] mbits;
    logic [8:0] wide;
    sh    = 5'd0;
    mbits = '0;
    wide  = 9'd0;
    if (MANT == 0) begin
      wide = 9'(bexp);
    end else if (bexp <= 5'(MANT)) begin
      wide = 9'(bexp);
    end else begin
      sh    = bexp - 5'(1 + MANT);
      mbits = MANT_W'(bmag >> sh);
      wide  = (9'(bexp - 5'(MANT)) << MANT) | 9'(mbits);
    end
    feat = (wide > 9'(FEAT_MAX)) ? FEAT_W'(FEAT_MAX) : FEAT_W'(wide);
  end

  // Frame-mean sampling. Band b is due when cnt[b-1:0] is zero, and taking
  // every 2^(FRAME_LOG2-b-AVG_SHIFT)-th tick of band b reduces to the same
  // test for every band: the low FRAME_LOG2-AVG_SHIFT bits of the frame
  // counter are zero. So one AND over counter bits that already exist gates
  // the whole ring, and every band contributes exactly 2^AVG_SHIFT samples.
  //
  // This holds only while every tap is due at those instants, i.e. while
  // TAP0+NBAND-1 <= FRAME_LOG2-AVG_SHIFT. At FRAME_LOG2=16, AVG_SHIFT=6 that
  // is band 10, and the deepest tap in any build here is 8.
  localparam SAMP_W = FRAME_LOG2 - AVG_SHIFT;
  wire avg_due = (cnt[SAMP_W-1:0] == '0);
  // The accumulator cannot overflow: 2^AVG_SHIFT samples of at most FEAT_MAX
  // sum to less than 2^(FEAT_W+AVG_SHIFT) = 2^AVG_W.
  wire [AVG_W-1:0] avg_nx = favg[0] + AVG_W'(feat);

  // ---------------------------------------------------------------------
  // Classifier -- one shared ternary adder tree, time-multiplexed over
  // NPHASE x NHID accumulators once per frame.
  // ---------------------------------------------------------------------
  wire [PH_W-1:0]   c_ph   = slot[SLOT_W-1 -: PH_W];
  wire [HD_W-1:0]   c_hd   = slot[HD_W-1:0];
  wire [FIDX_W-1:0] c_slot = frame_idx - FIDX_W'(c_ph * HOP);
  wire              win_end = (c_slot == FIDX_W'(NFRAME-1));
  wire              last_h  = (c_hd == HD_W'(NHID-1));

  // 2 bits per weight: 01 = +1, 11 = -1, else 0. WW_ROW packs one
  // (hidden unit, frame slot) row of NBAND weights; see drone_weights.svh.
  wire [$clog2(NHID*NFRAME)-1:0] wsel = ($clog2(NHID*NFRAME))'(c_hd*NFRAME + c_slot);
  wire [2*NFEAT-1:0] wrow = WW_ROW[2*NFEAT*wsel +: 2*NFEAT];

  // Feature b of the row: every band's maximum first, in band order, then the
  // AVG_N means, also in band order -- [max0..max(NBAND-1), avg of band
  // NBAND-AVG_N .. avg of band NBAND-1]. Changing this order would silently
  // mis-decode every weight. A mean is read as
  // the top FEAT_W bits of its accumulator, which is the divide by
  // 2^AVG_SHIFT.
  logic signed [HACC_W-1:0] dot;
  always_comb begin
    logic signed [HACC_W-1:0] fc;
    logic        [FEAT_W-1:0] fv;
    dot = '0;
    for (int b = 0; b < NFEAT; b++) begin
      logic [1:0] w2;
      w2 = wrow[2*b +: 2];
      fv = (b < NBAND) ? fmax[b]
                       : FEAT_W'(favg[b - NBAND][AVG_W-1 -: FEAT_W]);
      fc = HACC_W'($signed({1'b0, fv})) - HACC_W'(FEAT_OFF);
      if (w2[0]) dot = w2[1] ? dot - fc : dot + fc;
    end
  end

  // At slot 0 the accumulator resets to a hard-wired per-unit constant. That
  // constant carries both the learned bias and the feature-centring offset,
  // so centring costs nothing in silicon.
  wire signed [HACC_W-1:0] hb = $signed(WW_HBIAS[HACC_W*c_hd +: HACC_W]);
  wire signed [HACC_W-1:0] acc_cur = (c_slot == '0) ? hb : hacc[0];
  wire signed [HACC_W:0]   acc_wide = {acc_cur[HACC_W-1], acc_cur} + {dot[HACC_W-1], dot};
  localparam signed [HACC_W:0] HA_MAX =  (1 << (HACC_W-1)) - 1;
  localparam signed [HACC_W:0] HA_MIN = -(1 << (HACC_W-1));
  wire signed [HACC_W-1:0] acc_next =
      (acc_wide >  HA_MAX) ? HACC_W'(HA_MAX) :
      (acc_wide <  HA_MIN) ? HACC_W'(HA_MIN) : HACC_W'(acc_wide);

  // Hidden activation: clamp(acc >> HSHIFT, 0, 15) -- ReLU is free again.
  wire signed [HACC_W-1:0] hsh = acc_next >>> HSHIFT;
  wire [3:0] hval = acc_next[HACC_W-1]        ? 4'd0  :
                    (|hsh[HACC_W-1:4])        ? 4'd15 : hsh[3:0];

  // Output layer: one ternary constant per hidden unit, folded into the same
  // loop, so the window closes on the step that visits the last hidden unit.
  wire [1:0] w2c = WW_W2[2*c_hd +: 2];
  wire signed [OSUM_W-1:0] hval_s = OSUM_W'($signed({1'b0, hval}));
  wire signed [OSUM_W-1:0] o_term =
      w2c[0] ? (w2c[1] ? (~hval_s + 1'b1) : hval_s) : '0;
  wire signed [OSUM_W-1:0] osum_cur  = (c_hd == '0) ? '0 : osum;
  wire signed [OSUM_W-1:0] osum_next = osum_cur + o_term;

  // Threshold: the trained constant, trimmed by the board's DIP switches.
  // trim = (ui_in[7:1] - 64) * 4, range -256..+252, as a SCORE_W-bit signed value.
  wire signed [7:0]         trim8  = $signed({1'b0, ui_in[7:1]}) - 8'sd64;
  wire signed [SCORE_W-1:0] trim   = {{(SCORE_W-8){trim8[7]}}, trim8} <<< 2;
  wire signed [SCORE_W-1:0] thresh = $signed(WW_THRESH_PK) + trim;
  wire signed [SCORE_W-1:0] osum_w = {{(SCORE_W-OSUM_W){osum_next[OSUM_W-1]}}, osum_next};
  wire fire = win_end && last_h && (osum_w > thresh);

  // ---------------------------------------------------------------------
  // Sequencer
  // ---------------------------------------------------------------------
  integer i;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st   <= S_IDLE;
      cnt  <= '0;
      stg  <= '0;
      slot <= '0;
      for (i = 0; i < NSTAGE; i++) ring[i] <= '0;
      for (i = 0; i < NBAND;  i++) fmax[i]  <= '0;
      for (i = 0; i < NAVG;   i++) favg[i]  <= '0;
      for (i = 0; i < NSLOT;  i++) hacc[i]  <= '0;
      osum <= '0;
      hold <= '0;
    end else begin
      case (st)
        S_IDLE: if (tick) begin
          stg <= '0;
          st  <= S_CASC;
        end

        S_CASC: begin
          // Rotate one position; update the head only if this stage is due.
          for (i = 0; i < NSTAGE - 1; i++) ring[i] <= ring[i + 1];
          ring[NSTAGE-1] <= casc_due ? casc_nx : casc_st;
          // Tap stages rotate the fmax ring once each; the band under
          // update is at the head and its new maximum goes to the tail.
          if (is_tap) begin
            for (i = 0; i < NBAND - 1; i++) fmax[i] <= fmax[i + 1];
            fmax[NBAND-1] <= (casc_due && (feat > fmax[0])) ? feat : fmax[0];
          end
          if (AVG_N > 0 && stg >= STG_W'(AVG_STG0)
                        && stg < STG_W'(TAP0 + NBAND)) begin
            for (i = 0; i < NAVG - 1; i++) favg[i] <= favg[i + 1];
            favg[NAVG-1] <= (casc_due && avg_due) ? avg_nx : favg[0];
          end
          if (casc_last) begin
            if (frame_end) begin
              slot <= '0;
              st   <= S_CLASS;
            end else begin
              cnt <= cnt + 1'b1;
              st  <= S_IDLE;
            end
          end else begin
            stg <= stg + 1'b1;
          end
        end

        S_CLASS: begin
          // Rotate the accumulator ring; the slot just evaluated goes to
          // the tail so that after NSLOT steps the order is restored.
          for (i = 0; i < NSLOT - 1; i++) hacc[i] <= hacc[i + 1];
          hacc[NSLOT-1] <= acc_next;
          if (win_end) osum <= osum_next;
          if (fire) hold <= (FIDX_W+1)'(HOLD_FRAMES);
          if (slot == SLOT_W'(NSLOT - 1)) st <= S_ROLL;
          else                            slot <= slot + 1'b1;
        end

        S_ROLL: begin
          for (i = 0; i < NBAND; i++) fmax[i] <= '0;
          // The mean accumulator is a per-frame statistic like the max, so it
          // clears with it. (The leaky integrator this replaced did not, which
          // is part of why it measured the wrong thing.)
          if (AVG_N > 0) for (i = 0; i < NAVG; i++) favg[i] <= '0;
          if (hold != 0) hold <= hold - 1'b1;
          cnt <= cnt + 1'b1;
          st  <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------
  wire detect = (hold != 0);
  assign uo_out[1] = detect;
  assign uo_out[2] = detect;
  assign uo_out[3] = detect;                        // the LED
  generate
    if (DEBUG_PINS != 0) begin : g_dbg
      assign uo_out[7:4]  = fmax[0][FEAT_W-1 -: 4];  // scope hook: top band level
      assign uio_out[3:0] = 4'(frame_idx);
      assign uio_out[6:4] = {st, |detect};
      assign uio_out[7]   = tick;
      assign uio_oe       = 8'hFF;
    end else begin : g_nodbg
      assign uo_out[7:4]  = 4'b0;
      assign uio_out      = 8'b0;
      assign uio_oe       = 8'b0;
    end
  endgenerate

  wire _unused = &{ena, uio_in, 1'b0};

endmodule

`default_nettype wire
