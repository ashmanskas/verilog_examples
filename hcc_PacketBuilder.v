`default_nettype none
`timescale 1ns/1ps

module hcc_PacketBuilder
  (
   input  wire        clk160,  // all PB logic is driven by 160 MHz clk
   input  wire        rstb,    // asynchronous reset* for all flipflops

   input  wire [15:0] HPR_data_i,  // (ABC) HPR data input (4 words per HPR)
   input  wire [15:0] RR_data_i,   // (ABC) RR data input (4 words per RR)
   input  wire [11:0] PR_data_i,   // PR physics data input (1 word / cluster)
   input  wire [11:0] LP_data_i,   // LP physics data input

   input  wire [10:0] HPR_empties_i,  // HPR FIFO empty flags, one per IC
   input  wire [10:0] RR_empties_i,   // RR FIFO empty flags per IC
   input  wire [10:0] PR_empties_i,   // PR FIFO empty flags
   input  wire [10:0] LP_empties_i,   // LP FIFO empty flags

   input  wire [10:0] dispatch_tag_lp_i,    // next {BCID(4), LP tag(7)}
   input  wire        dispatch_empty_lp_i,  // LP tag FIFO empty
   output wire        dispatch_re_lp_o,     // increment LP tag FIFO
   input  wire [10:0] dispatch_tag_pr_i,    // next {BCID(4), PR tag(7)}
   input  wire        dispatch_empty_pr_i,  // PR tag FIFO empty
   output wire        dispatch_re_pr_o,     // increment PR tag FIFO

   input  wire [1:0]  operating_mode_i,  // Physics, Register Read Special,
                                         //   Pack Transparent, Full Transpar
   input  wire [10:0] IC_enabled_i,    // Input Channel enabled
   input  wire [10:0] BCMM_squelch_i,  // mask BC match from generating errblk
   input  wire        out_full_i,      // output FIFO (almost) full

   input  wire [39:0] HCC_HPR_line_i,   // HCC HPR data (one word per HPR)
   input  wire        HCC_HPR_empty_i,  // HCC HPR data FIFO empty
   input  wire [39:0] ReadReg_line_i,   // HCC Register Read data
   input  wire        ReadReg_empty_i,  // HCC Register Read FIFO empty

   output wire [3:0]  HPR_addr_o,  // HPR Input Channel address (mux select)
   output wire        HPR_rinc_o,  // HPR FIFO read-increment
   output wire [3:0]  RR_addr_o,   // RR Input Channel address
   output wire        RR_rinc_o,   // RR FIFO read-increment
   output wire [3:0]  PR_addr_o,   // PR Input Channel address
   output wire        PR_rinc_o,   // PR FIFO read-increment
   output wire [3:0]  LP_addr_o,   // LP Input Channel address
   output wire        LP_rinc_o,   // LP FIFO read-increment

   output wire        HCC_HPR_inc_o,  // increment FIFO for HCC HPR data
   output wire        ReadReg_inc_o,  // increment FIFO for HCC RR data

   output wire [8:0]  hcc_out_o,     // data output {K, word[7:0]}
   output wire        out_we_o       // data output FIFO write-enable
   );

`include "../modules/common/Common_Parameters.vh"
`include "HCCstar/HCCStar_Parameters.vh"
`include "OutputPath/Kontrol_8b10b.vh"

    // Convenient abbreviations: NE = NOT empty (i.e. data available)
    wire        dp_ne = !dispatch_empty_pr_i;  // dispatch PR not-empty
    wire        dl_ne = !dispatch_empty_lp_i;  // dispatch LP not-empty
    wire        hh_ne = !HCC_HPR_empty_i;      // HCC HPR not-empty
    // [15:0] (zero-padded), not [10:0], because of [ichan] multiplexing
    // I think Verilog would have zero-padded automagically, but this
    // makes it explicit that impossible input channels appear to be
    // permanently empty
    wire [15:0] hp_ne = {5'b0,~HPR_empties_i};  // (ABC) HPR FIFOs not-empty
    wire [15:0] pr_ne = {5'b0,~PR_empties_i};   // PR physics FIFOs not-empty
    wire [15:0] lp_ne = {5'b0,~LP_empties_i};   // LP physics FIFOs not-empty
    wire        hr_ne = !ReadReg_empty_i;       // HCC RR FIFO not-empty
    wire [15:0] rr_ne = {5'b0,~RR_empties_i};   // (ABC) RR FIFOs not-empty
    wire [39:0] hh_dat = HCC_HPR_line_i;  // HCC HPR data
    wire [15:0] hp_dat = HPR_data_i;      // (ABC) HPR data (4 words needed)
    wire [11:0] pr_dat = PR_data_i;       // PR physics data (one FIFO word ..)
    wire [11:0] lp_dat = LP_data_i;       // LP physics data (.. per cluster)
    wire [39:0] hr_dat = ReadReg_line_i;  // HPR RR data (only 1 word needed)
    wire [15:0] rr_dat = RR_data_i;       // (ABC) RR data (4 words needed)
    wire out_full = out_full_i;  // almost full (8 spaces left)
    // Keep track of list of pending Input Channels ("_d" = combinational)
    // Confusingly, the 'wire' vectors 'ic_todo' and 'ichan' are in fact
    // flipflop outputs (of dffe_nbit_t instances), while the 'reg' vectors
    // are in fact combinational logic that must be Verilog 'reg' variables,
    // as they are driven by a combinational always block.  I try to comment
    // 'comb' explicitly to indicate Verilog regs that are in fact not FFs
    // but combinational logic driven by a combinational always block.
    wire [15:0] ic_todo;    // zero-pad to [15:0] for [ichan] multiplexing
    reg  [10:0] ic_todo_d;  // combinational: next value of ic_todo
    wire  [3:0] ichan;      // ichan = address into "backbus" IC multiplexing
    reg   [3:0] ichan_d;    // combinational: next value of ichan
    dffe_nbit_t #(.W(11)) ic_todo_reg_t  // triplicated N-bit-wide register
      (.rstb(rstb), .clk(clk160), .ena(1'b1),
       .d(ic_todo_d), .q(ic_todo[10:0]), .serOut());  // soft-error out unused
    assign ic_todo[15:11] = 5'b0;
    dffe_nbit_t #(.W(4)) ichan_reg_t
      (.rstb(rstb), .clk(clk160), .ena(1'b1),
       .d(ichan_d), .q(ichan), .serOut());
    assign HPR_addr_o = ichan;  // all backbus addresses equal 'ichan'
    assign PR_addr_o  = ichan;
    assign LP_addr_o  = ichan;  // The 'backbus' multiplexes the outputs
    assign RR_addr_o  = ichan;  // of the 11 input channels' output FIFOs.
    // The 'IC_enabled_i' (which input channels are enabled) and
    // 'operating_mode_i[1:0]' (select among 4 possible modes of operation)
    // inputs get abbreviated names here and are registered such that their
    // apparent values only update in the START and IDLE states.
    wire [10:0] ic_ena;
    wire  [1:0] opmode;
    reg         ic_ena_ena;  // combinational: enables ic_ena and opmode FFs
    dffe_nbit_t #(.W(11)) ic_ena_reg_t
      (.rstb(rstb), .clk(clk160), .ena(ic_ena_ena),
       .d(IC_enabled_i), .q(ic_ena), .serOut());
    dffe_nbit_t #(.W(2)) opmode_reg_t
      (.rstb(rstb), .clk(clk160), .ena(ic_ena_ena),
       .d(operating_mode_i), .q(opmode), .serOut());
    // Registered copies of not-empty flags for LP and PR dispatch
    // FIFOs, to improve the timing margin on clk160 signal paths.
    // Also a doubly-registered copy, which is used to detect the
    // transition from empty to not-empty.
    //
    // Dear Reader: These registered copies of various inputs from
    // FIFOs and of a few 'rinc' outputs back to FIFOs seem to me to
    // be a likely potential source of bugs, as relevant information
    // can be misaligned in time by a clock cycle.  These timing
    // optimizations were made after the verification was quite
    // mature; bugs did appear in nighly "continuous integration" runs
    // and were fixed.  Subtler bugs may yet lurk?  All of these _ff
    // signals deserve close scrutiny.
    reg dl_ne_ff, dl_ne_ff1, dp_ne_ff, dp_ne_ff1;
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            dl_ne_ff  <= 1'b0;  // dispatch LP not-empty, one FF delay
            dl_ne_ff1 <= 1'b0;  // ditto, with second FF delay
            dp_ne_ff  <= 1'b0;  // dispatch PR not-empty, one FF delay
            dp_ne_ff1 <= 1'b0;  // ditto, with second FF delay
        end else begin
            dl_ne_ff  <= dl_ne;
            dl_ne_ff1 <= dl_ne_ff;
            dp_ne_ff  <= dp_ne;
            dp_ne_ff1 <= dp_ne_ff;
        end
    end
    reg  [15:0] lp_ne_ff, pr_ne_ff;
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            lp_ne_ff <= 16'b0;  // LP physics data not-empty, one FF delay
            pr_ne_ff <= 16'b0;  // PR physics data not-empty, one FF delay
        end else begin
            lp_ne_ff <= lp_ne;
            pr_ne_ff <= pr_ne;
        end
    end
    // Count clk160 cycles spent in the condition that some but not all
    // ICs have LP data waiting to be processed.  Whereas we leave the
    // IDLE state when one or more enabled PR channels have available
    // data, we do not leave the IDLE state until ALL enabled LP channels
    // have available data -- but this counter allows us to time out in
    // case some subset of the enabled LP channels arrive very late.
    // Counter resets to zero if no LP data are waiting; counter stops
    // incrementing to prevent rollover back to zero.
    wire lp_any_waiting = dl_ne_ff && (lp_ne_ff & ic_ena);
    wire lp_all_waiting = dl_ne_ff && ((lp_ne_ff & ic_ena)==ic_ena) && ic_ena;
    wire [PB_LP_TIMEOUT_BITS-1:0] lp_tmo_ticks;  // timeout ticks ..
    wire [PB_LP_TIMEOUT_BITS-1:0] lp_tmo_ticks_d;  // .. (nominally 1023)
    assign lp_tmo_ticks_d =  // counter sticks once all bits are 1
               !lp_any_waiting ? 1'b0 :
               ~lp_tmo_ticks   ? lp_tmo_ticks + 1'b1 : lp_tmo_ticks;
    wire lp_timedout = !(~lp_tmo_ticks);  // true if ALL bits are 1
    dffe_nbit_t #(.W(PB_LP_TIMEOUT_BITS)) lp_tmo_ticks_t
      (.rstb(rstb), .clk(clk160), .ena(1'b1),
       .d(lp_tmo_ticks_d), .q(lp_tmo_ticks), .serOut());
    // Register 4 consecutive ReadReg FIFO words, for one complete transaction
    wire [15:0] rr0, rr1, rr2, rr3;
    reg  [15:0] rr0_d, rr1_d, rr2_d, rr3_d;   // comb
    reg  rr0_ena, rr1_ena, rr2_ena, rr3_ena;  // comb
    dffe_nbit_t #(.W(16)) rr0_reg_t
      (.rstb(rstb), .clk(clk160), .ena(rr0_ena),
       .d(rr0_d), .q(rr0), .serOut());
    dffe_nbit_t #(.W(16)) rr1_reg_t
      (.rstb(rstb), .clk(clk160), .ena(rr1_ena),
       .d(rr1_d), .q(rr1), .serOut());
    dffe_nbit_t #(.W(16)) rr2_reg_t
      (.rstb(rstb), .clk(clk160), .ena(rr2_ena),
       .d(rr2_d), .q(rr2), .serOut());
    dffe_nbit_t #(.W(16)) rr3_reg_t
      (.rstb(rstb), .clk(clk160), .ena(rr3_ena),
       .d(rr3_d), .q(rr3), .serOut());
    // The following tracks header/metadata for LP/PR physics event in progress
    wire hdrwritten;  // Have we yet written out a header for this event?
    reg  hdrwritten_d, hdrwritten_ena;  // comb
    dffe_nbit_t #(.W(1)) hdrwritten_reg_t
      (.rstb(rstb), .clk(clk160), .ena(hdrwritten_ena),
       .d(hdrwritten_d), .q(hdrwritten), .serOut());
    wire [10:0] bcmismatch;  // Has a BCID mismatch been detected (per IC)?
    reg  [10:0] bcmismatch_d;  // comb
    reg  bcmismatch_ena;       // comb
    dffe_nbit_t #(.W(11)) bcmismatch_reg_t
      (.rstb(rstb), .clk(clk160), .ena(bcmismatch_ena),
       .d(bcmismatch_d), .q(bcmismatch), .serOut());
    wire [10:0] tmoerr;  // Has a timeout error been detected per IC?
    reg  [10:0] tmoerr_d;  // comb
    reg         tmoerr_ena;  // comb
    dffe_nbit_t #(.W(11)) tmoerr_reg_t
      (.rstb(rstb), .clk(clk160), .ena(tmoerr_ena),
       .d(tmoerr_d), .q(tmoerr), .serOut());
    wire [10:0] l0mismatch;  // Has a L0ID mismatch been detected per IC?
    reg  [10:0] l0mismatch_d;  // comb
    reg  l0mismatch_ena;       // comb
    dffe_nbit_t #(.W(11)) l0mismatch_reg_t
      (.rstb(rstb), .clk(clk160), .ena(l0mismatch_ena),
       .d(l0mismatch_d), .q(l0mismatch), .serOut());
    wire suppress;  // Should we suppress writing clusters for this ABC packet?
    reg  suppress_d, suppress_ena;  // comb
    dffe_nbit_t #(.W(1)) supress_reg_t
      (.rstb(rstb), .clk(clk160), .ena(suppress_ena),
       .d(suppress_d), .q(suppress), .serOut());
    wire [10:0] abcerr;  // Has the ABC error bit been detected per IC?
    reg  [10:0] abcerr_d;  // comb
    reg  abcerr_ena;       // comb
    dffe_nbit_t #(.W(11)) abcerr_reg_t
      (.rstb(rstb), .clk(clk160), .ena(abcerr_ena),
       .d(abcerr_d), .q(abcerr), .serOut());
    wire [3:0] bcid;  // known (true) BCID for physics event in progress
    reg  [3:0] bcid_d;  // comb
    reg  bcid_ena;      // comb
    dffe_nbit_t #(.W(4)) bcid_reg_t
      (.rstb(rstb), .clk(clk160), .ena(bcid_ena),
       .d(bcid_d), .q(bcid), .serOut());
    wire headerq;  // true if current LP/PR FIFO word is a header word
    reg  headerq_d, headerq_ena;  // comb
    dffe_nbit_t #(.W(1)) headerq_reg_t
      (.rstb(rstb), .clk(clk160), .ena(headerq_ena),
       .d(headerq_d), .q(headerq), .serOut());
    wire [2:0] clucnt;  // count clusters (wrap after 0,1,2,3,4) for this IC
    reg  clucnt_inc, clucnt_clr;  // comb
    dffe_nbit_t #(.W(3)) clucnt_reg_t
      (.rstb(rstb), .clk(clk160), .ena(clucnt_clr || clucnt_inc),
       .d(clucnt_clr ? 1'b0 : clucnt + 1'b1), .q(clucnt), .serOut());
    wire [6:0] l0tag;     // known (true) L0ID expected for current event
    reg  [6:0] l0tag_d;   // comb
    reg       l0tag_ena;  // comb
    dffe_nbit_t #(.W(7)) l0tag_reg_t
      (.rstb(rstb), .clk(clk160), .ena(l0tag_ena),
       .d(l0tag_d), .q(l0tag), .serOut());
    wire [5:0] ticks_since_lp;  // How long has LP dispatch FIFO been ..
    // .. non-empty?  Counters (no wrap: stop at 6'b111111) used in LP and PR
    // "flush" states: If LP or PR physics data FIFOs are non-empty when
    // corresponding dispatch FIFO is empty, data in FIFOs must be junk.
    // Flush it out.  Since ABC-to HCC event transmission takes at least 64
    // ticks, no need to interrupt flushing (once begun) until 63 ticks after
    // dispatch FIFO goes non-empty.
    dffe_nbit_t #(.W(6)) ticks_since_lp_reg_t
      (.rstb(rstb), .clk(clk160),
       .ena((dl_ne_ff && !dl_ne_ff1) || (~ticks_since_lp)),  // don't wrap to 0
       .d((dl_ne_ff && !dl_ne_ff1) ? 1'b0 : ticks_since_lp + 1'b1),
       .q(ticks_since_lp), .serOut());
    wire [5:0] ticks_since_pr;  // How long has PR dispatch FIFO been nonempty?
    dffe_nbit_t #(.W(6)) ticks_since_pr_reg_t
      (.rstb(rstb), .clk(clk160),
       .ena((dp_ne_ff && !dp_ne_ff1) || (~ticks_since_pr)),  // don't wrap to 0
       .d((dp_ne_ff && !dp_ne_ff1) ? 1'b0 : ticks_since_pr + 1'b1),
       .q(ticks_since_pr), .serOut());
    // LP (not PR) flag to let both physics modes share same FSM states
    wire lp_Npr;  // LP_notPR
    reg  lp_Npr_d, lp_Npr_ena;  // comb
    dffe_nbit_t #(.W(1)) lp_Npr_reg_t
      (.rstb(rstb), .clk(clk160), .ena(lp_Npr_ena),
       .d(lp_Npr_d), .q(lp_Npr), .serOut());
    // Abbreviate "PH" for physics to avoid duplicating LP and PR logic in FSM
    wire [15:0] ph_ne_ff = (lp_Npr ? lp_ne_ff : pr_ne_ff);
    wire        df_ne = (lp_Npr ? dl_ne_ff : dp_ne_ff);  // "dispatch FIFO" ne
    // Registered copies of not-empty flags for ABC HPR and ABC RR, to
    // improve the timing margin on clk160 signal paths
    reg [15:0] hp_ne_ff;  // per-IC (ABC) HPR not-empty flag, flipflop-delayed
    reg [15:0] rr_ne_ff;  // per-IC (ABC) RR not-empty flag, flipflop-delayed
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            hp_ne_ff <= 16'b0;
            rr_ne_ff <= 16'b0;
        end else begin
            hp_ne_ff <= hp_ne;
            rr_ne_ff <= rr_ne;
        end
    end
    // More multiplexing LP/PR info to allow shared "PH"ysics FSM states
    wire [6:0]  disp_tag = (lp_Npr ?  // expected BCID/L0ID truth info
                            dispatch_tag_lp_i[6:0] : dispatch_tag_pr_i[6:0]);
    wire [3:0]  ph_typ = (lp_Npr ? TYP_LP : TYP_PR);  // output event type tag
    wire [11:0] ph_dat = (lp_Npr ? lp_dat : pr_dat);  // PHysics cluster data
    reg  [11:0] ph_dat_ff;  // registered copy to improve clk160 timing margin
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            ph_dat_ff <= 12'b0;  // PHysics cluster data, flipflop-delayed
        end else begin
            ph_dat_ff <= ph_dat;
        end
    end
    wire errflag = 1'b0;  // obsolete output bit originally for ABC error flags
    // Combinational logic driven by FSM
    reg hh_rinc, hp_rinc, pr_rinc, lp_rinc, hr_rinc, rr_rinc;  // comb
    assign HCC_HPR_inc_o = hh_rinc;
    assign PR_rinc_o = pr_rinc;
    assign LP_rinc_o = lp_rinc;
    assign ReadReg_inc_o = hr_rinc;
    reg pr_dispatch_re, lp_dispatch_re;  // comb
    assign dispatch_re_pr_o = pr_dispatch_re;
    assign dispatch_re_lp_o = lp_dispatch_re;
    reg [8:0] obyte;  // comb
    reg obyte_we;  // comb
    assign hcc_out_o = obyte;
    assign out_we_o = obyte_we;
    // Register the 'hp_rinc' and 'rr_rinc' outputs to improve PnR timing
    reg hp_rinc_ff, rr_rinc_ff;
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            hp_rinc_ff <= 1'b0;  // (ABC) HPR FIFO read-increment, pipelined
            rr_rinc_ff <= 1'b0;  // (ABC) RR FIFO read-increment, pipelined
        end else begin
            hp_rinc_ff <= hp_rinc;
            rr_rinc_ff <= rr_rinc;
        end
    end
    assign HPR_rinc_o = hp_rinc_ff;
    assign RR_rinc_o = rr_rinc_ff;
    // State machine to keep track of what Packet Builder is doing
    localparam  // FSM state list
      START=7'd0, IDLE=7'd1,
      HH00=7'd2,  HH01=7'd3,  HH02=7'd4,  HH03=7'd5,  HH04=7'd6,  HH05=7'd7,
      HH06=7'd8,  HH07=7'd9,
      HP00=7'd10, HP01=7'd11, HP02=7'd12, HP03=7'd13, HP04=7'd14, HP05=7'd15,
      HP06=7'd16, HP07=7'd17, HP08=7'd18, HP09=7'd19, HP10=7'd20, HP11=7'd21,
      HP12=7'd22, HP13=7'd23, HP14=7'd24, HP15=7'd25,
      PH00=7'd26, PH02=7'd27, PH03=7'd28, PH04=7'd29, PH05=7'd30, PH06=7'd31,
      PH07=7'd32, PH08=7'd33, PH09=7'd34, PH10=7'd35, PH11=7'd36, PH12=7'd37,
      PH13=7'd38, PH14=7'd39, PH15=7'd40, PH16=7'd41, PH17=7'd42, PH18=7'd43,
      LF00=7'd44, LF01=7'd45, PF00=7'd46, PF01=7'd47,
      HR00=7'd48, HR01=7'd49, HR02=7'd50, HR03=7'd51, HR04=7'd52, HR05=7'd53,
      HR06=7'd54, HR07=7'd55,
      RR00=7'd56, RR01=7'd57, RR02=7'd58, RR03=7'd59, RR04=7'd60, RR05=7'd61,
      RR06=7'd62, RR07=7'd63, RR08=7'd64, RR09=7'd65, RR10=7'd66, RR11=7'd67,
      RR12=7'd68, RR13=7'd69, RR14=7'd70, RR15=7'd71,
      HF00=7'd72, HF01=7'd73, RF00=7'd74, RF01=7'd75;
    reg  [11:0] fsm_enc;   // FF: this is the (hamming-encoded) state register
    wire  [6:0] fsm;       // hamming-decoded version of fsm_enc
    reg   [6:0] fsm_prev;  // FF: (decoded) state on previous clock cycle
    reg   [6:0] fsm_d;     // combinational logic: next state
    wire [11:0] fsm_d_enc; // hamming-encoded version of fsm_d
    hamming_encode_12_7 h_enc(.Codeword_o(fsm_d_enc), .DataBits_i(fsm_d));
    hamming_decode_12_7 h_dec(.Codeword_i(fsm_enc),   .DataBits_o(fsm));
    localparam ENCODED_START = 12'd0;  // hamming-encoded START state code
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            fsm_enc <= ENCODED_START;
            fsm_prev <= START;
        end else begin
            // next state (hamming encoded) from combinational logic
            fsm_enc <= fsm_d_enc;
            // remember state (not encoded) from one cycle ago
            fsm_prev <= fsm;
        end
    end
    // Implement PB FSM timeout counter: used only in 'PH00' state
    wire [6:0] timeout;
    wire timedout = !(~timeout);  // timeout counter maxed out
    dffe_nbit_t #(.W(7)) timeout_reg_t
      (.rstb(rstb), .clk(clk160),
       .ena(fsm==IDLE || fsm!=fsm_prev || !timedout),  // don't wrap to 0
       .d((fsm==IDLE || fsm!=fsm_prev) ? 1'b0 : timeout + 1'b1),
       .q(timeout), .serOut());
    // Debug: to control verbose debug printout during simulation
`ifdef PB_VERBOSE
    wire [7:0] dbg0 = 8'd1;
`elsif PB_VERY_VERBOSE
    wire [7:0] dbg0 = 8'd100;
`else
    wire [7:0] dbg0 = 8'd0;
`endif
    reg  [7:0] dbg;  // combinational (hack)
    always @ (posedge clk160) begin
        // Since adding the Hamming encoding to the FSM state register,
        // the debug printout in the big combinational always @(*) block
        // below has been running twice per clock cycle, causing misleading
        // spurious printout of irrelevant intermediate information.  This
        // ugly hack causes the debug printout to occur only once, 1ns after
        // the rising edge of clk160.
        dbg = 8'b0;
        #1;
        dbg = dbg0;
        #0;
        dbg = 8'b0;
    end
    // Count ticks of clk160, so that debug printout can indicate time
    // both in nanoseconds and in integer clock ticks.  Used only in
    // debug printout.
    reg [31:0] ticks;
    always @ (posedge clk160 or negedge rstb) begin
        if (!rstb) begin
            ticks <= 32'b0;
        end else begin
            ticks <= ticks + 1'b1;
        end
    end
    /* synopsys translate_off */
    reg pause = 0;  // For RTL simulation only
    /* synopsys translate_on */
    // The following is a COMBINATIONAL always block
    always @ (*) begin
        // Assign default values to avoid risk of implicit latches
        abcerr_d = 11'b0;
        abcerr_ena = 1'b0;
        bcid_d = 4'b0;
        bcid_ena = 1'b0;
        bcmismatch_d = 11'b0;
        bcmismatch_ena = 1'b0;
        clucnt_clr = 1'b0;
        clucnt_inc = 1'b0;
        fsm_d = START;
        hdrwritten_d = 1'b0;
        hdrwritten_ena = 1'b0;
        headerq_d = 1'b0;
        headerq_ena = 1'b0;
        hh_rinc = 1'b0;
        hp_rinc = 1'b0;
        hr_rinc = 1'b0;
        ic_ena_ena = 1'b0;
        ic_todo_d = ic_todo[10:0];
        ichan_d = ichan;
        l0mismatch_d = 11'b0;
        l0mismatch_ena = 1'b0;
        l0tag_d = 7'b0;
        l0tag_ena = 1'b0;
        lp_Npr_d = 1'b0;
        lp_Npr_ena = 1'b0;
        lp_dispatch_re = 1'b0;
        lp_rinc = 1'b0;
        obyte = 9'd0;
        obyte_we = 1'b0;
        pr_dispatch_re = 1'b0;
        pr_rinc = 1'b0;
        rr0_d = 16'd0;
        rr1_d = 16'd0;
        rr2_d = 16'd0;
        rr3_d = 16'd0;
        {rr0_ena, rr1_ena, rr2_ena, rr3_ena} = 4'b0;
        rr_rinc = 1'b0;
        suppress_d = 1'b0;
        suppress_ena = 1'b0;
        tmoerr_d = 11'b0;
        tmoerr_ena = 1'b0;
        case (fsm)
            START:
              begin
                  if (dbg) $strobe("START(HCC PB)@%1d/%1d", ticks, $time);
                  ic_ena_ena = 1'b1;  // Update list of enabled ICs
                  if (hp_ne_ff & ic_ena) begin
                      // Flush any unprocessed data from ABC HPR FIFOs
                      fsm_d = HF00;
                  end else if (rr_ne_ff & ic_ena) begin
                      // Flush any unprocessed data from ABC RR FIFOs
                      fsm_d = RF00;
                  end else begin
                      fsm_d = IDLE;
                  end
              end
            IDLE:
              // Wait for input data to become available.
              begin
                  if (dbg>=8'd100)
                    $strobe(
                      "IDLE@%1d/%1d: pr_ne=%b/%d lp_ne=%b/%d ",
                      $time, ticks, pr_ne_ff, dp_ne_ff, lp_ne_ff, dl_ne_ff, 
                      "rr_ne=%b ic_ena=%b",
                      rr_ne, ic_ena);
                  fsm_d = IDLE;
                  ic_ena_ena = 1'b1;  // Update list of enabled ICs
                  clucnt_clr = 1'b1;  // Reset count of processed clusters
                  if (out_full) begin
                      // Stay IDLE until at least 8 spaces become available
                      // in the output FIFO.  This removes all need to pause
                      // when output FIFO becomes almost-full, except in the
                      // PHysics states.
                      fsm_d = IDLE;
                  end
                  /* synopsys translate_off */
                  else if (pause) begin
                      // Allow RTL simulation to pause PB (not currently used)
                      fsm_d = IDLE;
                  end
                  /* synopsys translate_on */
                  else if (hh_ne) begin
                      // Handle HCC HPR
                      fsm_d = HH00;
                  end else if (hp_ne_ff & ic_ena) begin
                      // Handle ABC HPR
                      fsm_d = HP00;
                  end else if ((pr_ne_ff & ic_ena) && !dp_ne_ff) begin
                      // PR dispatch FIFO is empty, but PR backbus
                      // FIFOs contain (spurious) data.  Flush them out.
                      ichan_d = 4'b0;  // 2018-05-13 added to match LP logic
                      fsm_d = PF00;
                  end else if ((lp_ne_ff & ic_ena) && !dl_ne_ff) begin
                      // LP dispatch FIFO is empty, but LP backbus
                      // FIFOs contain (spurious) data.  Flush them out.
                      ichan_d = 4'b0;
                      fsm_d = LF00;
                  end else if (dp_ne_ff && (pr_ne_ff & ic_ena)) begin
                      // Handle PR.  This runs if dispatch FIFO and at
                      // least one enabled IC have data.  Much simpler
                      // than the "at least two ICs, unless only one
                      // IC is enabled" that we had discussed.
                      ic_todo_d = ic_ena;  // mark all enabled IC as "to do"
                      ichan_d = 4'b1111;  // hack: start with invalid chnl
                      hdrwritten_ena = 1'b1;
                      hdrwritten_d = 1'b0;  // output header not yet written
                      tmoerr_ena = 1'b1;
                      tmoerr_d = 11'b0;  // zero timeout-error flags
                      bcmismatch_ena = 1'b1;
                      bcmismatch_d = 11'b0;  // zero BCID error flags
                      l0mismatch_ena = 1'b1;
                      l0mismatch_d = 11'b0;  // zero L0ID error flags
                      suppress_ena = 1'b1;
                      suppress_d = 1'b0;  // don't suppress cluster output
                      abcerr_ena = 1'b1;
                      abcerr_d = 11'b0;  // zero ABC-reported error flags
                      headerq_ena = 1'b1;
                      headerq_d = 1'b1;  // next word seen will be header word
                      lp_Npr_d = 1'b0;  // this is PR, not LP
                      lp_Npr_ena = 1'b1;
                      l0tag_d = dispatch_tag_pr_i[6:0];  // grab expected L0ID
                      l0tag_ena = 1'b1;
                      bcid_d = dispatch_tag_pr_i[10:7];  // grab expected BCID
                      bcid_ena = 1'b1;
                      // Our advancing the dispatch FIFO here allows us to
                      // peek at the L0ID tag of the next event in the queue
                      // (if one exists), as a means to recover event
                      // synchronization in case of missing data.
                      pr_dispatch_re = 1'b1;
                      fsm_d = PH00;  // unified PH(ysics) state
                  end else if (lp_all_waiting ||
                               (lp_timedout && lp_any_waiting)) begin
                      // Handle LP.  This runs if dispatch FIFO and all
                      // enabled IC have data, or if any enabled IC have
                      // had data for longer than timeout.
                      ic_todo_d = ic_ena;  // mark all enabled IC as "to do"
                      ichan_d = 4'b1111;  // hack: start with invalid chnl
                      hdrwritten_ena = 1'b1;
                      hdrwritten_d = 1'b0;  // output header not yet written
                      tmoerr_ena = 1'b1;
                      tmoerr_d = 11'b0;  // zero timeout-error flags
                      bcmismatch_ena = 1'b1;
                      bcmismatch_d = 11'b0;  // zero BCID error flags
                      l0mismatch_ena = 1'b1;
                      l0mismatch_d = 11'b0;  // zero L0ID error flags
                      suppress_ena = 1'b1;
                      suppress_d = 1'b0;  // don't suppress cluster output
                      abcerr_ena = 1'b1;
                      abcerr_d = 11'b0;  // zero ABC-reported error flags
                      headerq_ena = 1'b1;
                      headerq_d = 1'b1;  // next word seen will be header word
                      lp_Npr_d = 1'b1;  // this is LP, not PR
                      lp_Npr_ena = 1'b1;
                      l0tag_d = dispatch_tag_lp_i[6:0];  // grab expected L0ID
                      l0tag_ena = 1'b1;
                      bcid_d = dispatch_tag_lp_i[10:7];  // grab expected BCID
                      bcid_ena = 1'b1;
                      // Our advancing the dispatch FIFO here allows us to
                      // peek at the L0ID tag of the next event in the queue
                      // (if one exists), as a means to recover event
                      // synchronization in case of missing data.
                      lp_dispatch_re = 1'b1;  // advance to peek at next tags
                      fsm_d = PH00;  // unified PH(ysics) state
                  end else if (hr_ne) begin
                      // Handle HCC ReadReg
                      fsm_d = HR00;
                  end else if (rr_ne_ff & ic_ena) begin
                      // Handle ABC ReadReg
                      fsm_d = RR00;
                  end
              end
            LF00:
              // Flush junk data from LP FIFOs
              begin
                  if (dbg) $strobe("LF00@%1d/%1d", ticks, $time);
                  if (dl_ne_ff && !(~ticks_since_lp)) begin  // 2018-05-13 edit
                      // If LP event has been dispatched, long enough ago that
                      // 'ticks_since_lp' is maxed out, then abort flushing.
                      fsm_d = IDLE;
                  end else if (!(lp_ne_ff & ic_ena)) begin
                      // If LP physics data FIFOs corresponding to all enabled
                      // ICs are empty, then we are done flushing.
                      fsm_d = IDLE;
                  end else begin
                      ichan_d = (ic_ena[ 0] && lp_ne_ff[ 0]) ? 4'd0 :
                                (ic_ena[ 1] && lp_ne_ff[ 1]) ? 4'd1 :
                                (ic_ena[ 2] && lp_ne_ff[ 2]) ? 4'd2 :
                                (ic_ena[ 3] && lp_ne_ff[ 3]) ? 4'd3 :
                                (ic_ena[ 4] && lp_ne_ff[ 4]) ? 4'd4 :
                                (ic_ena[ 5] && lp_ne_ff[ 5]) ? 4'd5 :
                                (ic_ena[ 6] && lp_ne_ff[ 6]) ? 4'd6 :
                                (ic_ena[ 7] && lp_ne_ff[ 7]) ? 4'd7 :
                                (ic_ena[ 8] && lp_ne_ff[ 8]) ? 4'd8 :
                                (ic_ena[ 9] && lp_ne_ff[ 9]) ? 4'd9 :
                                (ic_ena[10] && lp_ne_ff[10]) ? 4'd10 : ichan;
                      fsm_d = LF01;
                  end
              end
            LF01:
              begin
                  if (dbg) $strobe("LF01: flushing LP ichan=%d", ichan);
                  if (lp_ne_ff[ichan]) begin
                      // Since we use lp_ne_ff (pipeline-delayed), we
                      // will 'rinc' one too many words here, but that
                      // should be of no consequence.
                      lp_rinc = 1'b1;
                      fsm_d = LF01;
                  end else begin
                      fsm_d = LF00;
                  end
              end
            PF00:
              // Flush junk data from PR FIFOs
              begin
                  if (dbg) $strobe("PF00@%1d/%1d", ticks, $time);
                  if (dp_ne_ff && !(~ticks_since_pr)) begin  // 2018-05-13 edit
                      // If PR event has been dispatched, long enough ago that
                      // 'ticks_since_pr' is maxed out, then abort flushing.
                      fsm_d = IDLE;
                  end else if (!(pr_ne_ff & ic_ena)) begin
                      // If PR physics data FIFOs corresponding to all enabled
                      // ICs are empty, then we are done flushing.
                      fsm_d = IDLE;
                  end else begin
                      ichan_d = (ic_ena[ 0] && pr_ne_ff[ 0]) ? 4'd0 :
                                (ic_ena[ 1] && pr_ne_ff[ 1]) ? 4'd1 :
                                (ic_ena[ 2] && pr_ne_ff[ 2]) ? 4'd2 :
                                (ic_ena[ 3] && pr_ne_ff[ 3]) ? 4'd3 :
                                (ic_ena[ 4] && pr_ne_ff[ 4]) ? 4'd4 :
                                (ic_ena[ 5] && pr_ne_ff[ 5]) ? 4'd5 :
                                (ic_ena[ 6] && pr_ne_ff[ 6]) ? 4'd6 :
                                (ic_ena[ 7] && pr_ne_ff[ 7]) ? 4'd7 :
                                (ic_ena[ 8] && pr_ne_ff[ 8]) ? 4'd8 :
                                (ic_ena[ 9] && pr_ne_ff[ 9]) ? 4'd9 :
                                (ic_ena[10] && pr_ne_ff[10]) ? 4'd10 : ichan;
                      fsm_d = PF01;
                  end
              end
            PF01:
              begin
                  if (dbg) $strobe("PF01: flushing PR ichan=%d", ichan);
                  if (pr_ne_ff[ichan]) begin
                      // This will 'rinc' one too many words, due to
                      // pipeline delay, but that should be OK here.
                      pr_rinc = 1'b1;
                      fsm_d = PF01;
                  end else begin
                      fsm_d = PF00;
                  end
              end
            HF00:
              // Flush junk data from ABC HPR FIFOs.  We can only get
              // here from the START state.  We implement this because a
              // complete ABC HPR consists of 4 FIFO words, but there
              // is no start/end delimiter, so an SEU could
              // conceivably leave a partial transaction in the FIFO.
              begin
                  if (dbg) $strobe("HF00@%1d/%1d", ticks, $time);
                  if (!(hp_ne_ff & ic_ena)) begin
                      fsm_d = START;
                  end else begin
                      ichan_d = (ic_ena[ 0] && hp_ne_ff[ 0]) ? 4'd0  :
                                (ic_ena[ 1] && hp_ne_ff[ 1]) ? 4'd1  :
                                (ic_ena[ 2] && hp_ne_ff[ 2]) ? 4'd2  :
                                (ic_ena[ 3] && hp_ne_ff[ 3]) ? 4'd3  :
                                (ic_ena[ 4] && hp_ne_ff[ 4]) ? 4'd4  :
                                (ic_ena[ 5] && hp_ne_ff[ 5]) ? 4'd5  :
                                (ic_ena[ 6] && hp_ne_ff[ 6]) ? 4'd6  :
                                (ic_ena[ 7] && hp_ne_ff[ 7]) ? 4'd7  :
                                (ic_ena[ 8] && hp_ne_ff[ 8]) ? 4'd8  :
                                (ic_ena[ 9] && hp_ne_ff[ 9]) ? 4'd9  :
                                (ic_ena[10] && hp_ne_ff[10]) ? 4'd10 : ichan;
                      fsm_d = HF01;
                  end
              end
            HF01:
              begin
                  if (dbg) $strobe("HF01: flushing ABC HPR ichan=%d", ichan);
                  if (hp_ne_ff[ichan]) begin
                      // Due to pipelining, we will flush two words
                      // too many (one extra for pipelined hp_ne_ff,
                      // one extra for pipelined hp_rinc), but 'rinc'ing
                      // an already-empty FIFO is of no consequence here.
                      hp_rinc = 1'b1;
                      fsm_d = HF01;
                  end else begin
                      fsm_d = HF00;
                  end
              end
            RF00:
              // Flush junk data from ABC RR FIFOs.  We can only get
              // here from the START state.  We implement this because a
              // complete ABC RR consists of 4 FIFO words, but there
              // is no start/end delimiter, so an SEU could
              // conceivably leave a partial transaction in the FIFO.
              begin
                  if (dbg) $strobe("RF00@%1d/%1d", ticks, $time);
                  if (!(rr_ne_ff & ic_ena)) begin
                      fsm_d = START;
                  end else begin
                      ichan_d = (ic_ena[ 0] && rr_ne_ff[ 0]) ? 4'd0  :
                                (ic_ena[ 1] && rr_ne_ff[ 1]) ? 4'd1  :
                                (ic_ena[ 2] && rr_ne_ff[ 2]) ? 4'd2  :
                                (ic_ena[ 3] && rr_ne_ff[ 3]) ? 4'd3  :
                                (ic_ena[ 4] && rr_ne_ff[ 4]) ? 4'd4  :
                                (ic_ena[ 5] && rr_ne_ff[ 5]) ? 4'd5  :
                                (ic_ena[ 6] && rr_ne_ff[ 6]) ? 4'd6  :
                                (ic_ena[ 7] && rr_ne_ff[ 7]) ? 4'd7  :
                                (ic_ena[ 8] && rr_ne_ff[ 8]) ? 4'd8  :
                                (ic_ena[ 9] && rr_ne_ff[ 9]) ? 4'd9  :
                                (ic_ena[10] && rr_ne_ff[10]) ? 4'd10 : ichan;
                      fsm_d = RF01;
                  end
              end
            RF01:
              begin
                  if (dbg) $strobe("RF01: flushing ABC RR ichan=%d", ichan);
                  if (rr_ne_ff[ichan]) begin
                      // Due to pipelining, we will flush two words
                      // too many (one extra for pipelined rr_ne_ff,
                      // one extra for pipelined rr_rinc), but 'rinc'ing
                      // an already-empty FIFO is of no consequence here.
                      rr_rinc = 1'b1;
                      fsm_d = RF01;
                  end else begin
                      fsm_d = RF00;
                  end
              end
            HH00:
              // Build an HCC HPR packet: start-of-frame K-code.
              begin
                  if (dbg) $strobe("HH00@%1d/%1d", ticks, $time);
                  obyte = SB_SOF;
                  obyte_we = 1'b1;
                  fsm_d = HH01;
              end
            HH01:
              // Header byte
              begin
                  if (dbg) $strobe("HH01");
                  obyte = {1'b0,TYP_HCC_HPR,hh_dat[39:36]};
                  obyte_we = 1'b1;
                  fsm_d = HH02;
              end
            HH02:
              // HCC HPR data
              begin
                  if (dbg) $strobe("HH02: hh_dat=%x", hh_dat);
                  obyte = {1'b0,hh_dat[35:28]};
                  obyte_we = 1'b1;
                  fsm_d = HH03;
              end
            HH03:
              begin
                  if (dbg) $strobe("HH03");
                  obyte ={1'b0, hh_dat[27:20]};
                  obyte_we = 1'b1;
                  fsm_d = HH04;
              end
            HH04:
              begin
                  if (dbg) $strobe("HH04");
                  obyte = {1'b0,hh_dat[19:12]};
                  obyte_we = 1'b1;
                  fsm_d = HH05;
              end
            HH05:
              begin
                  if (dbg) $strobe("HH05");
                  obyte = {1'b0,hh_dat[11:4]};
                  obyte_we = 1'b1;
                  fsm_d = HH06;
              end
            HH06:
              begin
                  if (dbg) $strobe("HH06");
                  obyte = {1'b0,hh_dat[3:0],4'b0};
                  obyte_we = 1'b1;
                  fsm_d = HH07;
              end
            HH07:
              begin
                  if (dbg) $strobe("HH07");
                  obyte = SB_EOF;
                  hh_rinc = 1'b1;
                  obyte_we = 1'b1;
                  fsm_d = IDLE;
              end
            HP00:
              // Build an (ABC) HPR packet
              begin
                  if (dbg) $strobe("HP00@%1d/%1d", ticks, $time);
                  if (hp_ne_ff[0] && ic_ena[0]) begin
                      ichan_d = 4'd0; fsm_d = HP01;
                  end else if (hp_ne_ff[1] && ic_ena[1]) begin
                      ichan_d = 4'd1; fsm_d = HP01;
                  end else if (hp_ne_ff[2] && ic_ena[2]) begin
                      ichan_d = 4'd2; fsm_d = HP01;
                  end else if (hp_ne_ff[3] && ic_ena[3]) begin
                      ichan_d = 4'd3; fsm_d = HP01;
                  end else if (hp_ne_ff[4] && ic_ena[4]) begin
                      ichan_d = 4'd4; fsm_d = HP01;
                  end else if (hp_ne_ff[5] && ic_ena[5]) begin
                      ichan_d = 4'd5; fsm_d = HP01;
                  end else if (hp_ne_ff[6] && ic_ena[6]) begin
                      ichan_d = 4'd6; fsm_d = HP01;
                  end else if (hp_ne_ff[7] && ic_ena[7]) begin
                      ichan_d = 4'd7; fsm_d = HP01;
                  end else if (hp_ne_ff[8] && ic_ena[8]) begin
                      ichan_d = 4'd8; fsm_d = HP01;
                  end else if (hp_ne_ff[9] && ic_ena[9]) begin
                      ichan_d = 4'd9; fsm_d = HP01;
                  end else if (hp_ne_ff[10] && ic_ena[10]) begin
                      ichan_d = 4'd10; fsm_d = HP01;
                  end else begin
                      fsm_d = IDLE;
                  end
              end
            HP01:
              // Latch in FIFO word 0
              begin
                  if (dbg) $strobe("HP01: ichan=%1d", ichan);
                  rr0_d = hp_dat;
                  rr0_ena = 1'b1;
                  hp_rinc = 1'b1;  // advance FIFO: NB added pipeline delay
                  fsm_d = HP02;
              end
            HP02:
              // Wait for FIFO word 1 to appear (NB added pipeline delay)
              begin
                  if (dbg) $strobe("HP02");
                  hp_rinc = 1'b1;  // NB added pipeline delay
                  fsm_d = HP03;
                  // Don't bother checking if FIFO is non-empty, b/c
                  // not seeing all 4 words can only happen if there
                  // is a bug in the InputChannel logic.
              end
            HP03:
              // Latch in FIFO word 1
              begin
                  if (dbg) $strobe("HP03");
                  rr1_d = hp_dat;
                  rr1_ena = 1'b1;
                  hp_rinc = 1'b1;  // NB added pipeline delay
                  fsm_d = HP04;
              end
            HP04:
              // Latch in FIFO word 2
              begin
                  if (dbg) $strobe("HP04");
                  rr2_d = hp_dat;
                  rr2_ena = 1'b1;
                  hp_rinc = 1'b1;  // NB added pipeline delay
                  fsm_d = HP05;
              end
            HP05:
              // Send start-of-frame K-code; latch in FIFO word 3
              begin
                  if (dbg) $strobe("HP05");
                  rr3_d = hp_dat;
                  rr3_ena = 1'b1;
                  obyte = SB_SOF;
                  obyte_we = 1'b1;
                  fsm_d = HP06;
              end
            HP06:
              // Payload byte 0
              begin
                  if (dbg)
                    $strobe("HP06: ichan=%1d rr0123=%x %x %x %x",
                             ichan, rr0, rr1, rr2, rr3);
                  obyte = {1'b0,TYP_ABC_HPR, ichan[3:0]};
                  obyte_we = 1'b1;
                  fsm_d = HP07;
              end
            HP07:
              // Payload byte 1
              begin
                  if (dbg) $strobe("HP07");
                  obyte = {1'b0,rr0[11:4]};
                  obyte_we = 1'b1;
                  fsm_d = HP08;
              end
            HP08:
              // Payload byte 2
              begin
                  if (dbg) $strobe("HP08");
                  obyte = {1'b0,rr0[3:0],rr1[15:12]};
                  obyte_we = 1'b1;
                  fsm_d = HP09;
              end
            HP09:
              // Payload byte 3
              begin
                  if (dbg) $strobe("HP09");
                  obyte = {1'b0,rr1[11:4]};
                  obyte_we = 1'b1;
                  fsm_d = HP10;
              end
            HP10:
              // Payload byte 4
              begin
                  if (dbg) $strobe("HP10");
                  obyte = {1'b0,rr1[3:0],rr2[15:12]};
                  obyte_we = 1'b1;
                  fsm_d = HP11;
              end
            HP11:
              // Payload byte 5
              begin
                  if (dbg) $strobe("HP11");
                  obyte = {1'b0,rr2[11:4]};
                  obyte_we = 1'b1;
                  fsm_d = HP12;
              end
            HP12:
              // Payload byte 6
              begin
                  if (dbg) $strobe("HP12");
                  obyte = {1'b0,rr2[3:0],rr3[15:12]};
                  // 2018-05-13 removed 'out_full' check
                  obyte_we = 1'b1;
                  fsm_d = HP13;
              end
            HP13:
              // Payload byte 7
              begin
                  if (dbg) $strobe("HP13");
                  obyte = {1'b0,rr3[11:4]};
                  obyte_we = 1'b1;
                  fsm_d = HP14;
              end
            HP14:
              // Payload byte 8
              begin
                  if (dbg) $strobe("HP14");
                  obyte = {1'b0,rr3[3:0],4'b0};
                  obyte_we = 1'b1;
                  fsm_d = HP15;
              end
            HP15:
              // Send end-of-frame K-code
              begin
                  if (dbg) $strobe("HP15");
                  obyte = SB_EOF;
                  obyte_we = 1'b1;
                  fsm_d = IDLE;
              end
            PH00:
              // Build (or continue building) a PR or LP packet
              //
              // TODO: Eventually, the PR and LP packet-building
              // should be made smart enough to bit-pack 15-bit
              // clusters into 8-bit words, avoiding a 6% waste of
              // physics readout bandwidth.
              begin
                  if (dbg)
                    $strobe("PH00: hdrw=%d ", hdrwritten,
                            "ich=%d todo=%b lp=%d ne=%b cc=%d",
                            ichan, ic_todo, lp_Npr, ph_ne_ff, clucnt,
                            "   \t t=%1d/%1d", ticks, $time);
                  if (!ic_todo) begin
                      // No ICs left to process for this event
                      if (l0mismatch || abcerr ||
                          (bcmismatch & ~BCMM_squelch_i)) begin
                          // BCID mismatch is detected regardless of BCMM
                          // squelch, but if 'ichan' is squelched, then a BCID
                          // mismatch on 'ichan' is not sufficient cause to
                          // justify writing an error block; if the error
                          // block is written for some other reason, the
                          // detected pattern of BCID mismatches is included.
                          fsm_d = PH11;  // write error block (then trailer)
                      end else begin
                          fsm_d = PH08;  // write event trailer
                      end
                  end else if (timedout) begin
                      if (dbg) $strobe("PH00: timeout ich=%d", ichan);
                      tmoerr_ena = 1'b1;         // any IC left ...
                      tmoerr_d = ic_todo[10:0];  // ... must have timed out
                      fsm_d = PH11;
                  end else if (!ic_todo[ichan] ||
                               (clucnt==3'd4 && !ph_ne_ff[ichan])) begin
                      // IC 'ichan' is done, or is disabled or stalled at the
                      // end of an ABC packet (4 clusters per ABC packet):
                      // switch to first non-empty IC that is still on the
                      // to-do list
                      ichan_d = ic_todo[ 0] && ph_ne_ff[ 0] ? 4'd0 :
                                ic_todo[ 1] && ph_ne_ff[ 1] ? 4'd1 :
                                ic_todo[ 2] && ph_ne_ff[ 2] ? 4'd2 :
                                ic_todo[ 3] && ph_ne_ff[ 3] ? 4'd3 :
                                ic_todo[ 4] && ph_ne_ff[ 4] ? 4'd4 :
                                ic_todo[ 5] && ph_ne_ff[ 5] ? 4'd5 :
                                ic_todo[ 6] && ph_ne_ff[ 6] ? 4'd6 :
                                ic_todo[ 7] && ph_ne_ff[ 7] ? 4'd7 :
                                ic_todo[ 8] && ph_ne_ff[ 8] ? 4'd8 :
                                ic_todo[ 9] && ph_ne_ff[ 9] ? 4'd9 :
                                ic_todo[10] && ph_ne_ff[10] ? 4'd10 : ichan;
                      headerq_ena = 1'b1;
                      headerq_d = 1'b1;  // next word seen will be header word
                      fsm_d = PH00;
                  end else if (ph_ne_ff[ichan] && !hdrwritten) begin
                      // An input word is available for 'ichan', but we have
                      // not yet written a header for this event
                      hdrwritten_ena = 1'b1;
                      hdrwritten_d = 1'b1;
                      fsm_d = PH02;  // Write packet header
                  end else if (ph_ne_ff[ichan]) begin
                      // An input word is available for 'ichan' (and we have
                      // already written a header for this event)
                      fsm_d = PH05;  // Packet header already written
                  end else begin
                      fsm_d = PH00;
                  end
                  // TODO: handle NO CLUSTER value, per PTK 2018-01-04;
                  // TODO: flag error for this IC if wrong event seen  ?done?
                  // TODO: pack the whole packet
              end
            PH02:
              // Write start-of-frame K-code (we have first word of first IC)
              begin
                  if (dbg) $strobe("PH02: ", obyte_we ? "WR SOF" : "",
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = SB_SOF;
                  if (out_full) begin  // not needed here, could omit
                      fsm_d = fsm;
                  end else begin
                      obyte_we = 1'b1;
                      fsm_d = PH03;
                  end
              end
            PH03:
              // Continue writing packet header
              begin
                  if (dbg)
                    $strobe("PH03: WR ichan=%1d bcid=%x(%d) l0tag=%x(%d)",
                            ichan, bcid, bcid, l0tag, l0tag,
                            "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,ph_typ,errflag,l0tag[6:4]};
                  obyte_we = 1'b1;
                  fsm_d = PH04;
              end
            PH04:
              // Continue writing packet header
              begin
                  if (dbg)
                    $strobe("PH04: WR ichan=%1d bcid=%x l0tag=%x",
                            ichan, bcid, l0tag,
                            "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,l0tag[3:0],bcid};
                  obyte_we = 1'b1;
                  fsm_d = PH05;
              end
            PH05:
              // We are looking at an input word, and packet header is written
              begin
                  if (dbg)
                    $strobe("PH05: ichan=%1d headerq=%d", ichan, headerq,
                            "   \t t=%1d/%1d", ticks, $time);
                  if (headerq || (clucnt==3'd4)) begin
                      // 'headerq' can be true because we just started
                      // building a new event, or because we just switched
                      // from a different IC (which we never do in the middle
                      // of an ABC packet).  But if 'clucnt' has reached 4,
                      // then we are looking at the header word for the next
                      // ABC packet for the current IC.  Either way, 'rinc'
                      // this FIFO word away without writing anything.
                      clucnt_clr = 1'b1;
                      headerq_ena = 1'b1;
                      headerq_d = 1'b0;  // next word will not be header word
                      suppress_d = 1'b0;  // by default DO NOT suppress output
                      suppress_ena = 1'b1;
                      if (lp_Npr) begin
                          lp_rinc = 1'b1;
                      end else begin
                          pr_rinc = 1'b1;
                      end
                      // This word contains BCID information, which we should
                      // check against the expected value.
                      if (ph_dat_ff[3:0] != bcid) begin
                          if (dbg)
                            $strobe("PH05: ichan=%1d bc mismatch %1x != %1x",
                                    ichan, ph_dat_ff[3:0], bcid,
                                    "   \t t=%1d/%1d", ticks, $time);
                          bcmismatch_d = bcmismatch | (11'b1<<ichan);
                          bcmismatch_ena = 1'b1;
                      end
                      // This word contains L0ID information, which we should
                      // check against the expected value.
                      if (ph_dat_ff[10:4] != l0tag) begin
                          if (dbg)
                            $strobe("PH05: ichan=%1d l0tag mismatch %x != %x",
                                    ichan, ph_dat_ff[10:4], l0tag,
                                    "   \t t=%1d/%1d", ticks, $time);
                          l0mismatch_d = l0mismatch | (11'b1<<ichan);
                          l0mismatch_ena = 1'b1;
                          suppress_d = 1'b1;  // suppress output of clusters ..
                          suppress_ena = 1'b1;  // .. for this ABC packet
                          if (df_ne && ph_dat_ff[10:4]==disp_tag) begin
                              // If header tag matches NEXT expected tag, then
                              // set this IC aside to process with next event.
                              if (dbg)
                                $strobe("PH05: l0tag == NEXT: %x", disp_tag);
                              ic_todo_d[ichan] = 1'b0;  // save for next event
                              lp_rinc = 1'b0;
                              pr_rinc = 1'b0;
                          end
                      end
                      // Handle error bit from ABC, if set
                      if (ph_dat_ff[11]) begin
                          if (dbg)
                            $strobe("PH05: abc[%d] error flag set", ichan);
                          abcerr_d = abcerr | (11'b1<<ichan);
                          abcerr_ena = 1'b1;
                      end
                      fsm_d = PH00;  // clever: wait state NOT needed (even
                      // though we may have just rinc'ed the LP or PR FIFO)
                      // because LP/PR FIFO will never be empty immediately
                      // after a header word
                  end else begin
                      // This IC word is not a header: write it out
                      clucnt_inc = 1'b1;
                      fsm_d = PH06;
                  end
              end
            PH06:
              // First output byte for given input word
              begin
                  if (dbg) $strobe("PH06: ",
                                   obyte_we ? "WR " : "",
                                   "ichan=%1d lp_Npr=%d dat=%x/%x",
                                   ichan, lp_Npr, ph_dat, ph_dat_ff,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {2'b00,ichan[3:0],ph_dat_ff[10:8]};
                  if (out_full) begin
                      fsm_d = fsm;  // Wait for room in output FIFO
                  end else begin
                      obyte_we = !suppress;  // suppress output in case of
                                             // mismatched L0ID
                      fsm_d = PH07;
                      if (lp_Npr) begin
                          lp_rinc = 1'b1;
                      end else begin
                          pr_rinc = 1'b1;
                      end
                  end
              end
            PH07:
              // Second output byte for given input word
              begin
                  if (dbg) $strobe("PH07: WR ichan=%1d", ichan,
                                   "   \t t=%1d/%1d", ticks, $time);
                      obyte = {1'b0,ph_dat_ff[7:0]};
                      if (ph_dat_ff[11]) begin
                          // last cluster from this IC
                          ic_todo_d[ichan] = 1'b0;
                      end
                  obyte_we = !suppress;  // suppress this ABC pkt if l0mismatch
                  fsm_d = PH00;  // jump back to process rest of this event
              end
            PH08:
              // No ICs left to process for this event: write packet trailer
              begin
                  if (dbg) $strobe("PH08: ", obyte_we ? "WR EOP=6F" : "",
                                   "   \t t=%1d/%1d", ticks, $time);
                  // insert 15'h6FED sentinel at end of packet
                  obyte = {1'b0,8'h6F};
                  if (out_full) begin  // is this needed here?
                      fsm_d = fsm;
                  end else begin
                      obyte_we = 1'b1;
                      fsm_d = PH09;
                  end
              end
            PH09:
              // 2nd byte of packet trailer
              begin
                  if (dbg) $strobe("PH09: WR EOP=ED",
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,8'hED};
                  obyte_we = 1'b1;
                  fsm_d = PH10;
              end
            PH10:
              // End-of-packet K-code
              begin
                  if (dbg) $strobe("PH10: WR EOF",
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = SB_EOF;
                  obyte_we = 1'b1;
                  fsm_d = IDLE;
              end
            PH11:
              // Write error block
              begin
                  if (dbg) $strobe("PH11: ", obyte_we ? "WR " : "",
                                   "errblk=77",
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,8'h77};  // first byte of errblk header
                  if (out_full) begin
                      fsm_d = fsm;
                  end else begin
                      obyte_we = 1'b1;
                      fsm_d = PH12;
                  end
              end
            PH12:
              begin
                  if (dbg) $strobe("PH12: WR errblk=F4",
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,8'hF4};  // 2nd byte of errblk header
                  obyte_we = 1'b1;
                  fsm_d = PH13;
              end
            PH13:
              begin
                  if (dbg) $strobe("PH13: WR errblk obyte=%2x ", obyte,
                                   "abcerr=%x bcmm=%x l0mm=%x tmo=%x",
                                   abcerr, bcmismatch, l0mismatch, tmoerr,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,1'b0,abcerr[10:4]};
                  obyte_we = 1'b1;
                  fsm_d = PH14;
              end
            PH14:
              begin
                  if (dbg) $strobe("PH14: WR errblk obyte=%2x", obyte,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,abcerr[3:0],1'b0,bcmismatch[10:8]};
                  obyte_we = 1'b1;
                  fsm_d = PH15;
              end
            PH15:
              begin
                  if (dbg) $strobe("PH15: WR errblk obyte=%2x", obyte,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,bcmismatch[7:0]};
                  obyte_we = 1'b1;
                  fsm_d = PH16;
              end

            PH16:
              begin
                  if (dbg) $strobe("PH16: WR errblk obyte=%2x", obyte,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,1'b0,l0mismatch[10:4]};
                  obyte_we = 1'b1;
                  fsm_d = PH17;
              end
            PH17:
              begin
                  if (dbg) $strobe("PH17: WR errblk obyte=%2x", obyte,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,l0mismatch[3:0],1'b0,tmoerr[10:8]};
                  obyte_we = 1'b1;
                  fsm_d = PH18;
              end
            PH18:
              // end of writing error block
              begin
                  if (dbg) $strobe("PH18: WR errblk obyte=%2x errblkend",
                                   obyte,
                                   "   \t t=%1d/%1d", ticks, $time);
                  obyte = {1'b0,tmoerr[7:0]};
                  obyte_we = 1'b1;
                  fsm_d = PH08;  // next, write event trailer
              end
            HR00:
              // Build an HCC ReadReg packet: start-of-frame K-code.
              begin
                  if (dbg) $strobe("HR00@%1d/%1d", ticks, $time);
                  obyte = SB_SOF;
                  obyte_we = 1'b1;
                  fsm_d = HR01;
              end
            HR01:
              // Header byte
              begin
                  if (dbg) $strobe("HR01");
                  obyte = {1'b0,TYP_HCC_RR,hr_dat[39:36]};
                  obyte_we = 1'b1;
                  fsm_d = HR02;
              end
            HR02:
              // HCC ReadReg data
              begin
                  if (dbg) $strobe("HR02: hr_dat=%x", hr_dat);
                  obyte = {1'b0,hr_dat[35:28]};
                  obyte_we = 1'b1;
                  fsm_d = HR03;
              end
            HR03:
              begin
                  if (dbg) $strobe("HR03");
                  obyte = {1'b0,hr_dat[27:20]};
                  obyte_we = 1'b1;
                  fsm_d = HR04;
              end
            HR04:
              begin
                  if (dbg) $strobe("HR04");
                  obyte = {1'b0,hr_dat[19:12]};
                  obyte_we = 1'b1;
                  fsm_d = HR05;
              end
            HR05:
              begin
                  if (dbg) $strobe("HR05");
                  obyte ={1'b0, hr_dat[11:4]};
                  obyte_we = 1'b1;
                  fsm_d = HR06;
              end
            HR06:
              begin
                  if (dbg) $strobe("HR06");
                  obyte = {1'b0,hr_dat[3:0],4'b0};
                  obyte_we = 1'b1;
                  hr_rinc = 1'b1;  // 2018-05-13 moved here from HR07
                  fsm_d = HR07;
              end
            HR07:
              begin
                  if (dbg) $strobe("HR07");
                  obyte = SB_EOF;
                  obyte_we = 1'b1;
                  fsm_d = IDLE;
              end
            RR00:
              // Build an (ABC) RR packet
              begin
                  if (dbg) $strobe("RR00@%1d/%1d: opmode=%d", 
                                   ticks, $time, opmode);
                  if (rr_ne_ff[0] && ic_ena[0]) begin
                      ichan_d = 4'd0; fsm_d = RR01;
                  end else if (rr_ne_ff[1] && ic_ena[1]) begin
                      ichan_d = 4'd1; fsm_d = RR01;
                  end else if (rr_ne_ff[2] && ic_ena[2]) begin
                      ichan_d = 4'd2; fsm_d = RR01;
                  end else if (rr_ne_ff[3] && ic_ena[3]) begin
                      ichan_d = 4'd3; fsm_d = RR01;
                  end else if (rr_ne_ff[4] && ic_ena[4]) begin
                      ichan_d = 4'd4; fsm_d = RR01;
                  end else if (rr_ne_ff[5] && ic_ena[5]) begin
                      ichan_d = 4'd5; fsm_d = RR01;
                  end else if (rr_ne_ff[6] && ic_ena[6]) begin
                      ichan_d = 4'd6; fsm_d = RR01;
                  end else if (rr_ne_ff[7] && ic_ena[7]) begin
                      ichan_d = 4'd7; fsm_d = RR01;
                  end else if (rr_ne_ff[8] && ic_ena[8]) begin
                      ichan_d = 4'd8; fsm_d = RR01;
                  end else if (rr_ne_ff[9] && ic_ena[9]) begin
                      ichan_d = 4'd9; fsm_d = RR01;
                  end else if (rr_ne_ff[10] && ic_ena[10]) begin
                      ichan_d = 4'd10; fsm_d = RR01;
                  end else begin
                      fsm_d = IDLE;
                  end
              end
            RR01:
              // Latch in FIFO word 0
              begin
                  if (dbg) $strobe("RR01: ichan=%1d", ichan);
                  rr0_d = rr_dat;
                  rr0_ena = 1'b1;
                  rr_rinc = 1'b1;  // advance FIFO: NB added pipeline delay
                  fsm_d = RR02;
              end
            RR02:
              // Wait for FIFO word 1 to appear (NB added pipeline delay)
              begin
                  if (dbg) $strobe("RR02");
                  rr_rinc = 1'b1;  // NB added pipeline delay
                  fsm_d = RR03;
                  // Don't bother checking if FIFO is non-empty, b/c
                  // not seeing all 4 words can only happen if there
                  // is a bug in the InputChannel logic.
              end
            RR03:
              // Latch in FIFO word 1
              begin
                  if (dbg) $strobe("RR03");
                  rr1_d = rr_dat;
                  rr1_ena = 1'b1;
                  rr_rinc = 1'b1;
                  fsm_d = RR04;
              end
            RR04:
              // Latch in FIFO word 2
              begin
                  if (dbg) $strobe("RR04");
                  rr2_d = rr_dat;
                  rr2_ena = 1'b1;
                  rr_rinc = 1'b1;
                  fsm_d = RR05;
              end
            RR05:
              // Send start-of-frame K-code; latch in FIFO word 3
              begin
                  if (dbg) $strobe("RR05");
                  rr3_d = rr_dat;
                  rr3_ena = 1'b1;
                  obyte = SB_SOF;
                  obyte_we = 1'b1;
                  fsm_d = RR06;
              end
            RR06:
              // Payload byte 0
              begin
                  if (dbg)
                    $strobe("RR06: ichan=%1d rr0123=%x %x %x %x",
                             ichan, rr0, rr1, rr2, rr3);
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,TYP_ABC_TRANSP, ichan[3:0]};
                  end else begin
                      obyte = {1'b0,TYP_ABC_RR, ichan[3:0]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR07;
              end
            RR07:
              // Payload byte 1
              begin
                  if (dbg) $strobe("RR07");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr0[15:8]};
                  end else begin
                      obyte = {1'b0,rr0[11:4]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR08;
              end
            RR08:
              // Payload byte 2
              begin
                  if (dbg) $strobe("RR08");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr0[7:0]};
                  end else begin
                      obyte = {1'b0,rr0[3:0],rr1[15:12]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR09;
              end
            RR09:
              // Payload byte 3
              begin
                  if (dbg) $strobe("RR09");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr1[15:8]};
                  end else begin
                      obyte = {1'b0,rr1[11:4]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR10;
              end
            RR10:
              // Payload byte 4
              begin
                  if (dbg) $strobe("RR10");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr1[7:0]};
                  end else begin
                      obyte = {1'b0,rr1[3:0],rr2[15:12]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR11;
              end
            RR11:
              // Payload byte 5
              begin
                  if (dbg) $strobe("RR11");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr2[15:8]};
                  end else begin
                      obyte = {1'b0,rr2[11:4]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR12;
              end
            RR12:
              // Payload byte 6
              begin
                  if (dbg) $strobe("RR12");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr2[7:0]};
                  end else begin
                      obyte = {1'b0,rr2[3:0],rr3[15:12]};
                  end
                  // 2018-05-13 removed 'out_full' check
                  obyte_we = 1'b1;
                  fsm_d = RR13;
              end
            RR13:
              // Payload byte 7
              begin
                  if (dbg) $strobe("RR13");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr3[15:8]};
                  end else begin
                      obyte = {1'b0,rr3[11:4]};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR14;
              end
            RR14:
              // Payload byte 8
              begin
                  if (dbg) $strobe("RR14");
                  if (opmode==OPMODE_PACKTRANS) begin
                      obyte = {1'b0,rr3[7:0]};
                  end else begin
                      obyte = {1'b0,rr3[3:0],4'b0};
                  end
                  obyte_we = 1'b1;
                  fsm_d = RR15;
              end
            RR15:
              // Send end-of-frame K-code
              begin
                  if (dbg) $strobe("RR15");
                  obyte = SB_EOF;
                  obyte_we = 1'b1;
                  fsm_d = IDLE;
              end
            default:
              begin
                  if ($time>0)
                    $strobe("INVALID STATE(t=%1d/%1d): fsm=%d fsm_prev=%d",
                            ticks, $time, fsm, fsm_prev);
                  fsm_d = START;
              end
        endcase
    end
endmodule // hcc_PacketBuilder

`default_nettype wire
