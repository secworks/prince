//======================================================================
//
// prince_core.v
// -------------
// Prince block cipher core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2019, Assured AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module prince_core(
                   input wire           clk,
                   input wire           reset_n,

                   input wire           encdec,
                   input wire           next,
                   output wire          ready,

                   input wire [127 : 0] key,

                   input wire [63 : 0]  block,
                   output wire [63 : 0] result
                  );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam CTRL_IDLE    = 2'h0;
  localparam CTRL_INIT    = 2'h1;
  localparam CTRL_ROUNDS0 = 2'h2;
  localparam CTRL_ROUNDS1 = 2'h3;

  localparam NUM_ROUNDS   = 10;

  localparam alpha = 64'hc0ac29b7c97c50dd;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [63 : 0] k0_reg;
  reg [63 : 0] k0_new;
  reg [63 : 0] k1_reg;
  reg [63 : 0] k1_new;
  reg [63 : 0] kp_reg;
  reg [63 : 0] kp_new;
  reg          k_we;

  reg          ready_reg;
  reg          ready_new;
  reg          ready_we;

  reg [63 : 0] state_reg;
  reg [63 : 0] state_new;
  reg           state_we;

  reg [5 : 0]  round_ctr_reg;
  reg [5 : 0]  round_ctr_new;
  reg          round_ctr_rst;
  reg          round_ctr_inc;
  reg          round_ctr_we;

  reg [1 : 0]  core_ctrl_reg;
  reg [1 : 0]  core_ctrl_new;
  reg          core_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg init_keys;
  reg init_state;
  reg round_state;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign ready  = ready_reg;
  assign result = state_reg;


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  function [3 : 0] sb(input [3 : 0] x);
    case(x)
      4'h0: sb = 4'hb;
      4'h1: sb = 4'hf;
      4'h2: sb = 4'h3;
      4'h3: sb = 4'h2;
      4'h4: sb = 4'ha;
      4'h5: sb = 4'hc;
      4'h6: sb = 4'h9;
      4'h7: sb = 4'h1;
      4'h8: sb = 4'h6;
      4'h9: sb = 4'h7;
      4'ha: sb = 4'h8;
      4'hb: sb = 4'h0;
      4'hc: sb = 4'he;
      4'hd: sb = 4'h5;
      4'he: sb = 4'hd;
      4'hf: sb = 4'h4;
    endcase // case (x)
  endfunction // sb


  function [63 : 0] sbox(input [63 : 0] x);
    sbox = {sb(x[63 : 60]), sb(x[59 : 56]),
            sb(x[55 : 52]), sb(x[51 : 48]),
            sb(x[47 : 44]), sb(x[43 : 40]),
            sb(x[39 : 36]), sb(x[35 : 32]),
            sb(x[31 : 28]), sb(x[27 : 24]),
            sb(x[23 : 20]), sb(x[19 : 16]),
            sb(x[15 : 12]), sb(x[11 : 08]),
            sb(x[07 : 04]), sb(x[03 : 00])};
  endfunction // sbox


  function [3 : 0] isb(input [3 : 0] x);
    case(x)
      4'h0: isb = 4'hb;
      4'h1: isb = 4'h7;
      4'h2: isb = 4'h3;
      4'h3: isb = 4'h2;
      4'h4: isb = 4'hf;
      4'h5: isb = 4'hd;
      4'h6: isb = 4'h8;
      4'h7: isb = 4'h9;
      4'h8: isb = 4'ha;
      4'h9: isb = 4'h6;
      4'ha: isb = 4'h4;
      4'hb: isb = 4'h0;
      4'hc: isb = 4'h5;
      4'hd: isb = 4'he;
      4'he: isb = 4'hc;
      4'hf: isb = 4'h1;
    endcase // case (x)
  endfunction // sb


  function [63 : 0] isbox(input [63 : 0] x);
    isbox = {isb(x[63 : 60]), isb(x[59 : 56]),
             isb(x[55 : 52]), isb(x[51 : 48]),
             isb(x[47 : 44]), isb(x[43 : 40]),
             isb(x[39 : 36]), isb(x[35 : 32]),
             isb(x[31 : 28]), isb(x[27 : 24]),
             isb(x[23 : 20]), isb(x[19 : 16]),
             isb(x[15 : 12]), isb(x[11 : 08]),
             isb(x[07 : 04]), isb(x[03 : 00])};
  endfunction // sbox


  function [63 : 0] sr(input [63 : 0] x);
    sr = {x[00 * 4 +:4], x[05 * 4 +:4], x[10 * 4 +:4], x[15 * 4 +:4],
          x[04 * 4 +:4], x[09 * 4 +:4], x[14 * 4 +:4], x[03 * 4 +:4],
          x[08 * 4 +:4], x[13 * 4 +:4], x[02 * 4 +:4], x[07 * 4 +:4],
          x[12 * 4 +:4], x[01 * 4 +:4], x[06 * 4 +:4], x[11 * 4 +:4]};
  endfunction // sr


  function [63 : 0] rc(input [3 : 0] round);
    begin
      case(round)
        00: rc = 64'h0000000000000000;
        01: rc = 64'h13198a2e03707344;
        02: rc = 64'ha4093822299f31d0;
        03: rc = 64'h082efa98ec4e6c89;
        04: rc = 64'h452821e638d01377;
        05: rc = 64'hbe5466cf34e90c6c;
        06: rc = 64'h7ef84f78fd955cb1;
        07: rc = 64'h85840851f1ac43aa;
        08: rc = 64'hc882d32f25323c54;
        09: rc = 64'h64a51195e0e3610d;
        10: rc = 64'hd3b5a399ca0c2399;
        11: rc = 64'hc0ac29b7c97c50dd;

        default:
          rc = 64'h0;
      endcase // case (round)
    end
  endfunction // rc


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin: reg_update
      if (!reset_n)
        begin
          ready_reg     <= 1'h1;
          k0_reg        <= 64'h0;
          k1_reg        <= 64'h0;
          kp_reg        <= 64'h0;
          state_reg     <= 64'h0;
          round_ctr_reg <= NUM_ROUNDS;
          core_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (ready_we)
            ready_reg <= ready_new;

          if (state_we)
            state_reg <= state_new;

          if (k_we)
            begin
              k0_reg <= k0_new;
              k1_reg <= k1_new;
              kp_reg <= kp_new;
            end

          if (round_ctr_we)
            round_ctr_reg <= round_ctr_new;

          if (core_ctrl_we)
            core_ctrl_reg <= core_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // prince_core_dp
  //
  // Datapath with state update logic.
  //----------------------------------------------------------------
  always @*
    begin : prince_core_dp
      state_new = 64'h0;
      state_we  = 1'h0;

      k0_new = 64'h0;
      k1_new = 64'h0;
      kp_new = 64'h0;
      k_we   = 1'h0;

      if (init_state)
        begin
          state_new = block;
          state_we  = 1'h1;
        end

      if (round_state)
        begin
          state_new = sbox(state_reg);
          state_we  = 1'h1;
        end

      if (init_keys)
        begin
          k0_new = key[127 : 64];
          k1_new = key[63 : 0];
          kp_new = {k0_new[0], k0_new[63 : 2], k0_new[63] ^ k0_new[1]};
          k_we   = 1'h1;
        end
    end // prince_core_dp


  //----------------------------------------------------------------
  // round_ctr
  //
  // Update logic for the round counter.
  //----------------------------------------------------------------
  always @*
    begin : round_ctr
      round_ctr_new = 6'h0;
      round_ctr_we  = 1'h0;

      if (round_ctr_rst)
        round_ctr_we  = 1'h1;

      if (round_ctr_inc)
        begin
          round_ctr_new = round_ctr_reg + 1'h1;
          round_ctr_we  = 1'h1;
        end
    end // round_ctr


  //----------------------------------------------------------------
  // prince_core_ctrl
  //
  // Control FSM for aes core.
  //----------------------------------------------------------------
  always @*
    begin : prince_core_ctrl
      ready_new     = 1'h0;
      ready_we      = 1'h0;
      init_state    = 1'h0;
      round_state   = 1'h0;
      init_keys     = 1'h0;
      round_ctr_rst = 1'h0;
      round_ctr_inc = 1'h0;
      core_ctrl_new = CTRL_IDLE;
      core_ctrl_we  = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (next)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                core_ctrl_new = CTRL_INIT;
                core_ctrl_we  = 1'h1;
              end
          end

        CTRL_INIT:
          begin
            init_state    = 1'h1;
            init_keys     = 1'h1;
            round_ctr_rst = 1'h1;
            ready_new     = 1'h1;
            ready_we      = 1'h1;
            core_ctrl_new = CTRL_IDLE;
            core_ctrl_we  = 1'h1;
          end

        default:
          begin
          end
      endcase // case (core_ctrl_reg)
    end // prince_core_ctrl

endmodule // prince_core

//======================================================================
// EOF prince_core.v
//======================================================================
