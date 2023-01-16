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

`default_nettype none

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
  localparam CTRL_IDLE   = 3'h0;
  localparam CTRL_PIPE0  = 3'h1;
  localparam CTRL_PIPE1  = 3'h2;
  localparam CTRL_PIPE2  = 3'h3;
  localparam CTRL_UPDATE = 3'h4;

  localparam ALPHA = 64'hc0ac29b7c97c50dd;


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
  reg          state_we;

  reg [63 : 0] r3_reg;
  reg [63 : 0] r3_new;

  reg [63 : 0] r8_reg;
  reg [63 : 0] r8_new;

  reg [63 : 0] mr_reg;
  reg [63 : 0] mr_new;

  reg [2 : 0]  core_ctrl_reg;
  reg [2 : 0]  core_ctrl_new;
  reg          core_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg init_state;
  reg update_state;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign ready  = ready_reg;
  assign result = state_reg;


  //----------------------------------------------------------------
  // Internal functions.
  //----------------------------------------------------------------
`include "prince_round_functions.vh"


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin: reg_update
      if (!reset_n)
        begin
          ready_reg     <= 1'h1;
          k0_reg        <= 64'h0;
          k1_reg        <= 64'h0;
          kp_reg        <= 64'h0;
          r3_reg        <= 64'h0;
          r8_reg        <= 64'h0;
          mr_reg        <= 64'h0;
          state_reg     <= 64'h0;
          core_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          r3_reg <= r3_new;
          r8_reg <= r8_new;
          mr_reg <= mr_new;

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

      reg [63 : 0] core_input;
      reg [63 : 0] core_output;

      reg [63 : 0] r0;
      reg [63 : 0] r1;
      reg [63 : 0] r2;
      reg [63 : 0] r4;
      reg [63 : 0] r5;
      reg [63 : 0] r6;
      reg [63 : 0] r7;
      reg [63 : 0] r9;
      reg [63 : 0] r10;
      reg [63 : 0] r11;

      state_new = 64'h0;
      state_we  = 1'h0;
      k0_new    = 64'h0;
      k1_new    = 64'h0;
      kp_new    = 64'h0;
      k_we      = 1'h0;


      // Pipeline stages.
      core_input = state_reg ^ k0_reg;
      r0 = round0(core_input, k1_reg);
      r1 = round(r0, k1_reg, 1);
      r2 = round(r1, k1_reg, 2);
      r3_new = round(r2, k1_reg, 3);

      r4 = round(r3_reg, k1_reg, 4);
      r5 = round(r4, k1_reg, 5);
      mr_new = middle_round(r5);

      r6 = iround(mr_reg, k1_reg, 6);
      r7 = iround(r6, k1_reg, 7);
      r8_new = iround(r7, k1_reg, 8);

      r9  = iround(r8_reg, k1_reg, 9);
      r10 = iround(r9, k1_reg, 10);
      r11 = round11(r10, k1_reg);
      core_output = r11 ^ kp_reg;


      if (init_state)
        begin
          k_we      = 1'h1;
          state_new = block;
          state_we  = 1'h1;

          if (encdec)
            begin
              k0_new = key[127 : 64];
              kp_new = {k0_new[0], k0_new[63 : 2], (k0_new[1] ^ k0_new[63])};
              k1_new = key[63 : 0];
            end
          else
            begin
              kp_new = key[127 : 64];
              k0_new = {kp_new[0], kp_new[63 : 2], (kp_new[1] ^ kp_new[63])};
              k1_new = key[63 : 0] ^ ALPHA;
            end
        end

      if (update_state)
        begin
          state_new = core_output;
          state_we  = 1'h1;
        end
    end // prince_core_dp


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
      update_state  = 1'h0;
      core_ctrl_new = CTRL_IDLE;
      core_ctrl_we  = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (next)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                init_state    = 1'h1;
                core_ctrl_new = CTRL_PIPE0;
                core_ctrl_we  = 1'h1;
              end
          end

        CTRL_PIPE0:
          begin
            core_ctrl_new = CTRL_PIPE1;
            core_ctrl_we  = 1'h1;
          end

        CTRL_PIPE1:
          begin
            core_ctrl_new = CTRL_PIPE2;
            core_ctrl_we  = 1'h1;
          end

        CTRL_PIPE2:
          begin
            core_ctrl_new = CTRL_UPDATE;
            core_ctrl_we  = 1'h1;
          end

        CTRL_UPDATE:
          begin
            ready_new     = 1'h1;
            ready_we      = 1'h1;
            update_state  = 1'h1;
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
