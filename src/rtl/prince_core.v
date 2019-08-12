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

  reg [127 : 0] state_reg;
  reg [127 : 0] state_new;
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


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign ready  = ready_reg;
  assign result = {v0_reg, v1_reg};


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
          state_reg     <= 128'h0;
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
      state_new = 128'h0;
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

      if (init_keys)
        begin
          k0_new = key[127 : 64];
          k1_new = key[63 : 0];
          kp_new = {k0_new[0], k0_new[63 : 2], k0_new[63] ^ k0_new[1]};
          k0_we = 1'h1;
          k1_we = 1'h1;
          kp_we = 1'h1;
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
