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
                   input wire           init,
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
  localparam CTRL_NEXT    = 2'h2;

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

  reg [1 : 0]  core_ctrl_reg;
  reg [1 : 0]  core_ctrl_new;
  reg          core_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg init_keys;
  reg init_state;
  reg update_state;


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

  function [63 : 0] mp(input [63 : 0] block);
    begin
      mp[63] = block[59] ^ block[55] ^ block[51];
      mp[62] = block[62] ^ block[54] ^ block[50];
      mp[61] = block[61] ^ block[57] ^ block[49];
      mp[60] = block[60] ^ block[56] ^ block[52];
      mp[59] = block[63] ^ block[59] ^ block[55];
      mp[58] = block[58] ^ block[54] ^ block[50];
      mp[57] = block[61] ^ block[53] ^ block[49];
      mp[56] = block[60] ^ block[56] ^ block[48];
      mp[55] = block[63] ^ block[59] ^ block[51];
      mp[54] = block[62] ^ block[58] ^ block[54];
      mp[53] = block[57] ^ block[53] ^ block[49];
      mp[52] = block[60] ^ block[52] ^ block[48];
      mp[51] = block[63] ^ block[55] ^ block[51];
      mp[50] = block[62] ^ block[58] ^ block[50];
      mp[49] = block[61] ^ block[57] ^ block[53];
      mp[48] = block[56] ^ block[52] ^ block[48];
      mp[47] = block[47] ^ block[43] ^ block[39];
      mp[46] = block[42] ^ block[38] ^ block[34];
      mp[45] = block[45] ^ block[37] ^ block[33];
      mp[44] = block[44] ^ block[40] ^ block[32];
      mp[43] = block[47] ^ block[43] ^ block[35];
      mp[42] = block[46] ^ block[42] ^ block[38];
      mp[41] = block[41] ^ block[37] ^ block[33];
      mp[40] = block[44] ^ block[36] ^ block[32];
      mp[39] = block[47] ^ block[39] ^ block[35];
      mp[38] = block[46] ^ block[42] ^ block[34];
      mp[37] = block[45] ^ block[41] ^ block[37];
      mp[36] = block[40] ^ block[36] ^ block[32];
      mp[35] = block[43] ^ block[39] ^ block[35];
      mp[34] = block[46] ^ block[38] ^ block[34];
      mp[33] = block[45] ^ block[41] ^ block[33];
      mp[32] = block[44] ^ block[40] ^ block[36];
      mp[31] = block[31] ^ block[27] ^ block[23];
      mp[30] = block[26] ^ block[22] ^ block[18];
      mp[29] = block[29] ^ block[21] ^ block[17];
      mp[28] = block[28] ^ block[24] ^ block[16];
      mp[27] = block[31] ^ block[27] ^ block[19];
      mp[26] = block[30] ^ block[26] ^ block[22];
      mp[25] = block[25] ^ block[21] ^ block[17];
      mp[24] = block[28] ^ block[20] ^ block[16];
      mp[23] = block[31] ^ block[23] ^ block[19];
      mp[22] = block[30] ^ block[26] ^ block[18];
      mp[21] = block[29] ^ block[25] ^ block[21];
      mp[20] = block[24] ^ block[20] ^ block[16];
      mp[19] = block[27] ^ block[23] ^ block[19];
      mp[18] = block[30] ^ block[22] ^ block[18];
      mp[17] = block[29] ^ block[25] ^ block[17];
      mp[16] = block[28] ^ block[24] ^ block[20];
      mp[15] = block[11] ^ block[7]  ^ block[3];
      mp[14] = block[14] ^ block[6]  ^ block[2];
      mp[13] = block[13] ^ block[9]  ^ block[1];
      mp[12] = block[12] ^ block[8]  ^ block[4];
      mp[11] = block[15] ^ block[11] ^ block[7];
      mp[10] = block[10] ^ block[6]  ^ block[2];
      mp[9]  = block[13] ^ block[5]  ^ block[1];
      mp[8]  = block[12] ^ block[8]  ^ block[0];
      mp[7]  = block[15] ^ block[11] ^ block[3];
      mp[6]  = block[14] ^ block[10] ^ block[6];
      mp[5]  = block[9]  ^ block[5]  ^ block[1];
      mp[4]  = block[12] ^ block[4]  ^ block[0];
      mp[3]  = block[15] ^ block[7]  ^ block[3];
      mp[2]  = block[14] ^ block[10] ^ block[2];
      mp[1]  = block[13] ^ block[9]  ^ block[5];
      mp[0]  = block[8]  ^ block[4]  ^ block[0];
    end
  endfunction // mp

  function [63 : 0] imp(input [63 : 0] block);
    begin
      imp[63] = block[59] ^ block[55] ^ block[51];
      imp[62] = block[62] ^ block[54] ^ block[50];
      imp[61] = block[61] ^ block[57] ^ block[49];
      imp[60] = block[60] ^ block[56] ^ block[52];
      imp[59] = block[63] ^ block[59] ^ block[55];
      imp[58] = block[58] ^ block[54] ^ block[50];
      imp[57] = block[61] ^ block[53] ^ block[49];
      imp[56] = block[60] ^ block[56] ^ block[48];
      imp[55] = block[63] ^ block[59] ^ block[51];
      imp[54] = block[62] ^ block[58] ^ block[54];
      imp[53] = block[57] ^ block[53] ^ block[49];
      imp[52] = block[60] ^ block[52] ^ block[48];
      imp[51] = block[63] ^ block[55] ^ block[51];
      imp[50] = block[62] ^ block[58] ^ block[50];
      imp[49] = block[61] ^ block[57] ^ block[53];
      imp[48] = block[56] ^ block[52] ^ block[48];
      imp[47] = block[47] ^ block[43] ^ block[39];
      imp[46] = block[42] ^ block[38] ^ block[34];
      imp[45] = block[45] ^ block[37] ^ block[33];
      imp[44] = block[44] ^ block[40] ^ block[32];
      imp[43] = block[47] ^ block[43] ^ block[35];
      imp[42] = block[46] ^ block[42] ^ block[38];
      imp[41] = block[41] ^ block[37] ^ block[33];
      imp[40] = block[44] ^ block[36] ^ block[32];
      imp[39] = block[47] ^ block[39] ^ block[35];
      imp[38] = block[46] ^ block[42] ^ block[34];
      imp[37] = block[45] ^ block[41] ^ block[37];
      imp[36] = block[40] ^ block[36] ^ block[32];
      imp[35] = block[43] ^ block[39] ^ block[35];
      imp[34] = block[46] ^ block[38] ^ block[34];
      imp[33] = block[45] ^ block[41] ^ block[33];
      imp[32] = block[44] ^ block[40] ^ block[36];
      imp[31] = block[31] ^ block[27] ^ block[23];
      imp[30] = block[26] ^ block[22] ^ block[18];
      imp[29] = block[29] ^ block[21] ^ block[17];
      imp[28] = block[28] ^ block[24] ^ block[16];
      imp[27] = block[31] ^ block[27] ^ block[19];
      imp[26] = block[30] ^ block[26] ^ block[22];
      imp[25] = block[25] ^ block[21] ^ block[17];
      imp[24] = block[28] ^ block[20] ^ block[16];
      imp[23] = block[31] ^ block[23] ^ block[19];
      imp[22] = block[30] ^ block[26] ^ block[18];
      imp[21] = block[29] ^ block[25] ^ block[21];
      imp[20] = block[24] ^ block[20] ^ block[16];
      imp[19] = block[27] ^ block[23] ^ block[19];
      imp[18] = block[30] ^ block[22] ^ block[18];
      imp[17] = block[29] ^ block[25] ^ block[17];
      imp[16] = block[28] ^ block[24] ^ block[20];
      imp[15] = block[11] ^ block[7]  ^ block[3];
      imp[14] = block[14] ^ block[6]  ^ block[2];
      imp[13] = block[13] ^ block[9]  ^ block[1];
      imp[12] = block[12] ^ block[8]  ^ block[4];
      imp[11] = block[15] ^ block[11] ^ block[7];
      imp[10] = block[10] ^ block[6]  ^ block[2];
      imp[9]  = block[13] ^ block[5]  ^ block[1];
      imp[8]  = block[12] ^ block[8]  ^ block[0];
      imp[7]  = block[15] ^ block[11] ^ block[3];
      imp[6]  = block[14] ^ block[10] ^ block[6];
      imp[5]  = block[9]  ^ block[5]  ^ block[1];
      imp[4]  = block[12] ^ block[4]  ^ block[0];
      imp[3]  = block[15] ^ block[7]  ^ block[3];
      imp[2]  = block[14] ^ block[10] ^ block[2];
      imp[1]  = block[13] ^ block[9]  ^ block[5];
      imp[0]  = block[8]  ^ block[4]  ^ block[0];
    end
  endfunction // imp

  function [63 : 0] round0(input [63 : 0] block, input [63 : 0] key);
    begin
      round0 = block ^ key ^ rc(0);
    end
  endfunction // round0

  function [63 : 0] round11(input [63 : 0] block, input [63 : 0] key);
    begin
      round11 = block;
    end
  endfunction // round11

  function [63 : 0] middle(input [63 : 0] block);
    begin
      middle = block;
    end
  endfunction // middle

  function [63 : 0] round(input [63 : 0] block, input [63 : 0] key, input [3 : 0] n);
    begin
      round = mp(sbox(block)) ^ rc(n) ^ key;
    end
  endfunction // round


  function [63 : 0] iround(input [63 : 0] block, input [63 : 0] key, input [3 : 0] n);
    begin
      iround = isbox(imp(rc(n) ^ key ^ block));
    end
  endfunction // iround


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
      reg [63 : 0] r3;
      reg [63 : 0] r4;
      reg [63 : 0] r5;
      reg [63 : 0] mr;
      reg [63 : 0] r6;
      reg [63 : 0] r7;
      reg [63 : 0] r8;
      reg [63 : 0] r9;
      reg [63 : 0] r10;
      reg [63 : 0] r11;


      core_input  = 64'h0;
      core_output = 64'h0;
      r0          = 64'h0;
      r1          = 64'h0;
      r2          = 64'h0;
      r3          = 64'h0;
      r4          = 64'h0;
      r5          = 64'h0;
      mr          = 64'h0;
      r6          = 64'h0;
      r7          = 64'h0;
      r8          = 64'h0;
      r9          = 64'h0;
      r10         = 64'h0;
      r11         = 64'h0;
      state_new   = 64'h0;
      state_we    = 1'h0;
      k0_new      = 64'h0;
      k1_new      = 64'h0;
      kp_new      = 64'h0;
      k_we        = 1'h0;

      if (init_keys)
        begin
          k_we = 1'h1;
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

      if (init_state)
        begin
          state_new = block;
          state_we  = 1'h1;
        end

      if (update_state)
        begin
          core_input = state_reg ^ k0_reg;
          r0  = round0(core_input, k1_reg);

          r1  = round(r0, k1_reg, 1);
          r2  = round(r1, k1_reg, 2);
          r3  = round(r2, k1_reg, 3);
          r4  = round(r3, k1_reg, 4);
          r5  = round(r4, k1_reg, 5);

          mr  = middle(r5);

          r6  = iround(mr, k1_reg, 6);
          r7  = iround(r6, k1_reg, 7);
          r8  = iround(r7, k1_reg, 8);
          r9  = iround(r8, k1_reg, 9);
          r10 = iround(r9, k1_reg, 10);

          r11 = round11(r10, k1_reg);
          core_output = r11 ^ kp_reg;

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
      init_keys     = 1'h0;
      core_ctrl_new = CTRL_IDLE;
      core_ctrl_we  = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (init)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                init_keys     = 1'h1;
                core_ctrl_new = CTRL_INIT;
                core_ctrl_we  = 1'h1;
              end

            if (next)
              begin
                ready_new     = 1'h0;
                ready_we      = 1'h1;
                init_state    = 1'h1;
                core_ctrl_new = CTRL_NEXT;
                core_ctrl_we  = 1'h1;
              end
          end

        CTRL_INIT:
          begin
            ready_new     = 1'h1;
            ready_we      = 1'h1;
            core_ctrl_new = CTRL_IDLE;
            core_ctrl_we  = 1'h1;
          end

        CTRL_NEXT:
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
