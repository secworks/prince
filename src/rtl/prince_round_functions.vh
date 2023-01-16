//======================================================================
//
// prince_round_functions.v
// ------------------------
// Functions used in the prince core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2023, Assured AB
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

  function [63 : 0] s(input [63 : 0] x);
    s = {sb(x[63 : 60]), sb(x[59 : 56]),
         sb(x[55 : 52]), sb(x[51 : 48]),
         sb(x[47 : 44]), sb(x[43 : 40]),
         sb(x[39 : 36]), sb(x[35 : 32]),
         sb(x[31 : 28]), sb(x[27 : 24]),
         sb(x[23 : 20]), sb(x[19 : 16]),
         sb(x[15 : 12]), sb(x[11 : 08]),
         sb(x[07 : 04]), sb(x[03 : 00])};
  endfunction // s

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
  endfunction // isb

  function [63 : 0] si(input [63 : 0] x);
    si = {isb(x[63 : 60]), isb(x[59 : 56]),
          isb(x[55 : 52]), isb(x[51 : 48]),
          isb(x[47 : 44]), isb(x[43 : 40]),
          isb(x[39 : 36]), isb(x[35 : 32]),
          isb(x[31 : 28]), isb(x[27 : 24]),
          isb(x[23 : 20]), isb(x[19 : 16]),
          isb(x[15 : 12]), isb(x[11 : 08]),
          isb(x[07 : 04]), isb(x[03 : 00])};
  endfunction // si

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

  function [63 : 0] mp(input [63 : 0] b);
    begin
      mp[63] = b[59] ^ b[55] ^ b[51];
      mp[62] = b[62] ^ b[54] ^ b[50];
      mp[61] = b[61] ^ b[57] ^ b[49];
      mp[60] = b[60] ^ b[56] ^ b[52];
      mp[59] = b[63] ^ b[59] ^ b[55];
      mp[58] = b[58] ^ b[54] ^ b[50];
      mp[57] = b[61] ^ b[53] ^ b[49];
      mp[56] = b[60] ^ b[56] ^ b[48];
      mp[55] = b[63] ^ b[59] ^ b[51];
      mp[54] = b[62] ^ b[58] ^ b[54];
      mp[53] = b[57] ^ b[53] ^ b[49];
      mp[52] = b[60] ^ b[52] ^ b[48];
      mp[51] = b[63] ^ b[55] ^ b[51];
      mp[50] = b[62] ^ b[58] ^ b[50];
      mp[49] = b[61] ^ b[57] ^ b[53];
      mp[48] = b[56] ^ b[52] ^ b[48];
      mp[47] = b[47] ^ b[43] ^ b[39];
      mp[46] = b[42] ^ b[38] ^ b[34];
      mp[45] = b[45] ^ b[37] ^ b[33];
      mp[44] = b[44] ^ b[40] ^ b[32];
      mp[43] = b[47] ^ b[43] ^ b[35];
      mp[42] = b[46] ^ b[42] ^ b[38];
      mp[41] = b[41] ^ b[37] ^ b[33];
      mp[40] = b[44] ^ b[36] ^ b[32];
      mp[39] = b[47] ^ b[39] ^ b[35];
      mp[38] = b[46] ^ b[42] ^ b[34];
      mp[37] = b[45] ^ b[41] ^ b[37];
      mp[36] = b[40] ^ b[36] ^ b[32];
      mp[35] = b[43] ^ b[39] ^ b[35];
      mp[34] = b[46] ^ b[38] ^ b[34];
      mp[33] = b[45] ^ b[41] ^ b[33];
      mp[32] = b[44] ^ b[40] ^ b[36];
      mp[31] = b[31] ^ b[27] ^ b[23];
      mp[30] = b[26] ^ b[22] ^ b[18];
      mp[29] = b[29] ^ b[21] ^ b[17];
      mp[28] = b[28] ^ b[24] ^ b[16];
      mp[27] = b[31] ^ b[27] ^ b[19];
      mp[26] = b[30] ^ b[26] ^ b[22];
      mp[25] = b[25] ^ b[21] ^ b[17];
      mp[24] = b[28] ^ b[20] ^ b[16];
      mp[23] = b[31] ^ b[23] ^ b[19];
      mp[22] = b[30] ^ b[26] ^ b[18];
      mp[21] = b[29] ^ b[25] ^ b[21];
      mp[20] = b[24] ^ b[20] ^ b[16];
      mp[19] = b[27] ^ b[23] ^ b[19];
      mp[18] = b[30] ^ b[22] ^ b[18];
      mp[17] = b[29] ^ b[25] ^ b[17];
      mp[16] = b[28] ^ b[24] ^ b[20];
      mp[15] = b[11] ^ b[07] ^ b[03];
      mp[14] = b[14] ^ b[06] ^ b[02];
      mp[13] = b[13] ^ b[09] ^ b[01];
      mp[12] = b[12] ^ b[08] ^ b[04];
      mp[11] = b[15] ^ b[11] ^ b[07];
      mp[10] = b[10] ^ b[06] ^ b[02];
      mp[09] = b[13] ^ b[05] ^ b[01];
      mp[08] = b[12] ^ b[08] ^ b[00];
      mp[07] = b[15] ^ b[11] ^ b[03];
      mp[06] = b[14] ^ b[10] ^ b[06];
      mp[05] = b[09] ^ b[05] ^ b[01];
      mp[04] = b[12] ^ b[04] ^ b[00];
      mp[03] = b[15] ^ b[07] ^ b[03];
      mp[02] = b[14] ^ b[10] ^ b[02];
      mp[01] = b[13] ^ b[09] ^ b[05];
      mp[00] = b[08] ^ b[04] ^ b[00];
    end
  endfunction // mp

  function [63 : 0] m(input [63 : 0] b);
    begin : m_func
      reg [63 : 0] t;
      t = mp(b);

      m = {t[63 : 60], t[43 : 40], t[23 : 20], t[03 : 00],
           t[47 : 44], t[27 : 24], t[07 : 04], t[51 : 48],
           t[31 : 28], t[11 : 08], t[55 : 52], t[35 : 32],
           t[15 : 12], t[59 : 56], t[39 : 36], t[19 : 16]};
    end
  endfunction // m

  function [63 : 0] mi(input [63 : 0] b);
    begin : mi_func
      reg [63 : 0] t;

      t = {b[63 : 60], b[11 : 08], b[23 : 20], b[35 : 32],
           b[47 : 44], b[59 : 56], b[07 : 04], b[19 : 16],
           b[31 : 28], b[43 : 40], b[55 : 52], b[03 : 00],
           b[15 : 12], b[27 : 24], b[39 : 36], b[51 : 48]};

      mi = mp(t);
    end
  endfunction // m

  function [63 : 0] round0(input [63 : 0] b, input [63 : 0] k);
    begin
      round0 = b ^ k ^ rc(0);
    end
  endfunction // round0

  function [63 : 0] round11(input [63 : 0] b, input [63 : 0] k);
    begin
      round11 =  b ^ k ^ rc(11);
    end
  endfunction // round11

  function [63 : 0] round(input [63 : 0] b, input [63 : 0] k, input [3 : 0] n);
    begin
      round = m(s(b)) ^ rc(n) ^ k;
    end
  endfunction // round

  function [63 : 0] middle_round(input [63 : 0] b);
    begin
      middle_round = si(mp(s(b)));
    end
  endfunction // middle_round

  function [63 : 0] iround(input [63 : 0] b, input [63 : 0] k, input [3 : 0] n);
    begin
      iround = si(mi(rc(n) ^ k ^ b));
    end
  endfunction // iround

//======================================================================
// EOF prince_round_functions.vh
//======================================================================
