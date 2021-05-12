//======================================================================
//
// tb_prince_core.v
// --------------
// Testbench for the prince block cipher core.
// Testvectors from:
// https://github.com/sebastien-riou/prince-c-ref/blob/master/log.txt
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

module tb_prince_core();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG     = 0;
  parameter DUMP_WAIT = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD = 2 * CLK_HALF_PERIOD;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;
  reg          tb_monitor;

  reg           tb_clk;
  reg           tb_reset_n;
  reg           tb_encdec;
  reg           tb_init;
  reg           tb_next;
  wire          tb_ready;
  reg [127 : 0] tb_key;
  reg [63 : 0]  tb_block;
  wire [63 : 0] tb_result;


  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  prince_core dut(
                .clk(tb_clk),
                .reset_n(tb_reset_n),

                .encdec(tb_encdec),
                .next(tb_next),
                .ready(tb_ready),

                .key(tb_key),

                .block(tb_block),
                .result(tb_result)
               );


  //----------------------------------------------------------------
  // clk_gen
  //
  // Always running clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD;
      tb_clk = !tb_clk;
    end // clk_gen


  //----------------------------------------------------------------
  // sys_monitor()
  //
  // An always running process that creates a cycle counter and
  // conditionally displays information about the DUT.
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      cycle_ctr = cycle_ctr + 1;
      #(CLK_PERIOD);
      if (tb_monitor)
        begin
          dump_dut_state();
        end
    end


  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state;
    begin
      $display("State of DUT");
      $display("------------");
      $display("Cycle: %08d", cycle_ctr);
      $display("Inputs and outputs:");
      $display("encdec = 0x%01x, next = 0x%01x", dut.encdec, dut.next);
      $display("key   = 0x%016x ", dut.key);
      $display("block = 0x%08x", dut.block);
      $display("ready  = 0x%01x", dut.ready);
      $display("result = 0x%08x", dut.result);
      $display("");
      $display("Internal states:");
      $display("k0: 0x%08x, k1: 0x%08x, kp: 0x%08x",
               dut.k0_reg, dut.k1_reg, dut.kp_reg);
      $display("");
      $display("core_input: 0x%08x", dut.prince_core_dp.core_input);
      $display("r0: 0x%08x, r1:  0x%08x, r2:  0x%08x",
               dut.prince_core_dp.r0, dut.prince_core_dp.r1, dut.prince_core_dp.r2);
      $display("r3: 0x%08x, r4:  0x%08x, r5:  0x%08x",
               dut.r3_reg, dut.prince_core_dp.r4, dut.prince_core_dp.r5);
      $display("mr: 0x%08x", dut.mr_reg);
      $display("r6: 0x%08x, r7:  0x%08x, r8:  0x%08x",
               dut.prince_core_dp.r6, dut.prince_core_dp.r7, dut.r8_reg);
      $display("r9: 0x%08x, r10: 0x%08x, r11: 0x%08x",
               dut.prince_core_dp.r9, dut.prince_core_dp.r10, dut.prince_core_dp.r11);
      $display("core_output: 0x%08x", dut.prince_core_dp.core_output);
      $display("state_reg: 0x%016x, state_new: 0x%016x, state_we: 0x%01x",
               dut.state_reg, dut.state_new, dut.state_we);
      $display("");
      $display("init_state: 0x%01x, update_state: 0x%01x",
               dut.init_state, dut.update_state);
      $display("core_ctrl_reg: 0x%02x, core_ctrl_new: 0x%02x, core_ctrl_we: 0x%01x",
               dut.core_ctrl_reg, dut.core_ctrl_new, dut.core_ctrl_we);
      $display("");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // reset_dut()
  //
  // Toggle reset to put the DUT into a well known state.
  //----------------------------------------------------------------
  task reset_dut;
    begin
      $display("*** DUT before reset:");
      dump_dut_state();
      $display("*** Toggling reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
      $display("*** DUT after reset:");
      dump_dut_state();
    end
  endtask // reset_dut


  //----------------------------------------------------------------
  // display_test_result()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_result;
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_result


  //----------------------------------------------------------------
  // wait_ready()
  //
  // Wait for the ready flag in the dut to be set.
  //
  // Note: It is the callers responsibility to call the function
  // when the dut is actively processing and will in fact at some
  // point set the flag.
  //----------------------------------------------------------------
  task wait_ready;
    begin
      #(2 * CLK_PERIOD);
      while (!tb_ready)
        begin
          #(CLK_PERIOD);
          if (DUMP_WAIT)
            begin
              dump_dut_state();
            end
        end
    end
  endtask // wait_ready


  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim;
    begin
      cycle_ctr  = 0;
      error_ctr  = 0;
      tc_ctr     = 0;
      tb_monitor = 0;

      tb_clk     = 0;
      tb_reset_n = 1;
      tb_encdec  = 0;
      tb_next    = 0;
      tb_key     = 128'h0;
      tb_block   = 64'h0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // test()
  // Runs an encipher, decipher test with given key and plaintext
  // The generated ciphertext is verified with the given ciphertet.
  // The generated plaintxt is also verified against the
  // given plaintext.
  //----------------------------------------------------------------
  task test(input integer tc, input reg [127 : 0] key,
            input reg [63 : 0] plaintext, input reg [63 : 0] ciphertext);
    begin
      tc_ctr = tc_ctr + 1;
      tb_monitor = 0;

      $display("*** TC%01d started.", tc);
      dump_dut_state();

      $display("*** TC%01d - encryption started.", tc);
      tb_key     = key;
      tb_encdec  = 1'h1;
      tb_block   = plaintext;
      tb_next    = 1'h1;
      #(CLK_PERIOD);
      tb_next    = 1'h0;
      wait_ready();
      $display("*** TC%01d - encryption completed.", tc);
      dump_dut_state();

      if (tb_result == ciphertext)
        $display("*** TC%01d correct ciphertext generated: 0x%016x",
                 tc, tb_result);
      else
        begin
          error_ctr = error_ctr + 1;
          $display("*** TC%01d incorrect ciphertext generated", tc);
          $display("*** TC%01d expected: 0x%016x", tc, ciphertext);
          $display("*** TC%01d got:      0x%016x", tc, tb_result);
        end

      $display("*** TC%01d - decryption started.", tc);
      tb_key    = key;
      tb_block  = ciphertext;
      tb_encdec = 1'h0;
      tb_next   = 1'h1;
      #(CLK_PERIOD);
      tb_next    = 1'h0;
      wait_ready();
      $display("*** TC%01d - decryption completed.", tc);

      if (tb_result == plaintext)
        $display("*** TC%01d correct plaintext generated: 0x%016x",
                 tc, tb_result);
      else
        begin
          error_ctr = error_ctr + 1;
          $display("*** TC%01d incorrect plaintext generated", tc);
          $display("*** TC%01d expected: 0x%016x", tc, plaintext);
          $display("*** TC%01d got:      0x%016x", tc, tb_result);
        end

      tb_monitor = 0;

      $display("*** TC%01d completed.", tc);
      $display("");
    end
  endtask // test


  //----------------------------------------------------------------
  // prince_core_test
  //
  // Test vectors from:
  //----------------------------------------------------------------
  initial
    begin : prince_core_test
      $display("*** Simulation of PRINCE core started.");
      $display("");

      init_sim();
      reset_dut();

      test(1, 128'h00000000_00000000_00000000_00000000,
           64'h00000000_00000000, 64'h818665aa_0d02dfda);

      test(2, 128'h00000000_00000000_00000000_00000000,
           64'hffffffff_ffffffff, 64'h604ae6ca_03c20ada);

      test(3, 128'hffffffff_ffffffff_00000000_00000000,
           64'h00000000_00000000, 64'h9fb51935_fc3df524);

      test(4, 128'h00000000_00000000_ffffffff_ffffffff,
           64'h00000000_00000000, 64'h78a54cbe_737bb7ef);

      test(5, 128'h00000000_00000000_fedcba98_76543210,
           64'h01234567_89abcdef, 64'hae25ad3c_a8fa9ccf);

      test(6, 128'h00112233_44556677_8899aabb_ccddeeff,
           64'h01234567_89abcdef, 64'hd6dcb597_8de756ee);

      test(7, 128'h01122334_45566778_899aabbc_cddeeff0,
           64'h01234567_89abcdef, 64'h392f599f_46761cd3);

      test(8, 128'h01122334_45566778_899aabbc_cddeeff0,
           64'hf0123456_789abcde, 64'h4fb5e332_b9b409bb);

      test(9, 128'hd8cdb780_70b4c55a_818665aa_0d02dfda,
           64'h69c4e0d8_6a7b0430, 64'h43c6b256_d79de7e8);

      display_test_result();
      $display("");
      $display("*** Simulation of PRINCE core completed.");
      $finish;
    end // prince_core_test
endmodule // tb_prince_core

//======================================================================
// EOF tb_prince_core.v
//======================================================================
