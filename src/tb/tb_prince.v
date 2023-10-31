//======================================================================
//
// tb_prince.v
// -----------
// Testbench for the prince top level wrapper
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

module tb_prince();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG     = 0;
  parameter DUMP_WAIT = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD = 2 * CLK_HALF_PERIOD;

  localparam ADDR_NAME0        = 8'h00;
  localparam ADDR_NAME1        = 8'h01;
  localparam ADDR_VERSION      = 8'h02;

  localparam ADDR_CTRL         = 8'h08;
  localparam CTRL_NEXT_BIT     = 0;

  localparam ADDR_STATUS       = 8'h09;
  localparam STATUS_READY_BIT  = 0;

  localparam ADDR_CONFIG       = 8'h0a;
  localparam CONFIG_ENCDEC_BIT = 0;

  localparam ADDR_KEY0         = 8'h10;
  localparam ADDR_KEY1         = 8'h11;
  localparam ADDR_KEY2         = 8'h12;
  localparam ADDR_KEY3         = 8'h13;

  localparam ADDR_BLOCK0       = 8'h20;
  localparam ADDR_BLOCK1       = 8'h21;

  localparam ADDR_RESULT0      = 8'h30;
  localparam ADDR_RESULT1      = 8'h31;

  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;
  reg          tb_monitor;

  reg           tb_clk;
  reg           tb_reset_n;
  reg           tb_cs;
  reg           tb_we;
  reg [7 : 0]   tb_address;
  reg [31 : 0]  tb_write_data;
  wire [31 : 0] tb_read_data;

  reg [31 : 0] read_data;


  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  prince dut(
           .clk(tb_clk),
           .reset_n(tb_reset_n),

           .cs(tb_cs),
           .we(tb_we),

           .address(tb_address),
           .write_data(tb_write_data),
           .read_data(tb_read_data)
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
      $display("");
      $display("Core inputs and outputs:");
      $display("next: 0x%01x, encdec: 0x%01x, ready: 0x%01x",
               dut.core.next, dut.core.encdec, dut.core.ready);
      $display("key:    0x%032x", dut.core.key);
      $display("block:  0x%016x", dut.core.block);
      $display("result: 0x%016x", dut.core.result);
      $display("");
      $display("Core state:");
      $display("k0: 0x%016x", dut.core.k0_reg);
      $display("k1: 0x%016x", dut.core.k1_reg);
      $display("kp: 0x%016x", dut.core.kp_reg);
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
      $display("--- Toggle reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
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
          $display("--- All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("--- %02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_result


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

      tb_clk        = 1'h0;
      tb_reset_n    = 1'h1;
      tb_cs         = 1'h0;
      tb_we         = 1'h0;
      tb_address    = 8'h0;
      tb_write_data = 32'h0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // write_word()
  //
  // Write the given word to the DUT using the DUT interface.
  //----------------------------------------------------------------
  task write_word(input [11 : 0] address,
                  input [31 : 0] word);
    begin
      if (DEBUG)
        begin
          $display("--- Writing 0x%08x to 0x%02x.", word, address);
          $display("");
        end

      tb_address = address;
      tb_write_data = word;
      tb_cs = 1;
      tb_we = 1;
      #(2 * CLK_PERIOD);
      tb_cs = 0;
      tb_we = 0;
    end
  endtask // write_word


  //----------------------------------------------------------------
  // read_word()
  //
  // Read a data word from the given address in the DUT.
  // the word read will be available in the global variable
  // read_data.
  //----------------------------------------------------------------
  task read_word(input [11 : 0]  address);
    begin
      tb_address = address;
      tb_cs = 1;
      tb_we = 0;
      #(CLK_PERIOD);
      read_data = tb_read_data;
      tb_cs = 0;

      if (DEBUG)
        begin
          $display("--- Reading 0x%08x from 0x%02x.", read_data, address);
          $display("");
        end
    end
  endtask // read_word


  //----------------------------------------------------------------
  // wait_ready()
  //
  // Wait for the ready flag to be set in dut.
  //----------------------------------------------------------------
  task wait_ready;
    begin : wready
      read_word(ADDR_STATUS);
      while (read_data == 0)
        read_word(ADDR_STATUS);
    end
  endtask // wait_ready


  //----------------------------------------------------------------
  // test()
  //----------------------------------------------------------------
  task test(input integer testcase, input [127 : 0] key,
            input [63 : 0] plaintext, input [63 : 0] ciphertext);
    begin : test
      reg [63 : 0] enc_res;
      reg [63 : 0] dec_res;

      tc_ctr = tc_ctr + 1;

      $display("");
      $display("--- TC%02d started.", testcase);

      // Encryption.
      write_word(ADDR_KEY0, key[031 : 00]);
      write_word(ADDR_KEY1, key[063 : 32]);
      write_word(ADDR_KEY2, key[095 : 64]);
      write_word(ADDR_KEY3, key[127 : 96]);
      write_word(ADDR_CONFIG, 32'h1);
      write_word(ADDR_BLOCK0, plaintext[31 : 00]);
      write_word(ADDR_BLOCK1, plaintext[63 : 32]);
      write_word(ADDR_CTRL, 32'h1);
      $display("--- TC%02d - encryption started.", testcase);
      wait_ready();
      $display("--- TC%02d - encryption completed.", testcase);

      read_word(ADDR_RESULT0);
      enc_res[31 : 0] = read_data;
      read_word(ADDR_RESULT1);
      enc_res[63 : 32] = read_data;

      if (enc_res == ciphertext)
        begin
          $display("--- Correct ciphertext received.");
        end
      else
        begin
          $display("--- Incorrect ciphertext received. Expected 0x%016x, got 0x%016x", ciphertext, enc_res);
          error_ctr = error_ctr + 1;
        end
      $display("");

      // Decryption.
      write_word(ADDR_BLOCK0, ciphertext[31 : 00]);
      write_word(ADDR_BLOCK1, ciphertext[63 : 32]);
      write_word(ADDR_CONFIG, 32'h0);
      write_word(ADDR_CTRL, 32'h1);
      $display("--- TC%02d - decryption started.", testcase);
      wait_ready();
      $display("--- TC%02d - decryption started.", testcase);

      read_word(ADDR_RESULT0);
      dec_res[31 : 0] = read_data;
      read_word(ADDR_RESULT1);
      dec_res[63 : 32] = read_data;

      if (dec_res == plaintext)
        begin
          $display("--- Correct plaintext received.");
        end
      else
        begin
          $display("--- Incorrect plaintext received. Expected 0x%016x, got 0x%016x", plaintext, dec_res);
          error_ctr = error_ctr + 1;
        end

      $display("--- TC%02d completed.", testcase);
      $display("");
    end
  endtask // test


  //----------------------------------------------------------------
  // prince_test
  //----------------------------------------------------------------
  initial
    begin : prince_test
      $display("   -= Testbench for prince started =-");
      $display("     ==============================");
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
      $display("   -= Testbench for prince completed =-");
      $display("     ================================");
      $finish;
    end // prince_test
endmodule // tb_prince

//======================================================================
// EOF tb_prince.v
//======================================================================
