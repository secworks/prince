//======================================================================
//
// prince.v
// --------
// Top level wrapper for the PRINCE block cipher core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2020, Assured AB
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

module prince(
              // Clock and reset.
              input wire           clk,
              input wire           reset_n,

              // Control.
              input wire           cs,
              input wire           we,

              // Data ports.
              input wire  [7 : 0]  address,
              input wire  [31 : 0] write_data,
              output wire [31 : 0] read_data
             );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
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
  localparam ADDR_KEY3         = 8'h13;

  localparam ADDR_BLOCK0       = 8'h20;
  localparam ADDR_BLOCK1       = 8'h21;

  localparam ADDR_RESULT0      = 8'h30;
  localparam ADDR_RESULT1      = 8'h31;

  localparam CORE_NAME0        = 32'h7072696e; // "prin"
  localparam CORE_NAME1        = 32'h63652020; // "ce  "
  localparam CORE_VERSION      = 32'h302e3130; // "0.10"


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg next_reg;
  reg next_new;

  reg encdec_reg;
  reg config_we;

  reg [31 : 0] block_reg [0 : 1];
  reg          block_we;

  reg [31 : 0] key_reg [0 : 3];
  reg          key_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0]   tmp_read_data;

  wire           core_ready;
  wire [127 : 0] core_key;
  wire [63 : 0]  core_block;
  wire [63 : 0]  core_result;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;

  assign core_key = {key_reg[3], key_reg[2], key_reg[1], key_reg[0]};
  assign core_block  = {block_reg[1], block_reg[0]};


  //----------------------------------------------------------------
  // core instantiation.
  //----------------------------------------------------------------
  prince_core core(
                   .clk(clk),
                   .reset_n(reset_n),

                   .encdec(encdec_reg),
                   .next(next_reg),
                   .ready(core_ready),

                   .key(core_key),

                   .block(core_block),
                   .result(core_result)
                  );


  //----------------------------------------------------------------
  // reg_update
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin : reg_update
      integer i;

      if (!reset_n)
        begin
          for (i = 0 ; i < 2 ; i = i + 1)
            block_reg[i] <= 32'h0;

          for (i = 0 ; i < 4 ; i = i + 1)
            key_reg[i] <= 32'h0;

          next_reg   <= 1'h0;
          encdec_reg <= 1'h0;
        end
      else
        begin
          next_reg <= next_new;

          if (config_we)
            begin
              encdec_reg <= write_data[CONFIG_ENCDEC_BIT];
            end

          if (key_we)
            key_reg[address[1 : 0]] <= write_data;

          if (block_we)
            block_reg[address[0]] <= write_data;
        end
    end // reg_update


  //----------------------------------------------------------------
  // api
  //
  // The interface command decoding logic.
  //----------------------------------------------------------------
  always @*
    begin : api
      next_new      = 1'h0;
      config_we     = 1'h0;
      key_we        = 1'h0;
      block_we      = 1'h0;
      tmp_read_data = 32'h0;

      if (cs)
        begin
          if (we)
            begin
              if (core_ready)
                begin
                  if (address == ADDR_CTRL)
                    next_new = write_data[CTRL_NEXT_BIT];

                  if (address == ADDR_CONFIG)
                    config_we = 1'h1;

                  if ((address >= ADDR_KEY0) && (address <= ADDR_KEY3))
                    key_we = 1'h1;

                  if ((address >= ADDR_BLOCK0) && (address <= ADDR_BLOCK1))
                    block_we = 1'h1;
                end
            end

          else
            begin
              case (address)
                ADDR_NAME0:   tmp_read_data = CORE_NAME0;
                ADDR_NAME1:   tmp_read_data = CORE_NAME1;
                ADDR_VERSION: tmp_read_data = CORE_VERSION;
                ADDR_STATUS:  tmp_read_data = {31'h0, core_ready};
                ADDR_RESULT0: tmp_read_data = core_result[31 : 0];
                ADDR_RESULT1: tmp_read_data = core_result[63 : 32];
                default:
                  begin
                  end
              endcase // case (address)
            end
        end
    end // addr_decoder
endmodule // prince

//======================================================================
// EOF prince.v
//======================================================================
