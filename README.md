# prince
The Prince lightweight block cipher in Verilog.

## Status
Core is functionally completed. Both core and top level provides correct
results for all test vectors. The core is lint clean. The core has not
yet been implemented in hardware.

Use with caution.


## Introduction
[Prince](https://eprint.iacr.org/2012/529.pdf) is a lightweight, low
latency block cipher suitable for Iot and embedded systems. The key is
128 bits and the block size is 64 bits.


## Implementation
The implementation is currently a pipelined designs with three pipeline
stages. With a cycle to initalize the state and a cycle for latching the
result the latency is five cycles. The design does not use any RAM
blocks nor DSPs.


## Implementation results
## Altera Cyclone V
* Device: 5CGXFC7C7F23C8
* Tool version: Intel Quartus Prime 19.1
* ALMs: 993
* Regs: 716
* Fmax: 102 MHz
