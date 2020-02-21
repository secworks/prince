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
The implementation is currently a single cyle designs with all rounds as
functions in the datapath. This will replicate the logic needed and will
create a low latency implementation. Right now the max clock frequency
is not known. Or the total size of the design.
