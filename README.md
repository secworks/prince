# prince
The Prince lightweight block cipher in Verilog.

## Status
Core is functionally completed. Both core and top level provides correct
results for all test vectors. The core is lint clean. The core has been
implemented in hardware.


## Introduction
[Prince](https://eprint.iacr.org/2012/529.pdf) is a lightweight, low
latency block cipher suitable for Iot and embedded systems. The key is
128 bits and the block size is 64 bits.


## Implementation details
The implementation is currently a pipelined designs with three pipeline
stages. With a cycle to initalize the state and a cycle for latching the
result the latency is five cycles. The design does not use any RAM
blocks nor DSPs.


## FuseSoC
This core is supported by the
[FuseSoC](https://github.com/olofk/fusesoc) core package manager and
build system. Some quick  FuseSoC instructions:

Install FuseSoC
~~~
pip install fusesoc
~~~

Create and enter a new workspace
~~~
mkdir workspace && cd workspace
~~~

Register prince as a library in the workspace
~~~
fusesoc library add prince /path/to/prince
~~~
...if repo is available locally or...
...to get the upstream repo
~~~
fusesoc library add prince https://github.com/secworks/prince
~~~

Run tb_prince testbench
~~~
fusesoc run --target=tb_prince secworks:crypto:prince
~~~

Run with modelsim instead of default tool (icarus)
~~~
fusesoc run --target=tb_prince --tool=modelsim secworks:crypto:prince
~~~


## Implementation results

### Altera Cyclone V
* Device: 5CGXFC7C7F23C8
* Tool version: Intel Quartus Prime 19.1
* ALMs: 993
* Regs: 716
* Fmax: 102 MHz

### Xilinx Artix-7
* Device: xc7a200tsbv484-2
* Tool version: Vivado 2019.2
* LUTs: 1587
* FFs: 646
* Fmax: 150 MHz
