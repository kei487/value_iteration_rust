# MATLAB HDL Coder Streaming Kernel

Third kernel variant for the Value Iteration FPGA accelerator, built with
MATLAB HDL Coder + SoC Blockset.

## Required Toolboxes

- MATLAB R2024b+
- Simulink
- HDL Coder
- HDL Verifier
- Fixed-Point Designer
- SoC Blockset
- Zynq UltraScale+ MPSoC support package (or Ultra96-V2 BSP)

## Quick Start

```matlab
% 1. Run the matlab.unittest suite (no toolboxes needed beyond base MATLAB)
run_matlab_tests

% 2. Fixed-point analysis (requires Fixed-Point Designer)
cd fixedpoint; fp_config

% 3. HDL cosimulation (requires HDL Verifier + Vivado Xsim)
cd cosim; cosim_tb

% 4. Bitstream generation (requires HDL Coder + SoC Blockset + Vivado)
cd soc; build_bitstream
```

## Directory Structure

```
matlab/
├── src/           MATLAB functions (HDL Coder targets)
├── test/          matlab.unittest test suite
├── testbench/     Test data generators and compatibility scripts
├── fixedpoint/    Fixed-Point Advisor configuration
├── cosim/         HDL Verifier cosimulation
├── model/         Simulink models (.slx)
├── run_matlab_tests.m
└── soc/           SoC Builder configuration
```

## Development Workflow

### Phase A: Floating-Point Verification

1. Edit algorithm in `src/*.m` (all signals are `double`)
2. Run `run_matlab_tests` to execute the MATLAB unit and integration suite
3. Iterate until all tests pass

### Phase B: Fixed-Point Conversion

1. Run `fixedpoint/fp_config.m` to analyze dynamic range
2. Open Simulink model -> Fixed-Point Tool -> apply proposed types
3. Re-run `run_matlab_tests` to verify zero-error conversion
4. Target bit widths: value=16, penalty=16, offset=8 (matching HLS)

### Phase C: HDL Generation and Cosimulation

1. Open `model/vi_sweep_stream_matlab.slx` in Simulink
2. HDL Workflow Advisor -> Generate HDL
3. Run `cosim/cosim_tb.m` with Xsim backend
4. Verify cycle-accurate match against golden MATLAB output

### Phase D: Bitstream and Hardware

1. Run `soc/build_bitstream.m` (or use HDL Workflow Advisor GUI)
2. Deploy .bit + .hwh to Ultra96-V2
3. Test via `vi_cli --verify` with MATLAB driver ops

## Makefile Targets

From project root:

```bash
make matlab-sim        # Run matlab.unittest suite
make matlab-hdl        # Generate HDL
make matlab-cosim      # Run cosimulation
make matlab-bitstream  # Build bitstream
```

## Constants

All constants are defined in `src/vi_params.m` and match
`fpga/hls/stream/src/vi_stream_types.h`. See the design spec at
`docs/superpowers/specs/2026-04-13-matlab-hdl-coder-streaming-design.md`.
