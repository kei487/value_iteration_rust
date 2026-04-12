FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SCRIPTS_DIR := $(FPGA_DIR)scripts/

.PHONY: csim_tile hls_tile vivado_tile bitstream_tile \
        csim_stream hls_stream vivado_stream bitstream_stream \
        clean_hls_tile clean_hls_stream clean_vivado_tile clean_vivado_stream clean

# ---------- Tile-based kernel ----------

csim_tile:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl run_csim_tile.tcl

hls_tile:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl export_hls_ip_tile.tcl

vivado_tile: hls_tile
	vivado -mode batch -source $(SCRIPTS_DIR)build_vivado.tcl -tclargs tile

bitstream_tile: vivado_tile

# ---------- Streaming kernel ----------

csim_stream:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl run_csim_stream.tcl

hls_stream:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl export_hls_ip_stream.tcl

vivado_stream: hls_stream
	vivado -mode batch -source $(SCRIPTS_DIR)build_vivado.tcl -tclargs stream

bitstream_stream: vivado_stream

# ---------- Clean ----------

clean_hls_tile:
	rm -rf $(SCRIPTS_DIR)hls_build_tile

clean_hls_stream:
	rm -rf $(SCRIPTS_DIR)hls_build_stream

clean_vivado_tile:
	rm -rf $(FPGA_DIR)vivado/ultra96v2/vi_tile

clean_vivado_stream:
	rm -rf $(FPGA_DIR)vivado/ultra96v2/vi_stream

clean: clean_hls_tile clean_hls_stream clean_vivado_tile clean_vivado_stream
