FPGA_DIR    := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
SCRIPTS_DIR := $(FPGA_DIR)scripts/
VIVADO_DIR  := $(FPGA_DIR)vivado/ultra96v2/

# ---------- Kernel selector ----------
# Usage: make -C fpga csim stream
#        make -C fpga bitstream tile

ifneq ($(filter stream,$(MAKECMDGOALS)),)
  KERNEL := stream
endif
ifneq ($(filter tile,$(MAKECMDGOALS)),)
  KERNEL := tile
endif

stream tile:
	@:

.PHONY: stream tile csim hls vivado bitstream clean_hls clean_vivado clean

# ---------- Build targets ----------

csim:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl run_csim_$(KERNEL).tcl

hls:
	cd $(SCRIPTS_DIR) && vitis-run --mode hls --tcl export_hls_ip_$(KERNEL).tcl
	cd $(VIVADO_DIR)ip_repo_$(KERNEL) && unzip -o export.zip

vivado: hls
	vivado -mode batch -source $(SCRIPTS_DIR)build_vivado.tcl -tclargs $(KERNEL)

bitstream: vivado

# ---------- Clean ----------

clean_hls:
	if exist "$(subst /,\,$(SCRIPTS_DIR)hls_build_$(KERNEL))" rmdir /s /q "$(subst /,\,$(SCRIPTS_DIR)hls_build_$(KERNEL))"

clean_vivado:
	if exist "$(subst /,\,$(VIVADO_DIR)vi_$(KERNEL))" rmdir /s /q "$(subst /,\,$(VIVADO_DIR)vi_$(KERNEL))"

ifdef KERNEL
clean: clean_hls clean_vivado
else
clean:
	$(MAKE) -f $(lastword $(MAKEFILE_LIST)) clean_hls clean_vivado stream
	$(MAKE) -f $(lastword $(MAKEFILE_LIST)) clean_hls clean_vivado tile
endif
