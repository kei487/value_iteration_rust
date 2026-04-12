.PHONY: driver host test-host test-hw \
       csim hls vivado bitstream sync-hw-header \
       edf-docker edf-shell edf-setup edf-build \
       clean clean-fpga clean-edf

# ---------- Software (driver + host) ----------

driver:
	$(MAKE) -C driver/uio all

host: driver
	$(MAKE) -C host all

test-host:
	$(MAKE) -C host test-host

test-hw:
	$(MAKE) -C host test-hw

# ---------- FPGA (HLS + Vivado) ----------

csim:
	$(MAKE) -C fpga csim

hls:
	$(MAKE) -C fpga hls

vivado: hls
	$(MAKE) -C fpga vivado

bitstream: vivado

sync-hw-header: hls
	$(MAKE) -C driver/uio sync-hw-header

# ---------- EDF / Linux (Docker) ----------

edf-docker:
	$(MAKE) -C petalinux docker-build

edf-shell:
	$(MAKE) -C petalinux docker-shell

edf-setup:
	$(MAKE) -C petalinux edf-setup XSA=$(XSA)

edf-build:
	$(MAKE) -C petalinux edf-build MACHINE=$(MACHINE)

# ---------- Clean ----------

clean-edf:
	$(MAKE) -C petalinux clean

clean-fpga:
	$(MAKE) -C fpga clean

clean:
	$(MAKE) -C driver/uio clean
	$(MAKE) -C host clean
