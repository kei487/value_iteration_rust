REPO_ROOT   := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/..)
IMAGE_NAME  := vi-petalinux:2025.2
CONTAINER   := vi-petalinux-builder
PROJECT_DIR := project/vi_petalinux

# PetaLinux installer filename (place in docker/ before building)
PETALINUX_INSTALLER ?= petalinux-v2025.2-11160223-installer.run

# Path to .xsa (set when running create target)
XSA ?=

# Path to .bit (optional, for BOOT.BIN packaging)
BITSTREAM ?=

.PHONY: docker-build docker-shell plnx-create plnx-build plnx-package clean help

help:
	@echo "PetaLinux Docker build targets:"
	@echo ""
	@echo "  make docker-build                          Build Docker image"
	@echo "  make docker-shell                          Interactive shell in container"
	@echo "  make plnx-create XSA=<path/to/design.xsa>  Create & configure project"
	@echo "  make plnx-build                            Build Linux images"
	@echo "  make plnx-package BITSTREAM=<path/to/file.bit>  Package BOOT.BIN with bitstream"
	@echo "  make clean                                 Remove project and output dirs"
	@echo ""
	@echo "Prerequisites:"
	@echo "  1. Download petalinux-v2025.2-*-installer.run from AMD"
	@echo "  2. Place it in petalinux/docker/"
	@echo "  3. Run: make docker-build"

# ---------- Docker image ----------

docker-build:
	@if [ ! -f docker/$(PETALINUX_INSTALLER) ]; then \
		echo "ERROR: docker/$(PETALINUX_INSTALLER) not found."; \
		echo "Download it from https://www.xilinx.com/support/download.html"; \
		exit 1; \
	fi
	docker build -t $(IMAGE_NAME) \
		--build-arg PETALINUX_INSTALLER=$(PETALINUX_INSTALLER) \
		docker/

# ---------- Run helpers ----------

define DOCKER_RUN
	docker run --rm -it \
		-v "$(REPO_ROOT):/work" \
		-w /work \
		$(IMAGE_NAME)
endef

docker-shell:
	$(DOCKER_RUN) bash

# ---------- PetaLinux workflow ----------

plnx-create:
	@if [ -z "$(XSA)" ]; then \
		echo "ERROR: XSA not set. Usage: make plnx-create XSA=path/to/design.xsa"; \
		exit 1; \
	fi
	$(DOCKER_RUN) bash /work/petalinux/scripts/create_project.sh "$(XSA)"

plnx-build:
	@if [ ! -d "$(PROJECT_DIR)" ]; then \
		echo "ERROR: Project not found. Run 'make plnx-create XSA=...' first."; \
		exit 1; \
	fi
	$(DOCKER_RUN) bash /work/petalinux/scripts/build.sh

plnx-package:
	@if [ -z "$(BITSTREAM)" ]; then \
		echo "ERROR: BITSTREAM not set. Usage: make plnx-package BITSTREAM=path/to/file.bit"; \
		exit 1; \
	fi
	$(DOCKER_RUN) bash /work/petalinux/scripts/build.sh --bitstream "$(BITSTREAM)"

# ---------- Clean ----------

clean:
	rm -rf project/ output/
