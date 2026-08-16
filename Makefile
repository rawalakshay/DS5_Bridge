# DS5 Bridge firmware build helper.
#
# Wraps the CMake/Ninja workflow documented in docs/development.md and drops the
# resulting UF2 into uf2_builds/ ready to copy onto a BOOTSEL-mounted Pico.
#
#   make pico-build               Pico 2 W, Release, companion interface on
#   make DIAGNOSTICS=traces       diagnostic build (see docs/diagnostics.md)
#   make test                     host-side firmware tests (no Pico SDK needed)
#   make clean                    remove build trees, keep built UF2s
#   make help                     list targets and current settings
#
# The Pico SDK is not vendored. Point PICO_SDK_PATH at a 2.3.0 checkout either
# in the environment or per invocation:
#
#   make PICO_SDK_PATH=/path/to/pico-sdk

OUTPUT_DIR      := uf2_builds
BUILD_ROOT      := build
GENERATOR       ?= Ninja
BUILD_TYPE      ?= Release
DIAGNOSTICS     ?= off

FIRMWARE_VERSION := $(shell tr -d ' \t\n\r' < firmware-version.txt)

# Optional machine-local settings, git-ignored. Use it to pin the SDK once
# instead of passing PICO_SDK_PATH on every invocation:
#
#   echo 'PICO_SDK_PATH ?= /path/to/pico-sdk' > local.mk
#
# Use ?= there so an explicit environment variable still wins.
-include local.mk

# Nothing set? Look in the usual places before giving up. The versioned
# .pico-sdk path is where the official VS Code extension installs, and matches
# the sdkVersion this project pins in CMakeLists.txt.
PICO_SDK_CANDIDATES := \
	$(HOME)/.pico-sdk/sdk/2.3.0 \
	$(HOME)/pico-sdk \
	$(CURDIR)/../pico-sdk \
	$(CURDIR)/../../pico-sdk

ifeq ($(strip $(PICO_SDK_PATH)),)
PICO_SDK_PATH := $(firstword $(foreach d,$(PICO_SDK_CANDIDATES),\
	$(if $(wildcard $(d)/pico_sdk_init.cmake),$(d))))
endif

# Non-default diagnostics presets get their own build tree so a traces build
# never reuses a release cache. Note the artifact name is timestamp-only, so a
# diagnostics image is not distinguishable from a release one by filename.
ifeq ($(DIAGNOSTICS),off)
DIAG_SUFFIX :=
else
DIAG_SUFFIX := -$(DIAGNOSTICS)
endif

PICO2W_BUILD_DIR := $(BUILD_ROOT)/pico2w$(DIAG_SUFFIX)
TEST_BUILD_DIR   := build-firmware-tests

# Artifacts are named ds5-bridge-DDMMYY-HHMM.uf2. The stamp is taken inside the
# recipe rather than at parse time so it reflects when the image was actually
# produced, not when make started.
STAMP_FMT := +%d%m%y-%H%M
LATEST_UF2 := ds5-bridge-latest.uf2

CMAKE_FLAGS = -G $(GENERATOR) \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DPICO_SDK_PATH=$(PICO_SDK_PATH) \
	-DENABLE_COMPANION=ON \
	-DDS5_DIAGNOSTICS_PRESET=$(DIAGNOSTICS)

.PHONY: pico-build test clean help check-sdk

pico-build: check-sdk | $(OUTPUT_DIR)
	cmake -S . -B $(PICO2W_BUILD_DIR) $(CMAKE_FLAGS)
	cmake --build $(PICO2W_BUILD_DIR) --target ds5-bridge
	@out="$(OUTPUT_DIR)/ds5-bridge-`date $(STAMP_FMT)`.uf2"; \
	cp $(PICO2W_BUILD_DIR)/ds5-bridge.uf2 "$$out" && \
	cp $(PICO2W_BUILD_DIR)/ds5-bridge.uf2 $(OUTPUT_DIR)/$(LATEST_UF2) && \
	echo "" && \
	echo "Pico 2 W firmware ready:" && \
	echo "  $$out" && \
	echo "  $(OUTPUT_DIR)/$(LATEST_UF2)"

# Host-side logic and source-guard tests. Deliberately has no SDK dependency so
# it runs anywhere, including CI containers without the ARM toolchain.
test:
	cmake -S tests/firmware -B $(TEST_BUILD_DIR) -G $(GENERATOR)
	cmake --build $(TEST_BUILD_DIR)
	ctest --test-dir $(TEST_BUILD_DIR) --output-on-failure

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

# Removes build trees only. Built UF2s in $(OUTPUT_DIR) are left alone so a
# clean never throws away a firmware image you may still need to flash.
clean:
	rm -rf $(BUILD_ROOT) $(TEST_BUILD_DIR)
	@echo "Removed build trees. $(OUTPUT_DIR)/ was left untouched."

check-sdk:
	@if [ -z "$(PICO_SDK_PATH)" ]; then \
		echo "No Pico SDK found. Looked in:"; \
		for d in $(PICO_SDK_CANDIDATES); do echo "  $$d"; done; \
		echo ""; \
		echo "Set it once for this machine (git-ignored):"; \
		echo "  echo 'PICO_SDK_PATH ?= /path/to/pico-sdk' > local.mk"; \
		echo ""; \
		echo "Or per invocation:"; \
		echo "  make pico-build PICO_SDK_PATH=/path/to/pico-sdk"; \
		echo ""; \
		echo "No SDK checkout yet? Pico SDK 2.3.0 is expected:"; \
		echo "  git clone --depth 1 --branch 2.3.0 \\"; \
		echo "      https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk"; \
		echo "  git -C ~/pico-sdk submodule update --init --recursive"; \
		echo ""; \
		echo "See docs/development.md."; \
		exit 1; \
	fi
	@if [ ! -f "$(PICO_SDK_PATH)/pico_sdk_init.cmake" ]; then \
		echo "PICO_SDK_PATH=$(PICO_SDK_PATH)"; \
		echo "does not look like a Pico SDK checkout (no pico_sdk_init.cmake)."; \
		exit 1; \
	fi

help:
	@echo "DS5 Bridge firmware $(FIRMWARE_VERSION)"
	@echo ""
	@echo "Targets:"
	@echo "  pico-build       Build the firmware for Pico 2 W (default)"
	@echo "  test             Run host-side firmware tests (no Pico SDK required)"
	@echo "  clean            Remove build trees, keeping built UF2s"
	@echo ""
	@echo "Settings:"
	@echo "  PICO_SDK_PATH = $(if $(PICO_SDK_PATH),$(PICO_SDK_PATH),(unset))"
	@echo "  DIAGNOSTICS   = $(DIAGNOSTICS)   (off, audio, traces, all, custom)"
	@echo "  BUILD_TYPE    = $(BUILD_TYPE)"
	@echo "  GENERATOR     = $(GENERATOR)"
	@echo ""
	@echo "Artifact name (DDMMYY-HHMM, stamped at copy time):"
	@echo "  $(OUTPUT_DIR)/ds5-bridge-`date $(STAMP_FMT)`.uf2"
