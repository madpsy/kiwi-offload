# Builds the eCPU assembler and assembles the eCPU code for the RX_CFG set in upstream/kiwi.config.
# Outputs go to build/. The assembler also regenerates upstream/verilog/kiwi.{cfg,gen}.vh and other.gen.vh.
BUILD := build
ASM   := $(BUILD)/tools/asm

all: ecode

$(ASM): $(wildcard upstream/e_cpu/asm/*.cpp upstream/e_cpu/asm/*.h)
	@mkdir -p $(BUILD)/tools
	g++ -Iupstream/e_cpu -Iupstream/e_cpu/asm -Iupstream -D_GNU_SOURCE -O1 -w upstream/e_cpu/asm/*.cpp -o $@

ecode: $(ASM)
	@mkdir -p $(BUILD)/gen
	cd upstream/e_cpu && ../../$(ASM) -2 -o ../../$(BUILD)/gen

clean:
	rm -rf $(BUILD)

.PHONY: all ecode clean

# Beagle-side bridge (static ARMv7 binary; the Kiwi runs Debian 11 / older glibc)
CROSS ?= arm-linux-gnueabihf-
bridge: $(BUILD)/arm/kbridge
$(BUILD)/arm/kbridge: bridge/kbridge.c bridge/kwf.c bridge/ksnap.c bridge/kgps.c bridge/kgps_run.c bridge/kgps_core.h bridge/kgeoid.h bridge/kleds.c bridge/khw.c bridge/klanes_dec.h bridge/kshm.h bridge/kconf.h $(BUILD)/arm/pffft.o
	@mkdir -p $(BUILD)/arm
	$(CROSS)gcc -O2 -Wall -Wextra -static -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=hard -Ithird_party/pffft \
		-o $@ $< $(BUILD)/arm/pffft.o -lm -pthread
# kgps_test: the GPS maths (bridge/kgps_core.h) on the build machine
kgps_test: $(BUILD)/kgps_test
	$(BUILD)/kgps_test
$(BUILD)/kgps_test: tools/kgps_test.c bridge/kgps_core.h bridge/kgeoid.h third_party/pffft/pffft.c
	@mkdir -p $(BUILD)
	gcc -O2 -Wall -Wextra -Ithird_party/pffft -o $@ tools/kgps_test.c third_party/pffft/pffft.c -lm
# kshm_probe: exercises kbridge's shared-memory interface (bridge/kshm.h) on the Beagle
kshm_probe: $(BUILD)/arm/kshm_probe
$(BUILD)/arm/kshm_probe: tools/kshm_probe.c bridge/kshm.h
	@mkdir -p $(BUILD)/arm
	$(CROSS)gcc -O2 -Wall -Wextra -static -march=armv7-a -mfpu=neon -mfloat-abi=hard -Ibridge -o $@ $< -lrt
.PHONY: kshm_probe
# pffft (third_party/pffft, FFTPACK licence): NEON real FFT for the waterfall computed on the Beagle
$(BUILD)/arm/pffft.o: third_party/pffft/pffft.c third_party/pffft/pffft.h
	@mkdir -p $(BUILD)/arm
	$(CROSS)gcc -O3 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=hard -c -o $@ $<

.PHONY: bridge

# PRU0 firmware (GNU PRU toolchain in vendor/gnupru, see README.md)
PRU_CC ?= vendor/gnupru/pru-elf/bin/pru-gcc
pru: $(BUILD)/pru/kpru.elf $(BUILD)/pru/kpru1.elf $(BUILD)/pru/klanes.elf
$(BUILD)/pru/kpru.elf: pru/kpru.c
	@mkdir -p $(BUILD)/pru
	$(PRU_CC) -O2 -mmcu=am335x.pru0 -Wall -Wno-array-bounds -o $@ $<
$(BUILD)/pru/kpru1.elf: pru/kpru.c
	@mkdir -p $(BUILD)/pru
	$(PRU_CC) -O2 -mmcu=am335x.pru1 -Wall -Wno-array-bounds -o $@ $<
$(BUILD)/pru/klanes.elf: pru/klanes.c
	@mkdir -p $(BUILD)/pru
	$(PRU_CC) -O2 -mmcu=am335x.pru0 -Wall -Wno-array-bounds -o $@ $<

.PHONY: pru

# Digital decoders for the web page (FT8, FT4, FT2: jt9; WSPR: wsprd), WebAssembly from ubersdr_wsprd's latest release:
# fetched once into build/wasm-dl/ ('make wasm-update' fetches again), unpacked into build/wasm/ (ft.mjs, ft.wasm,
# wspr.mjs, wspr.wasm, their licence). kweb serves them as /wasm/ (dist and deploys put them in kweb-www/wasm/).
WASM_URL := https://github.com/madpsy/ubersdr_wsprd/releases/download/latest
WASM_OUT := $(BUILD)/wasm
wasm: $(WASM_OUT)/ft.wasm $(WASM_OUT)/wspr.wasm
$(BUILD)/wasm-dl/%-wasm.tgz:
	@mkdir -p $(BUILD)/wasm-dl
	curl -fsSL -o $@.part $(WASM_URL)/$*-wasm.tgz && mv $@.part $@
$(WASM_OUT)/ft.wasm: $(BUILD)/wasm-dl/jt9-wasm.tgz
	@mkdir -p $(WASM_OUT)
	tar -xzf $< -C $(WASM_OUT) --strip-components=1 package/ft.mjs package/ft.wasm package/LICENSE
	mv $(WASM_OUT)/LICENSE $(WASM_OUT)/LICENSE-ubersdr_wsprd.txt; touch $@
$(WASM_OUT)/wspr.wasm: $(BUILD)/wasm-dl/wsprd-wasm.tgz
	@mkdir -p $(WASM_OUT)
	tar -xzf $< -C $(WASM_OUT) --strip-components=1 package/wspr.mjs package/wspr.wasm
	touch $@
wasm-update:
	rm -rf $(BUILD)/wasm-dl $(WASM_OUT)
	$(MAKE) wasm
.PHONY: wasm wasm-update

# Release for the Beagle: everything tools/install.sh needs, in the repository's own layout (so the same script also
# runs from a checkout with the ARM artifacts in build/). Result: build/dist/kiwi-offload-<version>/ and
# build/kiwi-offload-<version>.tar.gz; copy the tarball over, unpack, sudo ./tools/install.sh (README.md).
DIST_VER   := $(shell git describe --always --dirty 2>/dev/null || echo unknown)
DIST_NAME  := kiwi-offload-$(DIST_VER)
DIST_DIR   := $(BUILD)/dist/$(DIST_NAME)
DIST_SRC   := tools/install.sh tools/kiwi-offload.sh kweb/www kmod/kodma.c kmod/Makefile \
              config/kiwi-offload.conf.example config/chrony-kiwi-offload.conf config/kiwi-offload.target \
              config/kiwi-offload-kbridge.service config/kiwi-offload-kweb.service config/watchdog-kiwi-offload.conf \
              config/kiwi-offload-pps.service config/kiwi-offload-pps.dts config/journald-kiwi-offload.conf \
              firmware/stream_v25_13_lanes.bit upstream/bitstreams/KiwiSDR.rx8.wf3.aout README.md
DIST_BUILT := $(BUILD)/arm/kbridge $(BUILD)/arm/kweb/kweb $(BUILD)/pru/kpru.elf $(BUILD)/pru/kpru1.elf $(BUILD)/pru/klanes.elf
dist: bridge pru wasm
	$(MAKE) -C kweb
	rm -rf $(DIST_DIR)
	mkdir -p $(DIST_DIR)
	cp -r --parents $(DIST_SRC) $(DIST_BUILT) $(DIST_DIR)/
	mkdir -p $(DIST_DIR)/kweb/www/wasm && cp $(WASM_OUT)/* $(DIST_DIR)/kweb/www/wasm/
	echo $(DIST_VER) > $(DIST_DIR)/VERSION
	tar -C $(BUILD)/dist -czf $(BUILD)/$(DIST_NAME).tar.gz $(DIST_NAME)
	@echo "dist: $(BUILD)/$(DIST_NAME).tar.gz (on the Beagle: tar xzf, then sudo ./$(DIST_NAME)/tools/install.sh)"
.PHONY: dist
