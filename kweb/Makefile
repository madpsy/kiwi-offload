# kweb: static ARMv7 (Cortex-A8, NEON) build for the Beagle. Run from the repository root: make -C kweb
CROSS ?= arm-linux-gnueabihf-
BUILD ?= ../build/arm/kweb
ARCH  = -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=hard
# libopus from tools/build_opus.sh (fixed point: ~4x cheaper on the A8 than the float build)
OPUS ?= ../build/arm/opus-fixed
# -fno-math-errno: sqrtf etc. as instructions, not errno-setting calls; -fcx-limited-range: complex multiplies
# without the NaN recovery branch. Neither changes results for finite values.
CXXFLAGS = -O2 -std=c++17 -Wall -Wextra -fno-math-errno -fcx-limited-range $(ARCH) -I../bridge -I../third_party/pffft -I../third_party/civetweb -I$(OPUS)/include/opus
CFLAGS_CW = -O2 $(ARCH) -DNO_SSL -DNO_CGI -DUSE_WEBSOCKET -DNO_CACHING -I../third_party/civetweb
SRC = src/main.cpp
HDR = src/dsp.hpp src/wfenc.hpp src/codec.hpp src/shm.hpp ../bridge/kshm.h ../bridge/kconf.h

all: $(BUILD)/kweb

$(BUILD)/kweb: $(SRC) $(HDR) $(BUILD)/civetweb.o $(BUILD)/pffft.o $(OPUS)/lib/libopus.a
	@mkdir -p $(BUILD)
	$(CROSS)g++ $(CXXFLAGS) -static -o $@ $(SRC) $(BUILD)/civetweb.o $(BUILD)/pffft.o $(OPUS)/lib/libopus.a -lpthread -lrt -lm

$(OPUS)/lib/libopus.a:
	../tools/build_opus.sh

$(BUILD)/civetweb.o: ../third_party/civetweb/civetweb.c
	@mkdir -p $(BUILD)
	$(CROSS)gcc $(CFLAGS_CW) -c -o $@ $<

$(BUILD)/pffft.o: ../third_party/pffft/pffft.c
	@mkdir -p $(BUILD)
	$(CROSS)gcc -O3 $(ARCH) -c -o $@ $<

.PHONY: all

# DSP regression test, on the host (x86): tones through the Engine at every DDC rate
test: $(BUILD)/../vfo_test_host $(BUILD)/../vfo_modes_host $(BUILD)/../codes_test_host $(BUILD)/../timing_test_host $(BUILD)/../wfenc_test_host $(BUILD)/../zoomcap_test_host
	$(BUILD)/../wfenc_test_host
	$(BUILD)/../zoomcap_test_host
	$(BUILD)/../timing_test_host
	$(BUILD)/../vfo_modes_host
	$(BUILD)/../codes_test_host
	$(BUILD)/../vfo_test_host
$(BUILD)/../vfo_test_host: test/vfo_test.cpp src/dsp.hpp ../third_party/pffft/pffft.c
	g++ -O2 -std=c++17 -Wall -fno-math-errno -fcx-limited-range -Isrc -I../third_party/pffft -o $@ test/vfo_test.cpp ../third_party/pffft/pffft.c
.PHONY: test

# DSP cost on the Beagle (static ARM): make -C kweb bench, copy build/arm/kweb/engine_bench over, run it there
# (zoomcap_bench: the zoom captures' time per frame at each size, to refit Server::zoom_frame_ms)
bench: $(BUILD)/engine_bench $(BUILD)/codec_bench $(BUILD)/zoomcap_bench
$(BUILD)/zoomcap_bench: test/zoomcap_test.cpp src/dsp.hpp src/wfenc.hpp $(BUILD)/pffft.o
	$(CROSS)g++ $(CXXFLAGS) -Isrc -static -o $@ test/zoomcap_test.cpp $(BUILD)/pffft.o -lm
$(BUILD)/codec_bench: test/codec_bench.cpp src/codec.hpp $(OPUS)/lib/libopus.a
	$(CROSS)g++ $(CXXFLAGS) -Isrc -static -o $@ test/codec_bench.cpp $(OPUS)/lib/libopus.a -lm
$(BUILD)/engine_bench: test/engine_bench.cpp src/dsp.hpp src/codec.hpp $(BUILD)/pffft.o $(OPUS)/lib/libopus.a
	$(CROSS)g++ $(CXXFLAGS) -Isrc -static -o $@ test/engine_bench.cpp $(BUILD)/pffft.o $(OPUS)/lib/libopus.a -lm
.PHONY: bench

# the DSP tests built for the Beagle (static ARM, the NEON paths the host tests skip): make -C kweb armtest, copy
# build/arm/kweb/{vfo_test,vfo_modes,timing_test} over and run them there
armtest: $(BUILD)/vfo_test $(BUILD)/vfo_modes $(BUILD)/timing_test
$(BUILD)/vfo_test $(BUILD)/vfo_modes $(BUILD)/timing_test: $(BUILD)/%: test/%.cpp src/dsp.hpp $(BUILD)/pffft.o
	$(CROSS)g++ $(CXXFLAGS) -Isrc -static -o $@ $< $(BUILD)/pffft.o -lm
.PHONY: armtest
$(BUILD)/../vfo_modes_host: test/vfo_modes.cpp src/dsp.hpp ../third_party/pffft/pffft.c
	g++ -O2 -std=c++17 -Wall -fno-math-errno -fcx-limited-range -Isrc -I../third_party/pffft -o $@ test/vfo_modes.cpp ../third_party/pffft/pffft.c
$(BUILD)/../timing_test_host: test/timing_test.cpp src/dsp.hpp ../third_party/pffft/pffft.c
	g++ -O2 -std=c++17 -Wall -fno-math-errno -fcx-limited-range -Isrc -I../third_party/pffft -o $@ test/timing_test.cpp ../third_party/pffft/pffft.c
$(BUILD)/../wfenc_test_host: test/wfenc_test.cpp src/wfenc.hpp
	g++ -O2 -std=c++17 -Wall -Wextra -Isrc -o $@ test/wfenc_test.cpp
$(BUILD)/../zoomcap_test_host: test/zoomcap_test.cpp src/dsp.hpp src/wfenc.hpp ../third_party/pffft/pffft.c
	g++ -O2 -std=c++17 -Wall -Wextra -fno-math-errno -fcx-limited-range -Isrc -I../third_party/pffft -o $@ test/zoomcap_test.cpp ../third_party/pffft/pffft.c
$(BUILD)/../codes_test_host: test/codes_test.cpp src/dsp.hpp ../third_party/pffft/pffft.c
	g++ -O2 -std=c++17 -Wall -fno-math-errno -fcx-limited-range -Isrc -I../third_party/pffft -o $@ test/codes_test.cpp ../third_party/pffft/pffft.c
