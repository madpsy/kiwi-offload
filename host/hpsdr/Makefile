PREFIX=/usr/local

CC = gcc
CFLAGS = -std=gnu11 -Wall -O3 -D_GNU_SOURCE=1

LIBS = -lrt -lm -lpthread
LDFLAGS =

BINARY = kiwi-hpsdr-bridge

# The binaries land in build/, named after the architecture (from `uname -m`:
# a native build, for the PC the bridge runs on); objects and their .d files
# beside the sources. .gitignore covers both.
OUT = build
ARCH = $(shell uname -m | sed -e s/x86_64/amd64/ -e s/aarch64/arm64/)
TARGET = $(OUT)/$(BINARY)_$(ARCH)

DEPENDDIR = .
DEPENDFLAGS = -MM

SRCS = ka9q_hpsdr.c ko_source.c hpsdr_p1.c

OBJS = $(SRCS:.c=.o)

CLIENT = kiwi-hpsdr-client
CLIENT_TARGET = $(OUT)/$(CLIENT)_$(ARCH)

# Both binaries are release artefacts, built and published for every
# architecture, so both are what `all` means. The client links no library the
# bridge does not already need, and no sound library at all — it writes its
# audio to a player process — so it costs the build nothing.
#
# `make client` still builds only the client, which is what you want while
# working on it: the bridge is the slow half.
all: $(TARGET) $(CLIENT_TARGET)

client: $(CLIENT_TARGET)

$(CLIENT_TARGET): hpsdr_client.c
	@mkdir -p $(OUT)
	$(CC) $(CFLAGS) hpsdr_client.c -lm -o $(CLIENT_TARGET)

DEPS = $(patsubst %.o,$(DEPENDDIR)/%.d,$(OBJS))
-include $(DEPS)

$(DEPENDDIR)/%.d: %.c $(DEPENDDIR)
	$(CC) $(CFLAGS) $(DEPENDFLAGS) $< >$@

$(TARGET): $(OBJS)
	@mkdir -p $(OUT)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) $(PREFIX)/bin/$(BINARY)

clean:
	rm -f $(DEPS) $(OBJS) $(TARGET) $(CLIENT_TARGET)

.PHONY: all client clean install
