#ifndef _KA9Q_HPSDR_H
#define _KA9Q_HPSDR_H

/* Copyright (C)
*
*   11/2025 - Rick Koch, N1GP
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*/

#include <sched.h>
#include <unistd.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>
#include <libgen.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <netdb.h>
#include <complex.h>
#include <sys/wait.h>
#include <spawn.h>
#include <dirent.h>
#include <ifaddrs.h>
#include <getopt.h>
#include <ctype.h>


#define HERMES_FW_VER 18
#define MAX_BUFFER_LEN 2048
#define MAX_RCVRS 10
#define MAX_PRGMS 2
#define HERMES_LITE 6

/*
 * IQ accumulation buffer, in complex samples: what the kbridge source hands over
 * at once (one stream group resampled: at most a few dozen samples) plus up to one
 * HPSDR packet (238 samples) of leftover, with room to spare.
 */
#define IQ_RING_SAMPLES 4096

struct main_cb {
    int wideband;
    int debug;
    int num_rxs;
    int device_type;            /* 1=Hermes, 6=HermesLite (default 6) */
    char interface[15];
    char ip[16];
    /* The Kiwi: kbridge's host port (TCP control + UDP data), and what each
     * receiver's slice is asked for. */
    char kiwi_host[256];
    int  kiwi_port;             /* 9073 */
    int  bits;                  /* mantissa width per sample: 8/10/12/14/16 */
    int  gain;                  /* DDC output gain step 0..8, or -1: kbridge's default */
    int  keep_slices;           /* leave the host slices kbridge already has (else deleted) */

    struct rcvr_cb {
        int rcvr_num;
        int new_freq;
        int curr_freq;

        /* Protocol 2: this DDC's socket, bound to port 1035+n by its
         * rx_thread (-1 until then), and the sequence number of its next IQ
         * packet. load_packet() sends on the socket directly from the
         * kbridge source thread (ko_source.c). */
        int p2_sock;
        uint32_t p2_seq;
        int output_rate;
        u_int rcvr_mask;
        int reconnect_needed;   /* set on a rate change (the kbridge source resizes the slice) */
        struct main_cb* mcb;

        /* Stream bytes of this receiver's slice since the throughput reporter
         * last looked (64 per group). Written by the kbridge source thread and
         * read-and-cleared by the reporter, both through the relaxed atomic
         * builtins: a torn 64-bit count would report nonsense. */
        unsigned long long in_bytes;

        int iqSample_offset;
        int iqSamples_remaining;
        float complex iqSamples[IQ_RING_SAMPLES];
    } rcb[MAX_RCVRS];
};

void load_packet(struct rcvr_cb* rcb);
void sdr_sighandler(int signum);
void new_protocol_general_packet(unsigned char *buffer);

/* How far the receiver tunes (reported to the client, not enforced). */
extern long rx_min_freq;
extern long rx_max_freq;

/* For the kbridge source (ko_source.c): what the client wants of receiver k
 * (enabled, frequency in Hz, rate in kHz: the protocol threads' state), and
 * n new samples for it (complex, full scale +-1.0), which go out in HPSDR packets
 * of whichever protocol is running. */
void rcvr_state(int k, int *on, long *freq_hz, int *rate_khz);
/* The wideband (bandscope) stream: does a protocol 2 client want it now, and a raw ADC snapshot for it (16384
 * samples at 66.67 MS/s, 14-bit values sign-extended to int16), from kbridge. */
int  wb_wanted(void);
/* The client streaming now, as "192.168.1.20 (protocol 2)", or "local (protocol 2)" for one on this machine (for
 * kbridge's labels: kweb's admin page shows them); 0 = no client. */
int  hpsdr_client(char *buf, int n);
void wb_feed(const int16_t *s, int n);
void rcvr_deliver(struct rcvr_cb *rcb, const float complex *x, int n);
int bridge_exiting(void);

//
// message printing
//
#include <stdarg.h>
void t_print(const char *format, ...);
void t_perror(const char *string);

#endif // _KA9Q_HPSDR_H
