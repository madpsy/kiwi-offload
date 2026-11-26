/* Copyright (C)
*   11/2025 - Rick Koch, N1GP
*   Wrote ka9q_hpsdr with the help of various open sources on the internet.
*     Christoph v. Wüllen, https://github.com/dl1ycf/pihpsdr
*     John Melton, https://github.com/g0orx/linhpsdr
*     Phil Karn, https://github.com/ka9q/ka9q-radio
*
*   It uses HPSDR Protocol-2 defined here:
*     https://github.com/TAPR/OpenHPSDR-Firmware/blob/master/Protocol%202/Documentation/openHPSDR%20Ethernet%20Protocol%20v4.3.pdf
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

/*
 * This program simulates an HPSDR Hermes board with up to 10 DDC receiver
 * slices, bridging HPSDR Protocol-2 and Protocol-1 clients (Thetis, piHPSDR,
 * SparkSDR etc) to a KiwiSDR running kiwi-offload: each receiver is a slice of
 * the Kiwi's own DDCs, streamed by kbridge (ko_source.c).
 *
 * From UberSDR's HPSDR bridge (ka9q_ubersdr clients/hpsdr), which took Rick
 * Koch's ka9q-radio bridge to UberSDR's WebSocket API. Here the WebSocket
 * client is replaced by ko_source.c; the protocol side is UberSDR's:
 *   - both protocols on UDP 1024, one client at a time
 *   - Client disconnect watchdog: if nothing is received from the client for
 *     10 seconds, streaming is stopped and DDC state is cleared
 */

#include "ka9q_hpsdr.h"
#include "hpsdr_p1.h"
#include "ko_source.h"

static int do_exit = 0;
struct main_cb mcb;
static int sock_udp;
static int hp_sock;
static int ddcspec_sock = -1;   /* bound to port 1025; also the source for HP status sends */
static int interface_offset = 0;



static int running = 0;
static bool gen_rcvd = false;
static struct timespec last_client_activity = {0};  // updated by any UDP receive from client
static bool wbenable = false;
static int wide_len;
static int wide_size;
static int wide_rate;
static int wide_ppf;

static struct sockaddr_in addr_new;

// protocol2 stuff
static int bits = -1;
static long rxfreq[MAX_RCVRS] = {0,};
static int ddcenable[MAX_RCVRS] = {0,};
static int rxrate[MAX_RCVRS] = {0,};
static int ddc_port = 1025;
static int mic_port = 1026;
static int hp_port = 1027; // also wb_port
static int ddc0_port = 1035;

static pthread_t highprio_thread_id = 0;
static pthread_t ddc_specific_thread_id = 0;
static pthread_t mic_thread_id = 0;
static pthread_t wb_thread_id = 0;
static pthread_t hpstat_thread_id = 0;
static pthread_t sink_thread_id = 0;
static pthread_t throughput_thread_id = 0;
static pthread_t rx_thread_id[MAX_RCVRS] = {0,};
static void   *highprio_thread(void*);
static void   *ddc_specific_thread(void*);
static void   *mic_thread(void *);
static void   *wb_thread(void *);
static void   *hpstat_thread(void *);
static void   *sink_thread(void *);
static void   *rx_thread(void *);

/* How far the receiver tunes: the Kiwi's 0-30 MHz (its front end is low-passed at 30 MHz; kbridge
 * tunes to 33.3 MHz). Reported, not enforced: HPSDR has no field to tell a client its range. */
long rx_min_freq = 10000;
long rx_max_freq = 30000000;

/* ------------------------------------------------------------------------ */
/* The receiver plumbing protocol 1 drives.                                   */
/*                                                                            */
/* Declared in hpsdr_p1.h as an interface so the coupling runs one way: that   */
/* layer asks the bridge for things, and the bridge knows nothing about        */
/* protocol 1 beyond handing it datagrams.                                     */
/* ------------------------------------------------------------------------ */

static unsigned char p1_mac[6] = {0};

/*
 * Sample level: kbridge's samples are at the DDC FIR's 16-bit scale (+-1.0 here
 * after /32768) and go into the 24-bit field at its full scale, as a Hermes puts
 * its ADC's full scale there: a client's dBFS is the Kiwi's. (UberSDR's bridge
 * used a per-rate display gain instead: its full scale landed near -60 dBFS, and
 * this stream's weakest steps fell below one 24-bit step.)
 */
#define IQ_FULL_SCALE 8388607.0f

const unsigned char *p1_host_mac(void) { return p1_mac; }
int p1_host_board_type(void) { return mcb.device_type; }

static void put_be32(unsigned char *p, unsigned long v)
{
    p[0] = (unsigned char)(v >> 24);
    p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);
    p[3] = (unsigned char)v;
}

void p1_host_tag(unsigned char *reply)
{
    memcpy(&reply[44], "KIWISDR", 8);  /* with its NUL */
    put_be32(&reply[52], (unsigned long)rx_min_freq);
    put_be32(&reply[56], (unsigned long)rx_max_freq);
}

/* Is a protocol 2 client streaming? Both protocols drive the same receivers, so
 * the second one to arrive has to be turned away rather than allowed to
 * reconfigure the first one's session out from under it. */
bool p1_host_p2_busy(void) { return running && gen_rcvd; }

/* A rate change is applied the same way the protocol 2 path applies one: set the
 * rate (the kbridge source resizes the receiver's slice). The early return keeps a
 * no-op from resizing it for nothing.
 *
 * The display gain is deliberately NOT set here. It is a function of the
 * bandwidth and is derived where the samples are scaled, because a stored copy
 * has to be kept in step with the rate and once was not: the receivers are
 * seeded at 192 kHz, so a client selecting 192 kHz — the commonest case, and
 * what p1_start assumes when the client has not said — took this early return
 * and kept the placeholder gain of 700 rather than that rate's 4000. Fifteen
 * decibels quiet, at 192 kHz alone, since every other rate differs from the
 * seed and got past the return. */
void p1_host_set_rate(int rcvr, int rate_hz)
{
    if (rcvr < 0 || rcvr >= MAX_RCVRS) return;
    if (mcb.rcb[rcvr].output_rate == rate_hz) return;

    mcb.rcb[rcvr].output_rate = rate_hz;
    rxrate[rcvr] = rate_hz / 1000;
    mcb.rcb[rcvr].reconnect_needed = 1;
}

void p1_host_set_freq(int rcvr, long hz)
{
    if (rcvr < 0 || rcvr >= MAX_RCVRS) return;
    if (hz != 0 && (hz < rx_min_freq || hz > rx_max_freq)) {
        t_print("P1: WARNING tuned to %.3f MHz, outside the receiver's "
                "%.3f kHz - %.3f MHz\n", hz / 1e6,
                rx_min_freq / 1000.0, rx_max_freq / 1000000.0);
    }
    rxfreq[rcvr] = hz;
    mcb.rcb[rcvr].new_freq = hz;
    t_print("P1: DDC%d tuned to %ld Hz\n", rcvr, hz);
}

void p1_host_enable(int rcvr, bool on)
{
    if (rcvr < 0 || rcvr >= MAX_RCVRS) return;
    ddcenable[rcvr] = on ? 1 : 0;
    mcb.rcb[rcvr].rcvr_mask = on ? (1 << rcvr) : 0;
    /* `running` gates the receive threads and the activity watchdog, exactly as
     * the protocol 2 high-priority packet sets it. */
    running = on ? 1 : 0;
    if (!on) {
        rxrate[rcvr] = 0;
        rxfreq[rcvr] = 0;
    }
}

static const char *client_name(struct in_addr a)   /* a client on this machine (loopback): "local" */
{
    return (ntohl(a.s_addr) >> 24) == 127 ? "local" : inet_ntoa(a);
}
int hpsdr_client(char *buf, int n)
{
    struct sockaddr_in a;
    if (p1_client_addr(&a)) { snprintf(buf, n, "%s (protocol 1)", client_name(a.sin_addr)); return 1; }
    if (running && addr_new.sin_family == AF_INET) { snprintf(buf, n, "%s (protocol 2)", client_name(addr_new.sin_addr)); return 1; }
    buf[0] = 0; return 0;
}

void p1_host_stop_all(void)
{
    for (int i = 0; i < MAX_RCVRS; i++) {
        ddcenable[i] = 0;
        mcb.rcb[i].rcvr_mask = 0;
        rxrate[i] = 0;
        rxfreq[i] = 0;
    }
    running = 0;
}

void sdr_sighandler (int signum)
{
    t_print ("Signal:%d caught, exiting!\n", signum);
    do_exit = 1;
    running = 0;
}

int find_net(char *find)
{
    DIR* dir;
    struct dirent* ent;

    if (!(dir = opendir("/sys/class/net"))) {
        perror("can't open /sys/class/net");
        return 0;
    }

    while((ent = readdir(dir)) != NULL) {
        if (!strcmp("print", find) && strcmp(".", ent->d_name) && strcmp("..", ent->d_name)) {
            printf("%s ", ent->d_name);
        } else if (!strcmp(ent->d_name, find)) {
            closedir(dir);
            return 1;
        }
    }
    closedir(dir);
    return 0;
}

int proc_find(char name[][16], char *find)
{
    DIR* dir;
    struct dirent* ent;
    char* endptr;
    int i, name_found = 0;
    char buf[512];

    if (!(dir = opendir("/proc"))) {
        perror("can't open /proc");
        return 0;
    }

    while((ent = readdir(dir)) != NULL) {
        long lpid = strtol(ent->d_name, &endptr, 10);
        if (*endptr != '\0') {
            continue;
        }

        snprintf(buf, sizeof(buf), "/proc/%ld/cmdline", lpid);
        FILE* fp = fopen(buf, "r");

        if (fp) {
            if (fgets(buf, sizeof(buf), fp) != NULL) {
                // check the first token in the file, the program name
                char* first = strtok(buf, "\0");
                if (strstr(first, find) != NULL) {
                    for (i = 0; i < sizeof(buf); i++) {
                        if (!strcmp(&buf[i], "-i")) {
                            strcpy(name[name_found], &buf[i+3]);
                            if (++name_found > MAX_PRGMS)
                                goto finishup;
                            break;
                        }
                    }
                }
            }
            fclose(fp);
        }
    }

finishup:
    closedir(dir);
    return name_found;
}

/* How often the throughput line goes out, in seconds. */
#define THROUGHPUT_INTERVAL 5.0

/*
 * Reports what the IQ stream is actually costing, every five seconds while a
 * client is streaming.
 *
 * The slice's stream as kbridge sends it (64 bytes per group of 15-31 samples,
 * by the mantissa width asked for), before resampling to the client's rate.
 *
 * Nothing is printed while no client is connected. The counters are still
 * drained on every pass, so the first line after a client arrives covers its
 * own five seconds rather than everything the bridge ever received.
 */
static void *throughput_thread(void *arg)
{
    (void)arg;
    struct timespec last;
    clock_gettime(CLOCK_MONOTONIC, &last);
    bool was_running = false;

    while (!do_exit) {
        usleep(250000);

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        /* A client arriving restarts the interval rather than joining whatever
         * is left of the one in progress. Otherwise the first line after a
         * client connects divides its traffic by five seconds most of which it
         * was not there for, and reports a rate it never ran at. */
        const bool now_running = (running != 0);
        if (now_running != was_running) {
            was_running = now_running;
            for (int i = 0; i < mcb.num_rxs; i++)
                __atomic_store_n(&mcb.rcb[i].in_bytes, 0ULL, __ATOMIC_RELAXED);
            last = now;
            continue;
        }

        const double elapsed = (now.tv_sec - last.tv_sec) +
                               (now.tv_nsec - last.tv_nsec) * 1e-9;
        if (elapsed < THROUGHPUT_INTERVAL) continue;
        last = now;

        char line[256];
        int off = 0, active = 0;
        double total = 0.0;

        for (int i = 0; i < mcb.num_rxs; i++) {
            /* Read and cleared in one step, so the next interval starts from
             * zero and no traffic is counted twice or lost between the two. */
            const unsigned long long bytes =
                __atomic_exchange_n(&mcb.rcb[i].in_bytes, 0ULL, __ATOMIC_RELAXED);
            if (!running || bytes == 0) continue;

            const double kbps = (double)bytes * 8.0 / 1000.0 / elapsed;
            total += kbps;
            active++;
            if (off >= 0 && off < (int)sizeof(line)) {
                off += snprintf(line + off, sizeof(line) - (size_t)off,
                                "%sDDC%d %.1f kbps", off ? "  " : " ", i, kbps);
            }
        }

        if (active == 0) continue;
        if (off < 0 || off >= (int)sizeof(line)) continue;

        if (active > 1) {
            t_print("IQ:%s  total %.1f kbps\n", line, total);
        } else {
            t_print("IQ:%s\n", line);
        }
    }
    return NULL;
}

int main (int argc, char *argv[])
{
    struct sockaddr_in addr_udp;
    struct sockaddr_in addr_from;
    socklen_t lenaddr;
    struct timeval tv;
    int yes = 1;
    int bytes_read;
    uint32_t i, code;
    u_char buffer[MAX_BUFFER_LEN];
    uint32_t *code0;
    int CmdOption;
    struct sigaction sigact;


    code0 = (uint32_t *) buffer;
    memset(&mcb, 0, sizeof(mcb));
    // set defaults
    mcb.num_rxs = MAX_RCVRS;
    mcb.wideband = true;                /* the bandscope, on a client's request (--no-wideband: never) */
    mcb.device_type = HERMES_LITE;
    mcb.kiwi_port = 9073;
    mcb.bits = 12;
    mcb.gain = -1;

    static struct option long_options[] = {
        {"kiwi",       required_argument, 0, 'k'},
        {"bits",       required_argument, 0, 'b'},
        {"gain",       required_argument, 0, 'g'},
        {"keep-slices", no_argument,      0, 'K'},
        {"interface",  required_argument, 0, 'i'},
        {"receivers",  required_argument, 0, 'n'},
        {"device",     required_argument, 0, 'd'},
        {"wideband",   no_argument,       0, 'w'},
        {"no-wideband", no_argument,      0, 'W'},
        {"debug",      no_argument,       0, 'v'},
        {"help",       no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt_index = 0;
    while((CmdOption = getopt_long(argc, argv, "k:b:g:Ki:n:d:wWvh", long_options, &opt_index)) != -1) {
        switch(CmdOption) {
        case 'h':
            printf("Usage: %s --kiwi HOST[:PORT] [options]\n\n", basename(argv[0]));
            printf("Kiwi (kiwi-offload) Options:\n");
            printf("  --kiwi HOST[:PORT] the Kiwi running kbridge (port default 9073). The bridge is\n");
            printf("                     kbridge's host client: one at a time (stop other host tools)\n");
            printf("  --bits N           mantissa bits per sample: 8, 10, 12, 14 or 16 (default 12:\n");
            printf("                     block floating point, each group of samples has its own\n");
            printf("                     exponent). More bits, more of the Kiwi's link per receiver\n");
            printf("  --gain N           DDC output gain step 0..8 (default: kbridge's)\n");
            printf("  --keep-slices      leave the host slices kbridge already has (default: delete\n");
            printf("                     them at connect, so the link and DDCs are the receivers')\n");
            printf("\n");
            printf("HPSDR Emulation Options:\n");
            printf("  --interface IFACE  Network interface to bind to (auto-detected if omitted)\n");
            printf("  --receivers N      Number of receiver slices (default %d, max %d)\n", MAX_RCVRS, MAX_RCVRS);
            printf("  --device N         Device type: 1=Hermes, 6=HermesLite (default 6)\n");
            printf("  --no-wideband      No bandscope (default: the Kiwi's raw ADC snapshots, 16384 samples\n");
            printf("                     10 times a second, while a protocol 2 client asks for wideband data)\n");
            printf("  --debug            Log per-DDC frequency requests from the client\n");
            printf("\n");
            printf("Examples:\n");
            printf("  %s --kiwi 192.168.1.50\n", basename(argv[0]));
            printf("  %s --kiwi 192.168.1.50 --bits 14 --receivers 4 --interface eth0\n", basename(argv[0]));
            return EXIT_SUCCESS;
            break;

        case 'i':
            strcpy(mcb.interface, optarg);
            break;
        case 'n':
            mcb.num_rxs = atoi(optarg);
            break;
        case 'k': {
            strncpy(mcb.kiwi_host, optarg, sizeof(mcb.kiwi_host) - 1);
            char *c = strrchr(mcb.kiwi_host, ':');
            if (c) { *c = '\0'; mcb.kiwi_port = atoi(c + 1); }
            break;
        }
        case 'b':
            mcb.bits = atoi(optarg);
            if (mcb.bits != 8 && mcb.bits != 10 && mcb.bits != 12 && mcb.bits != 14 && mcb.bits != 16) {
                fprintf(stderr, "--bits must be 8, 10, 12, 14 or 16\n");
                return EXIT_FAILURE;
            }
            break;
        case 'g':
            mcb.gain = atoi(optarg);
            if (mcb.gain < 0 || mcb.gain > 8) { fprintf(stderr, "--gain must be 0..8\n"); return EXIT_FAILURE; }
            break;
        case 'K':
            mcb.keep_slices = 1;
            break;
        case 'd':
            mcb.device_type = atoi(optarg);
            break;
        case 'W':
            mcb.wideband = 0;
            break;
        case 'w':
            mcb.wideband = 1;
            break;
        case 'v':
            mcb.debug = 1;
            break;
        }
    }

    if (!mcb.kiwi_host[0]) {
        fprintf(stderr, "--kiwi HOST is needed: the Kiwi running kbridge (--help)\n");
        return EXIT_FAILURE;
    }
    if (mcb.num_rxs < 1 || mcb.num_rxs > MAX_RCVRS) {
        fprintf(stderr, "--receivers must be 1..%d\n", MAX_RCVRS);
        return EXIT_FAILURE;
    }
    printf("\n");

    int same_int = 0, prgms_found = 0;
    char myproc[MAX_PRGMS][16] = {0,};
    prgms_found = proc_find(myproc, "kiwi-hpsdr-bridge");
    if (prgms_found > MAX_PRGMS) {
        printf("These are already max: %d kiwi-hpsdr-bridge programs running.\n", MAX_PRGMS);
        return EXIT_FAILURE;
    }

    if (strlen(mcb.interface) == 0) {
        /* Auto-detect default route interface */
        FILE *pfd = popen("ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<NF;i++) if($i==\"dev\") print $(i+1)}'", "r");
        if (pfd != NULL) {
            if (fgets(mcb.interface, sizeof(mcb.interface) - 1, pfd) != NULL) {
                /* strip trailing newline */
                mcb.interface[strcspn(mcb.interface, "\n")] = '\0';
            }
            pclose(pfd);
        }
        if (strlen(mcb.interface) == 0) {
            printf("Could not auto-detect network interface. Use --interface to specify one.\n");
            printf("Available interfaces:\n\t");
            find_net("print");
            printf("\n");
            return EXIT_FAILURE;
        }
        printf("Auto-detected network interface: %s\n", mcb.interface);
    }

    if (find_net(mcb.interface) == 0) {
        printf("%s not found\n", mcb.interface);
        return EXIT_FAILURE;
    }

    // see how many different net interfaces these prgm's are
    // using and check before using the same one
    for (i = 0; i < prgms_found; i++) {
        if (!strcmp(myproc[i], mcb.interface))
            same_int++;
    }

    if (same_int > 1) {
        printf("interface %s already in use\n", mcb.interface);
        return EXIT_FAILURE;
    }

    if ((sock_udp = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        t_perror("socket");
        return EXIT_FAILURE;
    }

    if (prgms_found > 1) {
        interface_offset++;
        mcb.wideband = 0;
        if (setsockopt(sock_udp, SOL_SOCKET, SO_BINDTODEVICE,
                       mcb.interface, sizeof(mcb.interface)) < 0) {
            perror ("SO_BINDTODEVICE");
        }
    }

    struct ifreq hwaddr;
    memset(&hwaddr, 0, sizeof(hwaddr));
    strncpy(hwaddr.ifr_name, mcb.interface, IFNAMSIZ - 1);
    ioctl(sock_udp, SIOCGIFHWADDR, &hwaddr);
    /* Both protocols' discovery replies carry it, and protocol 1's builder is
     * in another file, so it is kept where either can reach it. */
    memcpy(p1_mac, hwaddr.ifr_addr.sa_data, 6);

    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;

    // get the IP address of the desired interface
    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family==AF_INET) {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            addr = inet_ntoa(sa->sin_addr);
            if (!strcmp(mcb.interface, ifa->ifa_name)) {
                strcpy(mcb.ip, addr);
            }
        }
    }
    freeifaddrs(ifap);

    setsockopt(sock_udp, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock_udp, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    setsockopt(sock_udp, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    memset(&addr_udp, 0, sizeof(addr_udp));
    addr_udp.sin_family = AF_INET;
    addr_udp.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_udp.sin_port = htons(1024);

    if (bind(sock_udp, (struct sockaddr *)&addr_udp, sizeof(addr_udp)) < 0) {
        t_perror("main ERROR: bind");
        return EXIT_FAILURE;
    }

    if (pthread_create(&highprio_thread_id, NULL, highprio_thread, NULL) < 0) {
        t_perror("***** ERROR: Create HighPrio thread");
    }

    if (pthread_create(&ddc_specific_thread_id, NULL, ddc_specific_thread, NULL) < 0) {
        t_perror("***** ERROR: Create DDC specific thread");
    }

    if (pthread_create(&mic_thread_id, NULL, mic_thread, NULL) < 0) {
        t_perror("***** ERROR: Create MIC thread");
    }

    if (pthread_create(&hpstat_thread_id, NULL, hpstat_thread, NULL) < 0) {
        t_perror("***** ERROR: Create HP status thread");
    }

    if (pthread_create(&sink_thread_id, NULL, sink_thread, NULL) < 0) {
        t_perror("***** ERROR: Create sink thread");
    }

    if (pthread_create(&throughput_thread_id, NULL, throughput_thread, NULL) < 0) {
        t_perror("***** ERROR: Create throughput thread");
    }

    if (mcb.wideband) {
        if (pthread_create(&wb_thread_id, NULL, wb_thread, NULL) < 0) {
            t_perror("***** ERROR: Create WB thread");
        }
    }

    sigact.sa_handler = sdr_sighandler;
    sigemptyset (&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigaction (SIGINT, &sigact, NULL);
    sigaction (SIGTERM, &sigact, NULL);
    sigaction (SIGQUIT, &sigact, NULL);
    sigaction (SIGPIPE, &sigact, NULL);


    for (i = 0; i < mcb.num_rxs; i++) {
        mcb.rcb[i].mcb = &mcb;
        mcb.rcb[i].new_freq = 0;
        mcb.rcb[i].curr_freq = 10000000;
        mcb.rcb[i].output_rate = 192000;
        mcb.rcb[i].rcvr_num = i;
        mcb.rcb[i].p2_sock = -1;
        mcb.rcb[i].p2_seq = 0;
        mcb.rcb[i].reconnect_needed = 0;
        mcb.rcb[i].rcvr_mask = 1 << i;
    }
    if (!ko_source_start(&mcb)) return EXIT_FAILURE;

    t_print("Waiting on Discovery...\n");

    while (!do_exit) {
        lenaddr = sizeof(addr_from);
        bytes_read = recvfrom(sock_udp, buffer, sizeof(buffer), 0, (struct sockaddr *)&addr_from, &lenaddr);

        if (bytes_read < 0 && errno != EAGAIN) {
            t_perror("recvfrom");
            continue;
        }

        if (bytes_read <= 0) {
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &last_client_activity);
        code = *code0;

        /*
         * Protocol 1 gets first refusal. The two formats cannot be confused —
         * every protocol 1 datagram opens EF FE and every protocol 2 one opens
         * with four zero bytes — so this consumes only what is unmistakably
         * protocol 1 and leaves everything else to the branches below.
         */
        if (p1_handle_datagram(sock_udp, buffer, (int)bytes_read, &addr_from))
            continue;

        /*
         * Here we have to handle the following "non standard" cases:
         * NewProtocol "Discovery" packet   60 bytes starting with 00 00 00 00 02
         * NewProtocol "General"   packet   60 bytes starting with 00 00 00 00 00
         *                                  ==> this starts NewProtocol radio
         */
        if (bytes_read == 60 && code == 0 && buffer[4] == 0x02) {
            /*
             * Always answer discovery — a running radio must reply with
             * status 3 (busy) so re-discovering clients still find it.
             * Never answering while running makes the radio invisible to
             * a client that restarts, until the activity watchdog fires.
             */
            t_print("NewProtocol discovery packet received from %s (status %d)\n",
                    inet_ntoa(addr_from.sin_addr), 2 + (running ? 1 : 0));
            // prepare response
            memset(buffer, 0, 60);
            buffer [4] = 0x02 + (running ? 1 : 0);
            for (i = 0; i < 6; ++i) buffer[i + 5] = hwaddr.ifr_addr.sa_data[i];
            buffer[11] = mcb.device_type;
            buffer[12] = 38;
            /*
             * Firmware version: for HermesLite (device 6) clients classify
             * version < 40 as "Hermes Lite V1" with reduced capabilities;
             * report 62 (a plausible HL2 gateware version) instead.
             */
            buffer[13] = (mcb.device_type == HERMES_LITE) ? 62 : HERMES_FW_VER;
            buffer[20] = mcb.num_rxs;
            buffer[21] = 1;
            /* Sample rate bitmask, bits 0-3 = 48/96/192/384 kHz. This is the
             * one field that tells the client which rates it may ask for, so a
             * stale 7 here is what made 384 kHz unreachable however wide the
             * receiver was. */
            buffer[22] = 0x0f;
            p1_host_tag(buffer);

            sendto(sock_udp, buffer, 60, 0, (struct sockaddr *)&addr_from, sizeof(addr_from));
            continue;
        }

        if (bytes_read == 60 && buffer[4] == 0x00) {
            /* Protocol 1 has the receivers. Taking the general packet now would
             * point the IQ at this client while the samples still leave as EP6
             * to the other one, so it is refused — and said out loud, because a
             * client that is being ignored deserves to know why. */
            if (p1_active()) {
                static bool said = false;
                if (!said) {
                    said = true;
                    t_print("P2: ignoring a client while a protocol 1 client is streaming\n");
                }
                continue;
            }
            // handle "general packet" of the new protocol
            memset(&addr_new, 0, sizeof(addr_new));
            addr_new.sin_family = AF_INET;
            addr_new.sin_addr.s_addr = addr_from.sin_addr.s_addr;
            addr_new.sin_port = addr_from.sin_port;
            new_protocol_general_packet(buffer);
            continue;
        }
    }

    close(sock_udp);

    return EXIT_SUCCESS;
}

void t_print(const char *format, ...)
{
    va_list(args);
    va_start(args, format);
    struct timespec ts;
    double now;
    static double starttime;
    static int first = 1;
    char line[1024];
    clock_gettime(CLOCK_MONOTONIC, &ts);
    now = ts.tv_sec + 1E-9 * ts.tv_nsec;

    if (first) {
        first = 0;
        starttime = now;
    }

    //
    // After 11 days, the time reaches 999999.999 so we simply wrap around
    //
    if (now - starttime >= 999999.995) {
        starttime += 1000000.0;
    }

    //
    // We have to use vsnt_print to handle the varargs stuff
    // g_print() seems to be thread-safe but call it only ONCE.
    //
    vsnprintf(line, 1024, format, args);
    printf("%10.6f %s", now - starttime, line);
    fflush(stdout);
}

void t_perror(const char *string)
{
    t_print("%s: %s\n", string, strerror(errno));
}

/*
 * Send one protocol 2 IQ packet: the next 238 samples of this receiver, from
 * its DDC's own port, which is how the client tells the receivers apart.
 *
 * Called from the kbridge source thread as the samples are decoded, and
 * sent from there. It used to hand each packet to the DDC's rx_thread through
 * a pair of condition variables that every receiver shared: each packet woke
 * all ten send threads so that one could claim it, and woke them again to hand
 * the buffer back. With ten receivers that handshake, not the sending, was
 * most of the bridge's CPU. The rx_thread now only owns the socket.
 *
 * Sent only while a protocol 2 stream is running, the DDC is enabled with a
 * rate and a frequency, and the client's address is known -- the conditions
 * the send thread waited on. While it is not, the sequence restarts at zero,
 * as it did there.
 */
void load_packet (struct rcvr_cb *rcb)
{
    const int k = rcb->rcvr_num;
    const int sock = __atomic_load_n(&rcb->p2_sock, __ATOMIC_ACQUIRE);
    const struct sockaddr_in to = addr_new;

    if (sock < 0 || !running || !gen_rcvd || ddcenable[k] <= 0 || rxrate[k] == 0
        || rxfreq[k] == 0 || to.sin_port == 0) {
        rcb->p2_seq = 0;
        return;
    }

    /*
     * P2 DDC IQ packet header (16 bytes):
     *   0-3   sequence number (big-endian)
     *   4-11  timestamp (unused, zero)
     *   12-13 bits per sample (16-bit BE = 24)
     *   14-15 samples per frame (16-bit BE = 238)
     */
    unsigned char pkt[1444];
    const uint32_t seq = rcb->p2_seq++;
    pkt[0] = (unsigned char)(seq >> 24);
    pkt[1] = (unsigned char)(seq >> 16);
    pkt[2] = (unsigned char)(seq >> 8);
    pkt[3] = (unsigned char)seq;
    memset(pkt + 4, 0, 8);   /* no time stamps */
    pkt[12] = 0;
    pkt[13] = 24;
    pkt[14] = 0;
    pkt[15] = 238;

    /*
     * P2 wire order is "I" first, then "Q" (3 bytes each, big-endian), but HPSDR
     * client software (the WDSP / piHPSDR family) reads the pair the other way
     * round from the textbook I + jQ: sending the real part first shows the
     * spectrum mirrored in a real client (seen in one, 2026-11-26), as UberSDR's
     * bridge had found. So the imaginary part goes first.
     */
    const float complex *out_buf = &rcb->iqSamples[rcb->iqSample_offset];
    unsigned char *q = pkt + 16;
    for (int i = 0; i < 238; i++, q += 6) {
        int IQData = (int)cimagf(out_buf[i]);
        q[0] = IQData >> 16;
        q[1] = IQData >> 8;
        q[2] = IQData & 0xff;
        IQData = (int)crealf(out_buf[i]);
        q[3] = IQData >> 16;
        q[4] = IQData >> 8;
        q[5] = IQData & 0xff;
    }

    if (sendto(sock, pkt, sizeof(pkt), 0, (const struct sockaddr *)&to, sizeof(to)) < 0) {
        /* Transient send errors must not kill the DDC stream. */
        t_perror("***** ERROR: load_packet sendto");
    }

}

/* ---- for the kbridge source (ko_source.c) ---- */
int bridge_exiting(void) { return do_exit; }

void rcvr_state(int k, int *on, long *freq_hz, int *rate_khz)
{
    *on = running && ddcenable[k] > 0 && rxrate[k] > 0 && rxfreq[k] > 0;
    *freq_hz = rxfreq[k];
    *rate_khz = rxrate[k];
}

/* n new samples of receiver rcb (+-1.0 full scale): into the 24-bit field at its
 * full scale (IQ_FULL_SCALE), sent in HPSDR packets of the running protocol. The stream is the pacing: samples
 * arrive at the receiver's rate and a packet goes out when there are enough. */
void rcvr_deliver(struct rcvr_cb *rcb, const float complex *x, int n)
{
    if (rcb->iqSamples_remaining + n > IQ_RING_SAMPLES) {     /* (cannot happen: delivered in small pieces) */
        rcb->iqSample_offset = rcb->iqSamples_remaining = 0;
        return;
    }
    for (int i = 0; i < n; i++) {
        float re = crealf(x[i]), im = cimagf(x[i]);
        re = re > 1 ? 1 : re < -1 ? -1 : re; im = im > 1 ? 1 : im < -1 ? -1 : im;
        rcb->iqSamples[rcb->iqSample_offset + rcb->iqSamples_remaining + i] = (re + I * im) * IQ_FULL_SCALE;
    }
    rcb->iqSamples_remaining += n;
    const bool p1 = p1_active();
    const int samps_packet = p1 ? P1_SAMPLES_PER_PACKET : 238;
    while (rcb->iqSamples_remaining >= samps_packet) {
        if (p1)
            p1_send_packet(rcb);
        else
            load_packet(rcb);
        rcb->iqSamples_remaining -= samps_packet;
        rcb->iqSample_offset     += samps_packet;
    }
    if (rcb->iqSample_offset > 0) {
        if (rcb->iqSamples_remaining > 0)
            memmove(&rcb->iqSamples[0], &rcb->iqSamples[rcb->iqSample_offset],
                    rcb->iqSamples_remaining * sizeof(float complex));
        rcb->iqSample_offset = 0;
    }
}

void new_protocol_general_packet(unsigned char *buffer)
{
    static unsigned long seqnum = 0;
    unsigned long seqold;
    int rc;

    gen_rcvd = true;

    seqold = seqnum;
    seqnum = ((unsigned long)buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];

    if ((seqnum != 0 && seqnum != seqold + 1 ) && seqold != 0) {
        t_print("GP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
    }

    /*
     * Byte 37, bit 3 (0x08): DDC/DUC frequency fields in the High Priority
     * packet are phase words (freq * 2^32 / 122.88e6) rather than raw Hz.
     * Most clients (Thetis, piHPSDR, deskHPSDR) set it; some send raw Hz.
     */
    rc = buffer[37];
    if (rc != bits) {
        bits = rc;
        t_print("GP: frequency mode byte37=0x%02x (%s)\n", bits,
                (bits & 0x08) ? "phase word" : "raw Hz");
    }

    /*
     * Bytes 5-22 carry the UDP port-override table (0 = default).
     * This bridge only implements the default ports — if a client
     * remaps them, nothing will work, so at least say so.
     */
    {
        static int port_override_warned = 0;
        int nonzero = 0;
        for (int k = 5; k <= 22; k++) {
            if (buffer[k]) { nonzero = 1; break; }
        }
        if (nonzero && !port_override_warned) {
            port_override_warned = 1;
            t_print("GP: WARNING client requests non-default UDP ports "
                    "(general packet bytes 5-22) — not supported, using defaults\n");
        }
    }

    if (!mcb.wideband && (buffer[23] & 1)) {
        static int wb_req_warned = 0;
        if (!wb_req_warned) {
            wb_req_warned = 1;
            t_print("GP: client requested wideband data but the bridge was started "
                    "with --no-wideband (ignored)\n");
        }
    }

    if (mcb.wideband) {
        rc = buffer[23] & 1;
        if (rc != wbenable) {
            wbenable = rc;
            t_print("GP: Wideband Enable Flag is %d\n", wbenable);
        }

        rc = (buffer[24] << 8) + buffer[25];
        if (rc != wide_len) {
            wide_len = rc;
            t_print("GP: WideBand Length is %d\n", rc);
        }

        rc = buffer[26];
        if (rc != wide_size) {
            wide_size = rc;
            t_print("GP: Wideband sample size is %d\n", rc);
        }

        rc = buffer[27];
        if (rc != wide_rate) {
            wide_rate = rc;
            t_print("GP: Wideband sample rate is %d\n", rc);
        }

        rc = buffer[28];
        if (rc != wide_ppf) {
            wide_ppf = rc;
            t_print("GP: Wideband PPF is %d\n", rc);
        }
    }
}

void *highprio_thread(void *data)
{
    struct sockaddr_in addr;
    socklen_t lenaddr = sizeof(addr);
    unsigned long seqnum = 0, seqold;
    unsigned char hp_buffer[2000];
    struct timeval tv;
    int i, rc, yes = 1;
    long freq;
    struct timespec last_hp_time = {0};
    int hp_watchdog_armed = 0;   // armed only after client proves it sends HP packets
    unsigned long hp_pkt_count = 0;
    struct timespec hp_rate_time = {0};

    hp_sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (hp_sock < 0) {
        t_perror("***** ERROR: HP: socket");
        return NULL;
    }

    setsockopt(hp_sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(hp_sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    setsockopt(hp_sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = (interface_offset > 0) ? inet_addr(mcb.ip) : htonl(INADDR_ANY);
    addr.sin_port = htons(hp_port);

    if (bind(hp_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("highprio_thread ERROR: bind");
        close(hp_sock);
        return NULL;
    }

    t_print("Starting highprio_thread()\n");
    while (!do_exit) {
        /* Protocol 1 owns the receivers; this client's high-priority packets
         * would reconfigure them under it. Drained and dropped. */
        if (p1_active()) { usleep(50000); continue; }

        if (!running) seqnum = 0;

        rc = recvfrom(hp_sock, hp_buffer, 1444, 0, (struct sockaddr *)&addr, &lenaddr);

        if (rc < 0 && errno != EAGAIN) {
            t_perror("***** ERROR: HighPrio thread: recvmsg");
            break;
        }

        if (rc < 0) {
            // EAGAIN timeout — check activity-based watchdog
            if (running && last_client_activity.tv_sec != 0) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                double since_any = (now.tv_sec - last_client_activity.tv_sec) +
                                   (now.tv_nsec - last_client_activity.tv_nsec) * 1e-9;
                if (since_any > 10.0) {
                    t_print("HP: no client UDP activity for %.1fs, client disconnected\n", since_any);
                    running = 0;
                    hp_watchdog_armed = 0;
                    hp_pkt_count = 0;
                    last_client_activity.tv_sec = 0;
                    for (i = 0; i < mcb.num_rxs; i++) {
                        ddcenable[i] = 0;
                        mcb.rcb[i].rcvr_mask = 0;
                        rxrate[i] = 0;
                        rxfreq[i] = 0;
                    }
                    /* Wake any thread blocked on the send handshake so it
                     * can observe running == 0 and park itself. */
                }
            }
            continue;
        }

        // Successful receive — reset watchdog timer and count packets
        clock_gettime(CLOCK_MONOTONIC, &last_hp_time);
        last_client_activity = last_hp_time;
        hp_pkt_count++;
        // Arm watchdog once we've seen 100+ packets (client is actively sending HP packets)
        if (!hp_watchdog_armed && hp_pkt_count >= 100) {
            hp_watchdog_armed = 1;
            t_print("HP: watchdog armed after %lu packets\n", hp_pkt_count);
        }
        if (hp_rate_time.tv_sec == 0) {
            hp_rate_time = last_hp_time;
        } else {
            double rate_elapsed = (last_hp_time.tv_sec - hp_rate_time.tv_sec) +
                                  (last_hp_time.tv_nsec - hp_rate_time.tv_nsec) * 1e-9;
            if (rate_elapsed >= 10.0) {
                t_print("HP: %.0f packets/sec from client\n", hp_pkt_count / rate_elapsed);
                hp_pkt_count = 0;
                hp_rate_time = last_hp_time;
            }
        }

        if (rc != 1444) {
            /* A stray/short datagram must not kill the control thread */
            t_print("Received HighPrio packet with incorrect length %d (ignored)\n", rc);
            continue;
        }

        seqold = seqnum;
        seqnum = ((unsigned long)hp_buffer[0] << 24) + (hp_buffer[1] << 16) + (hp_buffer[2] << 8) + hp_buffer[3];

        if ((seqnum != 0 && seqnum != seqold + 1 ) && seqold != 0) {
            t_print("HP: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
        }

        for (i = 0; i < mcb.num_rxs; i++) {
            /* assemble as uint32_t: byte << 24 on a signed int would
             * sign-extend for phase words with bit 31 set */
            uint32_t word = ((uint32_t)hp_buffer[ 9 + 4 * i] << 24)
                          + ((uint32_t)hp_buffer[10 + 4 * i] << 16)
                          + ((uint32_t)hp_buffer[11 + 4 * i] << 8)
                          +  (uint32_t)hp_buffer[12 + 4 * i];
            freq = (long)word;

            if (bits & 0x08) {
                freq = round(122880000.0 * (double) word / 4294967296.0);
            }

            if (freq != rxfreq[i]) {
                /* Reported rather than clamped, and not advertised at all: the
                 * HPSDR protocol has no field for a tuning range — the
                 * discovery reply carries a board type and firmware version and
                 * nothing else — so the client takes its limits from whatever
                 * hardware it thinks it is talking to. Passing the request on
                 * and saying so is the most this end can do. */
                if (freq != 0 && (freq < rx_min_freq || freq > rx_max_freq)) {
                    t_print("RX: WARNING DDC%d tuned to %.3f MHz, outside the receiver's "
                            "%.3f kHz - %.3f MHz\n", i, freq / 1e6,
                            rx_min_freq / 1000.0, rx_max_freq / 1000000.0);
                }
                mcb.rcb[i].new_freq = rxfreq[i] = freq;
                if (mcb.debug)
                    t_print("HP: DDC%d freq: %lu\n", i, freq);
            }
        }

        /* NOTE: bytes 5/6 of the host->radio HP packet are CWX keying
         * bits (CWX/dot/dash), NOT dither/random (those live in the
         * DDC-specific packet).  Byte 1443 is the ADC0 step attenuator.
         * None of these apply to this bridge: no TX and no controllable
         * front-end, so they are ignored. */

        rc = hp_buffer[4] & 0x01;
        if (rc != running) {
            running = rc;
            t_print("HP: Running = %d\n", rc);
            if (!running) {
                hp_watchdog_armed = 0;
                hp_pkt_count = 0;
                for (i = 0; i < mcb.num_rxs; i++) {
                    ddcenable[i] = 0;
                    mcb.rcb[i].rcvr_mask = 0;
                    rxrate[i] = 0;
                    rxfreq[i] = 0;
                }
                /* Wake any thread blocked on the send handshake so it
                 * can observe running == 0 and park itself. */
            } else {
                // running just went 1 — reset packet counter, watchdog arms after 100 packets
                hp_watchdog_armed = 0;
                hp_pkt_count = 0;
                for (i = 0; i < mcb.num_rxs; i++) {
                    if (rx_thread_id[i] == 0) {
                        if (pthread_create(&rx_thread_id[i], NULL, rx_thread, (void *) (uintptr_t) i) < 0) {
                            t_perror("***** ERROR: Create RX thread");
                        }
                    }
                }
            }
        }
    }

    t_print("Ending highprio_thread()\n");
    close(hp_sock);
    return NULL;
}

void *ddc_specific_thread(void *data)
{
    int sock;
    struct sockaddr_in addr;
    socklen_t lenaddr = sizeof(addr);
    unsigned long seqnum, seqold;
    struct timeval tv;
    unsigned char ddc_buffer[2000];
    int yes = 1;
    int rc;
    int i;

    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        t_perror("***** ERROR: ddc_specific_thread: socket");
        return NULL;
    }

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    tv.tv_sec = 0;
    tv.tv_usec = 10000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&tv, sizeof(tv));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = (interface_offset > 0) ? inet_addr(mcb.ip) : htonl(INADDR_ANY);
    addr.sin_port = htons(ddc_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("ddc_specific_thread ERROR: bind");
        close(sock);
        return NULL;
    }

    /* Export the fd: HP status packets must originate from source port
     * 1025, so hpstat_thread() sends on this same socket. */
    ddcspec_sock = sock;

    seqnum = 0;

    t_print("Starting ddc_specific_thread()\n");
    while (!do_exit) {
        /* Protocol 1 owns the receivers; this client's DDC-specific packets
         * would reconfigure them under it. Drained and dropped. */
        if (p1_active()) { usleep(50000); continue; }

        if (!running) {
            seqnum = 0;
            usleep(50000);
            continue;
        }

        rc = recvfrom(sock, ddc_buffer, 1444, 0, (struct sockaddr *)&addr, &lenaddr);
        if (rc < 0 && errno != EAGAIN) {
            t_perror("***** ERROR: DDC specific thread: recvmsg");
            break;
        }

        if (rc < 0) {
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &last_client_activity);

        if (rc != 1444) {
            /* A stray/short datagram must not kill the control thread */
            t_print("RXspec: Received DDC specific packet with incorrect length %d (ignored)\n", rc);
            continue;
        }

        seqold = seqnum;
        seqnum = ((unsigned long)ddc_buffer[0] << 24) + (ddc_buffer[1] << 16) + (ddc_buffer[2] << 8) + ddc_buffer[3];

        if ((seqnum != 0 && seqnum != seqold + 1 ) && seqold != 0) {
            t_print("RXspec: SEQ ERROR, old=%lu new=%lu\n", seqold, seqnum);
        }

        /* Bytes 5/6 carry the per-ADC dither/random enables — ignored,
         * as this bridge has no controllable ADC front-end. */

        /*
         * Bytes 1363+ddc: per-DDC sync bitmap (PureSignal/diversity).
         * Synced DDCs expect their samples interleaved in one stream,
         * which this bridge cannot produce — warn instead of failing
         * silently.
         */
        {
            static int sync_warned = 0;
            for (i = 0; i < mcb.num_rxs && !sync_warned; i++) {
                if (ddc_buffer[1363 + i]) {
                    sync_warned = 1;
                    t_print("RX: WARNING client requests synced DDCs "
                            "(byte %d = 0x%02x) — diversity/PureSignal not supported\n",
                            1363 + i, ddc_buffer[1363 + i]);
                }
            }
        }

        for (i = 0; i < mcb.num_rxs; i++) {
            int modified = 0;
            struct rcvr_cb *rcb = &mcb.rcb[i];

            rc = (ddc_buffer[18 + 6 * i] << 8) + ddc_buffer[19 + 6 * i];
            /* 384 kHz is the widest IQ mode the server offers, and the widest
             * rate the HPSDR protocol defines, so the two meet exactly. Note
             * the server only serves the wide modes to a bypassed session — an
             * unprivileged one is refused at /connection rather than here. */
            if (rc > 384) {
                t_print("RX: WARNING DDC%d requested %d kHz but the bridge's max is 384 kHz — "
                        "expect broken audio/spectrum at this rate\n", i, rc);
                rc = 384;
            }
            if (rc != rxrate[i] && rc != 0) {
                rxrate[i] = rc;
                mcb.rcb[i].output_rate = (rxrate[i] * 1000);
                modified = 1;

                /* the kbridge source resizes the slice */
                rcb->reconnect_needed = 1;
            }

            rc = (ddc_buffer[7 + (i / 8)] >> (i % 8)) & 0x01;
            if (rc != ddcenable[i]) {
                modified = 1;
                ddcenable[i] = rc;
                mcb.rcb[i].rcvr_mask = 1 << i;
            }

            if (modified) {
                t_print("RX: DDC%d Enable=%d Rate=%d\n", i, ddcenable[i], rxrate[i]);
                rc = 0;
            }
        }
    }

    ddcspec_sock = -1;
    close(sock);
    ddc_specific_thread_id = 0;
    t_print("Ending ddc_specific_thread()\n");
    return NULL;
}

/*
 * hpstat_thread — sends the radio->host High Priority status packet.
 *
 * The protocol requires the radio to send a 60-byte status packet from
 * source port 1025 (~every 50 ms during RX): PTT/dot/dash bits, ADC
 * overload, power and AIN readings.  This bridge has no TX and no real
 * ADC telemetry, so all fields are zero — but the stream itself (with
 * correct sequence numbers) must exist: clients use it for meters and
 * some treat its absence as a dead radio.
 *
 * Sends on ddcspec_sock, which is bound to port 1025 by
 * ddc_specific_thread(), so the source port is correct.
 */
void *hpstat_thread(void *data)
{
    unsigned long seqnum = 0;
    unsigned char buffer[60];

    t_print("Starting hpstat_thread()\n");
    while (!do_exit) {
        /* Protocol 1's client-silence watchdog rides this thread's cadence: it
         * is the one that already ticks whether or not a client is connected,
         * and a stopped protocol 1 client would otherwise hold a receiver open
         * on the Kiwi indefinitely. Protocol 1 sends its own
         * telemetry inside EP6, so nothing below this applies to it. */
        p1_check_watchdog();
        if (p1_active()) {
            usleep(50000);
            continue;
        }

        if (!running || !gen_rcvd || addr_new.sin_port == 0 || ddcspec_sock < 0) {
            seqnum = 0;
            usleep(50000);
            continue;
        }

        memset(buffer, 0, sizeof(buffer));
        buffer[0] = (seqnum >> 24) & 0xFF;
        buffer[1] = (seqnum >> 16) & 0xFF;
        buffer[2] = (seqnum >>  8) & 0xFF;
        buffer[3] = (seqnum >>  0) & 0xFF;
        seqnum++;
        /* byte 4: PTT/dot/dash = 0 (RX only); all telemetry fields zero */

        if (sendto(ddcspec_sock, buffer, sizeof(buffer), 0,
                   (struct sockaddr *)&addr_new, sizeof(addr_new)) < 0) {
            t_perror("***** ERROR: hpstat_thread sendto");
        }

        usleep(50000); /* 50 ms cadence (RX) */
    }

    t_print("Ending hpstat_thread()\n");
    return NULL;
}

/*
 * sink_thread — drains the host->radio ports this RX-only bridge does
 * not act on: 1028 (audio) and 1029 (TX IQ).
 *
 * Without listeners, every client audio/TX packet triggers an ICMP
 * port-unreachable, and clients send audio continuously during RX.
 * Draining also counts as client activity for the disconnect watchdog.
 *
 * (Port 1026 — DUC/TX specific — is drained in mic_thread, which owns
 * the socket bound to 1026 for sending mic data.)
 */
void *sink_thread(void *data)
{
    static const int sink_ports[] = { 1028, 1029 };
    enum { NSINK = sizeof(sink_ports) / sizeof(sink_ports[0]) };
    int socks[NSINK];
    unsigned char scratch[2048];
    struct sockaddr_in addr;
    int i, yes = 1;

    for (i = 0; i < NSINK; i++) {
        socks[i] = socket(AF_INET, SOCK_DGRAM, 0);
        if (socks[i] < 0) {
            t_perror("***** ERROR: sink_thread: socket");
            return NULL;
        }
        setsockopt(socks[i], SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
        setsockopt(socks[i], SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = (interface_offset > 0) ? inet_addr(mcb.ip) : htonl(INADDR_ANY);
        addr.sin_port = htons(sink_ports[i]);
        if (bind(socks[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            t_perror("sink_thread ERROR: bind");
            /* non-fatal: keep the other ports */
            close(socks[i]);
            socks[i] = -1;
        }
    }

    t_print("Starting sink_thread() (draining ports 1028/1029)\n");
    while (!do_exit) {
        fd_set fds;
        struct timeval tv;
        int maxfd = -1;

        FD_ZERO(&fds);
        for (i = 0; i < NSINK; i++) {
            if (socks[i] >= 0) {
                FD_SET(socks[i], &fds);
                if (socks[i] > maxfd) maxfd = socks[i];
            }
        }
        if (maxfd < 0) break;

        tv.tv_sec = 0;
        tv.tv_usec = 200000;
        if (select(maxfd + 1, &fds, NULL, NULL, &tv) <= 0) continue;

        for (i = 0; i < NSINK; i++) {
            if (socks[i] >= 0 && FD_ISSET(socks[i], &fds)) {
                while (recv(socks[i], scratch, sizeof(scratch), MSG_DONTWAIT) > 0) {
                    clock_gettime(CLOCK_MONOTONIC, &last_client_activity);
                }
            }
        }
    }

    for (i = 0; i < NSINK; i++) {
        if (socks[i] >= 0) close(socks[i]);
    }
    t_print("Ending sink_thread()\n");
    return NULL;
}

void *rx_thread(void *data)
{
    // One instance of this thread is started for each DDC
    int sock;
    struct sockaddr_in addr;
    int myddc;
    int yes = 1;
    struct rcvr_cb *rcb;

    myddc = (int) (uintptr_t) data;
    rcb = &mcb.rcb[myddc];

    if (myddc < 0 || myddc >= mcb.num_rxs) {
        return NULL;
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        t_perror("***** ERROR: RXthread: socket");
        return NULL;
    }

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = (interface_offset > 0) ? inet_addr(mcb.ip) : htonl(INADDR_ANY);
    addr.sin_port = htons(ddc0_port + myddc);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("rx_thread ERROR: bind");
        close(sock);
        return NULL;
    }

    t_print("Starting rx_thread(%d)\n", myddc);
    /* The socket is all this thread still owns: load_packet() sends on it,
     * from the kbridge source thread, as each packet's samples are
     * decoded. */
    __atomic_store_n(&rcb->p2_sock, sock, __ATOMIC_RELEASE);
    while (!do_exit)
        usleep(50000);
    __atomic_store_n(&rcb->p2_sock, -1, __ATOMIC_RELEASE);

    close(sock);
    t_print("Ending rx_thread(%d)\n", myddc);
    rx_thread_id[myddc] = 0;
    ddcenable[myddc] = 0;
    return NULL;
}

/* The bandscope: the Kiwi's raw ADC snapshots (16384 samples, 10 a second; kbridge forwards them while a client
 * wants them: ko_source.c), each sent as one wideband frame of wide_len-sample packets. HPSDR clients expect 16-bit
 * ADC samples (a Hermes' LTC2208): the LTC2248's 14 bits are scaled by 4 to the same full scale. */
static pthread_mutex_t wb_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wb_cv = PTHREAD_COND_INITIALIZER;
static int16_t wb_snap[16384];
static unsigned wb_gen;
int wb_wanted(void) { return mcb.wideband && !p1_active() && gen_rcvd && running && wbenable; }
void wb_feed(const int16_t *s, int n)
{
    if (n > 16384) n = 16384;
    pthread_mutex_lock(&wb_mu);
    memcpy(wb_snap, s, (size_t) n * sizeof *s);
    if (n < 16384) memset(wb_snap + n, 0, (size_t) (16384 - n) * sizeof *s);
    wb_gen++;
    pthread_cond_signal(&wb_cv);
    pthread_mutex_unlock(&wb_mu);
}

void *wb_thread(void *data)
{
    // NOTE: this thread reuses the hp_sock socket since two sockets
    //       can't send/recv on the same port/address (1027)
    (void) data;
    unsigned char wb_buffer[4 + 2 * 512];
    static int16_t snap[16384];
    unsigned seen = 0;

    t_print("Starting wb_thread()\n");
    while (!do_exit) {
        pthread_mutex_lock(&wb_mu);
        if (wb_gen == seen) {                       /* wait for the next snapshot (100 ms apart) */
            struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts); ts.tv_nsec += 200000000;
            if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
            pthread_cond_timedwait(&wb_cv, &wb_mu, &ts);
        }
        const int fresh = wb_gen != seen;
        if (fresh) { memcpy(snap, wb_snap, sizeof snap); seen = wb_gen; }
        pthread_mutex_unlock(&wb_mu);
        if (!fresh || !wb_wanted()) continue;

        /* the packet length the client asked for (General packet bytes 24-25, samples per packet; default 512) */
        int wlen = wide_len;
        if (wlen < 64 || wlen > 512 || (16384 % wlen) != 0) wlen = 512;
        const int npkts = 16384 / wlen;
        uint32_t seqnum = 0;                        /* per frame */
        for (int i = 0; i < npkts; i++) {
            uint32_t sq = htonl(seqnum++); memcpy(wb_buffer, &sq, 4);
            for (int j = 0; j < wlen; j++) {         /* 16-bit big-endian */
                int v = snap[i * wlen + j] * 4;
                v = v > 32767 ? 32767 : v < -32768 ? -32768 : v;
                wb_buffer[4 + 2 * j] = (unsigned char) ((v >> 8) & 0xff);
                wb_buffer[5 + 2 * j] = (unsigned char) (v & 0xff);
            }
            if (sendto(hp_sock, wb_buffer, (size_t) (4 + 2 * wlen), 0, (struct sockaddr *) &addr_new, sizeof(addr_new)) < 0) {
                t_perror("***** ERROR: WB thread sendto");
                break;
            }
        }
    }

    t_print("Ending wb_thread()\n");
    return NULL;
}

//
// The microphone thread just sends silence, that is
// a "zeroed" mic frame every 1.333 msec and needs to
// be sent for some app's timing purposes.
//
void *mic_thread(void *data)
{
    int sock;
    unsigned long seqnum = 0;
    struct sockaddr_in addr;
    unsigned char mic_buffer[132];
    unsigned char *p;
    int yes = 1;
    struct timespec delay;
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        t_perror("***** ERROR: Mic thread: socket");
        return NULL;
    }

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = (interface_offset > 0) ? inet_addr(mcb.ip) : htonl(INADDR_ANY);
    addr.sin_port = htons(mic_port);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        t_perror("mic_thread ERROR: bind");
        close(sock);
        return NULL;
    }

    memset(mic_buffer, 0, 132);
    clock_gettime(CLOCK_MONOTONIC, &delay);

    t_print("Starting mic_thread()\n");
    while (!do_exit) {
        /* Drain incoming TX-specific packets — the host sends them to
         * port 1026, which this socket owns.  We don't act on them
         * (RX-only bridge) but must not let them pile up unread. */
        {
            unsigned char scratch[2048];
            while (recv(sock, scratch, sizeof(scratch), MSG_DONTWAIT) > 0) {
                clock_gettime(CLOCK_MONOTONIC, &last_client_activity);
            }
        }

        if (!gen_rcvd || !running || addr_new.sin_port == 0) {
            usleep(500000);
            seqnum = 0;
            /* keep the pacing clock current so we don't burst on resume */
            clock_gettime(CLOCK_MONOTONIC, &delay);
            continue;
        }
        // update seq number
        p = mic_buffer;
        *(uint32_t*)p = htonl(seqnum++);
        p += 4;

        // 64 samples with 48000 kHz, makes 1333333 nsec
        delay.tv_nsec += 1333333;

        while (delay.tv_nsec >= 1000000000) {
            delay.tv_nsec -= 1000000000;
            delay.tv_sec++;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &delay, NULL);

        if (sendto(sock, mic_buffer, 132, 0, (struct sockaddr * )&addr_new, sizeof(addr_new)) < 0) {
            /* Transient send errors must not kill the mic stream */
            t_perror("***** ERROR: Mic thread sendto");
        }
    }

    t_print("Ending mic_thread()\n");
    close(sock);
    return NULL;
}
