/* ko_source.h: the HPSDR bridge's receivers from a KiwiSDR running kiwi-offload (kbridge).
 * GPLv3, as the rest of this directory. */
#ifndef KO_SOURCE_H
#define KO_SOURCE_H
#include "ka9q_hpsdr.h"

/* Starts the source thread: it connects to kbridge (mcb->kiwi_host, kiwi_port) and keeps a slice per enabled
 * receiver, at the client's frequency and rate, for as long as the bridge runs (reconnecting if kbridge goes away).
 * false: the thread could not be started. */
bool ko_source_start(struct main_cb *mcb);

#endif
