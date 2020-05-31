/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    LoRa concentrator : Timer synchronization
        Provides synchronization between unix, concentrator and gps clocks

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdio.h>        /* printf, fprintf, snprintf, fopen, fputs */
#include <stdint.h>        /* C99 types */
#include <pthread.h>

#include "trace.h"
#include "timersync.h"
#include "libloragw/loragw_hal.h"
#include "libloragw/loragw_reg.h"
#include "libloragw/loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define timersub(a, b, result)                                                \
  do {                                                                        \
    (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;                             \
    (result)->tv_usec = (a)->tv_usec - (b)->tv_usec;                          \
    if ((result)->tv_usec < 0) {                                              \
      --(result)->tv_sec;                                                     \
      (result)->tv_usec += 1000000;                                           \
    }                                                                         \
  } while (0)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

static pthread_mutex_t mx_timersync = PTHREAD_MUTEX_INITIALIZER; /* control access to timer sync offsets */
// static struct timeval offset_unix_concent = {0,0}; /* timer offset between unix host and concentrator */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE SHARED VARIABLES (GLOBAL) ------------------------------------ */
extern bool exit_sig;
extern bool quit_sig;
extern pthread_mutex_t mx_concent;
//extern lgw_context * ctx_tx;
extern lgw_context * g_ctx_arr[];
/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int get_concentrator_time(struct timeval *concent_time, struct timeval unix_time, lgw_context * ctx) {
    struct timeval local_timeval;

    if (concent_time == NULL) {
        MSG(LOG_INFO,"ERROR: %s invalid parameter\n", __FUNCTION__);
        return -1;
    }

    pthread_mutex_lock(&mx_timersync); /* protect global variable access */
    timersub(&unix_time, &(ctx->offset_unix_concent), &local_timeval);
    pthread_mutex_unlock(&mx_timersync);

    /* TODO: handle sx1301 coutner wrap-up !! */
    concent_time->tv_sec = local_timeval.tv_sec;
    concent_time->tv_usec = local_timeval.tv_usec;

    MSG_DEBUG(DEBUG_TIMERSYNC, " --> TIME: unix current time is   %ld,%ld\n", unix_time.tv_sec, unix_time.tv_usec);
    MSG_DEBUG(DEBUG_TIMERSYNC, "           offset is              %ld,%ld\n", ctx->offset_unix_concent.tv_sec, ctx->offset_unix_concent.tv_usec);
    MSG_DEBUG(DEBUG_TIMERSYNC, "           sx1301 current time is %ld,%ld\n", local_timeval.tv_sec, local_timeval.tv_usec);

    return 0;
}

/* ---------------------------------------------------------------------------------------------- */
/* --- THREAD 6: REGULARLAY MONITOR THE OFFSET BETWEEN UNIX CLOCK AND CONCENTRATOR CLOCK -------- */
extern lgw_context * g_ctx_arr[];
extern pthread_mutex_t mx_concent_sx1276;
extern lgw_context_sx1276 * g_ctx_sx1276_arr[];

void thread_timersync(void) {
    struct timeval unix_timeval;
    struct timeval concentrator_timeval;
    uint32_t sx1276_0_timecount = 0;
    uint32_t sx1276_timecount = 0;
    struct timeval offset_previous = {0,0};
    struct timeval offset_drift = {0,0}; /* delta between current and previous offset */
    int i;
    uint32_t uart;
    
    while (!exit_sig && !quit_sig) {
        for( i = 0 ;i < SUPPORT_SX1276_MAX; i++ ){
            if( NULL == g_ctx_sx1276_arr[i] )
                break;

            uart = g_ctr_sx1276_arr[i]->uart;

            /* Get current unix time */
            gettimeofday(&unix_timeval, NULL);

            /* Get current concentrator counter value (1MHz), tow 16bits timer make one 32bits timer */
            pthread_mutex_lock(&mx_concent_sx1276);
            if( i != 0 ){
                sx1276_0_timecount = lgw_uart_read_timer(g_ctx_sx1276_arr[0]->uart);
            }
            sx1301_timecount = lgw_uart_read_timer(uart);
            pthread_mutex_unlock(&mx_concent_sx1276);

            if (0 != i) {
                g_ctx_sx1276_arr[i]->offset_count_us = sx1276_0_timecount - sx1276_timecount;
            } else {
                g_ctx_sx1276_arr[i]->offset_count_us = 0;
            }
            
            concentrator_timeval.tv_sec = sx1276_timecount / 1000000UL;
            concentrator_timeval.tv_usec = sx1276_timecount - (concentrator_timeval.tv_sec * 1000000UL);

            /* Compute offset between unix and concentrator timers, with microsecond precision */
            offset_previous.tv_sec = g_ctx_sx1276_arr[i]->offset_unix_concent.tv_sec;
            offset_previous.tv_usec = g_ctx_sx1276_arr[i]->offset_unix_concent.tv_usec;

            /* TODO: handle sx1276 coutner wrap-up */
            pthread_mutex_lock(&mx_timersync); /* protect global variable access */
            timersub(&unix_timeval, &concentrator_timeval, &(g_ctx_sx1276_arr[i]->offset_unix_concent));
            pthread_mutex_unlock(&mx_timersync);

            timersub(&(g_ctx_sx1276_arr[i]->offset_unix_concent), &offset_previous, &offset_drift);

            MSG_DEBUG(DEBUG_TIMERSYNC, "  sx1276    = %u (µs) - timeval (%ld,%ld)\n",
                sx1276_timecount,
                concentrator_timeval.tv_sec,
                concentrator_timeval.tv_usec);
            MSG_DEBUG(DEBUG_TIMERSYNC, "  unix_timeval = %ld,%ld\n", unix_timeval.tv_sec, unix_timeval.tv_usec);

            MSG(LOG_INFO,"INFO: host/sx1276 time offset=(%lds:%ldµs) - drift=%ldµs\n",
                g_ctx_sx1276_arr[i]->offset_unix_concent.tv_sec,
                g_ctx_sx1276_arr[i]->offset_unix_concent.tv_usec,
                offset_drift.tv_sec * 1000000UL + offset_drift.tv_usec);
        }

        /* delay next sync */
        /* If we consider a crystal oscillator precision of about 20ppm worst case, and a clock
            running at 1MHz, this would mean 1µs drift every 50000µs (10000000/20).
            As here the time precision is not critical, we should be able to cope with at least 1ms drift,
            which should occur after 50s (50000µs * 1000).
            Let's set the thread sleep to 1 minute for now */
        wait_ms(60000);
    }
}
