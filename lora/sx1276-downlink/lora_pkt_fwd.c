/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Configure Lora concentrator and forward packets to a server
    Use GPS for packet timestamping.
    Send a becon at a regular interval without server intervention

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Michael Coracin
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#define  _BSD_SOURCE
#define  _GNU_SOURCE


#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>         /* C99 types */
#include <stdbool.h>        /* bool type */
#include <stdio.h>          /* printf, fprintf, snprintf, fopen, fputs */

#include <string.h>         /* memset */
#include <signal.h>         /* sigaction */
#include <time.h>           /* time, clock_gettime, strftime, gmtime */
#include <sys/time.h>       /* timeval */
#include <unistd.h>         /* getopt, access */
#include <stdlib.h>         /* atoi, exit */
#include <errno.h>          /* error messages */
#include <math.h>           /* modf */
#include <assert.h>
#include <stdarg.h>

#include <sys/socket.h>     /* socket specific definitions */
#include <netinet/in.h>     /* INET constants and stuff */
#include <arpa/inet.h>      /* IP address conversion stuff */
#include <netdb.h>          /* gai_strerror */
#include <fcntl.h>

#include <pthread.h>
#include <syslog.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <netinet/in.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <termios.h>


#include "trace.h"
#include "jitqueue.h"
#include "timersync.h"
#include "parson.h"
#include "base64.h"
#include "logger.h"

#include "libloragw/loragw_hal.h"
#include "libloragw/loragw_gps.h"
#include "libloragw/loragw_aux.h"
#include "libloragw/loragw_reg.h"
#include "rrd.h"

#include "filter_node.h"
#include "fbuffer.h"

typedef struct _lora_led{
    int fd;
    bool trigger;
}lora_led;

lgw_context * g_ctx_arr[SUPPORT_SX1301_MAX] = {NULL};
lgw_context * g_ctx_sx1276_arr[SUPPORT_SX1276_MAX] = {NULL};
lora_led g_led_arr[SUPPORT_SX1301_MAX] = {{0}};

//lgw_context * ctx_tx  = NULL; 
bool foreground = 0;

static int main_loop();
static void watch_child();

extern t_packet_table g_packet_table;
int g_debug_level = LOG_INFO;
/* --------DEBUG TO SYSLOG ------*/
void
_debug( int level, char *format, ...)
{

    va_list vlist;
    //char * fmt_cmd;
    char buff[1024];

    va_start(vlist, format);
    vsnprintf(buff, sizeof(buff), format, vlist);
    
    if( foreground ){
        printf("%s\n", buff);
       
    }
    else{
        if( level <= g_debug_level ){
            openlog("lora_pkt_fwd", LOG_PID, LOG_USER);
            syslog(level, buff);
            closelog();
        }
    }
    va_end(vlist);
    
}


/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define STRINGIFY(x)    #x
#define STR(x)          STRINGIFY(x)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
  #define VERSION_STRING "undefined"
#endif

#define DEFAULT_SERVER      127.0.0.1   /* hostname also supported */
#define DEFAULT_PORT_UP     1780
#define DEFAULT_PORT_DW     1782
#define DEFAULT_KEEPALIVE   5           /* default time interval for downstream keep-alive packet */
#define DEFAULT_STAT        30          /* default time interval for statistics */
#define PUSH_TIMEOUT_MS     100
#define PULL_TIMEOUT_MS     200
#define GPS_REF_MAX_AGE     30          /* maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable */
#define FETCH_SLEEP_MS      5          /* nb of ms waited when a fetch return no packets */
#define BEACON_POLL_MS      50          /* time in ms between polling of beacon TX status */

#define PROTOCOL_VERSION    2           /* v1.3 */

#define XERR_INIT_AVG       128         /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */

#define PKT_PUSH_DATA   0
#define PKT_PUSH_ACK    1
#define PKT_PULL_DATA   2
#define PKT_PULL_RESP   3
#define PKT_PULL_ACK    4
#define PKT_TX_ACK      5

#define NB_PKT_MAX      8 /* max number of packets per fetch/send cycle */

#define MIN_LORA_PREAMB 6 /* minimum Lora preamble length for this application */
#define STD_LORA_PREAMB 8
#define MIN_FSK_PREAMB  3 /* minimum FSK preamble length for this application */
#define STD_FSK_PREAMB  5

#define STATUS_SIZE     200
#define TX_BUFF_SIZE    ((540 * NB_PKT_MAX*(SUPPORT_SX1301_MAX + 1)) + 30 + STATUS_SIZE)

#define UNIX_GPS_EPOCH_OFFSET 315964800 /* Number of seconds ellapsed between 01.Jan.1970 00:00:00
                                                                          and 06.Jan.1980 00:00:00 */

#define DEFAULT_BEACON_FREQ_HZ      471900000
#define DEFAULT_BEACON_FREQ_NB      1
#define DEFAULT_BEACON_FREQ_STEP    0
#define DEFAULT_BEACON_DATARATE     10
#define DEFAULT_BEACON_BW_HZ        125000
#define DEFAULT_BEACON_POWER        14
#define DEFAULT_BEACON_INFODESC     0

/* Define value and TX_METADATA_NB is the same */
#define UART_METADATA_NB    16
#define UART_HEADER_LEN     28

/* Reading MCU version index */
#define VERSION_VAILD_SIZE      8
#define VERSION_RETURN_SIZE     12
#define VERSION_SIZE_INDEX      2
#define VERSION_FIRST_INDEX     3

/* Reading MCU RTC index */
#define RTC_VAILD_SIZE      6
#define RTC_RETURN_SIZE     10
#define RTC_SIZE_INDEX      2
#define RTC_FIRST_INDEX     3

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
volatile bool exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool quit_sig = false; /* 1 -> application terminates without shutting down the hardware */

static bool exit_err = false;

/* packets filtering configuration variables */
static bool fwd_valid_pkt = true; /* packets with PAYLOAD CRC OK are forwarded */
static bool fwd_error_pkt = false; /* packets with PAYLOAD CRC ERROR are NOT forwarded */
static bool fwd_nocrc_pkt = false; /* packets with NO PAYLOAD CRC are NOT forwarded */

/* network configuration variables */
static uint64_t lgwm = 0; /* Lora gateway MAC address */
static char serv_addr[64] = STR(DEFAULT_SERVER); /* address of the server (host name or IPv4/IPv6) */
static char serv_port_up[8] = STR(DEFAULT_PORT_UP); /* server port for upstream traffic */
static char serv_port_down[8] = STR(DEFAULT_PORT_DW); /* server port for downstream traffic */
static int keepalive_time = DEFAULT_KEEPALIVE; /* send a PULL_DATA request every X seconds, negative = disabled */

/* statistics collection configuration variables */
static unsigned stat_interval = DEFAULT_STAT; /* time interval (in sec) at which statistics are collected and displayed */


/* gateway <-> MAC protocol variables */
uint32_t net_mac_h; /* Most Significant Nibble, network order */
uint32_t net_mac_l; /* Least Significant Nibble, network order */


/* network sockets */
#ifdef _ALI_LINKWAN_
int sock_up; /* socket for upstream traffic */
#else
static int sock_up; /* socket for upstream traffic */
#endif

static int sock_down; /* socket for downstream traffic */

/* network protocol variables */
static struct timeval push_timeout_half = {0, (PUSH_TIMEOUT_MS * 500)}; /* cut in half, critical for throughput */
static struct timeval pull_timeout = {0, (PULL_TIMEOUT_MS * 1000)}; /* non critical for throughput */

bool is_lorawan = true;

/* hardware access control and correction */
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */
pthread_mutex_t mx_concent_sx1276 = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */
static pthread_mutex_t mx_xcorr = PTHREAD_MUTEX_INITIALIZER; /* control access to the XTAL correction */
static bool xtal_correct_ok = false; /* set true when XTAL correction is stable enough */
static double xtal_correct = 1.0;

/* GPS configuration and synchronization */
static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
static int gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
bool gps_enabled = false; /* is GPS enabled on that gateway ? */

/* GPS time reference */
pthread_mutex_t mx_timeref = PTHREAD_MUTEX_INITIALIZER; /* control access to GPS time reference */
bool gps_ref_valid; /* is GPS reference acceptable (ie. not too old) */
struct tref time_reference_gps; /* time reference used for GPS <-> timestamp conversion */

/* Reference coordinates, for broadcasting (beacon) */
struct coord_s reference_coord;

/* Enable faking the GPS coordinates of the gateway */
bool gps_fake_enable; /* enable the feature */

static int reset_pin;

static LGW_SPI_TYPE spi_device = LGW_SPI_NATIVE;

pthread_mutex_t mx_network_err = PTHREAD_MUTEX_INITIALIZER;
bool status_network_connect = false;

/* measurements to establish statistics */
pthread_mutex_t mx_meas_up = PTHREAD_MUTEX_INITIALIZER; /* control access to the upstream measurements */
static uint32_t meas_nb_rx_rcv = 0; /* count packets received */
static uint32_t meas_nb_rx_ok = 0; /* count packets received with PAYLOAD CRC OK */
static uint32_t meas_nb_rx_bad = 0; /* count packets received with PAYLOAD CRC ERROR */
static uint32_t meas_nb_rx_nocrc = 0; /* count packets received with NO PAYLOAD CRC */
static uint32_t meas_up_pkt_fwd = 0; /* number of radio packet forwarded to the server */
static uint32_t meas_up_network_byte = 0; /* sum of UDP bytes sent for upstream traffic */
static uint32_t meas_up_payload_byte = 0; /* sum of radio payload bytes sent for upstream traffic */
static uint32_t meas_up_dgram_sent = 0; /* number of datagrams sent for upstream traffic */
static uint32_t meas_up_ack_rcv = 0; /* number of datagrams acknowledged for upstream traffic */

uint32_t meas_g_nb_rx_ok = 0; /* count packets received with PAYLOAD CRC OK */
uint32_t meas_g_nb_rx_bad = 0; /* count packets received with PAYLOAD CRC ERROR */
uint32_t meas_g_nb_rx_nocrc = 0; /* count packets received with NO PAYLOAD CRC */


static pthread_mutex_t mx_meas_dw = PTHREAD_MUTEX_INITIALIZER; /* control access to the downstream measurements */
static uint32_t meas_dw_pull_sent = 0; /* number of PULL requests sent for downstream traffic */
static uint32_t meas_dw_ack_rcv = 0; /* number of PULL requests acknowledged for downstream traffic */
static uint32_t meas_dw_dgram_rcv = 0; /* count PULL response packets received for downstream traffic */
static uint32_t meas_dw_network_byte = 0; /* sum of UDP bytes sent for upstream traffic */
static uint32_t meas_dw_payload_byte = 0; /* sum of radio payload bytes sent for upstream traffic */
static uint32_t meas_nb_tx_ok = 0; /* count packets emitted successfully */
static uint32_t meas_nb_tx_fail = 0; /* count packets were TX failed for other reasons */
static uint32_t meas_nb_tx_requested = 0; /* count TX request from server (downlinks) */
static uint32_t meas_nb_tx_rejected_collision_packet = 0; /* count packets were TX request were rejected due to collision with another packet already programmed */
static uint32_t meas_nb_tx_rejected_collision_beacon = 0; /* count packets were TX request were rejected due to collision with a beacon already programmed */
static uint32_t meas_nb_tx_rejected_too_late = 0; /* count packets were TX request were rejected because it is too late to program it */
static uint32_t meas_nb_tx_rejected_too_early = 0; /* count packets were TX request were rejected because timestamp is too much in advance */
static uint32_t meas_nb_beacon_queued = 0; /* count beacon inserted in jit queue */
static uint32_t meas_nb_beacon_sent = 0; /* count beacon actually sent to concentrator */
static uint32_t meas_nb_beacon_rejected = 0; /* count beacon rejected for queuing */

#ifdef _ALI_LINKWAN_
/* Begin add for reset when no ack in specify time */
/* default count timeout of status packets had send but no ackstatus packets */ 
#define DEFAULT_STAT_NO_ACK_TIMEOUT     60
static pthread_mutex_t mx_stat_no_ack = PTHREAD_MUTEX_INITIALIZER;
static uint32_t stat_no_ack_timeout = DEFAULT_STAT_NO_ACK_TIMEOUT;
static uint32_t stat_no_ack_cnt = 0;
/* End */
#endif

pthread_mutex_t mx_meas_gps = PTHREAD_MUTEX_INITIALIZER; /* control access to the GPS statistics */
bool gps_coord_valid; /* could we get valid GPS coordinates ? */
struct coord_s meas_gps_coord; /* GPS position of the gateway */
struct coord_s meas_gps_err; /* GPS position of the gateway */

static pthread_mutex_t mx_stat_rep = PTHREAD_MUTEX_INITIALIZER; /* control access to the status report */
static bool report_ready = false; /* true when there is a new report to send to the server */
static char status_report[STATUS_SIZE]; /* status report as a JSON object */

/* beacon parameters */
#ifdef _ALI_LINKWAN_
static uint32_t beacon_period = 128; /* set beaconing period, must be a sub-multiple of 86400, the nb of sec in a day */
#else
static uint32_t beacon_period = 0;
#endif
static uint32_t beacon_freq_hz = DEFAULT_BEACON_FREQ_HZ; /* set beacon TX frequency, in Hz */
static uint8_t beacon_freq_nb = DEFAULT_BEACON_FREQ_NB; /* set number of beaconing channels beacon */
static uint32_t beacon_freq_step = DEFAULT_BEACON_FREQ_STEP; /* set frequency step between beacon channels, in Hz */
static uint8_t beacon_datarate = DEFAULT_BEACON_DATARATE; /* set beacon datarate (SF) */
static uint32_t beacon_bw_hz = DEFAULT_BEACON_BW_HZ; /* set beacon bandwidth, in Hz */
static int8_t beacon_power = DEFAULT_BEACON_POWER; /* set beacon TX power, in dBm */
static uint8_t beacon_infodesc = DEFAULT_BEACON_INFODESC; /* set beacon information descriptor */

/* auto-quit function */
static uint32_t autoquit_threshold = 30; /* enable auto-quit after a number of non-acknowledged PULL_DATA (0 = disabled)*/
static uint32_t network_error_threshold = 3;
bool data_recovery = false;
char * data_recovery_path = NULL;

/* auto-quit rx CRC error 100% */
static uint32_t autoquit_error_crc = 30;

/* Just In Time TX scheduling */
static struct jit_queue_s jit_queue[SUPPORT_SX1276_MAX];/* multi downlink support */

/* Gateway specificities */
static int8_t antenna_gain = 0;
LoRaMacRegion_t Region;


/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */


int g_sx1301_nb = 0;

#ifdef _ALI_LINKWAN_
/* Begin add for adapt iot lora sdk of ali */
static uint8_t push_ack_token_h; 
static uint8_t push_ack_token_l;
/* End */
#endif

/* Begin add for packet filtering by whitelist and blacklist */
#if defined(USE_FILTER_NODE)
static int filter_inited = 0;
#endif
/* End */


/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

static int parse_SX1301_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

static uint16_t crc16(const uint8_t * data, unsigned size);

static double difftimespec(struct timespec end, struct timespec beginning);

static void gps_process_sync(void);

static void gps_process_coords(void);

/* threads */
void thread_logger(void);
void thread_up(void);
void thread_down(void);
void thread_gps(void);
void thread_valid(void);
void thread_jit(void);
void thread_timersync(void);
void thread_rrd( void );
void thread_led(void);

#ifdef _ALI_LINKWAN_
// Begin add for get cpu/mem used ratio
extern int get_cpu_ratio(float* cpu_ratio);
extern int get_mem_ratio(float* mem_ratio);
// End
#endif


char * global_cfg_path = "/etc/lora/global_conf.json";
char * local_cfg_path = "/etc/lora/local_conf.json";;
char * debug_cfg_path = "/etc/lora/debug_conf.json";;


/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = true;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = true;
    }
    return;
}

static int parse_SX1301_configuration(const char * conf_file ) {
    int i, idx;
    char param_name[32]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX1301_conf";
    JSON_Value *root_val = NULL;
    JSON_Array *SX1301_array = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    JSON_Value *val = NULL;
    JSON_Array *conf_array = NULL;
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_lbt_s lbtconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    uint32_t sf, bw, fdev;
    
    
    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG(LOG_INFO,"ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    SX1301_array = json_object_get_array(json_value_get_object(root_val), conf_obj_name);

    /* point to the gateway configuration object */
    /*conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);*/
    for( idx = 0; idx < (int)json_array_get_count(SX1301_array);idx++ ){
        lgw_context * ctx_one = NULL;
        LGW_SPI_TYPE spi_type = LGW_SPI_NATIVE;
        struct lgw_tx_gain_lut_s iTxlut; 
        conf_obj = json_array_get_object(SX1301_array, idx);

        if (conf_obj == NULL) {
            MSG(LOG_INFO,"INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
            break;
        } else {
            MSG(LOG_INFO,"INFO: %s does contain a JSON object named %s, parsing SX1301 parameters\n", conf_file, conf_obj_name);
        }
        
        str = json_object_get_string( conf_obj, "spi_type");
        if( str )
        {
            if( 0 == strcmp("native", str) || 0 == strcmp("spi", str)){
                spi_type = LGW_SPI_NATIVE;
            }
            else if( 0 == strcmp("usb", str) || 0 == strcmp("ftdi", str)){
                spi_type = LGW_SPI_FTDI;
            }
        }

        MSG(LOG_INFO,"INFO: spi type %d", spi_type);
        ctx_one = lgw_context_init(spi_type, 0);
        if( NULL == ctx_one ){
            MSG(LOG_INFO,"ERROR: context init error!\n");
            exit(EXIT_FAILURE);
        }

        val = json_object_get_value(conf_obj, "reset_pin");
        if( json_value_get_type(val) == JSONNumber ){
            ctx_one->reset_pin = (uint32_t)json_value_get_number(val);
        } else {
            MSG(LOG_INFO,"WARNING: Data type for reset_pin seems wrong, please check\n");
            ctx_one->reset_pin = 0;
        }
        
        g_ctx_arr[idx] = ctx_one;
        /* set board configuration */
        memset(&boardconf, 0, sizeof boardconf); /* initialize configuration structure */
        val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONBoolean) {
            boardconf.lorawan_public = (bool)json_value_get_boolean(val);
        } else {
            MSG(LOG_INFO,"WARNING: Data type for lorawan_public seems wrong, please check\n");
            boardconf.lorawan_public = false;
        }
        val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONNumber) {
            boardconf.clksrc = (uint8_t)json_value_get_number(val);
        } else {
            MSG(LOG_INFO,"WARNING: Data type for clksrc seems wrong, please check\n");
            boardconf.clksrc = 0;
        }
        MSG(LOG_INFO,"INFO: lorawan_public %d, clksrc %d\n", boardconf.lorawan_public, boardconf.clksrc);
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_board_setconf(boardconf,ctx_one) != LGW_HAL_SUCCESS) {
            MSG(LOG_INFO,"ERROR: Failed to configure board\n");
            return -1;
        }

        /* set LBT configuration */
        memset(&lbtconf, 0, sizeof lbtconf); /* initialize configuration structure */
        conf_lbt_obj = json_object_get_object(conf_obj, "lbt_cfg"); /* fetch value (if possible) */
        if (conf_lbt_obj == NULL) {
            MSG(LOG_INFO,"INFO: no configuration for LBT\n");
        } else {
            val = json_object_get_value(conf_lbt_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                lbtconf.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG(LOG_INFO,"WARNING: Data type for lbt_cfg.enable seems wrong, please check\n");
                lbtconf.enable = false;
            }
            if (lbtconf.enable == true) {
                val = json_object_get_value(conf_lbt_obj, "rssi_target"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    lbtconf.rssi_target = (int8_t)json_value_get_number(val);
                } else {
                    MSG(LOG_INFO,"WARNING: Data type for lbt_cfg.rssi_target seems wrong, please check\n");
                    lbtconf.rssi_target = 0;
                }
                val = json_object_get_value(conf_lbt_obj, "sx127x_rssi_offset"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    lbtconf.rssi_offset = (int8_t)json_value_get_number(val);
                } else {
                    MSG(LOG_INFO,"WARNING: Data type for lbt_cfg.sx127x_rssi_offset seems wrong, please check\n");
                    lbtconf.rssi_offset = 0;
                }
                /* set LBT channels configuration */
                conf_array = json_object_get_array(conf_lbt_obj, "chan_cfg");
                if (conf_array != NULL) {
                    lbtconf.nb_channel = json_array_get_count( conf_array );
                    MSG(LOG_INFO,"INFO: %u LBT channels configured\n", lbtconf.nb_channel);
                }
                for (i = 0; i < (int)lbtconf.nb_channel; i++) {
                    /* Sanity check */
                    if (i >= LBT_CHANNEL_FREQ_NB)
                    {
                        MSG(LOG_INFO,"ERROR: LBT channel %d not supported, skip it\n", i );
                        break;
                    }
                    /* Get LBT channel configuration object from array */
                    conf_lbtchan_obj = json_array_get_object(conf_array, i);

                    /* Channel frequency */
                    val = json_object_dotget_value(conf_lbtchan_obj, "freq_hz"); /* fetch value (if possible) */
                    if (json_value_get_type(val) == JSONNumber) {
                        lbtconf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                    } else {
                        MSG(LOG_INFO,"WARNING: Data type for lbt_cfg.channels[%d].freq_hz seems wrong, please check\n", i);
                        lbtconf.channels[i].freq_hz = 0;
                    }

                    /* Channel scan time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); /* fetch value (if possible) */
                    if (json_value_get_type(val) == JSONNumber) {
                        lbtconf.channels[i].scan_time_us = (uint16_t)json_value_get_number(val);
                    } else {
                        MSG(LOG_INFO,"WARNING: Data type for lbt_cfg.channels[%d].scan_time_us seems wrong, please check\n", i);
                        lbtconf.channels[i].scan_time_us = 0;
                    }
                }

                /* all parameters parsed, submitting configuration to the HAL */
                if (lgw_lbt_setconf(lbtconf,ctx_one) != LGW_HAL_SUCCESS) {
                    MSG(LOG_INFO,"ERROR: Failed to configure LBT\n");
                    return -1;
                }
            } else {
                MSG(LOG_INFO,"INFO: LBT is disabled\n");
            }
        }

        /* set antenna gain configuration */
        val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
        if (val != NULL) {
            if (json_value_get_type(val) == JSONNumber) {
                antenna_gain = (int8_t)json_value_get_number(val);
            } else {
                MSG(LOG_INFO,"WARNING: Data type for antenna_gain seems wrong, please check\n");
                antenna_gain = 0;
            }
        }
        MSG(LOG_INFO,"INFO: antenna_gain %d dBi\n", antenna_gain);

        /* set configuration for tx gains */
        memset(&iTxlut, 0, sizeof iTxlut); /* initialize configuration structure */
        for (i = 0; i < TX_GAIN_LUT_SIZE_MAX; i++) {
            snprintf(param_name, sizeof param_name, "tx_lut_%i", i); /* compose parameter path inside JSON structure */
            val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
            if (json_value_get_type(val) != JSONObject) {
                MSG(LOG_INFO,"INFO: no configuration for tx gain lut %i\n", i);
                continue;
            }
            iTxlut.size++; /* update TX LUT size based on JSON object found in configuration file */
            /* there is an object to configure that TX gain index, let's parse it */
            snprintf(param_name, sizeof param_name, "tx_lut_%i.pa_gain", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONNumber) {
                iTxlut.lut[i].pa_gain = (uint8_t)json_value_get_number(val);
            } else {
                MSG(LOG_INFO,"WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                iTxlut.lut[i].pa_gain = 0;
            }
            snprintf(param_name, sizeof param_name, "tx_lut_%i.dac_gain", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONNumber) {
                iTxlut.lut[i].dac_gain = (uint8_t)json_value_get_number(val);
            } else {
                iTxlut.lut[i].dac_gain = 3; /* This is the only dac_gain supported for now */
            }
            snprintf(param_name, sizeof param_name, "tx_lut_%i.dig_gain", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONNumber) {
                iTxlut.lut[i].dig_gain = (uint8_t)json_value_get_number(val);
            } else {
                MSG(LOG_INFO,"WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                iTxlut.lut[i].dig_gain = 0;
            }
            snprintf(param_name, sizeof param_name, "tx_lut_%i.mix_gain", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONNumber) {
                iTxlut.lut[i].mix_gain = (uint8_t)json_value_get_number(val);
            } else {
                MSG(LOG_INFO,"WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                iTxlut.lut[i].mix_gain = 0;
            }
            snprintf(param_name, sizeof param_name, "tx_lut_%i.rf_power", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONNumber) {
                iTxlut.lut[i].rf_power = (int8_t)json_value_get_number(val);
            } else {
                MSG(LOG_INFO,"WARNING: Data type for %s[%d] seems wrong, please check\n", param_name, i);
                iTxlut.lut[i].rf_power = 0;
            }
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (iTxlut.size > 0) {
            MSG(LOG_INFO,"INFO: Configuring TX LUT with %u indexes\n", iTxlut.size);
            if (lgw_txgain_setconf(&iTxlut,ctx_one) != LGW_HAL_SUCCESS) {
                MSG(LOG_INFO,"ERROR: Failed to configure concentrator TX Gain LUT\n");
                return -1;
            }

            memcpy(&txlut, &iTxlut, sizeof txlut);
            
        } else {
            MSG(LOG_INFO,"WARNING: No TX gain LUT defined\n");
        }

        /* set configuration for RF chains */
        for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
            memset(&rfconf, 0, sizeof rfconf); /* initialize configuration structure */
            snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
            val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
            if (json_value_get_type(val) != JSONObject) {
                MSG(LOG_INFO,"INFO: no configuration for radio %i\n", i);
                continue;
            }
            /* there is an object to configure that radio, let's parse it */
            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (!strncmp(str, "SX1255", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1255;
            } else if (!strncmp(str, "SX1257", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1257;
            } else {
                MSG(LOG_INFO,"WARNING: invalid radio type: %s (should be SX1255 or SX1257)\n", str);
            }
                
            snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.enable = (bool)json_value_get_boolean(val);
            } else {
                rfconf.enable = false;
            }
            if (rfconf.enable == false) { /* radio disabled, nothing else to parse */
                MSG(LOG_INFO,"INFO: radio %i disabled\n", i);
            } else  { /* radio enabled, will parse the other parameters */
                snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
                rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
                rfconf.rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
                
                snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
                val = json_object_dotget_value(conf_obj, param_name);
                if (json_value_get_type(val) == JSONBoolean) {
                    rfconf.tx_enable = (bool)json_value_get_boolean(val);
                    if (rfconf.tx_enable == true) {
                        /* tx is enabled on this rf chain, we need its frequency range */
                        snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                        tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                        snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                        tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                        if ((tx_freq_min[i] == 0) || (tx_freq_max[i] == 0)) {
                            MSG(LOG_INFO,"WARNING: no frequency range specified for TX rf chain %d\n", i);
                        }
                        /* ... and the notch filter frequency to be set */
                        snprintf(param_name, sizeof param_name, "radio_%i.tx_notch_freq", i);
                        rfconf.tx_notch_freq = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    }
                } else {
                    rfconf.tx_enable = false;
                }
                MSG(LOG_INFO,"INFO: radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d, tx_notch_freq %u\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable, rfconf.tx_notch_freq);
            }
            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_rxrf_setconf(i, rfconf,ctx_one) != LGW_HAL_SUCCESS) {
                MSG(LOG_INFO,"ERROR: invalid configuration for radio %i\n", i);
                return -1;
            }
        }

        /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
        for (i = 0; i < LGW_MULTI_NB; ++i) {
            memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
            val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
            if (json_value_get_type(val) != JSONObject) {
                MSG(LOG_INFO,"INFO: no configuration for Lora multi-SF channel %i\n", i);
                continue;
            }
            
            /* there is an object to configure that Lora multi-SF channel, let's parse it */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                ifconf.enable = (bool)json_value_get_boolean(val);
            } else {
                ifconf.enable = false;
            }
            
            if (ifconf.enable == false) { /* Lora multi-SF channel disabled, nothing else to parse */
                MSG(LOG_INFO,"INFO: Lora multi-SF channel %i disabled\n", i);
            } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
                snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
                ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
                ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
                // TODO: handle individual SF enabling and disabling (spread_factor)
                MSG(LOG_INFO,"INFO: Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 7 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
            }
            
            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_rxif_setconf(i, ifconf,ctx_one) != LGW_HAL_SUCCESS) {
                MSG(LOG_INFO,"ERROR: invalid configuration for Lora multi-SF channel %i\n", i);
                return -1;
            }
        }

        /* set configuration for Lora standard channel */
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        val = json_object_get_value(conf_obj, "chan_Lora_std"); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG(LOG_INFO,"INFO: no configuration for Lora standard channel\n");
        } else {
            val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
            if (json_value_get_type(val) == JSONBoolean) {
                ifconf.enable = (bool)json_value_get_boolean(val);
            } else {
                ifconf.enable = false;
            }
            if (ifconf.enable == false) {
                MSG(LOG_INFO,"INFO: Lora standard channel %i disabled\n", i);
            } else  {
                ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
                ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
                bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
                switch(bw) {
                    case 500000: ifconf.bandwidth = BW_500KHZ; break;
                    case 250000: ifconf.bandwidth = BW_250KHZ; break;
                    case 125000: ifconf.bandwidth = BW_125KHZ; break;
                    default: ifconf.bandwidth = BW_UNDEFINED;
                }
                sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
                switch(sf) {
                    case  7: ifconf.datarate = DR_LORA_SF7;  break;
                    case  8: ifconf.datarate = DR_LORA_SF8;  break;
                    case  9: ifconf.datarate = DR_LORA_SF9;  break;
                    case 10: ifconf.datarate = DR_LORA_SF10; break;
                    case 11: ifconf.datarate = DR_LORA_SF11; break;
                    case 12: ifconf.datarate = DR_LORA_SF12; break;
                    default: ifconf.datarate = DR_UNDEFINED;
                }
                MSG(LOG_INFO,"INFO: Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf);
            }
            if (lgw_rxif_setconf(8, ifconf,ctx_one) != LGW_HAL_SUCCESS) {
                MSG(LOG_INFO,"ERROR: invalid configuration for Lora standard channel\n");
                return -1;
            }
        }

        /* set configuration for FSK channel */
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        val = json_object_get_value(conf_obj, "chan_FSK"); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG(LOG_INFO,"INFO: no configuration for FSK channel\n");
        } else {
            val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
            if (json_value_get_type(val) == JSONBoolean) {
                ifconf.enable = (bool)json_value_get_boolean(val);
            } else {
                ifconf.enable = false;
            }
            if (ifconf.enable == false) {
                MSG(LOG_INFO,"INFO: FSK channel %i disabled\n", i);
            } else  {
                ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
                ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
                bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
                fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
                ifconf.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

                /* if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
                if ((bw == 0) && (fdev != 0)) {
                    bw = 2 * fdev + ifconf.datarate;
                }
                if      (bw == 0)      ifconf.bandwidth = BW_UNDEFINED;
                else if (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
                else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
                else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
                else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
                else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
                else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
                else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
                else ifconf.bandwidth = BW_UNDEFINED;

                MSG(LOG_INFO,"INFO: FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
            }
            if (lgw_rxif_setconf(9, ifconf,ctx_one) != LGW_HAL_SUCCESS) {
                MSG(LOG_INFO,"ERROR: invalid configuration for FSK channel\n");
                return -1;
            }
        }
        g_sx1301_nb++;
    }
    json_value_free(root_val);

    return 0;
}

static int parse_gateway_configuration(const char * conf_file) {
    const char conf_obj_name[] = "gateway_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    unsigned long long ull = 0;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG(LOG_INFO,"ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG(LOG_INFO,"INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG(LOG_INFO,"INFO: %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj_name);
    }

    /* gateway unique identifier (aka MAC address) (optional) */
    str = json_object_get_string(conf_obj, "gateway_ID");
    if (str != NULL) {
        sscanf(str, "%llx", &ull);
        lgwm = ull;
        MSG(LOG_INFO,"INFO: gateway MAC address is configured to %016llX\n", ull);
    }

    /* server hostname or IP address (optional) */
    str = json_object_get_string(conf_obj, "server_address");
    if (str != NULL) {
        strncpy(serv_addr, str, sizeof serv_addr);
        MSG(LOG_INFO,"INFO: server hostname or IP address is configured to \"%s\"\n", serv_addr);
    }

    /* get up and down ports (optional) */
    val = json_object_get_value(conf_obj, "serv_port_up");
    if (val != NULL) {
        snprintf(serv_port_up, sizeof serv_port_up, "%u", (uint16_t)json_value_get_number(val));
        MSG(LOG_INFO,"INFO: upstream port is configured to \"%s\"\n", serv_port_up);
    }
    val = json_object_get_value(conf_obj, "serv_port_down");
    if (val != NULL) {
        snprintf(serv_port_down, sizeof serv_port_down, "%u", (uint16_t)json_value_get_number(val));
        MSG(LOG_INFO,"INFO: downstream port is configured to \"%s\"\n", serv_port_down);
    }

    /* get keep-alive interval (in seconds) for downstream (optional) */
    val = json_object_get_value(conf_obj, "keepalive_interval");
    if (val != NULL) {
        keepalive_time = (int)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: downstream keep-alive interval is configured to %u seconds\n", keepalive_time);
    }

    /* get interval (in seconds) for statistics display (optional) */
    val = json_object_get_value(conf_obj, "stat_interval");
    if (val != NULL) {
        stat_interval = (unsigned)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: statistics display interval is configured to %u seconds\n", stat_interval);
    }
#ifdef _ALI_LINKWAN_    
    /* Begin add for reset when no ack in specify time */
    /* get count timeout of status packets had send but no ack */
    val = json_object_get_value(conf_obj, "stat_no_ack_timeout_cnt");
    if (val != NULL) {
        stat_no_ack_timeout = (unsigned)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: timeout of status packets had send but no ack is configured to %u count\n", stat_no_ack_timeout);
    }
    /* End */
#endif

    /* get time-out value (in ms) for upstream datagrams (optional) */
    val = json_object_get_value(conf_obj, "push_timeout_ms");
    if (val != NULL) {
        push_timeout_half.tv_usec = 500 * (long int)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: upstream PUSH_DATA time-out is configured to %u ms\n", (unsigned)(push_timeout_half.tv_usec / 500));
    }

    val = json_object_get_value(conf_obj, "lorawan");
    if( val != NULL ){
        is_lorawan = json_value_get_number(val) == 0 ? false : true;
    }

    /* packet filtering parameters */
    val = json_object_get_value(conf_obj, "forward_crc_valid");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_valid_pkt = (bool)json_value_get_boolean(val);
    }
    MSG(LOG_INFO,"INFO: packets received with a valid CRC will%s be forwarded\n", (fwd_valid_pkt ? "" : " NOT"));
    val = json_object_get_value(conf_obj, "forward_crc_error");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_error_pkt = (bool)json_value_get_boolean(val);
    }
    MSG(LOG_INFO,"INFO: packets received with a CRC error will%s be forwarded\n", (fwd_error_pkt ? "" : " NOT"));
    val = json_object_get_value(conf_obj, "forward_crc_disabled");
    if (json_value_get_type(val) == JSONBoolean) {
        fwd_nocrc_pkt = (bool)json_value_get_boolean(val);
    }
    MSG(LOG_INFO,"INFO: packets received with no CRC will%s be forwarded\n", (fwd_nocrc_pkt ? "" : " NOT"));

    str = json_object_get_string(conf_obj, "region");
    if( str != NULL ){
        if( 0 == strcasecmp(str, "CN470")){
            Region = LORAMAC_REGION_CN470;
        }
        else if( 0 == strcasecmp(str, "EU868")){
            Region = LORAMAC_REGION_EU868;
        }
        else if( 0 == strcasecmp(str, "EU433")){
            Region = LORAMAC_REGION_EU433;
        }
        else if( 0 == strcasecmp(str, "KR920")){
            Region = LORAMAC_REGION_KR920;
        }
        else if( 0 == strcasecmp(str, "US915")){
            Region = LORAMAC_REGION_US915;
        }
        else if( 0 == strcasecmp(str, "AU915")){
            Region = LORAMAC_REGION_AU915;
        }
        else if( 0 == strcasecmp(str, "AS923")){
            Region = LORAMAC_REGION_AS923;
        }
        else if( 0 == strcasecmp(str, "IN865")){
            Region = LORAMAC_REGION_IN865;
        }
    }
    /* GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(gps_tty_path, str, sizeof gps_tty_path);
        MSG(LOG_INFO,"INFO: GPS serial port path is configured to \"%s\"\n", gps_tty_path);
    }

    /* get reference coordinates */
    val = json_object_get_value(conf_obj, "ref_latitude");
    if (val != NULL) {
        reference_coord.lat = (double)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Reference latitude is configured to %f deg\n", reference_coord.lat);
    }
    val = json_object_get_value(conf_obj, "ref_longitude");
    if (val != NULL) {
        reference_coord.lon = (double)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Reference longitude is configured to %f deg\n", reference_coord.lon);
    }
    val = json_object_get_value(conf_obj, "ref_altitude");
    if (val != NULL) {
        reference_coord.alt = (short)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Reference altitude is configured to %i meters\n", reference_coord.alt);
    }

    /* Gateway GPS coordinates hardcoding (aka. faking) option */
    val = json_object_get_value(conf_obj, "fake_gps");
    if (json_value_get_type(val) == JSONBoolean) {
        gps_fake_enable = (bool)json_value_get_boolean(val);
        if (gps_fake_enable == true) {
            MSG(LOG_INFO,"INFO: fake GPS is enabled\n");
        } else {
            MSG(LOG_INFO,"INFO: fake GPS is disabled\n");
        }
    }

    /* Beacon signal period (optional) */
    val = json_object_get_value(conf_obj, "beacon_period");
    if (val != NULL) {
        beacon_period = (uint32_t)json_value_get_number(val);
        if ((beacon_period > 0) && (beacon_period < 6)) {
            MSG(LOG_INFO,"ERROR: invalid configuration for Beacon period, must be >= 6s\n");
            return -1;
        } else {
            MSG(LOG_INFO,"INFO: Beaconing period is configured to %u seconds\n", beacon_period);
        }
    }

    /* Beacon TX frequency (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_hz");
    if (val != NULL) {
        beacon_freq_hz = (uint32_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing signal will be emitted at %u Hz\n", beacon_freq_hz);
    }

    /* Number of beacon channels (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_nb");
    if (val != NULL) {
        beacon_freq_nb = (uint8_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing channel number is set to %u\n", beacon_freq_nb);
    }

    /* Frequency step between beacon channels (optional) */
    val = json_object_get_value(conf_obj, "beacon_freq_step");
    if (val != NULL) {
        beacon_freq_step = (uint32_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing channel frequency step is set to %uHz\n", beacon_freq_step);
    }

    /* Beacon datarate (optional) */
    val = json_object_get_value(conf_obj, "beacon_datarate");
    if (val != NULL) {
        beacon_datarate = (uint8_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing datarate is set to SF%d\n", beacon_datarate);
    }

    /* Beacon modulation bandwidth (optional) */
    val = json_object_get_value(conf_obj, "beacon_bw_hz");
    if (val != NULL) {
        beacon_bw_hz = (uint32_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing modulation bandwidth is set to %dHz\n", beacon_bw_hz);
    }

    /* Beacon TX power (optional) */
    val = json_object_get_value(conf_obj, "beacon_power");
    if (val != NULL) {
        beacon_power = (int8_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing TX power is set to %ddBm\n", beacon_power);
    }

    /* Beacon information descriptor (optional) */
    val = json_object_get_value(conf_obj, "beacon_infodesc");
    if (val != NULL) {
        beacon_infodesc = (uint8_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Beaconing information descriptor is set to %u\n", beacon_infodesc);
    }

    /* Auto-quit threshold (optional) */
    val = json_object_get_value(conf_obj, "autoquit_threshold");
    if (val != NULL) {
        autoquit_threshold = (uint32_t)json_value_get_number(val);
        MSG(LOG_INFO,"INFO: Auto-quit after %u non-acknowledged PULL_DATA\n", autoquit_threshold);
    }

    val = json_object_get_value(conf_obj, "data_recovery");
    if (json_value_get_type(val) == JSONBoolean) {
        data_recovery = (bool)json_value_get_boolean(val);
        if (data_recovery == true) {
            MSG(LOG_INFO,"INFO: data_recovery is enabled\n");
        } else {
            MSG(LOG_INFO,"INFO: data_recovery is disabled\n");
        }
    }

    if( data_recovery == true ){
        val = json_object_get_value(conf_obj, "data_recovery_path");
        if (val != NULL) {
            data_recovery_path = strdup(json_value_get_string(val));
        }

        val = json_object_get_value(conf_obj, "network_failure_threshold");
        if( val != NULL ){
            network_error_threshold = (uint32_t)json_value_get_number(val);
        }
    }
    
    val = json_object_get_value(conf_obj, "reset_pin");
    if( val != NULL ){
        reset_pin = (uint32_t)json_value_get_number(val);
    }

    str = json_object_get_string(conf_obj, "device");
    if( str != NULL ){
        
        if( 0 == strcasecmp("ftdi", str) || 0 == strcasecmp("usb", str) ){
            spi_device = LGW_SPI_FTDI;
        }
        else{
            spi_device = LGW_SPI_NATIVE;
        }
    }
    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static int
do_command(const char *format, ...)
{
	va_list vlist;
	char *fmt_cmd;
	int rc;
    int status;
    
	va_start(vlist, format);
	vasprintf(&fmt_cmd, format, vlist);
	va_end(vlist);
    
	status = system(fmt_cmd);
    rc = WEXITSTATUS(status);
	if (rc!=0)
		MSG(LOG_INFO,"do command failed(%d): %s", rc, fmt_cmd);

	free(fmt_cmd);

	return rc;
}

static void reset_lgw(uint32_t pin){

    char path[256];
    if( pin <= 0 )
        return;

    snprintf(path, sizeof(path),"/sys/class/gpio/gpio%d", pin);
    if( 0 != access(path, F_OK)){
        do_command("echo %d > /sys/class/gpio/export", pin);
    }

    do_command("echo \"out\" > /sys/class/gpio/gpio%d/direction",pin);
    do_command("echo 1 > /sys/class/gpio/gpio%d/value",pin);
    sleep(1);
    do_command("echo 0 > /sys/class/gpio/gpio%d/value", pin);
    sleep(1);
    do_command("echo \"in\" > /sys/class/gpio/gpio%d/direction",pin);

    return;
}

static uint16_t crc16(const uint8_t * data, unsigned size) {
    const uint16_t crc_poly = 0x1021;
    const uint16_t init_val = 0x0000;
    uint16_t x = init_val;
    unsigned i, j;

    if (data == NULL)  {
        return 0;
    }

    for (i=0; i<size; ++i) {
        x ^= (uint16_t)data[i] << 8;
        for (j=0; j<8; ++j) {
            x = (x & 0x8000) ? (x<<1) ^ crc_poly : (x<<1);
        }
    }

    return x;
}

static double difftimespec(struct timespec end, struct timespec beginning) {
    double x;

    x = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    x += (double)(end.tv_sec - beginning.tv_sec);

    return x;
}

static int send_tx_ack(uint8_t token_h, uint8_t token_l, enum jit_error_e error) {
    uint8_t buff_ack[64]; /* buffer to give feedback to server */
    int buff_index;

    /* reset buffer */
    memset(buff_ack, 0, sizeof buff_ack);

    /* Prepare downlink feedback to be sent to server */
    buff_ack[0] = PROTOCOL_VERSION;
    buff_ack[1] = token_h;
    buff_ack[2] = token_l;
    buff_ack[3] = PKT_TX_ACK;
    *(uint32_t *)(buff_ack + 4) = net_mac_h;
    *(uint32_t *)(buff_ack + 8) = net_mac_l;
    buff_index = 12; /* 12-byte header */

    /* Put no JSON string if there is nothing to report */
    if (error != JIT_ERROR_OK) {
        /* start of JSON structure */
        memcpy((void *)(buff_ack + buff_index), (void *)"{\"txpk_ack\":{", 13);
        buff_index += 13;
        /* set downlink error status in JSON structure */
        memcpy((void *)(buff_ack + buff_index), (void *)"\"error\":", 8);
        buff_index += 8;
        switch (error) {
            case JIT_ERROR_FULL:
            case JIT_ERROR_COLLISION_PACKET:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"COLLISION_PACKET\"", 18);
                buff_index += 18;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_collision_packet += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_TOO_LATE:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TOO_LATE\"", 10);
                buff_index += 10;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_too_late += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_TOO_EARLY:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TOO_EARLY\"", 11);
                buff_index += 11;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_too_early += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_COLLISION_BEACON:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"COLLISION_BEACON\"", 18);
                buff_index += 18;
                /* update stats */
                pthread_mutex_lock(&mx_meas_dw);
                meas_nb_tx_rejected_collision_beacon += 1;
                pthread_mutex_unlock(&mx_meas_dw);
                break;
            case JIT_ERROR_TX_FREQ:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TX_FREQ\"", 9);
                buff_index += 9;
                break;
            case JIT_ERROR_TX_POWER:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"TX_POWER\"", 10);
                buff_index += 10;
                break;
            case JIT_ERROR_GPS_UNLOCKED:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"GPS_UNLOCKED\"", 14);
                buff_index += 14;
                break;
            default:
                memcpy((void *)(buff_ack + buff_index), (void *)"\"UNKNOWN\"", 9);
                buff_index += 9;
                break;
        }
        /* end of JSON structure */
        memcpy((void *)(buff_ack + buff_index), (void *)"}}", 2);
        buff_index += 2;
    }

    buff_ack[buff_index] = 0; /* add string terminator, for safety */

    /* send datagram to server */
#ifdef _ALI_LINKWAN_    
    /* Begin add for adapt iot lora sdk of ali 
     *  return send(sock_down, (void *)buff_ack, buff_index, 0);
     */
     return send(sock_up, (void *)buff_ack, buff_index, 0);
    /* End */
#else
    return send(sock_down, (void *)buff_ack, buff_index, 0);
#endif
}

static void lora_led_on(int idx){
    //system("echo 1 > /sys/class/leds/rak:green:lora/brightness");
    if( idx == 0 ){
        if( g_led_arr[idx].fd <= 0 ){
            g_led_arr[idx].fd = open("/sys/class/leds/rak:green:lora/brightness", O_RDWR);
            if( g_led_arr[idx].fd <= 0){
                MSG(LOG_ERR, "LED opt error");
                return;
            }
            
        }
        
        write(g_led_arr[idx].fd, "1", 1);
    }

    if( idx == 1 ){
        if( g_led_arr[idx].fd <= 0 ){
            g_led_arr[idx].fd = open("/sys/class/leds/rak:green:lora1/brightness", O_RDWR);
            if( g_led_arr[idx].fd <= 0){
                MSG(LOG_ERR, "LED opt error");
                return;
            }
            
        }
        
        write(g_led_arr[idx].fd, "1", 1);
    }
}

static void lora_led_trigger( int idx ){
    g_led_arr[idx].trigger = true;
}

static void lora_led_off(int idx){
    //system("echo 0 > /sys/class/leds/rak:green:lora/brightness");
    if( idx == 0 ){
        if( g_led_arr[idx].fd <= 0 ){
            g_led_arr[idx].fd = open("/sys/class/leds/rak:green:lora/brightness", O_RDWR);
            if( g_led_arr[idx].fd <= 0){
                MSG(LOG_ERR, "LED opt error");
                return;
            }
            
        }
        write(g_led_arr[idx].fd, "0", 1);
    }

    if( idx == 1 ){
        if( g_led_arr[idx].fd <= 0 ){
            g_led_arr[idx].fd = open("/sys/class/leds/rak:green:lora1/brightness", O_RDWR);
            if( g_led_arr[idx].fd <= 0){
                MSG(LOG_ERR, "LED opt error");
                return;
            }
            
        }
        write(g_led_arr[idx].fd, "0", 1);
    }
}

static pid_t monitor_pid = 0;
void
termination_handler(int s)
{
    
    MSG(LOG_INFO, "Handler for termination caught signal %d", s);
	MSG(LOG_INFO, "Exiting...");
	exit(s == 0 ? 1 : 0);	

}

void
sigchld_handler(int s)
{
	int	status;
    (void)s;
    (void)waitpid(-1, &status, WNOHANG);

}

void init_signals(void)
{
	struct sigaction sa;

	MSG(LOG_INFO, "Initializing signal handlers");
	
	sa.sa_handler = sigchld_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;
	if (sigaction(SIGCHLD, &sa, NULL) == -1) {
		MSG(LOG_INFO, "sigaction(): %s", strerror(errno));
		exit(1);
	}

	/* Trap SIGPIPE */
	/* This is done so that when libhttpd does a socket operation on
	 * a disconnected socket (i.e.: Broken Pipes) we catch the signal
	 * and do nothing. The alternative is to exit. SIGPIPE are harmless
	 * if not desirable.
	 */
	sa.sa_handler = SIG_IGN;
	if (sigaction(SIGPIPE, &sa, NULL) == -1) {
		MSG(LOG_INFO, "sigaction(): %s", strerror(errno));
		exit(1);
	}

	sa.sa_handler = termination_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;

	/* Trap SIGTERM */
	if (sigaction(SIGTERM, &sa, NULL) == -1) {
		MSG(LOG_INFO, "sigaction(): %s", strerror(errno));
		exit(1);
	}

	/* Trap SIGQUIT */
	if (sigaction(SIGQUIT, &sa, NULL) == -1) {
		MSG(LOG_INFO, "sigaction(): %s", strerror(errno));
		exit(1);
	}

	/* Trap SIGINT */
	if (sigaction(SIGINT, &sa, NULL) == -1) {
		MSG(LOG_INFO, "sigaction(): %s", strerror(errno));
		exit(1);
	}

    return;
}


static void
watch_child(void)
{
    int status;
    pid_t pid_main = -1;
    pid_t pid_exit = -1;
    monitor_pid = getpid();

    for (;;)
    {

        if (pid_exit == pid_main)
        {
        	if ((pid_main = fork()) == 0) 
            {
                /* child */
                sleep(3);
                signal(SIGCHLD, SIG_DFL);
                main_loop();
                exit(1);
        	}
        }

    	/* parent */
    	pid_exit = wait(&status);
        
    	if (WIFEXITED(status))
        {
    		MSG(LOG_INFO,"child process %d exited with status %d", pid_exit, WEXITSTATUS(status));
    	} 
        else if (WIFSIGNALED(status))
        {
    		MSG(LOG_INFO,"child process %d exited  due to signal %d", pid_exit, WTERMSIG(status));
    	} 
        else
        {
    		MSG(LOG_INFO,"child process %d exited", pid_exit);
    	}

    	if (WIFEXITED(status))
    	{
  
    	    if (WEXITSTATUS(status) == 0)
    		    exit(0);   
    	}

    	if (WIFSIGNALED(status))
        {
    	    switch (WTERMSIG(status))
            {
        	    case SIGKILL:
        		    exit(0);
        		    break;
        	    case SIGINT:
        	    case SIGTERM:
            		exit(1);
        	    default:
        		    break;
	        }
	    }
    }

    return;
}

void usage( void ){
    printf("\nUsage: lora_pkt_fwd -[gld]\n");
    printf("\t\t-g\tglobal_conf_path\n");
    printf("\t\t-l\tlocal_conf_path\n");
    printf("\t\t-d\tdebug_conf_path\n");
    printf("\n");
    return;
}
/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */
int main(int argc, char ** argv){
    int i;

    while((i = getopt(argc, argv, "hg:l:b:fv:")) != -1){
        switch(i){
            case 'h' :{
                usage();
                return -1;
                break;
            }
            case 'g' : {
                global_cfg_path = strdup(optarg);
                break;
            }
            case 'l' : {
                local_cfg_path = strdup(optarg);
                break;
            }
            case 'd' : {
                debug_cfg_path = strdup(optarg);
                break;
            }
            case 'f' :{
                foreground = true;
                break;
            }
            case 'v':{
                if( atoi(optarg) <= LOG_DEBUG )
                    g_debug_level =  atoi(optarg);
                break;
            }
            default:{
                usage();
                return -1;
                break;
            }
        }
    }
    
    init_signals();
    if( foreground == true ){
        signal(SIGCHLD, SIG_DFL);
        main_loop();
    }
    else{
        switch(fork()) {
        	case 0: /* child */
        		setsid();
        		watch_child();
        		break;

        	default: /* parent */
        		exit(0);
        		break;
        }
    }
    return 0;
}

int get_iface_ip(const char *ifname, struct sockaddr_in * addr)
{

	struct ifreq if_data;

	int sockd;
	u_int32_t ip;

	/* Create a socket */
	if ((sockd = socket (AF_INET, SOCK_PACKET, htons(0x8086))) < 0) {
		MSG(LOG_ERR, "socket(): %s", strerror(errno));
		return 1;
	}

	/* Get IP of internal interface */
	strcpy (if_data.ifr_name, ifname);

	/* Get the IP address */
	if (ioctl (sockd, SIOCGIFADDR, &if_data) < 0) {
		MSG(LOG_ERR, "ioctl(): SIOCGIFADDR %s", strerror(errno));
		close(sockd);
		return 1;
	}
	memcpy ((void *) &ip, (void *) &if_data.ifr_addr.sa_data + 2, 4);
	addr->sin_addr.s_addr = ip;

    return 0;
}

/*
 * UsbToUart communication interfaces
 */
const char *uart_dev[SUPPORT_SX1276_MAX] = {
    "/dev/ttyUSB0",
};
int uart_fd[SUPPORT_SX1276_MAX];

/* CRC8 polynomial expression 0x07(10001110) */
uint8_t crc_check(uint8_t *data, uint8_t len)
{
    uint8_t i, j;
    uint8_t crc_byte = 0x00;

    for (i = 0; i < len; i++) {
        crc_byte ^= data[i];

        for (j = 0; j < 8; j++) {
            if (crc_byte & 0x80) {
                crc_byte <<= 1;
                crc_byte ^= 0x07;
            } else
                crc_byte <<= 1;
        }
    }

    return crc_byte;
}

int lora_uart_open(const char *dev_name)
{
    return open(dev_name, O_RDWR);
}

int lora_uart_set_port(const int fd, int speed, int databits, int stopbits, int parity)
{
    struct termios opt;

    /* Getting device file attr */
    if (tcgetattr(fd, &opt) != 0)
        return -1;

    /* Setting speed */
    switch (speed) {
    case 115200:
        cfsetispeed(&opt, B115200);
        cfsetospeed(&opt, B115200);
        break;
    default:
        cfsetispeed(&opt, B115200);
        cfsetospeed(&opt, B115200);
        break;
    }
    /* Before setting attr, we must clear input/output queue by tcflush */
    tcflush(fd, TCIFLUSH);
    /* TCSANOW for setting attr is valid at once */
    if (tcsetattr(fd, TCSANOW, &opt) != 0)
        return -1;

    opt.c_cflag &= ~CSIZE;
    /* Setting databit for 8bits */
    switch (databits) {
    case 7:
        opt.c_cflag |= CS7;
        break;
    case 8:
        opt.c_cflag |= CS8;
        break;
    default:
        opt.c_cflag |= CS8;
        break;
    }

    /* Setting parity */
    switch (parity) {
    case 'n':
    case 'N':
        opt.c_cflag &= ~PARENB;     /* Clear parity enable */
        opt.c_iflag &= ~INPCK;      /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        opt.c_cflag |= (PARODD | PARENB);
        opt.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~PARODD;
        opt.c_iflag |= INPCK;
        break;
    case 'S':
    case 's':
        opt.c_cflag &= ~PARENB;     /* Clear parity enable */
        opt.c_iflag &= ~INPCK;      /* Enable parity checking */
        break;
    default:
        opt.c_cflag &= ~PARENB;     /* Clear parity enable */
        opt.c_iflag &= ~INPCK;      /* Enable parity checking */
        break;
    }

    /* Setting stopbit */
    switch (stopbits) {
    case 1:
        opt.c_cflag &= ~CSTOPB;
        break;
    case 2:
        opt.c_cflag |= CSTOPB;
        break;
    default:
        opt.c_cflag &= ~CSTOPB;
        break;
    }

    /* Setting local flag, ICANON for Nonstandard mode */
    opt.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

    /* Before setting attr, we must clear input/output queue by tcflush */
    tcflush(fd, TCIFLUSH);
    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN] = 0;
    /* TCSANOW for setting attr is valid at once */
    if (tcsetattr(fd, TCSADRAIN, &opt) != 0)
        return -1;

    return 0;
}

int lora_uart_write(int fd, uint8_t *buf, int size)
{
    return write(fd, buf, size);
}

/* Getting read buffer need some times */
int lora_uart_read(int fd, uint8_t *buf, int size)
{
    int i;
    int bytes = 0;
    int retry = 3000;

    //fcntl(fd, F_SETFL, FNDELAY);
    for (i = 0; i < retry; i++) {
        ioctl(fd, FIONREAD, &bytes);
        if (bytes > 0)
            break;
    }

    if (i == retry)
        return 0;

    return read(fd, buf, size);
}

void lora_uart_read_version(int fd)
{
    int i;
    int bytes;
    uint8_t retry = 3;
    uint8_t tmp[VERSION_VAILD_SIZE];
    uint8_t ver_buf[VERSION_RETURN_SIZE];
    uint8_t buf[] = { 0x00, 0x01, 0x00 }; /* Reading version command */

    for (i = 0; i < retry; i++) {
        lora_uart_write(fd, buf, sizeof(buf) / sizeof(uint8_t));
        bytes = lora_uart_read(fd, ver_buf, VERSION_RETURN_SIZE);
        if (bytes != VERSION_RETURN_SIZE) {
            MSG(LOG_NOTICE, "Reading MCU Version errors, reading count: %d\n", bytes);
            continue;
        }

        /* Checking datas CRC */
        if (crc_check(&ver_buf[VERSION_FIRST_INDEX], ver_buf[VERSION_SIZE_INDEX]) == \
                ver_buf[VERSION_RETURN_SIZE - 1])
            break;
    }

    if (i == retry) {
        MSG(LOG_NOTICE, "Reading MCU Version faild.\n");
        return;
    }

    memcpy(tmp, &ver_buf[VERSION_FIRST_INDEX], ver_buf[VERSION_SIZE_INDEX]);
    MSG(LOG_NOTICE, "MCU Version: %s\n", tmp);
}

void lora_uart_read_rtc(int fd)
{
    int i;
    int bytes;
    uint8_t retry = 3;
    uint32_t second;
    uint16_t msecond;
    uint8_t rtc_buf[RTC_RETURN_SIZE];
    uint8_t buf[] = { 0x00, 0x03, 0x00 }; /* Reading RTC command */

    for (i = 0; i < retry; i++) {
        lora_uart_write(fd, buf, sizeof(buf) / sizeof(uint8_t));
        bytes = lora_uart_read(fd, rtc_buf, RTC_RETURN_SIZE);
        if (bytes != RTC_RETURN_SIZE) {
            MSG(LOG_NOTICE, "Reading MCU RTC errors, reading count: %d\n", bytes);
        }

        /* Checking datas CRC */
        if (crc_check(&rtc_buf[RTC_FIRST_INDEX], rtc_buf[RTC_SIZE_INDEX]) == \
                rtc_buf[RTC_RETURN_SIZE - 1])
            break;
    }

    if (i == retry) {
        MSG(LOG_NOTICE, "Reading MCU RTC faild.\n");
        return;
    }

    second = ((((uint32_t)(rtc_buf[RTC_FIRST_INDEX]) << 24) & 0xff000000) | \
            (((uint32_t)(rtc_buf[RTC_FIRST_INDEX + 1]) << 16) & 0x00ff0000) | \
            (((uint32_t)(rtc_buf[RTC_FIRST_INDEX + 2]) << 8) & 0x0000ff00) | \
            (((uint32_t)(rtc_buf[RTC_FIRST_INDEX + 3]) << 0) & 0x000000ff));
    msecond = ((((uint16_t)(rtc_buf[RTC_FIRST_INDEX + 4]) << 8) & 0xff00) | \
            (((uint16_t)(rtc_buf[RTC_FIRST_INDEX + 5]) << 0) & 0x00ff));

    MSG(LOG_NOTICE, "MCU RTC: %d.%d\n", second, msecond);
}

int lora_uart_write_downlink(const int fd, uint8_t port, uint8_t func, struct lgw_pkt_tx_s pkt)
{
    int i;
    int size;

    /* Buffer prepare the packet to send + metadata + 2bytes(uart_header: port and function) */
    uint8_t buff[256 + UART_METADATA_NB + 2];

    /* Uart protocol header */
    buff[0] = port;
    buff[1] = func;

    /* Uart datas from pkt */
    buff[2] = (uint8_t)((pkt.freq_hz >> 24) & 0xff);
    buff[3] = (uint8_t)((pkt.freq_hz >> 16) & 0xff);
    buff[4] = (uint8_t)((pkt.freq_hz >> 8) & 0xff);
    buff[5] = (uint8_t)((pkt.freq_hz >> 0) & 0xff);
    buff[6] = pkt.tx_mode;
    buff[7] = (uint8_t)((pkt.count_us >> 24) & 0xff);
    buff[8] = (uint8_t)((pkt.count_us >> 16) & 0xff);
    buff[9] = (uint8_t)((pkt.count_us >> 8) & 0xff);
    buff[10] = (uint8_t)((pkt.count_us >> 0) & 0xff);
    buff[11] = pkt.rf_chain;
    buff[12] = pkt.rf_power;
    buff[13] = pkt.modulation;
    buff[14] = pkt.bandwidth;
    buff[15] = (uint8_t)((pkt.datarate >> 24) & 0xff);
    buff[16] = (uint8_t)((pkt.datarate >> 16) & 0xff);
    buff[17] = (uint8_t)((pkt.datarate >> 8) & 0xff);
    buff[18] = (uint8_t)((pkt.datarate >> 0) & 0xff);
    buff[19] = pkt.coderate;
    buff[20] = pkt.invert_pol;
    buff[21] = pkt.f_dev;
    buff[22] = (uint8_t)((pkt.preamble >> 8) & 0xff);
    buff[23] = (uint8_t)((pkt.preamble >> 0) & 0xff);
    buff[24] = pkt.no_crc;
    buff[25] = pkt.no_header;
    buff[26] = (uint8_t)((pkt.size >> 8) & 0xff);
    buff[27] = (uint8_t)((pkt.size >> 0) & 0xff);
    for (i = 0; i < pkt.size; i++)
        buff[UART_HEADER_LEN + i] = pkt.payload[i];

    size = UART_HEADER_LEN + pkt.size;

    return lora_uart_write(fd, buff, size);
}

lgw_context_sx1276 * lgw_context_sx1276_init(int index)
{
    lgw_context_sx1276 * ctx = NULL;

    ctx = (lgw_context_sx1276 *)malloc(sizeof(lgw_context_sx1276));
    if( NULL == ctx ){
        DEBUG_MSG("ERROR: malloc lgw context sx1276 error\n");
        return NULL;
    }

    memset(ctx, 0x00, sizeof(lgw_context_sx1276));

    ctx->uart = lora_uart_open(uart_dev[index]);
    if (ctx->uart == -1) {
        MSG(LOG_ERR, "ERROR: Open faild, index: %d.\n", index);
        exit(EXIT_FAILURE);
    }

    if (lora_uart_set_port(ctx->uart, 115200, 8, 1, 'N') == -1) {
        MSG(LOG_ERR, "ERROR: Set port failed, index: %d\n", index);
        exit(EXIT_FAILURE);
    }

    return ctx;
}

static int main_loop()
{
    struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
    int i; /* loop variable and temporary variable for return value */
    int x;
    int idx;
    /* configuration file related */

    /* threads */
    pthread_t thrid_logger;
    pthread_t thrid_up;
    pthread_t thrid_down;
    pthread_t thrid_gps;
    pthread_t thrid_valid;
    pthread_t thrid_jit;
    pthread_t thrid_timersync;
    pthread_t thrid_rrd;
    pthread_t thrid_led;
    /* network socket creation */
    struct addrinfo hints;
    struct addrinfo *result; /* store result of getaddrinfo */
    struct addrinfo *q; /* pointer to move into *result data */

    char host_name[64];
    char port_name[64];

    /* variables to get local copies of measurements */
    uint32_t cp_nb_rx_rcv;
    uint32_t cp_nb_rx_ok;
    uint32_t cp_nb_rx_bad;
    uint32_t cp_nb_rx_nocrc;
    uint32_t cp_up_pkt_fwd;
    uint32_t cp_up_network_byte;
    uint32_t cp_up_payload_byte;
    uint32_t cp_up_dgram_sent;
    uint32_t cp_up_ack_rcv;
    uint32_t cp_dw_pull_sent;
    uint32_t cp_dw_ack_rcv;
    uint32_t cp_dw_dgram_rcv;
    uint32_t cp_dw_network_byte;
    uint32_t cp_dw_payload_byte;
    uint32_t cp_nb_tx_ok;
    uint32_t cp_nb_tx_fail;
    uint32_t cp_nb_tx_requested = 0;
    uint32_t cp_nb_tx_rejected_collision_packet = 0;
    uint32_t cp_nb_tx_rejected_collision_beacon = 0;
    uint32_t cp_nb_tx_rejected_too_late = 0;
    uint32_t cp_nb_tx_rejected_too_early = 0;
    uint32_t cp_nb_beacon_queued = 0;
    uint32_t cp_nb_beacon_sent = 0;
    uint32_t cp_nb_beacon_rejected = 0;
    
#ifdef _ALI_LINKWAN_    
    /* Begin add for reset when no ack in specify time */
    uint32_t cp_stat_no_ack_cnt = 0;
    bool need_reset = false;
    /* End */
#endif

    /* GPS coordinates variables */
    bool coord_ok = false;
    struct coord_s cp_gps_coord = {0.0, 0.0, 0};

    /* SX1301 data variables */
    uint32_t trig_tstamp;

    /* statistics variable */
    time_t t;
    char stat_timestamp[24];
    float rx_ok_ratio;
    float rx_bad_ratio;
    float rx_nocrc_ratio;
    float up_ack_ratio;
    float dw_ack_ratio;    
    // Begin add for get cpu/mem used ratio
    float cpu_ratio = 0.0;
    float mem_ratio = 0.0;
    // End

    uint32_t rx_error_crc_num = 0;

    lgw_context_sx1276 * ctx_sx1276 = NULL;

    /* display version informations */
    MSG(LOG_INFO,"*** Beacon Packet Forwarder for Lora Gateway ***\nVersion: " VERSION_STRING "\n");
    MSG(LOG_INFO,"*** Lora concentrator HAL library version info ***\n%s\n***\n", lgw_version_info());

    /* display host endianness */
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        MSG(LOG_INFO,"INFO: Little endian host\n");
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        MSG(LOG_INFO,"INFO: Big endian host\n");
    #else
        MSG(LOG_INFO,"INFO: Host endianness unknown\n");
    #endif

    if (access(global_cfg_path, R_OK) == 0) { /* if there is a global conf, parse it and then try to parse local conf  */
        MSG(LOG_INFO,"INFO: found global configuration file %s, parsing it\n", global_cfg_path);
        x = parse_SX1301_configuration(global_cfg_path );
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        x = parse_gateway_configuration(global_cfg_path);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
    } 

    else {
        MSG(LOG_CRIT,"ERROR: [main] failed to find any configuration file named %s, %s OR %s\n", global_cfg_path, local_cfg_path, debug_cfg_path);
        exit(EXIT_FAILURE);
    }

    /* Start GPS a.s.a.p., to allow it to lock */
    if (gps_tty_path[0] != '\0') { /* do not try to open GPS device if no path set */
        i = lgw_gps_enable(gps_tty_path, "ubx7", 0, &gps_tty_fd); /* HAL only supports u-blox 7 for now */
        if (i != LGW_GPS_SUCCESS) {
            MSG(LOG_WARNING,"WARNING: [main] impossible to open %s for GPS sync (check permissions)\n", gps_tty_path);
            gps_enabled = false;
            gps_ref_valid = false;
        } else {
            MSG(LOG_INFO,"INFO: [main] TTY port %s open for GPS synchronization\n", gps_tty_path);
            gps_enabled = true;
            gps_ref_valid = false;
        }
    }

    /* get timezone info */
    tzset();

    /* sanity check on configuration variables */
    // TODO

    /* process some of the configuration variables */
    net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (lgwm>>32)));
    net_mac_l = htonl((uint32_t)(0xFFFFFFFF &  lgwm  ));

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL); /* Ctrl-\ */
    sigaction(SIGINT, &sigact, NULL); /* Ctrl-C */
    sigaction(SIGTERM, &sigact, NULL); /* default "kill" command */

    /* prepare hints to open network sockets */
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; /* WA: Forcing IPv4 as AF_UNSPEC makes connection on localhost to fail */
    hints.ai_socktype = SOCK_DGRAM;

    /* Init lgw */
    for( idx=0; idx < SUPPORT_SX1301_MAX; idx++ ){
        if( NULL == g_ctx_arr[idx])
            break;

        if( 0 == g_ctx_arr[idx]->reset_pin )
            continue;
        
        reset_lgw(g_ctx_arr[idx]->reset_pin);     
    }
    /* starting the concentrator */
 
    for( idx = 0; idx < SUPPORT_SX1301_MAX; idx++ ){
        if( NULL == g_ctx_arr[idx] )
            break;
        
        i = lgw_start(g_ctx_arr[idx]);
        if (i == LGW_HAL_SUCCESS) {
            MSG(LOG_NOTICE,"INFO: [main] concentrator started, packet can now be received\n");
            
            lora_led_on(idx);
            
        } else {
            MSG(LOG_CRIT,"ERROR: [main] failed to start the concentrator\n");
            exit(EXIT_FAILURE);
        }
    }
    
    /* Open UsbToUart device file */
    for (i = 0; i < SUPPORT_SX1276_MAX; i++) {
        ctx_sx1276 = lgw_context_sx1276_init(i);
        if (NULL == ctx_sx1276) {
            MSG(LOG_INFO, "ERROR: context sx1276 init error!\n");
            exit(EXIT_FAILURE);
        }

        g_ctx_sx1276_arr[i] = ctx_sx1276;
    }

    while(!exit_sig && !quit_sig )
    {
        sleep(10);
        
        /* look for server address w/ upstream port */
        i = getaddrinfo(serv_addr, serv_port_up, &hints, &result);
        if (i != 0) {
            MSG(LOG_ERR,"ERROR: [up] getaddrinfo on address %s (PORT %s) returned %s\n", serv_addr, serv_port_up, gai_strerror(i));
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }
        /* try to open socket for upstream traffic */
        for (q=result; q!=NULL; q=q->ai_next) {
            sock_up = socket(q->ai_family, q->ai_socktype,q->ai_protocol);
            if (sock_up == -1) 
                continue; /* try next field */
            else 
                break; /* success, get out of loop */
        }

        if (q == NULL) {
            MSG(LOG_ERR,"ERROR: [up] failed to open socket to any of server %s addresses (port %s)\n", serv_addr, serv_port_up);
            i = 1;
            for (q=result; q!=NULL; q=q->ai_next) {
                getnameinfo(q->ai_addr, q->ai_addrlen, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST);
                MSG(LOG_INFO,"INFO: [up] result %i host:%s service:%s\n", i, host_name, port_name);
                ++i;
            }
            freeaddrinfo(result);
            close(sock_up);
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }

        /* connect so we can send/receive packet with the server only */
        i = connect(sock_up, q->ai_addr, q->ai_addrlen);
        if (i != 0) {
            MSG(LOG_ERR,"ERROR: [up] connect returned %s\n", strerror(errno));
            freeaddrinfo(result);
            //exit(EXIT_FAILURE);
            close(sock_up);
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }
        
        /* set upstream socket RX timeout */
        i = setsockopt(sock_up, SOL_SOCKET, SO_RCVTIMEO, (void *)&push_timeout_half, sizeof push_timeout_half);
        if (i != 0) {
            MSG(LOG_CRIT,"ERROR: [up] setsockopt returned %s\n", strerror(errno));
            exit(EXIT_FAILURE);
        }
        
        freeaddrinfo(result);
        /* look for server address w/ downstream port */
        i = getaddrinfo(serv_addr, serv_port_down, &hints, &result);
        if (i != 0) {
            MSG(LOG_ERR,"ERROR: [down] getaddrinfo on address %s (port %s) returned %s\n", serv_addr, serv_port_up, gai_strerror(i));
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }

        /* try to open socket for downstream traffic */
        for (q=result; q!=NULL; q=q->ai_next) {
            sock_down = socket(q->ai_family, q->ai_socktype,q->ai_protocol);
            if (sock_down == -1)
                continue; /* try next field */
            else 
                break; /* success, get out of loop */
        }

        if (q == NULL) {
            MSG(LOG_ERR,"ERROR: [down] failed to open socket to any of server %s addresses (port %s)\n", serv_addr, serv_port_up);
            i = 1;
            for (q=result; q!=NULL; q=q->ai_next) {
                getnameinfo(q->ai_addr, q->ai_addrlen, host_name, sizeof host_name, port_name, sizeof port_name, NI_NUMERICHOST);
                MSG(LOG_INFO,"INFO: [down] result %i host:%s service:%s\n", i, host_name, port_name);
                ++i;
            }
            //exit(EXIT_FAILURE);
            close(sock_down);
            freeaddrinfo(result);
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }
#ifdef _ALI_LINKWAN_
         /* bind so we can receive packet with the server only */
        i = bind(sock_down, q->ai_addr, q->ai_addrlen);
        if (i != 0) {
             MSG(LOG_ERR,"ERROR: [down] bind returned %s\n", strerror(errno));
            close(sock_down);
            freeaddrinfo(result);
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }
        
        
#else
       /* connect so we can send/receive packet with the server only */
        i = connect(sock_down, q->ai_addr, q->ai_addrlen);
        if (i != 0) {
            MSG(LOG_CRIT,"ERROR: [down] connect returned %s\n", strerror(errno));
            //exit(EXIT_FAILURE);
            close(sock_down);
            freeaddrinfo(result);
            if( data_recovery ){
                continue;
            }
            else{
                exit(EXIT_FAILURE);
            }
        }
#endif

        freeaddrinfo(result);
        break;
    }
    


#if defined(USE_FILTER_NODE)
    if (0 == filter_init()) {
        filter_inited = 1;
    }
#endif
    rrd_init();  
    
    i = pthread_create( &thrid_up, NULL, (void * (*)(void *))thread_up, NULL);
    if (i != 0) {
        MSG(LOG_CRIT,"ERROR: [main] impossible to create upstream thread\n");
        exit(EXIT_FAILURE);
    }

    i = pthread_create(&thrid_rrd, NULL, (void * (*)(void *))thread_rrd, NULL);
    if( i != 0){
        MSG(LOG_CRIT,"ERROR: [main] impossible to create RRD thread\n");
        exit(EXIT_FAILURE);
    }
    
    /* spawn threads to manage upstream and downstream */
    i = pthread_create(&thrid_logger, NULL, ( void * (*)(void *))thread_logger, NULL);
    if( i != 0 ){
        MSG(LOG_CRIT,"ERROR: [main] impossible to create logger thread\n");
        exit(EXIT_FAILURE);
    }

    i = pthread_create( &thrid_down, NULL, (void * (*)(void *))thread_down, NULL);
    if (i != 0) {
        MSG(LOG_CRIT,"ERROR: [main] impossible to create downstream thread\n");
        exit(EXIT_FAILURE);
    }
    i = pthread_create( &thrid_jit, NULL, (void * (*)(void *))thread_jit, NULL);
    if (i != 0) {
        MSG(LOG_CRIT,"ERROR: [main] impossible to create JIT thread\n");
        exit(EXIT_FAILURE);
    }
    i = pthread_create( &thrid_timersync, NULL, (void * (*)(void *))thread_timersync, NULL);
    if (i != 0) {
        MSG(LOG_CRIT,"ERROR: [main] impossible to create Timer Sync thread\n");
        exit(EXIT_FAILURE);
    }

    
    i = pthread_create(&thrid_led, NULL, (void *(*)(void *))thread_led, NULL);
    if( i != 0 ){
        MSG(LOG_CRIT,"ERROR: [main] impossible to create logger thread\n");
        exit(EXIT_FAILURE);
    }
    
    /* spawn thread to manage GPS */
    if (gps_enabled == true) {
        i = pthread_create( &thrid_gps, NULL, (void * (*)(void *))thread_gps, NULL);
        if (i != 0) {
            MSG(LOG_CRIT,"ERROR: [main] impossible to create GPS thread\n");
            exit(EXIT_FAILURE);
        }
        i = pthread_create( &thrid_valid, NULL, (void * (*)(void *))thread_valid, NULL);
        if (i != 0) {
            MSG(LOG_CRIT,"ERROR: [main] impossible to create validation thread\n");
            exit(EXIT_FAILURE);
        }
    }
    
    
    /* main loop task : statistics collection */
    while (!exit_sig && !quit_sig) {
        /* wait for next reporting interval */
        wait_ms(1000 * stat_interval);

        /* get timestamp for statistics */
        t = time(NULL);
        strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

        /* access upstream statistics, copy and reset them */
        pthread_mutex_lock(&mx_meas_up);
        cp_nb_rx_rcv       = meas_nb_rx_rcv;
        cp_nb_rx_ok        = meas_nb_rx_ok;
        cp_nb_rx_bad       = meas_nb_rx_bad;
        cp_nb_rx_nocrc     = meas_nb_rx_nocrc;
        cp_up_pkt_fwd      = meas_up_pkt_fwd;
        cp_up_network_byte = meas_up_network_byte;
        cp_up_payload_byte = meas_up_payload_byte;
        cp_up_dgram_sent   = meas_up_dgram_sent;
        cp_up_ack_rcv      = meas_up_ack_rcv;
        meas_nb_rx_rcv = 0;
        meas_nb_rx_ok = 0;
        meas_nb_rx_bad = 0;
        meas_nb_rx_nocrc = 0;
        meas_up_pkt_fwd = 0;
        meas_up_network_byte = 0;
        meas_up_payload_byte = 0;
        meas_up_dgram_sent = 0;
        meas_up_ack_rcv = 0;
        pthread_mutex_unlock(&mx_meas_up);
        if (cp_nb_rx_rcv > 0) {
            rx_ok_ratio = (float)cp_nb_rx_ok / (float)cp_nb_rx_rcv;
            rx_bad_ratio = (float)cp_nb_rx_bad / (float)cp_nb_rx_rcv;
            rx_nocrc_ratio = (float)cp_nb_rx_nocrc / (float)cp_nb_rx_rcv;
            if( rx_ok_ratio == 0 ){
                rx_error_crc_num++;
            }
            else{
                rx_error_crc_num = 0;
            }
            
        } else {
            rx_ok_ratio = 0.0;
            rx_bad_ratio = 0.0;
            rx_nocrc_ratio = 0.0;
            rx_error_crc_num = 0;
        }

        if( rx_error_crc_num >= autoquit_error_crc ){
            MSG(LOG_CRIT,"ERROR : RX CRC ERROR quit!");
            exit_err = 1;
            exit_sig = 1;
        }
        
        if (cp_up_dgram_sent > 0) {
            up_ack_ratio = (float)cp_up_ack_rcv / (float)cp_up_dgram_sent;
        } else {
            up_ack_ratio = 0.0;
        }

        /* access downstream statistics, copy and reset them */
        pthread_mutex_lock(&mx_meas_dw);
        cp_dw_pull_sent    =  meas_dw_pull_sent;
        cp_dw_ack_rcv      =  meas_dw_ack_rcv;
        cp_dw_dgram_rcv    =  meas_dw_dgram_rcv;
        cp_dw_network_byte =  meas_dw_network_byte;
        cp_dw_payload_byte =  meas_dw_payload_byte;
        cp_nb_tx_ok        =  meas_nb_tx_ok;
        cp_nb_tx_fail      =  meas_nb_tx_fail;
        cp_nb_tx_requested                 +=  meas_nb_tx_requested;
        cp_nb_tx_rejected_collision_packet +=  meas_nb_tx_rejected_collision_packet;
        cp_nb_tx_rejected_collision_beacon +=  meas_nb_tx_rejected_collision_beacon;
        cp_nb_tx_rejected_too_late         +=  meas_nb_tx_rejected_too_late;
        cp_nb_tx_rejected_too_early        +=  meas_nb_tx_rejected_too_early;
        cp_nb_beacon_queued   +=  meas_nb_beacon_queued;
        cp_nb_beacon_sent     +=  meas_nb_beacon_sent;
        cp_nb_beacon_rejected +=  meas_nb_beacon_rejected;
        meas_dw_pull_sent = 0;
        meas_dw_ack_rcv = 0;
        meas_dw_dgram_rcv = 0;
        meas_dw_network_byte = 0;
        meas_dw_payload_byte = 0;
        meas_nb_tx_ok = 0;
        meas_nb_tx_fail = 0;
        meas_nb_tx_requested = 0;
        meas_nb_tx_rejected_collision_packet = 0;
        meas_nb_tx_rejected_collision_beacon = 0;
        meas_nb_tx_rejected_too_late = 0;
        meas_nb_tx_rejected_too_early = 0;
        meas_nb_beacon_queued = 0;
        meas_nb_beacon_sent = 0;
        meas_nb_beacon_rejected = 0;
        pthread_mutex_unlock(&mx_meas_dw);
        if (cp_dw_pull_sent > 0) {
            dw_ack_ratio = (float)cp_dw_ack_rcv / (float)cp_dw_pull_sent;
        } else {
            dw_ack_ratio = 0.0f;
        }
#ifdef _ALI_LINKWAN_        
        // Begin add for get cpu/mem used ratio
        get_mem_ratio(&mem_ratio);

        get_cpu_ratio(&cpu_ratio);
        // End
#endif
        /* access GPS statistics, copy them */
        if (gps_enabled == true) {
            pthread_mutex_lock(&mx_meas_gps);
            coord_ok = gps_coord_valid;
            cp_gps_coord = meas_gps_coord;
            pthread_mutex_unlock(&mx_meas_gps);
        }

        /* overwrite with reference coordinates if function is enabled */
        if (gps_fake_enable == true) {
            cp_gps_coord = reference_coord;
        }

        /* display a report */
        MSG(LOG_NOTICE,"\n##### %s #####\n", stat_timestamp);
        MSG(LOG_NOTICE,"### [UPSTREAM] ###\n");
        MSG(LOG_NOTICE,"# RF packets received by concentrator: %u\n", cp_nb_rx_rcv);
        MSG(LOG_NOTICE,"# CRC_OK: %.2f, CRC_FAIL: %.2f, NO_CRC: %.2f\n", 100.0 * rx_ok_ratio, 100.0 * rx_bad_ratio, 100.0 * rx_nocrc_ratio);
        MSG(LOG_NOTICE,"# RF packets forwarded: %u (%u bytes)\n", cp_up_pkt_fwd, cp_up_payload_byte);
        MSG(LOG_NOTICE,"# PUSH_DATA datagrams sent: %u (%u bytes)\n", cp_up_dgram_sent, cp_up_network_byte);
        MSG(LOG_NOTICE,"# PUSH_DATA acknowledged: %.2f\n", 100.0 * up_ack_ratio);
        MSG(LOG_NOTICE,"### [DOWNSTREAM] ###\n");
        MSG(LOG_NOTICE,"# PULL_DATA sent: %u (%.2f acknowledged)\n", cp_dw_pull_sent, 100.0f * dw_ack_ratio);
        MSG(LOG_NOTICE,"# PULL_RESP(onse) datagrams received: %u (%u bytes)\n", cp_dw_dgram_rcv, cp_dw_network_byte);
        MSG(LOG_NOTICE,"# RF packets sent to concentrator: %u (%u bytes)\n", (cp_nb_tx_ok+cp_nb_tx_fail), cp_dw_payload_byte);
        MSG(LOG_NOTICE,"# TX errors: %u\n", cp_nb_tx_fail);
        if (cp_nb_tx_requested != 0 ) {
            MSG(LOG_NOTICE,"# TX rejected (collision packet): %.2f (req:%u, rej:%u)\n", 100.0 * cp_nb_tx_rejected_collision_packet / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_collision_packet);
            MSG(LOG_NOTICE,"# TX rejected (collision beacon): %.2f (req:%u, rej:%u)\n", 100.0 * cp_nb_tx_rejected_collision_beacon / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_collision_beacon);
            MSG(LOG_NOTICE,"# TX rejected (too late): %.2f (req:%u, rej:%u)\n", 100.0 * cp_nb_tx_rejected_too_late / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_too_late);
            MSG(LOG_NOTICE,"# TX rejected (too early): %.2f (req:%u, rej:%u)\n", 100.0 * cp_nb_tx_rejected_too_early / cp_nb_tx_requested, cp_nb_tx_requested, cp_nb_tx_rejected_too_early);
        }
        MSG(LOG_NOTICE,"# BEACON queued: %u\n", cp_nb_beacon_queued);
        MSG(LOG_NOTICE,"# BEACON sent so far: %u\n", cp_nb_beacon_sent);
        MSG(LOG_NOTICE,"# BEACON rejected: %u\n", cp_nb_beacon_rejected);
        MSG(LOG_NOTICE,"### [JIT] ###\n");
        /* get timestamp captured on PPM pulse  */

        /* Downlink for SX1276 now */
        for( idx = 0; idx < SUPPORT_SX1276_MAX; idx++){
            if( NULL == g_ctx_sx1276_arr[idx] )
                break;
            pthread_mutex_lock(&mx_concent);
            trig_tstamp = lgw_uart_read_timer(g_ctx_sx1276_arr[idx]->uart);
            pthread_mutex_unlock(&mx_concent);
            MSG(LOG_NOTICE,"# SX1276 time (PPS): %u, offset us: %d\n", trig_tstamp, g_ctx_sx1276_arr[idx]->offset_count_us);
        }

        for( idx = 0; idx < SUPPORT_SX1276_MAX; idx++ ){
            jit_print_queue (&jit_queue[idx], false, DEBUG_LOG); 
        }

        MSG(LOG_NOTICE,"### [GPS] ###\n");
        if (gps_enabled == true) {
            /* no need for mutex, display is not critical */
            if (gps_ref_valid == true) {
                MSG(LOG_NOTICE,"# Valid time reference (age: %li sec)\n", (long)difftime(time(NULL), time_reference_gps.systime));
            } else {
                MSG(LOG_NOTICE,"# Invalid time reference (age: %li sec)\n", (long)difftime(time(NULL), time_reference_gps.systime));
            }
            if (coord_ok == true) {
                MSG(LOG_NOTICE,"# GPS coordinates: latitude %.5f, longitude %.5f, altitude %i m\n", cp_gps_coord.lat, cp_gps_coord.lon, cp_gps_coord.alt);
            } else {
                MSG(LOG_NOTICE,"# no valid GPS coordinates available yet\n");
            }
        } else if (gps_fake_enable == true) {
            MSG(LOG_NOTICE,"# GPS *FAKE* coordinates: latitude %.5f, longitude %.5f, altitude %i m\n", cp_gps_coord.lat, cp_gps_coord.lon, cp_gps_coord.alt);
        } else {
            MSG(LOG_NOTICE,"# GPS sync is disabled\n");
        }
        
        MSG(LOG_NOTICE,"##### END #####\n");

        /* generate a JSON report (will be sent to server by upstream thread) */
        pthread_mutex_lock(&mx_stat_rep);
        if (((gps_enabled == true) && (coord_ok == true)) || (gps_fake_enable == true)) {
            snprintf(status_report, STATUS_SIZE, "\"stat\":{\"time\":\"%s\",\"lati\":%.5f,\"long\":%.5f,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%.1f,\"dwnb\":%u,\"txnb\":%u,\"cpur\":%.1f,\"memr\":%.1f}", stat_timestamp, cp_gps_coord.lat, cp_gps_coord.lon, cp_gps_coord.alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 100.0 * up_ack_ratio, cp_dw_dgram_rcv, cp_nb_tx_ok, 100.0 * cpu_ratio, 100.0 * mem_ratio);
        } else {
            snprintf(status_report, STATUS_SIZE, "\"stat\":{\"time\":\"%s\",\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%.1f,\"dwnb\":%u,\"txnb\":%u,\"cpur\":%.1f,\"memr\":%.1f}", stat_timestamp, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 100.0 * up_ack_ratio, cp_dw_dgram_rcv, cp_nb_tx_ok, 100.0 * cpu_ratio, 100.0 * mem_ratio);
        }
        report_ready = true;
        pthread_mutex_unlock(&mx_stat_rep);
#ifdef _ALI_LINKWAN_
        /* Begin add for reset when no ack in specify time */
        pthread_mutex_lock(&mx_stat_no_ack);
        cp_stat_no_ack_cnt = stat_no_ack_cnt;
        pthread_mutex_unlock(&mx_stat_no_ack);
        if (cp_stat_no_ack_cnt > stat_no_ack_timeout) {
            exit_sig = true;
            need_reset = true;
            MSG(LOG_INFO,"INFO: status pkt no ack count:%u up to timeout:%u\n", cp_stat_no_ack_cnt, stat_no_ack_timeout);
        }
        /* End */
#endif        
    }

    /* wait for upstream thread to finish (1 fetch cycle max) */
    pthread_join(thrid_up, NULL);
    pthread_cancel(thrid_down); /* don't wait for downstream thread */
    pthread_cancel(thrid_jit); /* don't wait for jit thread */
    pthread_cancel(thrid_timersync); /* don't wait for timer sync thread */
    if (gps_enabled == true) {
        pthread_cancel(thrid_gps); /* don't wait for GPS thread */
        pthread_cancel(thrid_valid); /* don't wait for validation thread */

        i = lgw_gps_disable(gps_tty_fd);
        if (i == LGW_HAL_SUCCESS) {
            MSG(LOG_NOTICE,"INFO: GPS closed successfully\n");
        } else {
            MSG(LOG_WARNING,"WARNING: failed to close GPS successfully\n");
        }
    }

    /* if an exit signal was received, try to quit properly */
    if (exit_sig) {
        /* shut down network sockets */
        shutdown(sock_up, SHUT_RDWR);
        shutdown(sock_down, SHUT_RDWR);
        /* stop the hardware */
        for(idx = 0; idx < SUPPORT_SX1301_MAX; idx++){
            if( NULL == g_ctx_arr[idx])
                break;
            i = lgw_stop(g_ctx_arr[idx]);
            lora_led_off(idx);
            if (i == LGW_HAL_SUCCESS) {
                MSG(LOG_NOTICE,"INFO: concentrator stopped successfully\n");
            } else {
                MSG(LOG_WARNING,"WARNING: failed to stop concentrator successfully\n");
            }
        }
    }

    
    
        /* Begin add for packet filtering by whitelist and blacklist */
#if defined(USE_FILTER_NODE)
    if (1 == filter_inited) {
        filter_deinit();
    }
#endif
        /* End */

#ifdef _ALI_LINKWAN_
    /* Begin add for reset when no ack in specify time */
    if (need_reset == true) {
        sync();
        MSG(LOG_NOTICE,"INFO: no ack in specify time, need reset now\n");
        wait_ms(1000 * 5);
        system("reboot");
    }
    /* End */
#endif
    MSG(LOG_NOTICE,"INFO: Exiting packet forwarder program\n");
    
    if( exit_err )
        exit(EXIT_FAILURE);
    else
        exit(EXIT_SUCCESS);
}



void hex_dump( uint8_t * buff, uint16_t size){
    char s[256] = {0};
    int i, len;

    if( foreground == false && LOG_DEBUG > g_debug_level ){
        return;   
    }    
    
    len = 0;
    
    for( i = 1; i < size + 1; i++ ){
       len +=snprintf(s + len, sizeof(s) - len, "%02X ", buff[i-1]);
       if( (i % 8) == 0 || i == size ){
            MSG(LOG_DEBUG,"\t%s", s);
            len = 0;
       }
    }

    return;
}

#define MTYPE_JOIN_REQUEST          0
#define MTYPE_JOIN_ACCEPT           1
#define MTYPE_UNCONFIRM_DATA_UP     2
#define MTYPE_UNCONFIRM_DATA_DOWN   3
#define MTYPE_CONFIRM_DATA_UP       4
#define MTYPE_CONFIRM_DATA_DOWN     5
#define MTYPE_RFU                   6
#define MTYPE_PROPRITARY            7

#if 0
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static uint32_t rx_time_on_air( struct lgw_pkt_rx_s * packet, uint8_t fsk_sync_word_size){
    int32_t val;
    uint8_t SF, H, DE;
    uint16_t BW;
    uint32_t payloadSymbNb, Tpacket;
    double Tsym, Tpreamble, Tpayload, Tfsk;

    if (packet == NULL) {
        MSG(LOG_INFO,"ERROR: Failed to compute time on air, wrong parameter\n");
        return 0;
    }

    if (packet->modulation == MOD_LORA) {
        /* Get bandwidth */
        val = lgw_bw_getval(packet->bandwidth);
        if (val != -1) {
            BW = (uint16_t)(val / 1E3);
        } else {
            MSG(LOG_INFO,"ERROR: Cannot compute time on air for this packet, unsupported bandwidth (0x%02X)\n", packet->bandwidth);
            return 0;
        }

        /* Get datarate */
        val = lgw_sf_getval(packet->datarate);
        if (val != -1) {
            SF = (uint8_t)val;
        } else {
            MSG(LOG_INFO,"ERROR: Cannot compute time on air for this packet, unsupported datarate (0x%02X)\n", packet->datarate);
            return 0;
        }

        /* Duration of 1 symbol */
        Tsym = pow(2, SF) / BW;

        /* Duration of preamble */
        Tpreamble = ((double)(STD_LORA_PREAMB) + 4.25) * Tsym;

        /* Duration of payload */
        H =  1; /* header is always enabled, except for beacons */
        DE = (SF >= 11) ? 1 : 0; /* Low datarate optimization enabled for SF11 and SF12 */

        payloadSymbNb = 8 + (ceil((double)(8*packet->size - 4*SF + 28 + 16 - 20*H) / (double)(4*(SF - 2*DE))) * (packet->coderate + 4)); /* Explicitely cast to double to keep precision of the division */

        Tpayload = payloadSymbNb * Tsym;

        /* Duration of packet */
        Tpacket = Tpreamble + Tpayload;
    } else if (packet->modulation == MOD_FSK) {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        Tfsk = (8 * (double)(STD_FSK_PREAMB + fsk_sync_word_size + 1 + packet->size + 2) / (double)packet->datarate) * 1E3;

        /* Duration of packet */
        Tpacket = (uint32_t)Tfsk + 1; /* add margin for rounding */
    } else {
        Tpacket = 0;
        MSG(LOG_INFO,"ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    return Tpacket;
}
#endif
/* -------------------------------------------------------------------------- */
/* --- THREAD 1: RECEIVING PACKETS AND FORWARDING THEM ---------------------- */
struct lgw_recev_pkts {
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX];
    int nb_pkt;
};

extern pthread_mutex_t mx_rrd;

void thread_led(void){

    int i = 0;
    bool idx[SUPPORT_SX1301_MAX] = {false};

    while (!exit_sig && !quit_sig) {
        for( i = 0; i < SUPPORT_SX1301_MAX; i++ ){
            if( g_led_arr[i].trigger ){
                lora_led_off(i);
                g_led_arr[i].trigger = false;
                idx[i] = true;
            }
        }
        
        wait_ms(70);    
        for( i = 0; i < SUPPORT_SX1301_MAX; i++ ){
            if( idx[i] ){
                lora_led_on(i);
            }
        }
        wait_ms(30);
    }
}

void thread_up(void) {
    int i, j, n; /* loop variables */
    unsigned pkt_in_dgram; /* nb on Lora packet in the current datagram */
    struct lgw_recev_pkts  ctx_pkts[SUPPORT_SX1301_MAX]; // add 1 for fbuffer
    struct lgw_recev_pkts  buffer_pkts;
    uint8_t MType = 0;
    /* allocate memory for packet fetching and processing */
    struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
    int nb_pkt;
    /* local copy of GPS time reference */
    bool ref_ok = false; /* determine if GPS time reference must be used or not */
    struct tref local_ref; /* time reference used for UTC <-> timestamp conversion */
    int ret = 0;
    
    /* data buffers */
    uint8_t buff_up[TX_BUFF_SIZE]; /* buffer to compose the upstream packet */
    
    
    int buff_index;
#ifndef _ALI_LINKWAN_
    uint8_t buff_ack[32]; /* buffer to receive acknowledges */
#endif

    /* protocol variables */
    uint8_t token_h; /* random token for acknowledgement matching */
    uint8_t token_l; /* random token for acknowledgement matching */
    /* ping measurement variables */
    struct timespec send_time;

    /* GPS synchronization variables */
    struct timespec pkt_utc_time;
    struct tm * x; /* broken-up UTC time */
    struct timespec pkt_gps_time;
    uint64_t pkt_gps_time_ms;

    /* report management variable */
    bool send_report = false;

    
#ifndef _ALI_LINKWAN_    
    struct timespec recv_time; 
#endif    

    if( data_recovery ){
        fbuff_init(data_recovery_path);
    }
    
    /* pre-fill the data buffer with fixed fields */
    buff_up[0] = PROTOCOL_VERSION;
    buff_up[3] = PKT_PUSH_DATA;
    *(uint32_t *)(buff_up + 4) = net_mac_h;
    *(uint32_t *)(buff_up + 8) = net_mac_l;

    while (!exit_sig && !quit_sig) {
        bool network_st = false;

        pthread_mutex_lock( &mx_network_err );
        network_st = status_network_connect;
        pthread_mutex_unlock( &mx_network_err );
        
        /* fetch packets */   
        nb_pkt = 0;
        for( i = 0; i < SUPPORT_SX1301_MAX; i++){
            if( NULL == g_ctx_arr[i] )
                break;
            pthread_mutex_lock(&mx_concent);
            ret = lgw_receive(NB_PKT_MAX, ctx_pkts[i].rxpkt, g_ctx_arr[i]);
            pthread_mutex_unlock(&mx_concent);
            if( LGW_HAL_ERROR == ret ){
                MSG(LOG_CRIT,"ERROR: [up] failed packet fetch, exiting\n");
                exit(EXIT_FAILURE);
            }

            ctx_pkts[i].nb_pkt = ret;
            nb_pkt += ret;
        }

        if( sock_up > 0 && network_st == true ){
            if( data_recovery){
                buffer_pkts.nb_pkt = fbuff_dequeue(buffer_pkts.rxpkt, NB_PKT_MAX);
                nb_pkt += buffer_pkts.nb_pkt;
            }
        }
        else{
            buffer_pkts.nb_pkt = 0;
        }
        /* check if there are status report to send */
        send_report = report_ready; /* copy the variable so it doesn't change mid-function */
        /* no mutex, we're only reading */

        
        /* wait a short time if no packets, nor status report */
        if ((nb_pkt == 0) && (send_report == false)) {
            wait_ms(FETCH_SLEEP_MS);
            continue;
        }

        if(  sock_up <= 0 || network_st == false ){
            if( data_recovery ){
                for( i = 0; i < SUPPORT_SX1301_MAX; i++){
                    if( ctx_pkts[i].nb_pkt > 0)
                        fbuff_enqueue(ctx_pkts[i].rxpkt, ctx_pkts[i].nb_pkt);
                }
            }

            if( sock_up <= 0 || data_recovery ){
                if(send_report){
                    pthread_mutex_lock(&mx_stat_rep);
                    report_ready = false;
                    pthread_mutex_unlock(&mx_stat_rep);
                   
                }

                 continue;
            }
        }
        
        
        /* get a copy of GPS time reference (avoid 1 mutex per packet) */
        if ((nb_pkt > 0) && (gps_enabled == true)) {
            pthread_mutex_lock(&mx_timeref);
            ref_ok = gps_ref_valid;
            local_ref = time_reference_gps;
            pthread_mutex_unlock(&mx_timeref);
        } else {
            ref_ok = false;
        }

        /* start composing datagram with the header */
        token_h = (uint8_t)rand(); /* random token */
        token_l = (uint8_t)rand(); /* random token */
        buff_up[1] = token_h;
        buff_up[2] = token_l;
        buff_index = 12; /* 12-byte header */

        /* start of JSON structure */
        memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
        buff_index += 9;


        /* End */
        /* serialize Lora packets metadata and payload */
        pkt_in_dgram = 0;

        for( n = 0; n < SUPPORT_SX1301_MAX; n++ ){
            if( g_ctx_arr[n] == NULL )
                break;
            if( ctx_pkts[n].nb_pkt == 0 )
                continue;

            lora_led_trigger(n);
            for (i=0; i < ctx_pkts[n].nb_pkt; ++i) {
                           
                p = &(ctx_pkts[n].rxpkt[i]);

                MType = ( p->payload[0] & 0xE0 ) >> 5;

                if( is_lorawan ){
                    if( MType == MTYPE_CONFIRM_DATA_DOWN || MType == MTYPE_UNCONFIRM_DATA_DOWN || MType == MTYPE_JOIN_ACCEPT){
                        //MSG(LOG_WARNING, "[up] Receive Downlink Packet! Drop it");
                        continue;
                    }
                }
                
                

/* Begin add for packet filtering by whitelist and blacklist */
#if defined(USE_FILTER_NODE)
                if ( is_lorawan && (1 == filter_inited) && (STAT_CRC_OK == p->status  || STAT_NO_CRC == p->status)) {
                    j = filter_up_proc(p->payload, p->size);
                    if (1 != j) {
                        uint32_t mote_addr = 0;
                        mote_addr  = p->payload[1];
                        mote_addr |= p->payload[2] << 8;
                        mote_addr |= p->payload[3] << 16;
                        mote_addr |= p->payload[4] << 24;
                        MSG(LOG_INFO,"INFO: [up] the pkt from mote: %08X discard by packet filter\n", mote_addr);
                        if( g_packet_table.enable ){
                            logger_packet_add_up(p, TYPE_FILTERED);
                        }
                        continue; /* discard packet */
                    }
                }
#endif

                if( g_packet_table.enable ){
                    logger_packet_add_up(p, TYPE_NORMAL);
                }
                /* Get mote information from current packet (addr, fcnt) */


                /* basic packet filtering */
                pthread_mutex_lock(&mx_meas_up);
                meas_nb_rx_rcv += 1;
                switch(p->status) {
                    case STAT_CRC_OK:
                        meas_nb_rx_ok += 1;
                        meas_g_nb_rx_ok += 1;
                        if (!fwd_valid_pkt) {
                            pthread_mutex_unlock(&mx_meas_up);
                            continue; /* skip that packet */
                        }
                        break;
                    case STAT_CRC_BAD:
                        MSG(LOG_INFO, "INFO: Received pkt CRC BAD\n" );
                        meas_nb_rx_bad += 1;
                        meas_g_nb_rx_bad += 1;
                        if (!fwd_error_pkt) {
                            pthread_mutex_unlock(&mx_meas_up);
                            continue; /* skip that packet */
                        }
                        hex_dump(p->payload, p->size);
                        break;
                    case STAT_NO_CRC:
                        
                        meas_nb_rx_nocrc += 1;
                        meas_g_nb_rx_nocrc +=1;
                        MSG(LOG_INFO, "INFO: Received pkt NO CRC\n" );
                        
                        if (!fwd_nocrc_pkt) {                            
                            pthread_mutex_unlock(&mx_meas_up);
                            continue; /* skip that packet */
                        }
                        
                        hex_dump(p->payload, p->size);
                        break;
                    default:
                        pthread_mutex_unlock(&mx_meas_up);
                        MSG(LOG_WARNING,"WARNING: [up] received packet with unknown status %u (size %u, modulation %u, BW %u, DR %u, RSSI %.1f)\n", p->status, p->size, p->modulation, p->bandwidth, p->datarate, p->rssi);
                        //hex_dump(p->payload, p->size);
                        
                        continue; /* skip that packet */
                        // exit(EXIT_FAILURE);
                }
                meas_up_pkt_fwd += 1;
                meas_up_payload_byte += p->size;
                pthread_mutex_unlock(&mx_meas_up);

                
                MSG(LOG_DEBUG, "Uplink Frame : " );
                hex_dump(p->payload, p->size);

                /* Start of packet, add inter-packet separator if necessary */
                if (pkt_in_dgram == 0) {
                    buff_up[buff_index] = '{';
                    
                    ++buff_index;
                    
                } else {
                    buff_up[buff_index] = ',';
                    buff_up[buff_index+1] = '{';
                 
                    
                    buff_index += 2;
                }
                
                p->count_us += (g_ctx_sx1276_arr[n]->offset_count_us);
                
                p->rf_chain += (n * 2);

                /* RAW timestamp, 8-17 useful chars */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%u", p->count_us);

                if (j > 0) {
                    buff_index += j;
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                    exit(EXIT_FAILURE);
                }

                /* Packet RX time (GPS based), 37 useful chars */
                if (ref_ok == true) {
                    /* convert packet timestamp to UTC absolute time */
                    j = lgw_cnt2utc(local_ref, p->count_us, &pkt_utc_time);
                    if (j == LGW_GPS_SUCCESS) {
                        /* split the UNIX timestamp to its calendar components */
                        x = gmtime(&(pkt_utc_time.tv_sec));
                        j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"time\":\"%04i-%02i-%02iT%02i:%02i:%02i.%06liZ\"", (x->tm_year)+1900, (x->tm_mon)+1, x->tm_mday, x->tm_hour, x->tm_min, x->tm_sec, (pkt_utc_time.tv_nsec)/1000); /* ISO 8601 format */
                        if (j > 0) {
                            buff_index += j;
                        } else {
                            MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                            exit(EXIT_FAILURE);
                        }
                    }
                    /* convert packet timestamp to GPS absolute time */
                    j = lgw_cnt2gps(local_ref, p->count_us, &pkt_gps_time);
                    if (j == LGW_GPS_SUCCESS) {
                        pkt_gps_time_ms = pkt_gps_time.tv_sec * 1E3 + pkt_gps_time.tv_nsec / 1E6;
                        j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"tmms\":%llu",
                                        pkt_gps_time_ms); /* GPS time in milliseconds since 06.Jan.1980 */
                        if (j > 0) {
                            buff_index += j;
                        } else {
                            MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                            exit(EXIT_FAILURE);
                        }
                    }
                }

                /* Packet concentrator channel, RF chain & RX frequency, 34-36 useful chars */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", p->if_chain, p->rf_chain, ((double)p->freq_hz / 1e6));
                if (j > 0) {
                    buff_index += j;
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                    exit(EXIT_FAILURE);
                }

                
                /* Packet status, 9-10 useful chars */
                switch (p->status) {
                    case STAT_CRC_OK:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
                        buff_index += 9;
                        break;
                    case STAT_CRC_BAD:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":-1", 10);
                        buff_index += 10;
                        break;
                    case STAT_NO_CRC:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":0", 9);
                        buff_index += 9;
                        break;
                    default:
                        MSG(LOG_CRIT,"ERROR: [up] received packet with unknown status\n");
                        memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":?", 9);
                        buff_index += 9;
                        exit(EXIT_FAILURE);
                }

                /* Packet modulation, 13-14 useful chars */
                if (p->modulation == MOD_LORA) {
                    memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
                    buff_index += 14;

                    /* Lora datarate & bandwidth, 16-19 useful chars */
                    switch (p->datarate) {
                        case DR_LORA_SF7:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                            buff_index += 12;
                            break;
                        case DR_LORA_SF8:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                            buff_index += 12;
                            break;
                        case DR_LORA_SF9:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                            buff_index += 12;
                            break;
                        case DR_LORA_SF10:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                            buff_index += 13;
                            break;
                        case DR_LORA_SF11:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                            buff_index += 13;
                            break;
                        case DR_LORA_SF12:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                            buff_index += 13;
                            break;
                        default:
                            MSG(LOG_CRIT,"ERROR: [up] lora packet with unknown datarate\n");
                            memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                            buff_index += 12;
                            exit(EXIT_FAILURE);
                    }
                    switch (p->bandwidth) {
                        case BW_125KHZ:
                            memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
                            buff_index += 6;
                            break;
                        case BW_250KHZ:
                            memcpy((void *)(buff_up + buff_index), (void *)"BW250\"", 6);
                            buff_index += 6;
                            break;
                        case BW_500KHZ:
                            memcpy((void *)(buff_up + buff_index), (void *)"BW500\"", 6);
                            buff_index += 6;
                            break;
                        default:
                            MSG(LOG_CRIT,"ERROR: [up] lora packet with unknown bandwidth\n");
                            memcpy((void *)(buff_up + buff_index), (void *)"BW?\"", 4);
                            buff_index += 4;
                            exit(EXIT_FAILURE);
                    }

                    /* Packet ECC coding rate, 11-13 useful chars */
                    switch (p->coderate) {
                        case CR_LORA_4_5:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
                            buff_index += 13;
                            break;
                        case CR_LORA_4_6:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/6\"", 13);
                            buff_index += 13;
                            break;
                        case CR_LORA_4_7:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/7\"", 13);
                            buff_index += 13;
                            break;
                        case CR_LORA_4_8:
                            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/8\"", 13);
                            buff_index += 13;
                            break;
                        case 0: /* treat the CR0 case (mostly false sync) */
                            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"OFF\"", 13);
                            buff_index += 13;
                            break;
                        default:
                            MSG(LOG_CRIT,"ERROR: [up] lora packet with unknown coderate\n");
                            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"?\"", 11);
                            buff_index += 11;
                            exit(EXIT_FAILURE);
                    }

                    /* Lora SNR, 11-13 useful chars */
                    j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%.1f", p->snr);
                    if (j > 0) {
                        buff_index += j;
                    } else {
                        MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                        exit(EXIT_FAILURE);
                    }
                } else if (p->modulation == MOD_FSK) {
                    memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"FSK\"", 13);
                    buff_index += 13;

                    /* FSK datarate, 11-14 useful chars */
                    j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"datr\":%u", p->datarate);
                    if (j > 0) {
                        buff_index += j;
                    } else {
                        MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                        exit(EXIT_FAILURE);
                    }
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] received packet with unknown modulation\n");
                    exit(EXIT_FAILURE);
                }

                /* Packet RSSI, payload size, 18-23 useful chars */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%.0f,\"size\":%u", p->rssi, p->size);
                if (j > 0) {
                    buff_index += j;
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                    exit(EXIT_FAILURE);
                }

                /* Packet base64-encoded payload, 14-350 useful chars */
                memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
                buff_index += 9;
                j = bin_to_b64(p->payload, p->size, (char *)(buff_up + buff_index), 341); /* 255 bytes = 340 chars in b64 + null char */
                if (j>=0) {
                    buff_index += j;
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] bin_to_b64 failed line %u\n", (__LINE__ - 5));
                    exit(EXIT_FAILURE);
                }
                buff_up[buff_index] = '"';
                ++buff_index;

                /* End of packet serialization */
                buff_up[buff_index] = '}';
                ++buff_index;

                buff_up[buff_index] = 0;
                ++pkt_in_dgram;

                rrd_statistic_up(p, n);
            }
        }
        
        /* buffer packets */
        for(i = 0; i < buffer_pkts.nb_pkt; i++ ){
            p = &(buffer_pkts.rxpkt[i]);

            if( g_packet_table.enable ){
                logger_packet_add_up(p, TYPE_OUT_BUFFER);
            }
            /* Start of packet, add inter-packet separator if necessary */
            if (pkt_in_dgram == 0) {
                buff_up[buff_index] = '{';
                
                ++buff_index;
                
            } else {
                buff_up[buff_index] = ',';
                buff_up[buff_index+1] = '{';
             
                
                buff_index += 2;
            }
            
            
            /* RAW timestamp, 8-17 useful chars */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%u", p->count_us);

            if (j > 0) {
                buff_index += j;
            } else {
                MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                exit(EXIT_FAILURE);
            }

            

            /* Packet concentrator channel, RF chain & RX frequency, 34-36 useful chars */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", p->if_chain, p->rf_chain, ((double)p->freq_hz / 1e6));
            if (j > 0) {
                buff_index += j;
            } else {
                MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                exit(EXIT_FAILURE);
            }

            
            /* Packet status, 9-10 useful chars */
            switch (p->status) {
                case STAT_CRC_OK:
                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
                    buff_index += 9;
                    break;
                case STAT_CRC_BAD:
                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":-1", 10);
                    buff_index += 10;
                    break;
                case STAT_NO_CRC:
                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":0", 9);
                    buff_index += 9;
                    break;
                default:
                    MSG(LOG_CRIT,"ERROR: [up] received packet with unknown status\n");
                    memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":?", 9);
                    buff_index += 9;
                    exit(EXIT_FAILURE);
            }

            /* Packet modulation, 13-14 useful chars */
            if (p->modulation == MOD_LORA) {
                memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
                buff_index += 14;

                /* Lora datarate & bandwidth, 16-19 useful chars */
                switch (p->datarate) {
                    case DR_LORA_SF7:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                        buff_index += 12;
                        break;
                    case DR_LORA_SF8:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                        buff_index += 12;
                        break;
                    case DR_LORA_SF9:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                        buff_index += 12;
                        break;
                    case DR_LORA_SF10:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                        buff_index += 13;
                        break;
                    case DR_LORA_SF11:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                        buff_index += 13;
                        break;
                    case DR_LORA_SF12:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                        buff_index += 13;
                        break;
                    default:
                        MSG(LOG_CRIT,"ERROR: [up] lora packet with unknown datarate\n");
                        memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                        buff_index += 12;
                        exit(EXIT_FAILURE);
                }
                switch (p->bandwidth) {
                    case BW_125KHZ:
                        memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
                        buff_index += 6;
                        break;
                    case BW_250KHZ:
                        memcpy((void *)(buff_up + buff_index), (void *)"BW250\"", 6);
                        buff_index += 6;
                        break;
                    case BW_500KHZ:
                        memcpy((void *)(buff_up + buff_index), (void *)"BW500\"", 6);
                        buff_index += 6;
                        break;
                    default:
                        MSG(LOG_CRIT,"ERROR: [up] lora packet with unknown bandwidth\n");
                        memcpy((void *)(buff_up + buff_index), (void *)"BW?\"", 4);
                        buff_index += 4;
                        exit(EXIT_FAILURE);
                }

                /* Packet ECC coding rate, 11-13 useful chars */
                switch (p->coderate) {
                    case CR_LORA_4_5:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
                        buff_index += 13;
                        break;
                    case CR_LORA_4_6:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/6\"", 13);
                        buff_index += 13;
                        break;
                    case CR_LORA_4_7:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/7\"", 13);
                        buff_index += 13;
                        break;
                    case CR_LORA_4_8:
                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/8\"", 13);
                        buff_index += 13;
                        break;
                    case 0: /* treat the CR0 case (mostly false sync) */
                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"OFF\"", 13);
                        buff_index += 13;
                        break;
                    default:
                        MSG(LOG_CRIT,"ERROR: [up] lora packet with unknown coderate\n");
                        memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"?\"", 11);
                        buff_index += 11;
                        exit(EXIT_FAILURE);
                }

                /* Lora SNR, 11-13 useful chars */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%.1f", p->snr);
                if (j > 0) {
                    buff_index += j;
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                    exit(EXIT_FAILURE);
                }
            } else if (p->modulation == MOD_FSK) {
                memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"FSK\"", 13);
                buff_index += 13;

                /* FSK datarate, 11-14 useful chars */
                j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"datr\":%u", p->datarate);
                if (j > 0) {
                    buff_index += j;
                } else {
                    MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                    exit(EXIT_FAILURE);
                }
            } else {
                MSG(LOG_CRIT,"ERROR: [up] received packet with unknown modulation\n");
                exit(EXIT_FAILURE);
            }

            /* Packet RSSI, payload size, 18-23 useful chars */
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%.0f,\"size\":%u", p->rssi, p->size);
            if (j > 0) {
                buff_index += j;
            } else {
                MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 4));
                exit(EXIT_FAILURE);
            }

            /* Packet base64-encoded payload, 14-350 useful chars */
            memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
            buff_index += 9;
            j = bin_to_b64(p->payload, p->size, (char *)(buff_up + buff_index), 341); /* 255 bytes = 340 chars in b64 + null char */
            if (j>=0) {
                buff_index += j;
            } else {
                MSG(LOG_CRIT,"ERROR: [up] bin_to_b64 failed line %u\n", (__LINE__ - 5));
                exit(EXIT_FAILURE);
            }
            buff_up[buff_index] = '"';
            ++buff_index;

            /* End of packet serialization */
            buff_up[buff_index] = '}';
            ++buff_index;

            buff_up[buff_index] = 0;
            ++pkt_in_dgram;
        }
        
        /* restart fetch sequence without sending empty JSON if all packets have been filtered out */
        if (pkt_in_dgram == 0) {
            if (send_report == true) {
                /* need to clean up the beginning of the payload */
                buff_index -= 8; /* removes "rxpk":[ */
            } else {
                /* all packet have been filtered out and no report, restart loop */
                continue;
            }
        } else {
            /* end of packet array */
            buff_up[buff_index] = ']';
            ++buff_index;
            /* add separator if needed */
            if (send_report == true) {
                buff_up[buff_index] = ',';
                ++buff_index;
            }
        }

        /* add status report if a new one is available */
        if (send_report == true) {
            pthread_mutex_lock(&mx_stat_rep);
            report_ready = false;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "%s", status_report);
            pthread_mutex_unlock(&mx_stat_rep);
            if (j > 0) {
                buff_index += j;
            } else {
                MSG(LOG_CRIT,"ERROR: [up] snprintf failed line %u\n", (__LINE__ - 5));
                exit(EXIT_FAILURE);
            }
#ifdef _ALI_LINKWAN_
            /* Begin add for reset when no ack in specify time */
            pthread_mutex_lock(&mx_stat_no_ack);
            stat_no_ack_cnt++;
            pthread_mutex_unlock(&mx_stat_no_ack);
            MSG(LOG_INFO,"INFO: [up] status pkt no ack count: %u\n", stat_no_ack_cnt);
            /* End */
#endif
        }

        /* end of JSON datagram payload */
        buff_up[buff_index] = '}';
        ++buff_index;
        buff_up[buff_index] = 0; /* add string terminator, for safety */

        
        MSG(LOG_INFO,"\nJSON up: %s\n", (char *)(buff_up + 12)); /* DEBUG: display JSON payload */
        
        /* send datagram to server */
        send(sock_up, (void *)buff_up, buff_index, 0);
        
        clock_gettime(CLOCK_MONOTONIC, &send_time);
        pthread_mutex_lock(&mx_meas_up);
        meas_up_dgram_sent += 1;
        meas_up_network_byte += buff_index;
        pthread_mutex_unlock(&mx_meas_up);
        
#ifdef _ALI_LINKWAN_
        //Begin add for adapt iot lora sdk
        push_ack_token_h = token_h;
        push_ack_token_l = token_l;
#else

        /* wait for acknowledge (in 2 times, to catch extra packets) */
        for (i=0; i<2; ++i) {
            j = recv(sock_up, (void *)buff_ack, sizeof buff_ack, 0);
            clock_gettime(CLOCK_MONOTONIC, &recv_time);
            if (j == -1) {
                if (errno == EAGAIN) { /* timeout */
                    if( i == 2 ){
                        MSG(LOG_INFO, "[up] push timeout");
                    }
                    continue;
                } else { /* server connection error */
                    MSG(LOG_INFO, "[up] server connect error");
                    break;
                }
            } else if ((j < 4) || (buff_ack[0] != PROTOCOL_VERSION) || (buff_ack[3] != PKT_PUSH_ACK)) {
                MSG(LOG_INFO,"WARNING: [up] ignored invalid non-ACL packet\n");
                continue;
            } else if ((buff_ack[1] != token_h) || (buff_ack[2] != token_l)) {
                MSG(LOG_INFO,"WARNING: [up] ignored out-of sync ACK packet\n");
                continue;
            } else {
                MSG(LOG_INFO,"INFO: [up] PUSH_ACK received in %i ms\n", (int)(1000 * difftimespec(recv_time, send_time)));
                pthread_mutex_lock(&mx_meas_up);
                meas_up_ack_rcv += 1;
                pthread_mutex_unlock(&mx_meas_up);
                
                if( buffer_pkts.nb_pkt > 0 ){
                    fbuff_drop(buffer_pkts.nb_pkt);
                }
                break;
            }
        }
           
#endif
/* End */        
        
    }
    if( data_recovery ){
        fbuff_deinit();
    }
    MSG(LOG_INFO,"\nINFO: End of upstream thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 2: POLLING SERVER AND ENQUEUING PACKETS IN JIT QUEUE ---------- */

void thread_down(void) {
    int i; /* loop variables */

    /* configuration and metadata for an outbound packet */
    struct lgw_pkt_tx_s txpkt;
    uint32_t o_count_us = 0;
    bool sent_immediate = false; /* option to sent the packet immediately */
    
    /* local timekeeping variables */
    struct timespec send_time; /* time of the pull request */
    struct timespec recv_time; /* time of return from recv socket call */

    /* data buffers */
    uint8_t buff_down[1000]; /* buffer to receive downstream packets */
    
#ifdef _ALI_LINKWAN_
    uint8_t buff_req[256]; /* buffer to compose pull requests */
#else
    uint8_t buff_req[12];  /* buffer to compose pull requests */
#endif

    int msg_len;

    /* protocol variables */
    uint8_t token_h; /* random token for acknowledgement matching */
    uint8_t token_l; /* random token for acknowledgement matching */
    bool req_ack = false; /* keep track of whether PULL_DATA was acknowledged or not */

    /* JSON parsing variables */
    JSON_Value *root_val = NULL;
    JSON_Object *txpk_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    short x0, x1;
    uint64_t x2;
    double x3, x4;

    int buff_index;

    /* variables to send on GPS timestamp */
    struct tref local_ref; /* time reference used for GPS <-> timestamp conversion */
    struct timespec gps_tx; /* GPS time that needs to be converted to timestamp */

    /* beacon variables */
    struct lgw_pkt_tx_s beacon_pkt;
    uint8_t beacon_chan;
    uint8_t beacon_loop;
    size_t beacon_RFU1_size = 0;
    size_t beacon_RFU2_size = 0;
    uint8_t beacon_pyld_idx = 0;
    time_t diff_beacon_time;
    struct timespec next_beacon_gps_time; /* gps time of next beacon packet */
    struct timespec last_beacon_gps_time; /* gps time of last enqueued beacon packet */
    int retry;

    /* beacon data fields, byte 0 is Least Significant Byte */
    int32_t field_latitude; /* 3 bytes, derived from reference latitude */
    int32_t field_longitude; /* 3 bytes, derived from reference longitude */
    uint16_t field_crc1, field_crc2;

    /* auto-quit variable */
    uint32_t autoquit_cnt = 0; /* count the number of PULL_DATA sent since the latest PULL_ACK */

    /* Just In Time downlink */
    struct timeval current_unix_time;
    struct timeval current_concentrator_time;
    enum jit_error_e jit_result = JIT_ERROR_OK;
    enum jit_pkt_type_e downlink_type;
    uint8_t target_rf_chain = 0;
    
    /* set downstream socket RX timeout */
    i = setsockopt(sock_down, SOL_SOCKET, SO_RCVTIMEO, (void *)&pull_timeout, sizeof pull_timeout);
    if (i != 0) {
        MSG(LOG_CRIT,"ERROR: [down] setsockopt returned %s\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* pre-fill the pull request buffer with fixed fields */
    buff_req[0] = PROTOCOL_VERSION;
    buff_req[3] = PKT_PULL_DATA;
    *(uint32_t *)(buff_req + 4) = net_mac_h;
    *(uint32_t *)(buff_req + 8) = net_mac_l;

    /* beacon variables initialization */
    last_beacon_gps_time.tv_sec = 0;
    last_beacon_gps_time.tv_nsec = 0;

    /* beacon packet parameters */
    beacon_pkt.tx_mode = ON_GPS; /* send on PPS pulse */
    beacon_pkt.rf_chain = 0; /* antenna A */
    beacon_pkt.rf_power = beacon_power;
    beacon_pkt.modulation = MOD_LORA;
    switch (beacon_bw_hz) {
        case 125000:
            beacon_pkt.bandwidth = BW_125KHZ;
            break;
        case 500000:
            beacon_pkt.bandwidth = BW_500KHZ;
            break;
        default:
            /* should not happen */
            MSG(LOG_CRIT,"ERROR: [down] unsupported bandwidth for beacon\n");
            exit(EXIT_FAILURE);
    }
    switch (beacon_datarate) {
        case 8:
            beacon_pkt.datarate = DR_LORA_SF8;
            beacon_RFU1_size = 1;
            beacon_RFU2_size = 3;
            break;
        case 9:
            beacon_pkt.datarate = DR_LORA_SF9;
#if 0            
            beacon_RFU1_size = 3;
            beacon_RFU2_size = 1;
#else
            beacon_RFU1_size = 2;
            beacon_RFU2_size = 0;
#endif
            break;
        case 10:
            beacon_pkt.datarate = DR_LORA_SF10;
            beacon_RFU1_size = 3;
            beacon_RFU2_size = 1;
            break;
        case 12:
            beacon_pkt.datarate = DR_LORA_SF12;
            beacon_RFU1_size = 5;
            beacon_RFU2_size = 3;
            break;
        default:
            /* should not happen */
            MSG(LOG_CRIT,"ERROR: unsupported datarate for beacon\n");
            exit(EXIT_FAILURE);
    }
    beacon_pkt.size = beacon_RFU1_size + 4 + 2 + 7 + beacon_RFU2_size + 2;
    beacon_pkt.coderate = CR_LORA_4_5;
    beacon_pkt.invert_pol = false;
    beacon_pkt.preamble = 10;
    beacon_pkt.no_crc = true;
    beacon_pkt.no_header = true;

    /* network common part beacon fields (little endian) */
    for (i = 0; i < (int)beacon_RFU1_size; i++) {
        beacon_pkt.payload[beacon_pyld_idx++] = 0x0;
    }
    
#ifdef _ALI_LINKWAN_
   /*delete for beacon gps info*/
#else
    /* network common part beacon fields (little endian) */
    beacon_pyld_idx += 4; /* time (variable), filled later */
    beacon_pyld_idx += 2; /* crc1 (variable), filled later */

    /* calculate the latitude and longitude that must be publicly reported */
    field_latitude = (int32_t)((reference_coord.lat / 90.0) * (double)(1<<23));
    if (field_latitude > (int32_t)0x007FFFFF) {
        field_latitude = (int32_t)0x007FFFFF; /* +90 N is represented as 89.99999 N */
    } else if (field_latitude < (int32_t)0xFF800000) {
        field_latitude = (int32_t)0xFF800000;
    }
    field_longitude = (int32_t)((reference_coord.lon / 180.0) * (double)(1<<23));
    if (field_longitude > (int32_t)0x007FFFFF) {
        field_longitude = (int32_t)0x007FFFFF; /* +180 E is represented as 179.99999 E */
    } else if (field_longitude < (int32_t)0xFF800000) {
        field_longitude = (int32_t)0xFF800000;
    }

    /* gateway specific beacon fields */
    beacon_pkt.payload[beacon_pyld_idx++] = beacon_infodesc;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_latitude;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_latitude >>  8);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_latitude >> 16);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_longitude;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_longitude >>  8);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_longitude >> 16);

    /* RFU */
    for (i = 0; i < (int)beacon_RFU2_size; i++) {
        beacon_pkt.payload[beacon_pyld_idx++] = 0x0;
    }

    /* CRC of the beacon gateway specific part fields */
    field_crc2 = crc16((beacon_pkt.payload + 6 + beacon_RFU1_size), 7 + beacon_RFU2_size);
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_crc2;
    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_crc2 >> 8);
#endif
    /* JIT queue initialization */
    for( i = 0; i < g_sx1301_nb; i++)
        jit_queue_init(&jit_queue[i]);

    while (!exit_sig && !quit_sig) {
        /* */
        if( autoquit_cnt >= network_error_threshold ){
           pthread_mutex_lock( &mx_network_err );
           status_network_connect = false;
           pthread_mutex_unlock( &mx_network_err );
        }
        /* auto-quit if the threshold is crossed */
        if ((autoquit_threshold > 0) && (autoquit_cnt >= autoquit_threshold)) {
            exit_err = true;
            exit_sig = true;
            MSG(LOG_CRIT,"INFO: [down] the last %u PULL_DATA were not ACKed, exiting application\n", autoquit_threshold);
            break;
        }

        /* generate random token for request */
        token_h = (uint8_t)rand(); /* random token */
        token_l = (uint8_t)rand(); /* random token */
        buff_req[1] = token_h;
        buff_req[2] = token_l;
        buff_index = 12;
        
        /* send PULL request and record time */
#ifdef _ALI_LINKWAN_
        send(sock_up, (void *)buff_req, buff_index, 0);
#else
        send(sock_down, (void *)buff_req, sizeof buff_req, 0);
#endif    
        clock_gettime(CLOCK_MONOTONIC, &send_time);
        pthread_mutex_lock(&mx_meas_dw);
        meas_dw_pull_sent += 1;
        pthread_mutex_unlock(&mx_meas_dw);
        req_ack = false;
        autoquit_cnt++;
        
        /* listen to packets and process them until a new PULL request must be sent */
        recv_time = send_time;
        while ((int)difftimespec(recv_time, send_time) < keepalive_time) {

            /* try to receive a datagram */
            memset(buff_down, 0x0, sizeof(buff_down));
            msg_len = recv(sock_down, (void *)buff_down, (sizeof buff_down)-1, 0);
            clock_gettime(CLOCK_MONOTONIC, &recv_time);

            /* Pre-allocate beacon slots in JiT queue, to check downlink collisions */
            beacon_loop = JIT_NUM_BEACON_IN_QUEUE - jit_queue[0].num_beacon; // Send beacon on sx1301 0
            retry = 0;
            while (beacon_loop && (beacon_period != 0)) {
                pthread_mutex_lock(&mx_timeref);
                /* Wait for GPS to be ready before inserting beacons in JiT queue */
                if ((gps_ref_valid == true) && (xtal_correct_ok == true)) {

                    /* compute GPS time for next beacon to come      */
                    /*   LoRaWAN: T = k*beacon_period + TBeaconDelay */
                    /*            with TBeaconDelay = [1.5ms +/- 1µs]*/
                    if (last_beacon_gps_time.tv_sec == 0) {
                        /* if no beacon has been queued, get next slot from current GPS time */
                        diff_beacon_time = time_reference_gps.gps.tv_sec % ((time_t)beacon_period);
                        next_beacon_gps_time.tv_sec = time_reference_gps.gps.tv_sec +
                                                        ((time_t)beacon_period - diff_beacon_time);
                    } else {
                        /* if there is already a beacon, take it as reference */
                        next_beacon_gps_time.tv_sec = last_beacon_gps_time.tv_sec + beacon_period;
                    }
                    /* now we can add a beacon_period to the reference to get next beacon GPS time */
                    next_beacon_gps_time.tv_sec += (retry * beacon_period);
                    next_beacon_gps_time.tv_nsec = 0;

#if DEBUG_BEACON
                    {
                        time_t time_unix;

                        time_unix = time_reference_gps.gps.tv_sec + UNIX_GPS_EPOCH_OFFSET;
                        MSG_DEBUG(DEBUG_BEACON, "GPS-now : %s", ctime(&time_unix));
                        time_unix = last_beacon_gps_time.tv_sec + UNIX_GPS_EPOCH_OFFSET;
                        MSG_DEBUG(DEBUG_BEACON, "GPS-last: %s", ctime(&time_unix));
                        time_unix = next_beacon_gps_time.tv_sec + UNIX_GPS_EPOCH_OFFSET;
                        MSG_DEBUG(DEBUG_BEACON, "GPS-next: %s", ctime(&time_unix));
                    }
#endif

                    /* convert GPS time to concentrator time, and set packet counter for JiT trigger */
                    lgw_gps2cnt(time_reference_gps, next_beacon_gps_time, &(beacon_pkt.count_us));
                    pthread_mutex_unlock(&mx_timeref);

                    /* apply frequency correction to beacon TX frequency */
                    if (beacon_freq_nb > 1) {
                        beacon_chan = (next_beacon_gps_time.tv_sec / beacon_period) % beacon_freq_nb; /* floor rounding */
                    } else {
                        beacon_chan = 0;
                    }
                    /* Compute beacon frequency */
                    beacon_pkt.freq_hz = beacon_freq_hz + (beacon_chan * beacon_freq_step);

                    /* load time in beacon payload */
                    beacon_pyld_idx = beacon_RFU1_size;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  next_beacon_gps_time.tv_sec;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (next_beacon_gps_time.tv_sec >>  8);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (next_beacon_gps_time.tv_sec >> 16);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (next_beacon_gps_time.tv_sec >> 24);

                    /* calculate CRC */
                    field_crc1 = crc16(beacon_pkt.payload, 4 + beacon_RFU1_size); /* CRC for the network common part */
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & field_crc1;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_crc1 >> 8);
#ifdef _ALI_LINKWAN_
                    //Begin add for beacon gps info
                    /* GPS coordinates variables */
                    struct coord_s cp_gps_coord = {0.0, 0.0, 0};

                    /* access GPS statistics, copy them */
                    if (gps_enabled == true) {
                        pthread_mutex_lock(&mx_meas_gps);
                        cp_gps_coord = meas_gps_coord;
                        pthread_mutex_unlock(&mx_meas_gps);
                    }
                    
                    /* overwrite with reference coordinates if function is enabled */
                    if (gps_fake_enable == true) {
                        cp_gps_coord = reference_coord;
                    }
                    
                    /* calculate the latitude and longitude that must be publicly reported */
                    field_latitude = (int32_t)((cp_gps_coord.lat / 90.0) * (double)(1<<23));
                    if (field_latitude > (int32_t)0x007FFFFF) {
                        field_latitude = (int32_t)0x007FFFFF; /* +90 N is represented as 89.99999 N */
                    } else if (field_latitude < (int32_t)0xFF800000) {
                        field_latitude = (int32_t)0xFF800000;
                    }
                    field_longitude = (int32_t)((cp_gps_coord.lon / 180.0) * (double)(1<<23));
                    if (field_longitude > (int32_t)0x007FFFFF) {
                        field_longitude = (int32_t)0x007FFFFF; /* +180 E is represented as 179.99999 E */
                    } else if (field_longitude < (int32_t)0xFF800000) {
                        field_longitude = (int32_t)0xFF800000;
                    }
                    
                    /* gateway specific beacon fields */
                    beacon_pkt.payload[beacon_pyld_idx++] = beacon_infodesc;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_latitude;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_latitude >>  8);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_latitude >> 16);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_longitude;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_longitude >>  8);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_longitude >> 16);
                    
                    /* RFU */
                    for (i = 0; i < (int)beacon_RFU2_size; i++) {
                        beacon_pkt.payload[beacon_pyld_idx++] = 0x0;
                    }
                    
                    /* CRC of the beacon gateway specific part fields */
                    field_crc2 = crc16((beacon_pkt.payload + 6 + beacon_RFU1_size), 7 + beacon_RFU2_size);
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF &  field_crc2;
                    beacon_pkt.payload[beacon_pyld_idx++] = 0xFF & (field_crc2 >> 8);
                    //End
#endif

                    /* Insert beacon packet in JiT queue */
                    gettimeofday(&current_unix_time, NULL);
                    /* tx beacon on sx1301 0 */
                    get_concentrator_time(&current_concentrator_time, current_unix_time, g_ctx_arr[0]);
                    jit_result = jit_enqueue(&jit_queue[0], &current_concentrator_time, &beacon_pkt, JIT_PKT_TYPE_BEACON);
                    if (jit_result == JIT_ERROR_OK) {
                        /* update stats */
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_nb_beacon_queued += 1;
                        pthread_mutex_unlock(&mx_meas_dw);

                        /* One more beacon in the queue */
                        beacon_loop--;
                        retry = 0;
                        last_beacon_gps_time.tv_sec = next_beacon_gps_time.tv_sec; /* keep this beacon time as reference for next one to be programmed */

                        /* display beacon payload */
                        MSG(LOG_DEBUG,"INFO: Beacon queued (count_us=%u, freq_hz=%u, size=%u):\n", beacon_pkt.count_us, beacon_pkt.freq_hz, beacon_pkt.size);
                        MSG(LOG_DEBUG, "   => " );
                        for (i = 0; i < beacon_pkt.size; ++i) {
                            MSG(LOG_DEBUG,"%02X ", beacon_pkt.payload[i]);
                        }
                        MSG(LOG_DEBUG,"\n");
                    } else {
                        MSG_DEBUG(DEBUG_BEACON, "--> beacon queuing failed with %d\n", jit_result);
                        /* update stats */
                        pthread_mutex_lock(&mx_meas_dw);
                        if (jit_result != JIT_ERROR_COLLISION_BEACON) {
                            meas_nb_beacon_rejected += 1;
                        }
                        pthread_mutex_unlock(&mx_meas_dw);
                        /* In case previous enqueue failed, we retry one period later until it succeeds */
                        /* Note: In case the GPS has been unlocked for a while, there can be lots of retries */
                        /*       to be done from last beacon time to a new valid one */
                        retry++;
                        MSG_DEBUG(DEBUG_BEACON, "--> beacon queuing retry=%d\n", retry);
                        if( retry > 3 ){
                            break;
                        }
                    }
                } else {
                    pthread_mutex_unlock(&mx_timeref);
                    break;
                }
            }

            /* if no network message was received, got back to listening sock_down socket */
            if (msg_len == -1) {
                //MSG(LOG_INFO,"WARNING: [down] recv returned %s\n", strerror(errno)); /* too verbose */
                continue;
            }
#ifdef _ALI_LINKWAN_            
            /* Begin add for reset when no ack in specify time */
            pthread_mutex_lock(&mx_stat_no_ack);
            stat_no_ack_cnt = 0;
            pthread_mutex_unlock(&mx_stat_no_ack);
            /* End */
            
            /* Begin add for packet filtering by whitelist and blacklist */
#if defined(USE_FILTER_NODE)
            if (1 == filter_inited) {
                i = filter_down_proc(buff_down, msg_len);
                if (1 == i) {
                    MSG(LOG_INFO,"INFO: [down] the filter down msg\n");
                    continue;
                }
            } else {
                if (0 == filter_init()) {
                    filter_inited = 1;
                }
            }
#endif
            /* End */
            //Begin add for adapt iot lora sdk
            if ((msg_len < 4) || (buff_down[0] != PROTOCOL_VERSION) || ((buff_down[3] != PKT_PULL_RESP) && (buff_down[3] != PKT_PULL_ACK) && (buff_down[3] != PKT_PUSH_ACK))) {
            //End
#else
            /* if the datagram does not respect protocol, just ignore it */
            if ((msg_len < 4) || (buff_down[0] != PROTOCOL_VERSION) || ((buff_down[3] != PKT_PULL_RESP) && (buff_down[3] != PKT_PULL_ACK))) {

#endif
                MSG(LOG_WARNING,"WARNING: [down] ignoring invalid packet len=%d, protocol_version=%d, id=%d\n",
                        msg_len, buff_down[0], buff_down[3]);
                continue;
            }

            /* if the datagram is an ACK, check token */
            if (buff_down[3] == PKT_PULL_ACK) {
                if ((buff_down[1] == token_h) && (buff_down[2] == token_l)) {
                    if (req_ack) {
                        MSG(LOG_INFO,"INFO: [down] duplicate ACK received :)\n");
                    } else { /* if that packet was not already acknowledged */
                        req_ack = true;
                        autoquit_cnt = 0;
                        
                        pthread_mutex_lock( &mx_network_err );
                        status_network_connect = true;
                        pthread_mutex_unlock( &mx_network_err );
                        
                        pthread_mutex_lock(&mx_meas_dw);
                        meas_dw_ack_rcv += 1;
                        pthread_mutex_unlock(&mx_meas_dw);
                        MSG(LOG_INFO,"INFO: [down] PULL_ACK received in %i ms\n", (int)(1000 * difftimespec(recv_time, send_time)));
                    }
                } else { /* out-of-sync token */
                    MSG(LOG_INFO,"INFO: [down] received out-of-sync ACK\n");
                }
                continue;
#ifdef _ALI_LINKWAN_                
            //Begin add for adapt iot lora sdk
            } else if (buff_down[3] == PKT_PUSH_ACK) {
                if ((buff_down[1] == push_ack_token_h) && (buff_down[2] == push_ack_token_l)) {
                    MSG(LOG_INFO,"INFO: [down] PUSH_ACK received\n");
                    pthread_mutex_lock(&mx_meas_up);
                    meas_up_ack_rcv += 1;
                    pthread_mutex_unlock(&mx_meas_up);
                } else {
                    MSG(LOG_WARNING, "WARNING: [down] ignored out-of sync ACK packet\n");
                }
                continue;
            //End
#endif            
            }
            
            /* the datagram is a PULL_RESP */
            buff_down[msg_len] = 0; /* add string terminator, just to be safe */
            MSG(LOG_INFO,"INFO: [down] PULL_RESP received  - token[%d:%d] :)\n", buff_down[1], buff_down[2]); /* very verbose */
            MSG(LOG_INFO,"\nJSON down: %s\n", (char *)(buff_down + 4)); /* DEBUG: display JSON payload */
            

            /* initialize TX struct and try to parse JSON */
            memset(&txpkt, 0, sizeof(txpkt));
            root_val = json_parse_string_with_comments((const char *)(buff_down + 4)); /* JSON offset */
            if (root_val == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] invalid JSON, TX aborted\n");
                continue;
            }

            /* look for JSON sub-object 'txpk' */
            txpk_obj = json_object_get_object(json_value_get_object(root_val), "txpk");
            if (txpk_obj == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] no \"txpk\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }

            /* Parse "immediate" tag, or target timestamp, or UTC time to be converted by GPS (mandatory) */
            i = json_object_get_boolean(txpk_obj,"imme"); /* can be 1 if true, 0 if false, or -1 if not a JSON boolean */
            if (i == 1) {
                /* TX procedure: send immediately */
                sent_immediate = true;
                downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_C;
                MSG(LOG_INFO,"INFO: [down] a packet will be sent in \"immediate\" mode\n");
            } else {
                sent_immediate = false;
                val = json_object_get_value(txpk_obj,"tmst");
                if (val != NULL) {
                    /* TX procedure: send on timestamp value */
                    txpkt.count_us = (uint32_t)json_value_get_number(val);

                    /* Concentrator timestamp is given, we consider it is a Class A downlink */
                    downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_A;
                } else {
                    /* TX procedure: send on GPS time (converted to timestamp value) */
                    val = json_object_get_value(txpk_obj, "tmms");
                    if (val == NULL) {
                        MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.tmst\" or \"txpk.tmms\" objects in JSON, TX aborted\n");
                        json_value_free(root_val);
                        continue;
                    }
                    if (gps_enabled == true) {
                        pthread_mutex_lock(&mx_timeref);
                        if (gps_ref_valid == true) {
                            local_ref = time_reference_gps;
                            pthread_mutex_unlock(&mx_timeref);
                        } else {
                            pthread_mutex_unlock(&mx_timeref);
                            MSG(LOG_WARNING,"WARNING: [down] no valid GPS time reference yet, impossible to send packet on specific GPS time, TX aborted\n");
                            json_value_free(root_val);

                            /* send acknoledge datagram to server */
                            send_tx_ack(buff_down[1], buff_down[2], JIT_ERROR_GPS_UNLOCKED);
                            continue;
                        }
                    } else {
                        MSG(LOG_WARNING,"WARNING: [down] GPS disabled, impossible to send packet on specific GPS time, TX aborted\n");
                        json_value_free(root_val);

                        /* send acknoledge datagram to server */
                        send_tx_ack(buff_down[1], buff_down[2], JIT_ERROR_GPS_UNLOCKED);
                        continue;
                    }

                    /* Get GPS time from JSON */
                    x2 = (uint64_t)json_value_get_number(val);

                    /* Convert GPS time from milliseconds to timespec */
                    x3 = modf((double)x2/1E3, &x4);
                    gps_tx.tv_sec = (time_t)x4; /* get seconds from integer part */
                    gps_tx.tv_nsec = (long)(x3 * 1E9); /* get nanoseconds from fractional part */

                    /* transform GPS time to timestamp */
                    i = lgw_gps2cnt(local_ref, gps_tx, &(txpkt.count_us));// TODO: local_ref of sx1301[i] --- class B
                    if (i != LGW_GPS_SUCCESS) {
                        MSG(LOG_WARNING,"WARNING: [down] could not convert GPS time to timestamp, TX aborted\n");
                        json_value_free(root_val);
                        continue;
                    } else {
                        MSG(LOG_INFO,"INFO: [down] a packet will be sent on timestamp value %u (calculated from GPS time)\n", txpkt.count_us);
                    }

                    /* GPS timestamp is given, we consider it is a Class B downlink */
                    downlink_type = JIT_PKT_TYPE_DOWNLINK_CLASS_B;
                }
            }

            /* Parse "No CRC" flag (optional field) */
            val = json_object_get_value(txpk_obj,"ncrc");
            if (val != NULL) {
                txpkt.no_crc = (bool)json_value_get_boolean(val);
            }

            /* parse target frequency (mandatory) */
            val = json_object_get_value(txpk_obj,"freq");
            if (val == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.freq\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            txpkt.freq_hz = (uint32_t)((double)(1.0e6) * json_value_get_number(val));

            /* parse RF chain used for TX (mandatory) */
            val = json_object_get_value(txpk_obj,"rfch");
            if (val == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.rfch\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            /* RAK: TX on radio 0 */
            //txpkt.rf_chain = (uint8_t)json_value_get_number(val);
            txpkt.rf_chain = 0;
            target_rf_chain = (uint8_t)json_value_get_number(val);
            /* parse TX power (optional field) */
            val = json_object_get_value(txpk_obj,"powe");
            if (val != NULL) {
                txpkt.rf_power = (int8_t)json_value_get_number(val) - antenna_gain;
            }

            /* Parse modulation (mandatory) */
            str = json_object_get_string(txpk_obj, "modu");
            if (str == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.modu\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            if (strcmp(str, "LORA") == 0) {
                /* Lora modulation */
                txpkt.modulation = MOD_LORA;

                /* Parse Lora spreading-factor and modulation bandwidth (mandatory) */
                str = json_object_get_string(txpk_obj, "datr");
                if (str == NULL) {
                    MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.datr\" object in JSON, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                i = sscanf(str, "SF%2hdBW%3hd", &x0, &x1);
                if (i != 2) {
                    MSG(LOG_WARNING,"WARNING: [down] format error in \"txpk.datr\", TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                switch (x0) {
                    case  7: txpkt.datarate = DR_LORA_SF7;  break;
                    case  8: txpkt.datarate = DR_LORA_SF8;  break;
                    case  9: txpkt.datarate = DR_LORA_SF9;  break;
                    case 10: txpkt.datarate = DR_LORA_SF10; break;
                    case 11: txpkt.datarate = DR_LORA_SF11; break;
                    case 12: txpkt.datarate = DR_LORA_SF12; break;
                    default:
                        MSG(LOG_WARNING,"WARNING: [down] format error in \"txpk.datr\", invalid SF, TX aborted\n");
                        json_value_free(root_val);
                        continue;
                }
                switch (x1) {
                    case 125: txpkt.bandwidth = BW_125KHZ; break;
                    case 250: txpkt.bandwidth = BW_250KHZ; break;
                    case 500: txpkt.bandwidth = BW_500KHZ; break;
                    default:
                        MSG(LOG_WARNING,"WARNING: [down] format error in \"txpk.datr\", invalid BW, TX aborted\n");
                        json_value_free(root_val);
                        continue;
                }

                /* Parse ECC coding rate (optional field) */
                str = json_object_get_string(txpk_obj, "codr");
                if (str == NULL) {
                    MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.codr\" object in json, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                if      (strcmp(str, "4/5") == 0) txpkt.coderate = CR_LORA_4_5;
                else if (strcmp(str, "4/6") == 0) txpkt.coderate = CR_LORA_4_6;
                else if (strcmp(str, "2/3") == 0) txpkt.coderate = CR_LORA_4_6;
                else if (strcmp(str, "4/7") == 0) txpkt.coderate = CR_LORA_4_7;
                else if (strcmp(str, "4/8") == 0) txpkt.coderate = CR_LORA_4_8;
                else if (strcmp(str, "1/2") == 0) txpkt.coderate = CR_LORA_4_8;
                else {
                    MSG(LOG_WARNING,"WARNING: [down] format error in \"txpk.codr\", TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }

                /* Parse signal polarity switch (optional field) */
                val = json_object_get_value(txpk_obj,"ipol");
                if (val != NULL) {
                    txpkt.invert_pol = (bool)json_value_get_boolean(val);
                }

                /* parse Lora preamble length (optional field, optimum min value enforced) */
                val = json_object_get_value(txpk_obj,"prea");
                if (val != NULL) {
                    i = (int)json_value_get_number(val);
                    if (i >= MIN_LORA_PREAMB) {
                        txpkt.preamble = (uint16_t)i;
                    } else {
                        txpkt.preamble = (uint16_t)MIN_LORA_PREAMB;
                    }
                } else {
                    txpkt.preamble = (uint16_t)STD_LORA_PREAMB;
                }

            } else if (strcmp(str, "FSK") == 0) {
                /* FSK modulation */
                txpkt.modulation = MOD_FSK;

                /* parse FSK bitrate (mandatory) */
                val = json_object_get_value(txpk_obj,"datr");
                if (val == NULL) {
                    MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.datr\" object in JSON, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                txpkt.datarate = (uint32_t)(json_value_get_number(val));

                /* parse frequency deviation (mandatory) */
                val = json_object_get_value(txpk_obj,"fdev");
                if (val == NULL) {
                    MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.fdev\" object in JSON, TX aborted\n");
                    json_value_free(root_val);
                    continue;
                }
                txpkt.f_dev = (uint8_t)(json_value_get_number(val) / 1000.0); /* JSON value in Hz, txpkt.f_dev in kHz */

                /* parse FSK preamble length (optional field, optimum min value enforced) */
                val = json_object_get_value(txpk_obj,"prea");
                if (val != NULL) {
                    i = (int)json_value_get_number(val);
                    if (i >= MIN_FSK_PREAMB) {
                        txpkt.preamble = (uint16_t)i;
                    } else {
                        txpkt.preamble = (uint16_t)MIN_FSK_PREAMB;
                    }
                } else {
                    txpkt.preamble = (uint16_t)STD_FSK_PREAMB;
                }

            } else {
                MSG(LOG_WARNING,"WARNING: [down] invalid modulation in \"txpk.modu\", TX aborted\n");
                json_value_free(root_val);
                continue;
            }

            /* Parse payload length (mandatory) */
            val = json_object_get_value(txpk_obj,"size");
            if (val == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.size\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            txpkt.size = (uint16_t)json_value_get_number(val);

            /* Parse payload data (mandatory) */
            str = json_object_get_string(txpk_obj, "data");
            if (str == NULL) {
                MSG(LOG_WARNING,"WARNING: [down] no mandatory \"txpk.data\" object in JSON, TX aborted\n");
                json_value_free(root_val);
                continue;
            }
            i = b64_to_bin(str, strlen(str), txpkt.payload, sizeof txpkt.payload);
            
            if (i != txpkt.size) {
                MSG(LOG_WARNING,"WARNING: [down] mismatch between .size and .data size once converter to binary\n");
            }

            
            MSG(LOG_DEBUG,"DownLink Frame :");
            hex_dump(txpkt.payload, i);
           
            /* free the JSON parse tree from memory */
            json_value_free(root_val);

            /* select TX mode */
            if (sent_immediate) {
                txpkt.tx_mode = IMMEDIATE;
            } else {
                txpkt.tx_mode = TIMESTAMPED;
            }

            /* record measurement data */
            pthread_mutex_lock(&mx_meas_dw);
            meas_dw_dgram_rcv += 1; /* count only datagrams with no JSON errors */
            meas_dw_network_byte += msg_len; /* meas_dw_network_byte */
            meas_dw_payload_byte += txpkt.size;
            pthread_mutex_unlock(&mx_meas_dw);

            /* check TX parameter before trying to queue packet */
            jit_result = JIT_ERROR_OK;
            if ((txpkt.freq_hz < tx_freq_min[txpkt.rf_chain]) || (txpkt.freq_hz > tx_freq_max[txpkt.rf_chain])) {
                jit_result = JIT_ERROR_TX_FREQ;
                MSG(LOG_ERR,"ERROR: Packet REJECTED, unsupported frequency - %u (min:%u,max:%u)\n", txpkt.freq_hz, tx_freq_min[txpkt.rf_chain], tx_freq_max[txpkt.rf_chain]);
            }
            if (jit_result == JIT_ERROR_OK) {
                for (i=0; i<txlut.size; i++) {
                    if (txlut.lut[i].rf_power >= txpkt.rf_power) {
                        /* this RF power is supported, we can continue */
                        break;
                    }
                }
                
                if (i == txlut.size) {
                    /* this RF power is not supported 
                    jit_result = JIT_ERROR_TX_POWER;
                    MSG(LOG_ERR,"ERROR: Packet REJECTED, unsupported RF power for TX - %d\n", txpkt.rf_power);
                    */
                    txpkt.rf_power = txlut.lut[txlut.size - 1].rf_power; 
                }
                else if( txlut.lut[i].rf_power != txpkt.rf_power ){
                    txpkt.rf_power = txlut.lut[i>0?(i-1):i].rf_power;
                }
                
            }

            /* How to insert down queue */
            o_count_us = txpkt.count_us;
            /* insert packet to be sent into JIT queue */
            if (jit_result == JIT_ERROR_OK) {
                /* First try to transmit on the first sx1276 */
                ctx_id = 0;
                gettimeofday(&current_unix_time, NULL);
                get_concentrator_time(&current_concentrator_time, current_unix_time, g_ctx_sx1276_arr[ctx_id]);
                txpkt.count_us = o_count_us - g_ctx_sx1276_arr[ctx_id]->offset_count_us; // count_us sx1276[0] --> sx1276[i]
                
                jit_result = jit_enqueue(&jit_queue[ctx_id], &current_concentrator_time, &txpkt, downlink_type);
                if (jit_result != JIT_ERROR_OK && jit_result != JIT_ERROR_TOO_EARLY && jit_result != JIT_ERROR_TOO_LATE) {
                    for (i = 1; i < SUPPROT_SX1276_MAX; i++) {
                        if (i == ctx_id)
                            continue;
                                                  
                        gettimeofday(&current_unix_time, NULL);
                        get_concentrator_time(&current_concentrator_time, current_unix_time, g_ctx_sx1276_arr[i]);
                        txpkt.count_us = o_count_us - g_ctx_sx1276_arr[i]->offset_count_us; // count_us sx1276[0] --> sx1276[i]
                        
                        jit_result = jit_enqueue(&jit_queue[i], &current_concentrator_time, &txpkt, downlink_type);
                        if (jit_result != JIT_ERROR_OK) {
                            //MSG(LOG_ERR,"ERROR: Packet REJECTED (jit error=%d) by sx1301 %d\n", jit_result, i);
                            continue;
                        }
                        MSG(LOG_INFO,"INFO: Packet ENQUENUE SUCCESS on sx1301 %d\n", i);
                        
                        break;
                    }
                    if(jit_result != JIT_ERROR_OK)
                        MSG(LOG_ERR,"ERROR: Packet REJECTED, all SX1301 busy\n");
                }
                else{
                    MSG(LOG_INFO,"INFO: Packet ENQUENUE SUCCESS on sx1301 %d\n", ctx_id);
                } 

                if( JIT_ERROR_OK == jit_result ){
                    pthread_mutex_lock(&mx_meas_dw);
                    meas_nb_tx_requested += 1;
                    pthread_mutex_unlock(&mx_meas_dw);
                    rrd_statistic_down(&txpkt, 0);
                    if( g_packet_table.enable ){
                        logger_packet_add_down(&txpkt, TYPE_NORMAL);
                    }
                }
                else{
                    if( g_packet_table.enable ){
                        switch(jit_result){
                            case JIT_ERROR_TOO_EARLY:logger_packet_add_down(&txpkt, TYPE_TO_EARLY);break;
                            case JIT_ERROR_TOO_LATE:logger_packet_add_down(&txpkt, TYPE_TO_LATE);break;
                            default:logger_packet_add_down(&txpkt, TYPE_BUSY);break;
                        }
                    }
                }
            }
        
            /* Send acknoledge datagram to server */
            send_tx_ack(buff_down[1], buff_down[2], jit_result);
            
        }
    }
    
    MSG(LOG_INFO,"\nINFO: End of downstream thread\n");
}

void print_tx_status(uint8_t tx_status) {
    switch (tx_status) {
        case TX_OFF:
            MSG(LOG_INFO,"INFO: [jit] lgw_status returned TX_OFF\n");
            break;
        case TX_FREE:
            MSG(LOG_INFO,"INFO: [jit] lgw_status returned TX_FREE\n");
            break;
        case TX_EMITTING:
            MSG(LOG_INFO,"INFO: [jit] lgw_status returned TX_EMITTING\n");
            break;
        case TX_SCHEDULED:
            MSG(LOG_INFO,"INFO: [jit] lgw_status returned TX_SCHEDULED\n");
            break;
        default:
            MSG(LOG_INFO,"INFO: [jit] lgw_status returned UNKNOWN (%d)\n", tx_status);
            break;
    }
}


/* -------------------------------------------------------------------------- */
/* --- THREAD 3: CHECKING PACKETS TO BE SENT FROM JIT QUEUE AND SEND THEM --- */
/* -------------------------------------------------------------------------- */
void thread_jit(void) {
    int result = LGW_HAL_SUCCESS;
    struct lgw_pkt_tx_s pkt;
    int pkt_index = -1;
    struct timeval current_unix_time;
    struct timeval current_concentrator_time;
    enum jit_error_e jit_result;
    enum jit_pkt_type_e pkt_type;
    int i = 0;
    int j = 0;
    
    while (!exit_sig && !quit_sig) {
        wait_ms(10);
        for( i = 0; i<g_sx1301_nb; i++){
            /* transfer data and metadata to the concentrator, and schedule TX */
            gettimeofday(&current_unix_time, NULL);
            get_concentrator_time(&current_concentrator_time, current_unix_time, g_ctx_arr[i]);
            jit_result = jit_peek(&jit_queue[i], &current_concentrator_time, &pkt_index);
            if (jit_result == JIT_ERROR_OK) {
                if (pkt_index > -1) {
                    jit_result = jit_dequeue(&jit_queue[i], pkt_index, &pkt, &pkt_type);
                    if (jit_result == JIT_ERROR_OK) {
                        /* update beacon stats */
                        if (pkt_type == JIT_PKT_TYPE_BEACON) {
                            /* Compensate breacon frequency with xtal error */
                            pthread_mutex_lock(&mx_xcorr);
                            pkt.freq_hz = (uint32_t)(xtal_correct * (double)pkt.freq_hz);
                            MSG_DEBUG(DEBUG_BEACON, "beacon_pkt.freq_hz=%u (xtal_correct=%.15lf)\n", pkt.freq_hz, xtal_correct);
                            pthread_mutex_unlock(&mx_xcorr);

                            /* Update statistics */
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_nb_beacon_sent += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG(LOG_INFO,"INFO: Beacon dequeued (count_us=%u)\n", pkt.count_us);
                        }

                        /* Sending packet into stm32 mini-nodes by usbtouart */
                        pthread_mutex_lock(&mx_concent); /* may have to wait for a fetch to finish */
                        result = lora_uart_write_downlink(uart_fd[0], 0, 0x04, pkt);
                        pthread_mutex_unlock(&mx_concent); /* free concentrator ASAP */

                        printf("freq_hz: %d, 0x%x\n", pkt.freq_hz, pkt.freq_hz);
                        printf("tx_mode: %d, 0x%x\n", pkt.tx_mode, pkt.tx_mode);
                        printf("count_us: %d, 0x%x\n", pkt.count_us, pkt.count_us);
                        printf("rf_chain: %d, 0x%x\n", pkt.rf_chain, pkt.rf_chain);
                        printf("rf_power: %d, 0x%x\n", pkt.rf_power, pkt.rf_power);
                        printf("modulation: %d, 0x%x\n", pkt.modulation, pkt.modulation);
                        printf("bandwidth: %d, 0x%x\n", pkt.bandwidth, pkt.bandwidth);
                        printf("datarate: %d, 0x%x\n", pkt.datarate, pkt.datarate);
                        printf("coderate: %d, 0x%x\n", pkt.coderate, pkt.coderate);
                        printf("invert_pol: %d, 0x%x\n", pkt.invert_pol, pkt.invert_pol);
                        printf("f_dev: %d, 0x%x\n", pkt.f_dev, pkt.f_dev);
                        printf("preamble: %d, 0x%x\n", pkt.preamble, pkt.preamble);
                        printf("no_crc: %d, 0x%x\n", pkt.no_crc, pkt.no_crc);
                        printf("no_header: %d, 0x%x\n", pkt.no_header, pkt.no_header);
                        printf("size: %d, 0x%x\n", pkt.size, pkt.size);
                        for (j = 0; j < pkt.size; j++)
                            printf("payload[%d]: %d, 0x%x\n", j, pkt.payload[j], pkt.payload[j]);

                        if (result == LGW_HAL_ERROR) {
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_nb_tx_fail += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG(LOG_WARNING, "WARNING: [jit] lora_uart_write_downlink failed.\n");
                            continue;
                        } else {
                            pthread_mutex_lock(&mx_meas_dw);
                            meas_nb_tx_ok += 1;
                            pthread_mutex_unlock(&mx_meas_dw);
                            MSG_DEBUG(DEBUG_PKT_FWD, "lora_uart_write_downlink done: count_us=%u\n", pkt.count_us);
                        }
                    } else {
                        MSG(LOG_ERR,"ERROR: jit_dequeue failed with %d\n", jit_result);
                    }
                }
            } else if (jit_result == JIT_ERROR_EMPTY) {
                /* Do nothing, it can happen */
            } else {
                MSG(LOG_ERR,"ERROR: jit_peek failed with %d\n", jit_result);
            }
        }
    }

}

/* -------------------------------------------------------------------------- */
/* --- THREAD 4: PARSE GPS MESSAGE AND KEEP GATEWAY IN SYNC ----------------- */

static void gps_process_sync(void) {
    struct timespec gps_time;
    struct timespec utc;
    uint32_t trig_tstamp; /* concentrator timestamp associated with PPM pulse */
    int i = lgw_gps_get(&utc, &gps_time, NULL, NULL);
    
    /* get GPS time for synchronization */
    if (i != LGW_GPS_SUCCESS) {
        MSG(LOG_INFO,"WARNING: [gps] could not get GPS time from GPS\n");
        return;
    }
    
    /* get timestamp captured on PPM pulse  */
    pthread_mutex_lock(&mx_concent);
    i = lgw_get_trigcnt(&trig_tstamp,&(g_ctx_arr[0]->spi));
    pthread_mutex_unlock(&mx_concent);
    if (i != LGW_HAL_SUCCESS) {
        MSG(LOG_INFO,"WARNING: [gps] failed to read concentrator timestamp\n");
        return;
    }

    /* try to update time reference with the new GPS time & timestamp */
    pthread_mutex_lock(&mx_timeref);
    i = lgw_gps_sync(&time_reference_gps, trig_tstamp, utc, gps_time);
    pthread_mutex_unlock(&mx_timeref);
    if (i != LGW_GPS_SUCCESS) {
        MSG(LOG_INFO,"WARNING: [gps] GPS out of sync, keeping previous time reference\n");
    }
    
}

static void gps_process_coords(void) {
    /* position variable */
    struct coord_s coord;
    struct coord_s gpserr;
    int    i = lgw_gps_get(NULL, NULL, &coord, &gpserr);

    /* update gateway coordinates */
    pthread_mutex_lock(&mx_meas_gps);
    if (i == LGW_GPS_SUCCESS) {
        gps_coord_valid = true;
        meas_gps_coord = coord;
        meas_gps_err = gpserr;
        // TODO: report other GPS statistics (typ. signal quality & integrity)
    } else {
        gps_coord_valid = false;
    }
    pthread_mutex_unlock(&mx_meas_gps);
}

void thread_gps(void) {
    /* serial variables */
    char serial_buff[128]; /* buffer to receive GPS data */
    size_t wr_idx = 0;     /* pointer to end of chars in buffer */

    /* variables for PPM pulse GPS synchronization */
    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);

    while (!exit_sig && !quit_sig) {
        size_t rd_idx = 0;
        size_t frame_end_idx = 0;

        /* blocking non-canonical read on serial port */
        ssize_t nb_char = read(gps_tty_fd, serial_buff + wr_idx, LGW_GPS_MIN_MSG_SIZE);
        if (nb_char <= 0) {
            MSG(LOG_INFO,"WARNING: [gps] read() returned value %d\n", nb_char);
            continue;
        }
        wr_idx += (size_t)nb_char;

        /*******************************************
         * Scan buffer for UBX/NMEA sync chars and *
         * attempt to decode frame if one is found *
         *******************************************/
        
        while(rd_idx < wr_idx) {
            size_t frame_size = 0;
#ifdef GPS_UBX
            /* Scan buffer for UBX sync char */
            if(serial_buff[rd_idx] == (char)LGW_GPS_UBX_SYNC_CHAR) {

                /***********************
                 * Found UBX sync char *
                 ***********************/
                latest_msg = lgw_parse_ubx(&serial_buff[rd_idx], (wr_idx - rd_idx), &frame_size);
                
                if (frame_size > 0) {
                    if (latest_msg == INCOMPLETE) {
                        /* UBX header found but frame appears to be missing bytes */
                        frame_size = 0;
                    } else if (latest_msg == INVALID) {
                        /* message header received but message appears to be corrupted */
                        MSG(LOG_INFO,"WARNING: [gps] could not get a valid message from GPS (no time)\n");
                        frame_size = 0;
                    } else if (latest_msg == UBX_NAV_TIMEGPS) {
                        gps_process_sync();
                    }
                }
            } else 
#endif            
            if(serial_buff[rd_idx] == LGW_GPS_NMEA_SYNC_CHAR) {
                /************************
                 * Found NMEA sync char *
                 ************************/
                /* scan for NMEA end marker (LF = 0x0a) */
                char* nmea_end_ptr = memchr(&serial_buff[rd_idx],(int)0x0a, (wr_idx - rd_idx));

                if(nmea_end_ptr) {
                    /* found end marker */
                    frame_size = nmea_end_ptr - &serial_buff[rd_idx] + 1;
                    latest_msg = lgw_parse_nmea(&serial_buff[rd_idx], frame_size);

                    if(latest_msg == INVALID || latest_msg == UNKNOWN) {
                        /* checksum failed */
                        frame_size = 0;
                    } else if (latest_msg == NMEA_RMC) { /* Get location from RMC frames */
#ifndef     GPS_UBX
                        gps_process_sync();
#endif
                        gps_process_coords();
                    }
                }
            }

            if(frame_size > 0) {
                /* At this point message is a checksum verified frame
                   we're processed or ignored. Remove frame from buffer */
                rd_idx += frame_size;
                frame_end_idx = rd_idx;
            } else {
                rd_idx++;
            }
        } /* ...for(rd_idx = 0... */

        if(frame_end_idx) {
          /* Frames have been processed. Remove bytes to end of last processed frame */
          memcpy(serial_buff, &serial_buff[frame_end_idx], wr_idx - frame_end_idx);
          wr_idx -= frame_end_idx;
        } /* ...for(rd_idx = 0... */

        /* Prevent buffer overflow */
        if((sizeof(serial_buff) - wr_idx) < LGW_GPS_MIN_MSG_SIZE) {
            memcpy(serial_buff, &serial_buff[LGW_GPS_MIN_MSG_SIZE], wr_idx - LGW_GPS_MIN_MSG_SIZE);
            wr_idx -= LGW_GPS_MIN_MSG_SIZE;
        }
    }
    MSG(LOG_INFO,"\nINFO: End of GPS thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 5: CHECK TIME REFERENCE AND CALCULATE XTAL CORRECTION --------- */

void thread_valid(void) {

    /* GPS reference validation variables */
    long gps_ref_age = 0;
    bool ref_valid_local = false;
    double xtal_err_cpy;

    /* variables for XTAL correction averaging */
    unsigned init_cpt = 0;
    double init_acc = 0.0;
    double x;

    /* correction debug */
    // FILE * log_file = NULL;
    // time_t now_time;
    // char log_name[64];

    /* initialization */
    // time(&now_time);
    // strftime(log_name,sizeof log_name,"xtal_err_%Y%m%dT%H%M%SZ.csv",localtime(&now_time));
    // log_file = fopen(log_name, "w");
    // setbuf(log_file, NULL);
    // fprintf(log_file,"\"xtal_correct\",\"XERR_INIT_AVG %u XERR_FILT_COEF %u\"\n", XERR_INIT_AVG, XERR_FILT_COEF); // DEBUG

    /* main loop task */
    while (!exit_sig && !quit_sig) {
        wait_ms(1000);

        /* calculate when the time reference was last updated */
        pthread_mutex_lock(&mx_timeref);
        gps_ref_age = (long)difftime(time(NULL), time_reference_gps.systime);
        if ((gps_ref_age >= 0) && (gps_ref_age <= GPS_REF_MAX_AGE)) {
            /* time ref is ok, validate and  */
            gps_ref_valid = true;
            ref_valid_local = true;
            xtal_err_cpy = time_reference_gps.xtal_err;
            //MSG(LOG_INFO,"XTAL err: %.15lf (1/XTAL_err:%.15lf)\n", xtal_err_cpy, 1/xtal_err_cpy); // DEBUG
        } else {
            /* time ref is too old, invalidate */
            gps_ref_valid = false;
            ref_valid_local = false;
        }
        pthread_mutex_unlock(&mx_timeref);

        /* manage XTAL correction */
        if (ref_valid_local == false) {
            /* couldn't sync, or sync too old -> invalidate XTAL correction */
            pthread_mutex_lock(&mx_xcorr);
            xtal_correct_ok = false;
            xtal_correct = 1.0;
            pthread_mutex_unlock(&mx_xcorr);
            init_cpt = 0;
            init_acc = 0.0;
        } else {
            if (init_cpt < XERR_INIT_AVG) {
                /* initial accumulation */
                init_acc += xtal_err_cpy;
                ++init_cpt;
            } else if (init_cpt == XERR_INIT_AVG) {
                /* initial average calculation */
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = (double)(XERR_INIT_AVG) / init_acc;
                //MSG(LOG_INFO,"XERR_INIT_AVG=%d, init_acc=%.15lf\n", XERR_INIT_AVG, init_acc);
                xtal_correct_ok = true;
                pthread_mutex_unlock(&mx_xcorr);
                ++init_cpt;
                // fprintf(log_file,"%.18lf,\"average\"\n", xtal_correct); // DEBUG
            } else {
                /* tracking with low-pass filter */
                x = 1 / xtal_err_cpy;
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = xtal_correct - xtal_correct/XERR_FILT_COEF + x/XERR_FILT_COEF;
                pthread_mutex_unlock(&mx_xcorr);
                // fprintf(log_file,"%.18lf,\"track\"\n", xtal_correct); // DEBUG
            }
        }
        // MSG(LOG_INFO,"Time ref: %s, XTAL correct: %s (%.15lf)\n", ref_valid_local?"valid":"invalid", xtal_correct_ok?"valid":"invalid", xtal_correct); // DEBUG
    }
    MSG(LOG_INFO,"\nINFO: End of validation thread\n");
}

/* --- EOF ------------------------------------------------------------------ */
