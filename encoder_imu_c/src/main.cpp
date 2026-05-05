#include <stdio.h>
#include <cinttypes>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "QuadratureDecoder.h"
#include "QuadratureDecoder.pio.h"
#include "pps_capture.pio.h"

// =======================
// BNO085 UART-RVC config
// =======================
// UART-RVC mode gives yaw/pitch/roll directly. On the Pico 2, GPIO1 is UART0 RX.
// Wiring for UART-RVC:
//   BNO085 SDA -> Pico GPIO1 / UART0 RX
//   BNO085 P0  -> 3V3
//   BNO085 P1  -> low / unconnected
//   BNO085 SCL -> not connected
#define BNO_RVC_UART        uart0
#define BNO_RVC_RX_PIN      1
#define BNO_RVC_BAUDRATE    115200

#define RVC_DEGREE_SCALE    0.01f
#define RVC_MILLI_G_TO_MS2  0.0098067f

// =========
// Pinning & Rates
// =========
#define ENCODER_PIN_BASE    18
#define ENCODER_INTERVAL_US 1000     // 1 kHz encoder sample
#define LOG_INTERVAL_US     1000     // flush up to 1 kHz

#define PPS_PIN             9
#define PPS_PIO             pio1      // keep PPS separate from quadrature decoder on pio0

// ============
// Global state
// ============
static volatile int32_t  encoder_count   = 0;
static volatile int32_t  encoder_offset  = 0;
static volatile uint64_t pps_counter     = 0;
static volatile uint64_t time_offset_us  = 0;
static volatile bool     logging_enabled = false;

// Sync request from core 0, executed by core 1 so encoder/PPS/ring reset is atomic
// relative to motion sampling.
static volatile bool     sync_request    = false;
static volatile bool     sync_ack        = false;

// Latest IMU snapshot (Euler, degrees), IMU_TS relative to sync.
static volatile float    g_roll_deg  = 0.0f;
static volatile float    g_pitch_deg = 0.0f;
static volatile float    g_yaw_deg   = 0.0f;
static volatile uint64_t g_imu_t_us  = 0;

// PPS PIO state machine, initialized on core 0 and serviced on core 1.
static int pps_sm = -1;

// ============
// Ring buffer: complete motion row snapshot
// ============
static constexpr size_t BUF_SIZE = 2048; // power of two
struct Event {
    uint64_t timestamp;  // us since last sync
    int32_t  count;      // encoder count relative to sync
    float    roll_deg;
    float    pitch_deg;
    float    yaw_deg;
    uint64_t imu_t_us;   // IMU packet timestamp, us since sync
    uint64_t pps;        // PPS count captured at encoder sample time
};
static Event ring_buf[BUF_SIZE];
static volatile uint32_t buf_head = 0, buf_tail = 0;
static inline uint32_t next_idx(uint32_t i) { return (i + 1) & (BUF_SIZE - 1); }

// Print buffer
static char  print_buffer[8192];
static size_t print_pos = 0;

// =========================
// BNO085 UART-RVC helpers
// =========================
struct BNO085RvcData {
    float yaw;
    float pitch;
    float roll;
    float x_accel;
    float y_accel;
    float z_accel;
};

static inline int16_t read_le_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void initBNO085_RVC() {
    uart_init(BNO_RVC_UART, BNO_RVC_BAUDRATE);
    uart_set_format(BNO_RVC_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BNO_RVC_UART, true);

    // Only RX is required in UART-RVC mode. Do not drive a TX pin into the BNO085.
    gpio_set_function(BNO_RVC_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(BNO_RVC_RX_PIN);
}

// Non-blocking UART-RVC parser.
// Packet format used by Adafruit's BNO08x_RVC library:
//   0xAA 0xAA, then 17 payload bytes. payload[16] is checksum over payload[0..15].
//   yaw/pitch/roll are little-endian int16 values at payload[1..6], scaled by 0.01 deg.
static bool readBNO085_RVC(BNO085RvcData &out) {
    enum ParseState : uint8_t { WAIT_AA1 = 0, WAIT_AA2 = 1, READ_PAYLOAD = 2 };
    static ParseState state = WAIT_AA1;
    static uint8_t payload[17];
    static uint8_t idx = 0;

    bool got_packet = false;

    while (uart_is_readable(BNO_RVC_UART)) {
        uint8_t b = uart_getc(BNO_RVC_UART);

        switch (state) {
            case WAIT_AA1:
                if (b == 0xAA) state = WAIT_AA2;
                break;

            case WAIT_AA2:
                if (b == 0xAA) {
                    idx = 0;
                    state = READ_PAYLOAD;
                } else {
                    state = WAIT_AA1;
                }
                break;

            case READ_PAYLOAD:
                payload[idx++] = b;
                if (idx >= sizeof(payload)) {
                    uint8_t sum = 0;
                    for (uint8_t i = 0; i < 16; i++) sum += payload[i];

                    if (sum == payload[16]) {
                        int16_t yaw_raw     = read_le_i16(&payload[1]);
                        int16_t pitch_raw   = read_le_i16(&payload[3]);
                        int16_t roll_raw    = read_le_i16(&payload[5]);
                        int16_t x_accel_raw = read_le_i16(&payload[7]);
                        int16_t y_accel_raw = read_le_i16(&payload[9]);
                        int16_t z_accel_raw = read_le_i16(&payload[11]);

                        out.yaw     = (float)yaw_raw * RVC_DEGREE_SCALE;
                        out.pitch   = (float)pitch_raw * RVC_DEGREE_SCALE;
                        out.roll    = (float)roll_raw * RVC_DEGREE_SCALE;
                        out.x_accel = (float)x_accel_raw * RVC_MILLI_G_TO_MS2;
                        out.y_accel = (float)y_accel_raw * RVC_MILLI_G_TO_MS2;
                        out.z_accel = (float)z_accel_raw * RVC_MILLI_G_TO_MS2;
                        got_packet = true;
                    }

                    idx = 0;
                    state = WAIT_AA1;
                }
                break;
        }
    }

    return got_packet;
}

static void service_pps_fifo() {
    while (!pio_sm_is_rx_fifo_empty(PPS_PIO, pps_sm)) {
        (void)pio_sm_get(PPS_PIO, pps_sm);
        __atomic_add_fetch(&pps_counter, 1, __ATOMIC_RELAXED);
    }
}

static void clear_pps_fifo() {
    while (!pio_sm_is_rx_fifo_empty(PPS_PIO, pps_sm)) {
        (void)pio_sm_get(PPS_PIO, pps_sm);
    }
}

static void handle_sync_request(QuadratureDecoder &dec, int32_t idx) {
    if (!__atomic_load_n(&sync_request, __ATOMIC_ACQUIRE)) {
        return;
    }

    // Stop row capture while the shared state is reset.
    __atomic_store_n(&logging_enabled, false, __ATOMIC_RELAXED);

    int32_t cnt = dec.getCount(idx);
    __atomic_store_n(&encoder_count, cnt, __ATOMIC_RELAXED);
    __atomic_store_n(&encoder_offset, cnt, __ATOMIC_RELAXED);
    __atomic_store_n(&time_offset_us, time_us_64(), __ATOMIC_RELAXED);

    clear_pps_fifo();
    __atomic_store_n(&pps_counter, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&g_imu_t_us, 0, __ATOMIC_RELAXED);

    buf_head = 0;
    buf_tail = 0;

    __atomic_store_n(&logging_enabled, true, __ATOMIC_RELAXED);
    __atomic_store_n(&sync_request, false, __ATOMIC_RELEASE);
    __atomic_store_n(&sync_ack, true, __ATOMIC_RELEASE);
}

// ==============================
// Core 1: real-time motion service
// ==============================
void core1_entry() {
    pio_gpio_init(pio0, ENCODER_PIN_BASE);
    pio_gpio_init(pio0, ENCODER_PIN_BASE + 1);
    gpio_set_dir(ENCODER_PIN_BASE, GPIO_IN);
    gpio_set_dir(ENCODER_PIN_BASE + 1, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_BASE);
    gpio_pull_up(ENCODER_PIN_BASE + 1);

    QuadratureDecoder dec;
    dec.init(pio0);
    int32_t idx = dec.addQuadratureEncoder(ENCODER_PIN_BASE);

    absolute_time_t next_enc = get_absolute_time();

    while (true) {
        handle_sync_request(dec, idx);

        // Drain PPS promptly on the same core that snapshots encoder rows.
        service_pps_fifo();

        // Drain BNO085 UART promptly. At ~100 Hz this is light work.
        BNO085RvcData rvc;
        if (readBNO085_RVC(rvc)) {
            uint64_t rel_t = time_us_64() - __atomic_load_n(&time_offset_us, __ATOMIC_RELAXED);
            g_roll_deg  = rvc.roll;
            g_pitch_deg = rvc.pitch;
            g_yaw_deg   = rvc.yaw;
            g_imu_t_us  = rel_t;
        }

        while (absolute_time_diff_us(next_enc, get_absolute_time()) >= 0) {
            int32_t cnt = dec.getCount(idx);
            encoder_count = cnt;

            if (__atomic_load_n(&logging_enabled, __ATOMIC_RELAXED)) {
                uint64_t rel_time = time_us_64() - __atomic_load_n(&time_offset_us, __ATOMIC_RELAXED);
                int32_t enc_rel = cnt - __atomic_load_n(&encoder_offset, __ATOMIC_RELAXED);

                uint32_t h = buf_head;
                uint32_t t = buf_tail;
                uint32_t n = next_idx(h);

                ring_buf[h].timestamp = rel_time;
                ring_buf[h].count     = enc_rel;
                ring_buf[h].roll_deg  = g_roll_deg;
                ring_buf[h].pitch_deg = g_pitch_deg;
                ring_buf[h].yaw_deg   = g_yaw_deg;
                ring_buf[h].imu_t_us  = g_imu_t_us;
                ring_buf[h].pps       = __atomic_load_n(&pps_counter, __ATOMIC_RELAXED);
                __atomic_thread_fence(__ATOMIC_RELEASE);

                if (n == t) buf_tail = next_idx(buf_tail);
                buf_head = n;
            }

            next_enc = delayed_by_us(next_enc, ENCODER_INTERVAL_US);
        }

        sleep_us(50);
    }
}

// ============
// Core 0: setup + command + printing
// ============
int main() {
    sleep_ms(5000);
    stdio_init_all();

    // ====== PPS capture via PIO1 ======
    pio_gpio_init(PPS_PIO, PPS_PIN);
    gpio_set_dir(PPS_PIN, GPIO_IN);
    gpio_pull_up(PPS_PIN);

    uint pps_offset = pio_add_program(PPS_PIO, &pps_capture_program);
    pps_sm = pio_claim_unused_sm(PPS_PIO, true);
    pio_sm_config pps_cfg = pps_capture_program_get_default_config(pps_offset);
    sm_config_set_in_pins(&pps_cfg, PPS_PIN);
    pio_sm_init(PPS_PIO, pps_sm, pps_offset, &pps_cfg);
    pio_sm_clear_fifos(PPS_PIO, pps_sm);
    pio_sm_set_enabled(PPS_PIO, pps_sm, true);

    // ====== UART-RVC + IMU ======
    initBNO085_RVC();

    // Start motion core
    multicore_launch_core1(core1_entry);

    absolute_time_t next_print = get_absolute_time();

    // PPS is intentionally kept last for compatibility with older display/debug tools.
    printf("TS_US,ENC,ROLL_DEG,PITCH_DEG,YAW_DEG,IMU_TS_US,PPS\n");

    while (true) {
        // 1) Sync/reset on 's'. Core 1 performs the actual reset.
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT && (char)c == 's') {
            __atomic_store_n(&sync_ack, false, __ATOMIC_RELEASE);
            __atomic_store_n(&sync_request, true, __ATOMIC_RELEASE);

            absolute_time_t deadline = make_timeout_time_ms(500);
            while (!__atomic_load_n(&sync_ack, __ATOMIC_ACQUIRE) &&
                   absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
                sleep_us(100);
            }

            printf("event,SYNC_RESET\n");
            printf("event,SYNC_RESET_ACK\n");
        }

        // 2) Dequeue already-snapshotted rows and print.
        while (__atomic_load_n(&logging_enabled, __ATOMIC_RELAXED) &&
               absolute_time_diff_us(next_print, get_absolute_time()) >= 0) {
            while (buf_tail != buf_head) {
                __atomic_thread_fence(__ATOMIC_ACQUIRE);
                const auto &e = ring_buf[buf_tail];

                print_pos += snprintf(
                    print_buffer + print_pos,
                    sizeof(print_buffer) - print_pos,
                    "l,%" PRIu64 ",%d,%.2f,%.2f,%.2f,%" PRIu64 ",%" PRIu64 "\n",
                    e.timestamp,
                    e.count,
                    (double)e.roll_deg,
                    (double)e.pitch_deg,
                    (double)e.yaw_deg,
                    e.imu_t_us,
                    e.pps
                );

                buf_tail = next_idx(buf_tail);

                if (print_pos > sizeof(print_buffer) - 160) {
                    printf("%s", print_buffer);
                    print_pos = 0;
                }
            }

            if (print_pos > 0) {
                printf("%s", print_buffer);
                print_pos = 0;
            }

            next_print = delayed_by_us(next_print, LOG_INTERVAL_US);
        }

        sleep_us(50);
    }

    return 0;
}
