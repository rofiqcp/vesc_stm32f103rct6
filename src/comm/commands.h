#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "datatypes.h"

typedef void (*vesc_appdata_handler_t)(const uint8_t *data, uint16_t len, motor_id_t motor);

/* Canonical VESC command API names used by upstream modules. The reduced
 * STM32F103 port still exposes board-specific vesc_comm_* helpers below for
 * transport readiness and the dual-motor USART3 integration. */
void commands_init(void);
bool commands_is_initialized(void);
void commands_send_packet(unsigned char *data, unsigned int len);
#if defined(__GNUC__) || defined(__clang__)
int commands_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#else
int commands_printf(const char *fmt, ...);
#endif
void commands_send_rotor_pos(float rotor_pos);
void commands_send_experiment_samples(float *samples, int len);
void commands_init_plot(const char *namex, const char *namey);
void commands_plot_add_graph(const char *name);
void commands_plot_set_graph(int graph);
void commands_send_plot_points(float x, float y);

bool vesc_comm_task_init(void);
void vesc_comm_set_thread_ids(osThreadId_t packet, osThreadId_t blocking);
void vesc_comm_set_config_ready(bool ready);
void vesc_comm_set_motor_ready(bool ready);
bool vesc_comm_motor_ready(void);
void vesc_comm_send_payload(const uint8_t *payload, uint16_t len);
void vesc_comm_periodic_100hz(void);
void vesc_comm_send_sample_buffer(const debug_sample_t *samples, uint16_t count);
void vesc_comm_send_sample_buffer_to(void (*reply)(unsigned char *data, unsigned int len),
                                     uint16_t count);

void vesc_comm_register_appdata_handler(vesc_appdata_handler_t handler);

/* VESC COMM_PRINT transport hook used by motor diagnostics. */
void commands_send_print(const char *msg);

typedef struct {
    uint32_t packet_stack_free_bytes;
    uint32_t blocking_stack_free_bytes;
} vesc_comm_resource_stats_t;
void vesc_comm_get_resource_stats(vesc_comm_resource_stats_t *out);
