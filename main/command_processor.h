#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*command_print_fn_t)(const char *str);

void command_processor_init(void);
void command_process(const char *cmd_line, command_print_fn_t print_fn);
void command_notify_status(void);
void command_on_ble_received(const char *cmd, void (*reply_fn)(const char *reply));

// Writes the "STATUS:..." line (with trailing newline) into buf; returns the length written.
size_t command_build_status(char *buf, size_t len);

#ifdef __cplusplus
}
#endif
