#ifndef UART_CMD_H
#define UART_CMD_H

#include "cross_compiler.h"

#ifdef UART

void uart_send_status(void);
void uart_cmd_proc(void);

#endif /* UART */

#endif /* UART_CMD_H */
