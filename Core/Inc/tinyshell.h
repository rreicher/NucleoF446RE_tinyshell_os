/*
 * tinyshell.h
 *
 *  Created on: Dec 8, 2020
 *      Author: reicherr
 *      Brief: Port from "Building a Tiny CLI Shell for Tiny Firmware" by Tyler Hoffman
 *      https://interrupt.memfault.com/blog/firmware-shell
 */

#ifndef INC_TINYSHELL_H_
#define INC_TINYSHELL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stddef.h>

/**> tinyShell Command Structure Declaration */
typedef struct ShellCommand {
  const char *command;
  int (*handler)(int argc, char *argv[]);
  const char *help;
} sShellCommand;

/**> External declaration of pointer to shell_command array defined into tinyshell_command.c */
extern const sShellCommand *const g_shell_commands;
extern const size_t g_num_shell_commands;

/**> Structure for low level API Send_char() */
typedef struct ShellImpl {
  //! Function to call whenever a character needs to be sent out.
  int (*send_char)(char c);
} sShellImpl;

/**> Initializes shell, must be called one time and early at boot */
void shell_boot(const sShellImpl *impl);

/**> Call this when incoming byte is received */
void shell_receive_char(char c);

/**> Print help command to console */
int shell_help_handler(int argc, char *argv[]);

/**> Prints a line then a newline */
void shell_put_line(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* INC_TINYSHELL_H_ */
