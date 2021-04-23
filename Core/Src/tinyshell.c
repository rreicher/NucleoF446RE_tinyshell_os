/*
 * tinyshell.c
 *
 *  Created on: Dec 8, 2020
 *      Author: reicherr
 *      Brief: Port from "Building a Tiny CLI Shell for Tiny Firmware" by Tyler Hoffman
 *      https://interrupt.memfault.com/blog/firmware-shell
 *
 */

#include "tinyshell.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/**> Global Defines */
#define SHELL_RX_BUFFER_SIZE 		(256u)
#define SHELL_MAX_ARGS 					(16u)
#define SHELL_PROMPT 						"Shell> "

/**> Global Macro */
#define SHELL_FOR_EACH_COMMAND(command)                 \
  for (const sShellCommand *command = g_shell_commands; \
    command < &g_shell_commands[g_num_shell_commands];  \
    ++command)

/**> Shell Context Structure Declaration */
static struct ShellContext
{
  int (*send_char)(char c);							// Pointer to send char uart
  size_t rx_size;												// Receive uart buffer size
  char rx_buffer[SHELL_RX_BUFFER_SIZE];	// Receive uart buffer array
} s_shell;

/**
 * Local function used to check if char are waiting to be transmit
 * Arg0: None
 * Return: True of False
 */
static bool prv_booted(void)
{
  return s_shell.send_char != NULL;
}

/**
 * Local function used to echo back received char to console
 * Arg0: Incoming char
 * Return: None
 */
static void prv_send_char(char c)
{
  if (!prv_booted()) {
    return;
  }
  s_shell.send_char(c);
}

/**
 * Local function used to echo back received char to console
 * Arg0: Incoming char
 * Return: None
 */
static void prv_echo(char c)
{
  if ('\n' == c) {
    prv_send_char('\r');
    prv_send_char('\n');
  } else if ('\b' == c) {
    prv_send_char('\b');
    prv_send_char(' ');
    prv_send_char('\b');
  } else {
    prv_send_char(c);
  }
}

/**
 * Local function used to get last char stored in the buffer
 * Arg0: None
 * Return: Last char received
 */
static char prv_last_char(void)
{
  return s_shell.rx_buffer[s_shell.rx_size - 1];
}

/**
 * Local function used to check if uart rx buffer is full
 * Arg0: None
 * Return: Boolean True or False
 */
static bool prv_is_rx_buffer_full(void)
{
  return s_shell.rx_size >= SHELL_RX_BUFFER_SIZE;
}

/**
 * Local function used to reset uart rx buffer
 * Arg0: None
 * Return: None
 */
static void prv_reset_rx_buffer(void)
{
  memset(s_shell.rx_buffer, 0, sizeof(s_shell.rx_buffer));
  s_shell.rx_size = 0;
}

/**
 * Local function used to transmit string to console
 * Arg0: Incoming array char
 * Return: None
 */
static void prv_echo_str(const char *str)
{
  for (const char *c = str; *c != '\0'; ++c) {
    prv_echo(*c);
  }
}

/**
 * Local function used to display Shell Prompt to console
 * Arg0: None
 * Return: None
 */
static void prv_send_prompt(void)
{
  prv_echo_str(SHELL_PROMPT);
}

/**
 * Local function used to find existing command
 * Arg0: Command Name string
 * Return: ShellCommand Struct pointer
 */
static const sShellCommand *prv_find_command(const char *name)
{
  SHELL_FOR_EACH_COMMAND(command) {
    if (strcmp(command->command, name) == 0) {
      return command;
    }
  }
  return NULL;
}

/**
 * Local function used to parse uart Rx buffer
 * Arg0: None
 * Return: None
 */
static void prv_process(void)
{
  if (prv_last_char() != '\n' && !prv_is_rx_buffer_full()) {
    return;
  }

  char *argv[SHELL_MAX_ARGS] = {0};
  int argc = 0;

  char *next_arg = NULL;
  for (size_t i = 0; i < s_shell.rx_size && argc < SHELL_MAX_ARGS; ++i) {
    char *const c = &s_shell.rx_buffer[i];
    if (*c == ' ' || *c == '\n' || i == s_shell.rx_size - 1) {
      *c = '\0';
      if (next_arg) {
        argv[argc++] = next_arg;
        next_arg = NULL;
      }
    } else if (!next_arg) {
      next_arg = c;
    }
  }

  if (s_shell.rx_size == SHELL_RX_BUFFER_SIZE) {
    prv_echo('\n');
  }

  if (argc >= 1) {
    const sShellCommand *command = prv_find_command(argv[0]);
    if (!command) {
      prv_echo_str("Unknown command: ");
      prv_echo_str(argv[0]);
      prv_echo('\n');
      prv_echo_str("Type 'help' to list all commands\r\n");
    } else {
      command->handler(argc, argv);
    }
  }
  prv_reset_rx_buffer();
  prv_send_prompt();
}

/**
 * Function used to Initialize Shell
 * Arg0: sShellImpl structure
 * Return: None
 */
void shell_boot(const sShellImpl *impl)
{
  s_shell.send_char = impl->send_char;
  prv_reset_rx_buffer();
  prv_echo_str("STM32 tinyShell");
  prv_echo_str("\n" SHELL_PROMPT);
}

/**
 * Local function used to store and parse received char into buffer
 * Arg0: Incoming char
 * Return: None
 */
void shell_receive_char(char c)
{
  if (c == '\r' || prv_is_rx_buffer_full() || !prv_booted()) {
    return;
  }
  prv_echo(c);

  if (c == '\b') {
    s_shell.rx_buffer[--s_shell.rx_size] = '\0';
    return;
  }

  s_shell.rx_buffer[s_shell.rx_size++] = c;

  prv_process();
}

/**
 * Function used send string terminated with Line Feed
 * Arg0: Incoming string
 * Return: None
 */
void shell_put_line(const char *str)
{
  prv_echo_str(str);
  prv_echo('\n');
}

/**
 * Function used to display help
 * Arg0: argc number of index of argv[]
 * Arg1: argv[] array of parameters
 * Return: Always 0!
 */
int shell_help_handler(int argc, char *argv[])
{
	int32_t spaceLenght;

	prv_echo_str("Command:            ; Usage:\n");

	SHELL_FOR_EACH_COMMAND(command)
	{
		spaceLenght = 20 - (int32_t)(strlen(command->command));
		prv_echo_str(command->command);
		/**> Format and align usage column */
    do {
    	prv_echo_str(" ");
    } while (--spaceLenght);

    prv_echo_str("; ");
    prv_echo_str(command->help);
    prv_echo('\n');
  }
  prv_echo('\n');
  return 0;
}

