/*
 * tinyshell_commands.c
 *
 *  Created on: Dec 8, 2020
 *      Author: reicherr
 *      Brief: Port from "Building a Tiny CLI Shell for Tiny Firmware" by Tyler Hoffman
 *      https://interrupt.memfault.com/blog/firmware-shell
 */

#include "tinyshell.h"
#include <stdbool.h>		// boolean variable
#include <string.h>			// strlen()
#include <stdlib.h>     // strtoul()

/**> tinyShell specific define */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#define FW_REVISION			"STM32_tinyShell V1.0"

/**> Application specific define */
#define VDAC_REF_MV     (3300u)
#define VDAC_RANGE			(4096u)
#define VDAC_MAX_MV			VDAC_REF_MV

/**> Application variables */
uint32_t vdacValue;
extern DAC_HandleTypeDef hdac;

/**
 * Clear terminal screen
 */
int cli_clear(int argc, char *argv[])
{
	shell_put_line("\033[2J\033[1H");
	return 0;
}

/**
 * Reboot the MCU
 */
int cli_reboot(int argc, char *argv[])
{
	shell_put_line("System rebooting");
#if (__CORTEX_M >= 0x03)
	/**> Prevents activation of all exceptions except for Non-Maskable Interrupt NMI, not for Cortex-M0 */
	__set_FAULTMASK(1);
#endif
	NVIC_SystemReset();
	return 0;
}

/**
 *  Display firmware version
 */
int cli_fw_ver(int argc, char *argv[])
{
  shell_put_line(FW_REVISION);
  return 0;
}

/**
 * Set LED2
 */
int cli_led_on(int argc, char *argv[])
{
  shell_put_line("LED2 On");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  return 0;
}

/**
 * reset LED2
 */
int cli_led_off(int argc, char *argv[])
{
  shell_put_line("LED2 Off");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  return 0;
}


/**
 * Set DAC1 Channel 1 Value arg function
 */
bool kv_store_write(const char *key, const void *val, uint32_t len)
{
  uint32_t vdacTmp;

  /**> Convert argv[2] to integer */
  vdacTmp = strtoul(val, 0, 10);
  /**> Check for limit of value */
  if (vdacTmp > VDAC_MAX_MV) {
    return false;
  }

  /**> Compute the 12-bit output value */
  vdacValue = (uint32_t)((vdacTmp * VDAC_RANGE) / VDAC_REF_MV);

  /**> Write the output value to VDAC DATA register */
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, vdacValue);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

  return true;
}

/**
 * Set DAC1 Channel 1 Value parser cmd
 */
int cli_cmd_dac_write(int argc, char *argv[])
{
  // We expect 3 arguments:
  // 1. Command name
  // 2. Key
  // 3. Value
  if (argc != 3) {
    shell_put_line("> FAIL,1");
    return 1;
  }

  const char *key = argv[1];
  const char *value = argv[2];

  bool result = kv_store_write(key, value, strlen(value));
  if (!result) {
    shell_put_line("> FAIL,2");
    return 1;
  }
  shell_put_line("> Done");
  return 0;
}

/**
 * Array of tinyShell help commands
 */
static const sShellCommand s_shell_commands[] =
{
	{"help", 				shell_help_handler, 	"Lists all commands"										},
	{"clear", 			cli_clear, 						"Clear terminal"												},
	{"reboot", 			cli_reboot,						"Reboot mcu"														},
  {"version",		  cli_fw_ver, 					"Firmware revision"											},
  {"led_on", 			cli_led_on, 					"Set LED2"															},
  {"led_off", 		cli_led_off,	 				"Reset LED2"														},
  {"dac_set", 	  cli_cmd_dac_write, 		"<command> <DAC_CHANNEL_X> <mv_value>"	},
};

const sShellCommand *const g_shell_commands = s_shell_commands;
const size_t g_num_shell_commands = ARRAY_SIZE(s_shell_commands);

