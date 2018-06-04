# Washing Machine HAL for STM32F411RE

## Purpose

This project contains the pinout model for the "washing machine" kata,
along with the generated HAL, using ST micro's STM32CubeMX tool, in
standalone mode.

It's not expected you'll use this as-is for a project, but incorporate
it into one of your own, either by copying, or by using as a Git
submodule (the latter probably the best option).

## Pinout

The full report on pin assignments and configuration are stored in
this repository as a [text file](nucleo-f411re.txt) and a [PDF
file](nucleo-f411re.pdf). Here are the edited highlights though:

![pinout](pinout_image.png)

## Integration with your project

The HAL code is generated by a tool. Few people would consider it
aesthetically pleasing - it's designed for parameterisation and code
generation by a tool, rather than for maintenance by hand. Most
developers will choose to do their implementation entirely outside of
this framework.

A hook is provided that calls a (declared, not defined) function:

```
extern void run_application(ADC_HandleTypeDef*, SPI_HandleTypeDef*,
                            TIM_HandleTypeDef*, UART_HandleTypeDef*);
```

Following initialisation in the tool-generated `main.c`, the
peripheral hardware is set to safe defaults - the PWM duty cycle is
set to zero, all discrete LEDs are off, and the 7-segment diplay set
to a low-intensity setting with a dash character across all four
digits.

The run_application function is called, passing the initialised
hardware resource handles for SPI, ADC, PWM (htim2) and UART. It's up
to you to either implement that function call, or comment/delete it,
if you'd prefer to implement directly in the `main.c` - and I'd advise
against that if you want to do TDD.

It's expected that the call will never return, because it will start
the FreeRTOS scheduler. Consequently, the code further down `main()`
will never be called.

```
  ...
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  ...
  osKernelStart();
```

It's left in place, but you can delete it if you plan to start the
scheduler in run_application.

## Segger RTT and FreeRTOS instrumentation

The FreeRTOS version in this implementation is instrumented for Segger
"Real Time Transfer" (RTT). This is a library for transmitting
high-resolution telemetry from the target device back to a debugger
over the JTAG/SWD debugger.

Note: if you regenerate the code from STMCube, be aware that you will
overwrite these modifications, so you will need to cherry-pick the
hunks you want to commit if you want the RTT telemetry preserved.

## License: "It's complicated"

Virtually all the generated code in here is pumped out from the
STMCube tool:

- ST Micro's source files all have their own copyright statement.

- The FreeRTOS sources are under a modified version of the GNU GPL
  license (for v9.0 - this changes in the v10.x kernel to
  MIT...).

- The design of the washing machine kata itself is a Free Hardware
  Design, released under the GNU GPL v3 (c) 13coders Limited. The only
  artifact in here that could be said to be covered by that licesnse
  is the pinout model itself, in the .ioc file.

