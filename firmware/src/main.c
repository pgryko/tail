#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_leuart.h"
#include "em_emu.h"
#include "em_rmu.h" // Reset Management Unit

// Include project-specific modules
#include "common.h" // Common definitions and utilities
#include "radio.h" // UWB Radio high-level functions
#include "radio_spi.h" // Radio SPI communication functions
#include "radio_reg.h" // Radio register definitions
#include "accel.h" // Accelerometer functions
#include "uart.h" // UART communication functions
#include "cli.h" // Command Line Interface handler
#include "time.h" // Timekeeping functions
#include "config.h" // Configuration storage functions
#include "proto.h" // Tail communication protocol handler
#include "timer.h" // Software timer functions
#include "event.h" // Event handling system
#include "entropy.h" // Entropy source for random numbers
#include "crypto.h" // Cryptographic functions (AES, CMAC, DRBG)

// --- Debugging ---
#define CLOCK_DEBUG 0 // Set to 1 to output internal clocks on GPIO pins for debugging

// --- Constants ---
/* XXX no longer used - Antenna delay values are now read from config store */
// #define ANTENNA_DELAY_TX 16434
// #define ANTENNA_DELAY_RX 16434

// --- Global Variables ---

// Default radio configuration structure, values can be overridden by config store
radio_config_t radio_config = {
		/* chan */        5,     // UWB Channel 5
		/* prf_high */    false, // Use 16 MHz PRF
		/* tx_plen */     RADIO_PLEN_128, // 128 symbols preamble length
		/* rx_pac */      RADIO_PAC_8,    // 8 symbols PAC size
		/* tx_pcode */    4,     // Preamble code 4
		/* rx_pcode */    4,     // Preamble code 4
		/* ns_sfd */      false, // Use standard SFD
		/* data_rate */   RADIO_RATE_6M8, // 6.8 Mbps data rate
		/* long_frames */ false, // Use standard frame length (<= 127 bytes)
		/* sfd_timeout */ 0      // SFD timeout (0 means default)
};

// Variable to store the crystal oscillator trim value (read from config)
uint8_t xtal_trim;

// --- External Symbols (from linker script) ---
extern void *__StackTop;   // Top of the stack memory region
extern void *__StackLimit; // Bottom of the stack memory region (unused?)
extern void *__HeapLimit;  // End of the heap memory region (unused?)
extern void *__HeapBase;   // Start of the heap memory region

// --- Main Function ---
/**
 * @brief Main entry point for the Tail Tag firmware.
 *
 * Initializes hardware peripherals, software modules, reads configuration,
 * and enters the main low-power loop.
 */
int main(void)
{
  /* --- Basic MCU Initialization --- */

  /* Apply chip errata fixes */
  CHIP_Init();

  /* Enable Low Frequency Crystal Oscillator (LFXO) early */
  /* UART init will block until it's stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, false);

  /* Optional: Fill a portion of RAM with a pattern for debugging memory issues */
  /* Fills from HeapBase up to StackTop - 0x100 bytes */
  // memset(&__HeapBase, 0x5A, &__StackTop - &__HeapBase - 0x100); // Commented out, potentially for release builds

  /* Set High Frequency RC Oscillator to 21 MHz */
  CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);

  /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Example GPIO Pin setup (Purpose unclear without context, maybe LED or test pin?) */
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 1);

#if CLOCK_DEBUG
  /* Debug code to output internal clocks on GPIO pins */
  /* Select HFRCO as source for CMU_CLK0 pin */
    CMU->CTRL =(CMU->CTRL &~_CMU_CTRL_CLKOUTSEL0_MASK)| CMU_CTRL_CLKOUTSEL0_HFRCO;
//    CMU->CTRL =(CMU->CTRL &~_CMU_CTRL_CLKOUTSEL1_MASK)| CMU_CTRL_CLKOUTSEL1_HFCLK;
//    CMU->CTRL =(CMU->CTRL &~_CMU_CTRL_CLKOUTSEL0_MASK)| CMU_CTRL_CLKOUTSEL0_HFRCO;
//    CMU->CTRL =(CMU->CTRL &~_CMU_CTRL_CLKOUTSEL1_MASK)| CMU_CTRL_CLKOUTSEL1_LFRCO;
    CMU->CTRL =(CMU->CTRL &~_CMU_CTRL_CLKOUTSEL1_MASK)| CMU_CTRL_CLKOUTSEL1_LFXO;
  CMU->ROUTE = CMU_ROUTE_LOCATION_LOC0 | CMU_ROUTE_CLKOUT0PEN | CMU_ROUTE_CLKOUT1PEN;
  /* Configure PA2 and PA1 as push-pull outputs for clock signals */
  GPIO_PinModeSet(gpioPortA, 2, gpioModePushPull ,0);
  GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull ,0);
#endif
  /* Example GPIO Pin setup (Purpose unclear without context) */
  GPIO_PinModeSet(gpioPortA, 1, gpioModePushPull ,0);

  /* Disable Battery monitor input pin (PD7) to save power */
  GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0);

  /* --- Initialize Core Software Modules --- */
  delay(100); // Short delay after power-up/reset
  event_clear(); // Clear any pending events
  time_init();   // Initialize timekeeping system (requires LFXO)
  config_init(); // Initialize configuration storage (e.g., NVM)
  radio_init(true); // Initialize the DW1000 radio module
  bool accel_present = accel_init(); // Initialize accelerometer, check if present

  /* Load crystal trim value from config, apply if found */
  xtal_trim = 0x10; // Default value if not found in config
  if (config_get(config_key_xtal_trim, &xtal_trim, 1)) {
	  radio_xtal_trim(xtal_trim); // Apply trim value to radio
  }

  /* Initialize UART (waits for LFXO stability) */
  uart_init(config_get8(config_key_uart_drive)); // Use drive strength from config

  /* --- Reset Cause Handling --- */
  uint32_t reset_cause = RMU_ResetCauseGet(); // Get the cause of the last reset
  RMU_ResetCauseClear(); // Clear the reset cause flags

  /* Print reset cause to UART for debugging */
  // (Masks seem specific to EFM32G/GG, might need adjustment for other families)
#define RMU_RSTCAUSE_PORST_XMASK         0x00000000UL // Power-On Reset
#define RMU_RSTCAUSE_BODUNREGRST_XMASK   0x00000081UL // Brown-out Detector Unregulated Domain
#define RMU_RSTCAUSE_BODREGRST_XMASK     0x00000091UL // Brown-out Detector Regulated Domain
#define RMU_RSTCAUSE_EXTRST_XMASK        0x00000001UL // External Pin Reset
#define RMU_RSTCAUSE_WDOGRST_XMASK       0x00000003UL // Watchdog Timer Reset
#define RMU_RSTCAUSE_LOCKUPRST_XMASK     0x0000EFDFUL // Cortex-M Lockup Reset
#define RMU_RSTCAUSE_SYSREQRST_XMASK     0x0000EF9FUL // System Request Reset
#define RMU_RSTCAUSE_EM4RST_XMASK        0x00000719UL // EM4 Wake-up Reset
#define RMU_RSTCAUSE_EM4WURST_XMASK      0x00000619UL // EM4 Wake-up Reset by Pin
#define RMU_RSTCAUSE_BODAVDD0_XMASK      0x0000041FUL // AVDD0 Brown-out
#define RMU_RSTCAUSE_BODAVDD1_XMASK      0x0000021FUL // AVDD1 Brown-out (if applicable)

  // Print messages based on reset cause flags
  if (reset_cause & RMU_RSTCAUSE_PORST)
	  write_string("Power on reset\r\n");
  if (reset_cause & RMU_RSTCAUSE_BODUNREGRST)
	  write_string("Brownout in unregulated domain\r\n");
  if (reset_cause & RMU_RSTCAUSE_BODREGRST)
	  write_string("Brownout in regulated domain\r\n");
  if (reset_cause & RMU_RSTCAUSE_EXTRST)
	  write_string("External reset\r\n");
  if (reset_cause & RMU_RSTCAUSE_WDOGRST)
	  write_string("Watchdog reset\r\n");
  if (reset_cause & RMU_RSTCAUSE_LOCKUPRST)
	  write_string("Lockup reset\r\n");
  if (reset_cause & RMU_RSTCAUSE_SYSREQRST)
	  write_string("System requested reset\r\n");
  if (reset_cause & RMU_RSTCAUSE_EM4RST)
	  write_string("Woke up from EM4\r\n");
  if (reset_cause & RMU_RSTCAUSE_EM4WURST)
	  write_string("Woken from EM4 by pin\r\n");
  if (reset_cause & RMU_RSTCAUSE_BODAVDD0)
	  write_string("Brownout in analogue domain 0\r\n");
  if (reset_cause & RMU_RSTCAUSE_BODAVDD1)
	  write_string("Brownout in analogue domain 1\r\n");

  /* Report accelerometer status */
  if (accel_present)
      write_string("Accelerometer detected\r\n");
  else
	  write_string("No accelerometer detected\r\n");

  /* Initialize Command Line Interface */
  cli_init();

  /* --- Accelerometer Configuration (Sniff Mode) --- */
  // Read sniff mode parameters from config store, using defaults if not found
  uint8_t sniff_sensitivity = 2;
  uint8_t sniff_exponent = 0;
  uint8_t sniff_mode = 1;
  uint8_t sniff_count = 0;
  (void) config_get(config_key_accel_sensitivity, &sniff_sensitivity, 1);
  (void) config_get(config_key_accel_exponent, &sniff_exponent, 1);
  (void) config_get(config_key_accel_mode, &sniff_mode, 1);
  (void) config_get(config_key_accel_count, &sniff_count, 1);

  // Configure accelerometer for low-power motion detection (sniff mode)
  accel_enter_mode(ACCEL_STANDBY); // Enter standby to allow configuration
  accel_config_power_mode(ACCEL_ULP, ACCEL_ULP); // Ultra-Low Power mode
  accel_config_range_resolution(ACCEL_2G, ACCEL_6BITS); // +/- 2g range, 6-bit resolution
  accel_config_rate(ACCEL_ULP_ODR_25); // 25 Hz Output Data Rate in ULP mode
  accel_config_threshold(sniff_sensitivity, sniff_sensitivity, sniff_sensitivity, sniff_exponent); // Set motion detection thresholds
  if (sniff_count > 0) // Configure detection count if specified (count-1 written to register)
      accel_config_detection_count(sniff_count-1, sniff_count-1, sniff_count-1, true);
  else // Otherwise, disable count feature
      accel_config_detection_count(0, 0, 0, false);
  accel_config_sniff_mode(false, sniff_mode); // Configure sniff mode details (AND/OR logic, etc.)

  accel_enter_mode(ACCEL_SNIFF); // Enter sniff mode to detect motion

  /* --- Radio Configuration --- */
  // Configure radio LEDs based on config setting
  radio_leds(true, config_get8(config_key_radio_leds));

  // Load radio parameters from config store, overriding defaults in radio_config
  uint8_t byte;
  (void) config_get(config_key_chan, &radio_config.chan, sizeof(radio_config.chan));
  if (config_get(config_key_prf_high, &byte, 1) > 0)
    radio_config.prf_high = byte?true:false;
  (void) config_get(config_key_tx_plen, &radio_config.tx_plen, sizeof(radio_config.tx_plen));
  (void) config_get(config_key_rx_pac, &radio_config.rx_pac, sizeof(radio_config.rx_pac));
  (void) config_get(config_key_tx_pcode, &radio_config.tx_pcode, sizeof(radio_config.tx_pcode));
  (void) config_get(config_key_rx_pcode, &radio_config.rx_pcode, sizeof(radio_config.rx_pcode));
  if (config_get(config_key_ns_sfd, &byte, 1) > 0)
    radio_config.ns_sfd = byte?true:false;
  (void) config_get(config_key_data_rate, &radio_config.data_rate, sizeof(radio_config.data_rate));
  if (config_get(config_key_long_frames, &byte, 1) > 0)
    radio_config.long_frames = byte?true:false;
  (void) config_get(config_key_sfd_timeout, (uint8_t *)&radio_config.sfd_timeout, sizeof(radio_config.sfd_timeout));

  // Apply the loaded/default configuration to the radio hardware
  radio_configure(&radio_config);
  radio_doublebuffer(true); // Enable radio double buffering

  // Set radio SPI speed (true likely means high speed)
  radio_spi_speed(true);

  // Set antenna delays from config store
  set_antenna_delay_tx(config_get16(config_key_antenna_delay_tx));
  set_antenna_delay_rx(config_get16(config_key_antenna_delay_rx));

  // Configure TX power and smart TX power control from config
  uint32_t word = 0;
  if (config_get(config_key_tx_power, (uint8_t *)&word, 4) > 0)
    radio_settxpower(word);
  if (config_get(config_key_smart_tx_power, &byte, 1) > 0)
    radio_smarttxpowercontrol(byte);

  // Configure protocol timings (turnaround, timeouts) from config
  word = 0;
  if (config_get(config_key_turnaround_delay, (uint8_t *)&word, 4) > 0)
      proto_turnaround_delay(word);
  word = 0;
  if (config_get(config_key_rxtimeout, (uint8_t *)&word, 4) > 0)
    proto_rx_timeout(word);
  word = 0;
  if (config_get(config_key_rxdelay, (uint8_t *)&word, 4) > 0)
      proto_rx_delay(word);

  /* --- Initialize Final Modules --- */
  crypto_init(); // Initialize cryptographic module (e.g., load keys)
  proto_init();  // Initialize the Tail protocol state machine

  /* --- Main Loop --- */
  // This is a low-power loop that polls modules and enters sleep (EM2) when idle.
  while (1) {
    // Poll various modules for pending work/events
    time_event_poll(); // Check for time-based events
    cli_poll();        // Process command line interface input/output
    proto_poll();      // Process protocol state machine events (radio TX/RX, etc.)
    accel_poll();      // Check accelerometer events (e.g., motion detection interrupt)
    entropy_poll();    // Gather entropy for random number generation

    // Check if modules are ready to enter low-power sleep (EM2)
    // Each module indicates if it has pending work that prevents sleep.
    if (!cli_prepare_sleep())
      continue; // CLI needs processing, loop again
    if (!uart_prepare_sleep())
      continue; // UART needs processing (e.g., TX buffer not empty), loop again
    if (!time_prepare_sleep())
      continue; // Time module has pending events, loop again
    if (!timer_prepare_sleep())
      continue; // Software timer has pending events, loop again

    // All modules ready for sleep, enter Energy Mode 2 (EM2)
    // Wake-up will occur on configured interrupts (e.g., LFXO for time, GPIO for accel/UART)
    EMU_EnterEM2(true); // true = restore clocks automatically on wake-up
  }
}
