#!/usr/bin/python3
#
# xtaltrim.py - DW1000 Crystal Oscillator Frequency Offset Estimation Tool
#
# This script measures the frequency offset (in Parts Per Million - PPM) of a
# transmitting DW1000 device's crystal oscillator relative to one or more
# receiving devices. It assumes the receiving devices have clocks synchronized
# via NTP (Network Time Protocol) or a similar mechanism.
#
# It works by sending pairs of blinks from the transmitter (DUT) and recording
# both the hardware timestamps (from DW1000) and software timestamps (from the OS,
# assumed to be NTP-synced) at the receivers. By comparing the time difference
# between blinks as measured by the hardware clocks vs. the software clocks over
# an interval, the relative frequency offset can be estimated.
#
# Usage: xtaltrim.py xmit* recv1 recv2 recv3 ... [options]
#   - xmit*: The hostname/IP of the transmitting device (DUT). Mark with '*' if also a receiver.
#   - recvN: Hostnames/IPs of receiving devices (references).
#
# Output Columns (CSV):
#	0,1: Date, Time (YYYY/MM/DD, HH:MM:SS)
#	2:   Measurement Index
#	3:   Timestamp (seconds since start)
#	4:   Period (seconds, duration of calculation window)
#	5:   Erx (Avg Rx HW vs NTP clock drift, ppm)
#	6:   Etx (Avg Tx HW vs NTP clock drift, ppm)
#	7:   Ediff (Avg Erx - Etx, ppm)
#	8:   Eblks (Avg Rx/Tx HW clock ratio drift from blink pairs, ppm)
#	9:   Ettck (Avg Rx/Tx ratio from DW1000 TTICK registers, ppm)
#	10:  Pwr (Avg Rx Power, dBm)
#

import sys
import math
import argparse

import numpy as np
import subprocess as sub # Used only if programming via SSH was intended

from tail import *
from dwarf import *
from blinks import *


class cfg():
    """
    Configuration class holding global settings for the xtaltrim script.
    Defaults can be overridden by command-line arguments.
    """
    # --- Script Behavior ---
    debug            = 0       # Debug message level
    verbose          = 0       # Verbosity level

    # --- Output ---
    output           = None    # File object for CSV output (if specified)

    # --- Network ---
    rpc_port         = 9812    # Default TCP port for RPC

    # --- Measurement Timing ---
    blink_count      = 1000000 # Maximum number of measurement cycles to run
    blink_delay      = 0.100   # Delay (seconds) between the two blinks sent by the transmitter in each cycle
    blink_wait       = 1.0     # Timeout (seconds) waiting for blink responses from all receivers
    blink_interval   = 10      # Interval (seconds) between the start of consecutive measurement cycles

    # --- Analysis Window ---
    window_length    = 100     # Number of past measurements to include in the sliding window for calculation
    window_skip      = 10      # Minimum number of measurements in the window before calculating/printing results

# Global dictionary to store measurement data, keyed by measurement index
DATA = {}

# ---------------------------------------------------------------------------
# Logging/Printing Helpers
# ---------------------------------------------------------------------------

def dprint(level, *args, **kwargs):
    """Prints debug messages if the debug level is high enough."""
    if cfg.debug >= level:
        print(*args, file=sys.stderr, flush=True, **kwargs)

def veprint(level, *args, **kwargs):
    """Prints verbose messages if the verbosity level is high enough."""
    if cfg.verbose >= level:
        print(*args, file=sys.stderr, flush=True, **kwargs)

def veprints(level, *args, **kwargs):
    """Prints verbose messages without a newline if the verbosity level is high enough."""
    if cfg.verbose >= level:
        print(*args, file=sys.stderr, end='', flush=True, **kwargs)

# ---------------------------------------------------------------------------
# Output Function
# ---------------------------------------------------------------------------

def print_csv(index,timex,period,Ravg,Tavg,Davg,Xavg,Favg,Pavg):
    """
    Formats and writes a line of calculated results to the CSV output file (if configured).

    Args:
        index (int): Current measurement index.
        timex (float): Timestamp of the current measurement.
        period (float): Time difference since the start of the calculation window.
        Ravg (float): Average Rx HW vs NTP clock drift (ppm).
        Tavg (float): Average Tx HW vs NTP clock drift (ppm).
        Davg (float): Average difference (Erx - Etx) (ppm).
        Xavg (float): Average Rx/Tx HW clock ratio drift (ppm).
        Favg (float): Average Rx/Tx ratio from DW1000 TTICK (ppm).
        Pavg (float): Average Rx Power (dBm).
    """
    if cfg.output is not None:
        msg  = time.strftime('%Y/%m/%d,%H:%M:%S')               # 0,1 Date, Time
        msg += ',{}'.format(index)                              # 2 Index
        msg += ',{:.3f}'.format(timex)                          # 3 Timestamp (s)
        msg += ',{:.3f}'.format(period)                         # 4 Period (s)
        msg += ',{:.3}'.format(Ravg)                            # 5 Erx (ppm)
        msg += ',{:.3}'.format(Tavg)                            # 6 Etx (ppm)
        msg += ',{:.3}'.format(Davg)                            # 7 Ediff (ppm)
        msg += ',{:.3}'.format(Xavg)                            # 8 Eblks (ppm)
        msg += ',{:.3}'.format(Favg)                            # 9 Ettck (ppm)
        msg += ',{:.3}'.format(Pavg)                            # 10 Pwr (dBm)
        msg += '\n'
        cfg.output.write(msg)

# ---------------------------------------------------------------------------
# Measurement & Calculation Functions
# ---------------------------------------------------------------------------

def estimate_xtal_ppm(blk, tx, rxs, devs, INDEX):
    """
    Performs one measurement cycle: sends two blinks from the transmitter (tx)
    and collects timestamps (HW and SW) and diagnostic info from all receivers (rxs).
    Stores the collected data in the global DATA dictionary under the given INDEX.

    Args:
        blk (Blinks): The Blinks manager instance.
        tx (DW1000): The transmitting device (DUT).
        rxs (list): List of receiving DW1000 devices (references).
        devs (list): List of all devices involved (tx + rxs).
        INDEX (int): The index for the current measurement cycle.

    Returns:
        bool: True if measurements were successfully collected from all receivers, False otherwise.
    """

    OK = True # Flag to track if data was received from all receivers

    DATA[INDEX] = {} # Create entry for this measurement index
    DATA[INDEX]['TIME'] = blk.time() # Record the start time of this cycle

    # Send two blinks from the transmitter with a short delay
    i1 = blk.blink(tx)
    blk.nap(cfg.blink_delay) # Wait for the configured delay
    i2 = blk.blink(tx)

    # Wait for all devices (transmitter and receivers) to report receiving both blinks
    blk.wait_blinks((i1,i2), devs, cfg.blink_wait)

    # Initialize sums for averaging later (unused in this function, but kept for structure?)
    Fcnt = 0
    Xsum = 0.0
    Fsum = 0.0
    Psum = 0.0
    Rsum = 0.0
    Tsum = 0.0

    # Process data received by each receiver
    for rx in rxs:
        try:
            key = rx.eui # Use receiver EUI as the key

            # Retrieve timestamps for both blinks (i1, i2) from both devices (tx, rx)
            # Note: get_hwts(i, tx) might fail if tx isn't also listed as a receiver
            T1 = blk.get_hwts(i1, tx) # HW time of blink 1 at Tx (from Tx report if available, else error)
            R1 = blk.get_hwts(i1, rx) # HW time of blink 1 at Rx
            S1 = blk.get_swts(i1, rx) # SW time of blink 1 at Rx (NTP synced)

            T2 = blk.get_hwts(i2, tx) # HW time of blink 2 at Tx
            R2 = blk.get_hwts(i2, rx) # HW time of blink 2 at Rx
            S2 = blk.get_swts(i2, rx) # SW time of blink 2 at Rx

            # Get crystal ratio and power info from receiver reports
            F1 = blk.get_xtal_ratio(i1, rx) # TTICK ratio from blink 1
            F2 = blk.get_xtal_ratio(i2, rx) # TTICK ratio from blink 2

            P1 = blk.get_rx_level(i1, rx) # Estimated Rx power level from blink 1
            P2 = blk.get_rx_level(i2, rx) # Estimated Rx power level from blink 2

            # Calculate time differences
            T21 = T2 - T1 # HW time difference at Tx
            R21 = R2 - R1 # HW time difference at Rx

            # Calculate instantaneous Rx/Tx HW clock ratio drift (ppm)
            Xrt = (R21 - T21) / R21

            # Average the crystal ratio and power readings from the two blinks
            F12 = (F1 + F2) / 2
            P12 = (P1 + P2) / 2

            # Convert average power level to dBm
            Pdb = RxPower2dBm(P12, cfg.prf)

            # Store relevant data for this receiver and index
            DATA[INDEX][key] = {}
            DATA[INDEX][key]['HWTx']  = T1 # HW timestamp of first blink at Tx
            DATA[INDEX][key]['HWRx']  = R1 # HW timestamp of first blink at Rx
            DATA[INDEX][key]['SWRx']  = S1 # SW timestamp of first blink at Rx
            DATA[INDEX][key]['Xrt']   = Xrt # Instantaneous Rx/Tx HW clock ratio drift
            DATA[INDEX][key]['Pwr']   = P12 # Average raw Rx power
            DATA[INDEX][key]['Fpt']   = F12 # Average TTICK ratio
            DATA[INDEX][key]['Pdb']   = Pdb # Average Rx power in dBm

        except Exception as e: # Catch any error during data retrieval/calculation for this receiver
            # dprint(1, f"Error processing data for {rx.name} at index {INDEX}: {e}") # Optional debug print
            OK = False # Mark this cycle as potentially incomplete

    return OK


def calculate_xtal_sync(blk, tx, rxs, devs, INDEX, START):
    """
    Calculates and prints clock drift estimates based on data from two measurement points.
    Compares timestamps between a starting index (START) and the current index (INDEX)
    to determine drift over the measurement period.

    Calculates:
    - Erx: Drift of receiver HW clock relative to receiver SW (NTP) clock (ppm).
    - Etx: Drift of transmitter HW clock relative to receiver SW (NTP) clock (ppm).
    - Ediff: Difference between Erx and Etx (ppm).
    - Eblks: Instantaneous drift estimate based on HW timestamp ratios within one cycle (ppm).
    - Ettck: Instantaneous drift estimate based on DW1000 TTICK registers (ppm).

    Args:
        blk (Blinks): The Blinks manager instance (unused here, data comes from DATA dict).
        tx (DW1000): The transmitting device.
        rxs (list): List of receiving DW1000 devices.
        devs (list): List of all devices involved (unused here).
        INDEX (int): The index of the current (later) measurement cycle.
        START (int): The index of the starting (earlier) measurement cycle.
    """

    # Get timestamps for the start and end of the calculation window
    t0 = DATA[START]['TIME']
    t1 = DATA[INDEX]['TIME']

    # Format current time for display
    t = int(t1)
    h  = (t // 3600)
    m  = (t % 3600) // 60
    s  = t % 60

    period = t1 - t0 # Duration of the calculation window

    veprint(1,'\n** BLINK #{} @ {}:{}:{} -- TIME:{:.3f}s PERIOD:{:.3f}s\n'.format(INDEX, h,m,s,t1,period))

    # Initialize sums for averaging results across receivers
    Fcnt = 0    # Count of successful calculations for this window
    Xsum = 0.0  # Sum of Eblks
    Fsum = 0.0  # Sum of Ettck
    Psum = 0.0  # Sum of raw power
    Rsum = 0.0  # Sum of Erx
    Tsum = 0.0  # Sum of Etx

    # Print header for detailed output
    veprint(1,'    ANCHOR          Erx        Etx        Ediff      Eblks      Ettck      Pwr')
    veprint(1,'    ===============================================================================')

    # Calculate drift for each receiver over the window
    for rx in rxs:
        try:
            key = rx.eui

            # Retrieve stored data for the start and end points
            T0 = DATA[START][key]['HWTx'] # Tx HW time at START
            R0 = DATA[START][key]['HWRx'] # Rx HW time at START
            S0 = DATA[START][key]['SWRx'] # Rx SW time at START

            T1 = DATA[INDEX][key]['HWTx'] # Tx HW time at INDEX
            R1 = DATA[INDEX][key]['HWRx'] # Rx HW time at INDEX
            S1 = DATA[INDEX][key]['SWRx'] # Rx SW time at INDEX

            # Retrieve instantaneous estimates from the current index
            Xrt = DATA[INDEX][key]['Xrt'] # Eblks (instantaneous)
            Pwr = DATA[INDEX][key]['Pwr'] # Raw power
            Fpt = DATA[INDEX][key]['Fpt'] # Ettck (instantaneous)
            Pdb = DATA[INDEX][key]['Pdb'] # Power in dBm

            # Calculate time differences over the window period
            T10 = T1 - T0 # Tx HW time difference
            R10 = R1 - R0 # Rx HW time difference
            S10 = S1 - S0 # Rx SW time difference (NTP based)

            # Calculate drift rates relative to NTP time (S10)
            XrtRxNtp = (R10 - S10) / S10 # Erx: Rx HW drift vs NTP
            XrtTxNtp = (T10 - S10) / S10 # Etx: Tx HW drift vs NTP (as seen by Rx NTP)

            # Accumulate sums for averaging
            Fcnt += 1
            Rsum += XrtRxNtp
            Tsum += XrtTxNtp
            Xsum += Xrt # Accumulate instantaneous Eblks
            Fsum += Fpt # Accumulate instantaneous Ettck
            Psum += Pwr # Accumulate raw power

            # Format and print results for this receiver
            msg = '    '
            msg += '{:<12s}  '.format(rx.host)
            msg += '{:7.3f}ppm '.format(XrtRxNtp*1E6) # Erx
            msg += '{:7.3f}ppm '.format(XrtTxNtp*1E6) # Etx
            msg += '{:7.3f}ppm '.format((XrtRxNtp-XrtTxNtp)*1E6) # Ediff
            msg += '{:7.3f}ppm '.format(Xrt*1E6) # Eblks
            msg += '{:7.3f}ppm '.format(Fpt*1E6) # Ettck
            msg += '{:6.1f}dBm '.format(Pdb) # Pwr

            veprint(1, msg)

        except (ZeroDivisionError,KeyError,IndexError): # Handle cases where data might be missing or invalid
            veprint(1,'    {:<12s}  -- Error calculating drift --'.format(rx.host))

    # Calculate averages if any successful calculations were made
    if Fcnt > 0:
        Ravg = Rsum/Fcnt * 1E6
        Tavg = Tsum/Fcnt * 1E6
        Xavg = Xsum/Fcnt * 1E6
        Favg = Fsum/Fcnt * 1E6

        Pavg = Psum/Fcnt
        Pwr_dBm  = RxPower2dBm(Pavg,cfg.prf) # Convert average raw power to dBm

        # Write averages to CSV file
        print_csv(INDEX, t1, period, Ravg, Tavg, Ravg-Tavg, Xavg, Favg, Pwr_dBm)

        # Format and print average results
        msg = '    AVERAGE       '
        msg += '{:7.3f}ppm '.format(Ravg)
        msg += '{:7.3f}ppm '.format(Tavg)
        msg += '{:7.3f}ppm '.format(Ravg-Tavg)
        msg += '{:7.3f}ppm '.format(Xavg)
        msg += '{:7.3f}ppm '.format(Favg)
        msg += '{:6.1f}dBm '.format(Pwr_dBm)

        veprint(1,'    ===============================================================================')
        veprint(1, msg)
    else:
        veprint(1, "    -- No valid data for averaging in this window --")


# ---------------------------------------------------------------------------
# Main Execution
# ---------------------------------------------------------------------------

def main():
    """
    Main function: Parses arguments, connects to devices, runs the measurement loop,
    calculates drift over a sliding window, and prints/saves results.
    """
    # --- Argument Parsing ---
    parser = argparse.ArgumentParser(description="DW1000 Crystal Trim (XTAL) NTP Comparison Tool")

    # Add arguments for setting DW1000 parameters (defined in blinks.py)
    DW1000.add_device_arguments(parser)

    # Script specific arguments
    parser.add_argument('-D', '--debug', action='count', default=0)
    parser.add_argument('-v', '--verbose', action='count', default=0)
    parser.add_argument('-n', '--count', type=int, default=cfg.blink_count, help="Max number of measurement cycles")
    parser.add_argument('-d', '--delay', type=float, default=cfg.blink_delay, help="Delay between blink pairs (s)")
    parser.add_argument('-w', '--wait', type=float, default=cfg.blink_wait, help="Timeout waiting for blinks (s)")
    parser.add_argument('-i', '--interval', type=float, default=cfg.blink_interval, help="Interval between measurement cycles (s)")
    parser.add_argument('-l', '--length', type=int, default=cfg.window_length, help="Sliding window length (samples)") # Changed type to int
    parser.add_argument('-p', '--port', type=int, default=cfg.rpc_port, help="RPC port")
    parser.add_argument('-s', '--skip', type=int, default=cfg.window_skip, help="Min samples in window before calc")
    parser.add_argument('-o', '--output', type=str, default=None, help="Output CSV file path")
    parser.add_argument('remote', type=str, nargs='+', help="Remote address (transmitter marked with *, e.g., tag01*)")

    args = parser.parse_args()

    # --- Apply Configuration ---
    cfg.debug = args.debug
    cfg.verbose = args.verbose
    WPANFrame.verbosity = args.verbose # Set verbosity for underlying TailFrame printing

    # Open output file if specified
    if args.output is not None:
        try:
            cfg.output = open(args.output, 'w')
            # Write CSV header
            cfg.output.write("Date,Time,Index,Timestamp,Period,Erx_ppm,Etx_ppm,Ediff_ppm,Eblks_ppm,Ettck_ppm,Pwr_dBm\n")
        except IOError as e:
            eprint(f"Error opening output file {args.output}: {e}")
            sys.exit(1)

    # Apply timing and window parameters from args
    cfg.blink_count    = args.count
    cfg.blink_delay    = args.delay
    cfg.blink_wait     = args.wait
    # Adjust interval: total time between starts of cycles = interval arg
    cfg.blink_interval = args.interval - args.delay # Time to wait *after* sending blink pair
    cfg.window_skip    = args.skip
    cfg.window_length  = args.length

    # --- Device Initialization ---

    rpc = RPC(udp_port=args.port + 1 if args.port else cfg.rpc_port + 1) # Use different UDP port based on RPC port

    remotes  = [ ] # List of all connected DW1000 objects
    xmitters = [ ] # List of transmitters (DUTs)
    rceivers = [ ] # List of receivers (references)

    # Connect to and categorize devices specified on command line
    for host_arg in args.remote:
        try:
            is_xmit = host_arg.startswith('*') or host_arg.endswith('*')
            host = host_arg.strip('*')
            name = host.split('.')[0] # Simple name from hostname
            # Create DW1000 object (connects via RPC)
            anchor = DW1000(rpc, name, host, args.port if args.port else cfg.rpc_port)
            anchor.connect()
            remotes.append(anchor)
            if is_xmit:
                xmitters.append(anchor)
            # Allow a device to be both transmitter and receiver if marked with '*'
            if not is_xmit or (is_xmit and len(host_arg) > len(host)): # Check if '*' was actually present
                 rceivers.append(anchor)

        except (ValueError, ConnectionError, OSError) as err: # Catch potential errors during connection
            eprint(f'Error connecting to remote host {host}: {err}')
            rpc.stop() # Stop RPC thread before exiting
            sys.exit(1)
        except Exception as err: # Catch unexpected errors
             eprint(f'Unexpected error setting up host {host}: {err}')
             rpc.stop()
             sys.exit(1)


    # Apply any device-specific arguments passed via command line
    DW1000.handle_device_arguments(args,remotes)

    # Print initial device status if verbose
    if args.verbose > 0:
        DW1000.print_all_remote_attrs(remotes,True)

    # --- Measurement Loop ---

    blk = Blinks(rpc) # Initialize Blinks manager to handle TX/RX messages

    # Assume the first transmitter specified is the Device Under Test (DUT)
    if not xmitters:
        eprint("Error: No transmitter specified (use '*' suffix).")
        rpc.stop()
        sys.exit(1)
    if not rceivers:
        eprint("Error: No receivers specified.")
        rpc.stop()
        sys.exit(1)

    dut = xmitters[0]

    # Store PRF for power calculations
    try:
        cfg.prf = int(dut.get_dw1000_attr('prf'))
    except Exception as e:
        eprint(f"Error getting PRF from DUT {dut.name}: {e}")
        rpc.stop()
        sys.exit(1)


    INDEX = 0 # Initialize measurement index
    try:
        # Perform initial measurement (INDEX=0) to establish baseline
        veprint(1, "Performing initial measurement...")
        while not estimate_xtal_ppm(blk, dut, rceivers, remotes, INDEX=0):
            eprint('First measurement failed. Trying again.')
            blk.nap(1.0) # Wait a second before retrying

        # Main measurement loop
        for i in range(1, cfg.blink_count):
            INDEX = i
            blk.nap(cfg.blink_interval) # Wait for the interval between cycles
            try:
                # Perform the measurement for the current cycle
                if not estimate_xtal_ppm(blk, dut, rceivers, remotes, INDEX=i):
                    eprint(f"Warning: Incomplete data for measurement index {i}")
                    # Optionally skip calculation if data is incomplete
                    # continue

                # Determine the start index for the sliding window
                start_index = max(0, i - cfg.window_length)
                window_size = i - start_index

                # Calculate and print/save results if window is large enough
                if window_size >= cfg.window_skip:
                    calculate_xtal_sync(blk, dut, rceivers, remotes, INDEX=i, START=start_index)

            except (TimeoutError,ValueError,TypeError,KeyError,IndexError,ZeroDivisionError) as e:
                eprint(f'Error during measurement/calculation at index {i}: {e}. Continuing...')
            except Exception as e: # Catch unexpected errors
                 eprint(f'Unexpected error at index {i}: {e}. Continuing...')


    except KeyboardInterrupt: # Allow graceful exit on Ctrl+C
        eprint('\nStopping...')
    except RuntimeError as err: # Catch runtime errors (e.g., connection issues)
        errhandler('Runtime error', err)
    except Exception as e: # Catch any other unexpected errors
        errhandler('Unexpected error in main loop', e)

    # --- Cleanup ---
    veprint(1, "Stopping RPC...")
    blk.stop() # Stop Blinks manager (currently no-op)
    rpc.stop() # Stop RPC thread
    if cfg.output:
        cfg.output.close() # Close output file


if __name__ == "__main__":
    main()
