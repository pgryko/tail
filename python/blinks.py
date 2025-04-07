#!/usr/bin/python3
#
# blinks.py - Helper classes and functions for managing Tail UWB devices and blinks.
#
# This module provides:
# - Timer: A simple class for managing timed operations and delays.
# - DW1000: Represents a remote DW1000 device, handling configuration and communication via RPC.
# - RPC: A basic Remote Procedure Call mechanism over TCP/UDP for interacting with DW1000 devices.
# - Blink: Represents a single received UWB transmission (blink) with associated metadata.
# - Blinks: Manages collections of Blinks, correlating them by Beacon ID (bid) and handling RPC callbacks.
#

import sys
import math
import time
import json
import socket
import select
import threading
import traceback

import numpy as np

from tail import *
from dwarf import *


class Timer:
    """
    A simple timer class for managing delays and measuring elapsed time.
    Uses time.time() and provides methods for precise(ish) sleeping ('nap')
    and synchronization.
    """
    SLEEP_MIN = 500E-6   # Minimum sleep time threshold (500us) - avoids busy-waiting for very short delays.

    def __init__(self):
        """Initializes the timer, setting the start time."""
        self.start = time.time()
        self.timer = self.start

    def get(self):
        """Returns the elapsed time since the timer was started or last synced."""
        return self.timer - self.start

    def nap(self,delay):
        """
        Sleeps until a target time (current timer + delay) is reached.
        Uses iterative sleeping for potentially better precision than a single sleep.

        Args:
            delay (float): The duration to add to the current timer target.

        Returns:
            float: The total elapsed time since the timer started.
        """
        self.timer += delay
        while True:
            sleep = self.timer - time.time()
            if sleep < Timer.SLEEP_MIN:
                break
            time.sleep(sleep/2)
        return self.get()

    def sync(self,delay=0.0):
        """
        Waits for a specified delay using nap() and then resets the timer's
        internal target to the current time. Useful for synchronizing loops.

        Args:
            delay (float, optional): Delay to wait before syncing. Defaults to 0.0.

        Returns:
            float: The total elapsed time just before the sync occurred.
        """
        self.nap(delay)
        self.timer = time.time()
        return self.get()


class DW1000:
    """
    Represents a remote DW1000 device (likely an anchor) accessible via RPC.
    Handles connection, configuration (getting/setting attributes), and sending commands.
    """
    # Tuple of DW1000 attributes that can be set via command-line arguments or config
    DEV_ATTRS = (
        'channel',
        'prf',
        'pcode',
        'txpsr',
        'rate',
        'smart_power',
        'tx_power',
        'xtalt',
        'antd',
        'profile',
        'snr_threshold',
        'fpr_threshold',
        'noise_threshold', # Noise threshold for detection
    )

    # Tuple of DW1000 attributes typically printed for information
    PRINT_ATTRS = (
        'channel',
        'prf',
        'pcode',
        'txpsr',
        'rate',
        'smart_power',
        'tx_power',
        'xtalt',
        'antd', # Antenna delay
    )

    def __init__(self, rpc, name, host, port=None, coord=None):
        """
        Initializes a DW1000 device representation.

        Args:
            rpc (RPC): The RPC instance used for communication.
            name (str): A user-friendly name for this device.
            host (str): The hostname or IP address of the device.
            port (int, optional): The TCP port for RPC communication. Defaults to None (must be set later or via config).
            coord (np.array, optional): The 3D coordinates of the device. Defaults to None.
        """
        self.rpc = rpc
        self.pipe = None
        self.name = name
        self.host = host
        self.port = port
        self.coord = coord # 3D coordinates (numpy array)
        self.eui = None    # EUI-64 address (retrieved after connection)

    def load_config(self, **kwargs):
        """Loads configuration parameters from keyword arguments."""
        for (attr,value) in kwargs.items():
            try:
                getattr(self,attr)
                setattr(self,attr,value)
            except AttributeError:
                eprint(f'DW1000: Invalid config option: {attr} := {value}')

    @classmethod
    def from_config(cls, rpc, **kwargs):
        """Class method to create and configure a DW1000 instance from kwargs."""
        dev = cls(rpc)
        dev.load_config(kwargs)

    def connect(self):
        """Establishes a TCP connection to the device, registers with RPC, gets EUI, and registers for UDP broadcasts."""
        self.pipe = TCPTailPipe()
        self.pipe.connect(self.host,self.port)
        self.rpc.add_device(self)
        self.eui = self.get_eui()
        self.register_udp_bcast()

    def disconnect(self):
        """Disconnects from the device, unregisters from RPC, and closes the pipe."""
        self.rpc.del_device(self)
        self.pipe.close()
        self.pipe = None

    def sendmsg(self,msg):
        """Sends a pre-formatted JSON string message over the TCP pipe."""
        self.pipe.sendmsg(msg)

    def sendudp(self,msg):
        """Sends a pre-formatted JSON string message via UDP to the device's address."""
        self.rpc.sendudpmsgto(msg,self.pipe.remote)

    def sendargs(self,**kwargs):
        """Sends keyword arguments as a JSON message over the TCP pipe."""
        msg = json.dumps(kwargs)
        self.sendmsg(msg)

    def sendargsudp(self,**kwargs):
        """Sends keyword arguments as a JSON message via UDP."""
        msg = json.dumps(kwargs)
        self.sendudp(msg)

    def register_udp_bcast(self):
        """Sends a command to the device to start sending UDP broadcasts to the RPC server's UDP port."""
        self.sendargs(Type='UDP', Port=self.rpc.uport)

    def unregister_udp_bcast(self):
        """Sends a command to the device to stop sending UDP broadcasts."""
        self.sendargs(Type='NOUDP')

    def get_eui(self):
        """Retrieves the EUI-64 address of the remote device via RPC."""
        args = self.rpc.call(self, Func='GETEUI')
        return args.get('Value',None)

    def get_dt_attr(self,attr,form):
        """Retrieves a device tree attribute from the remote device via RPC."""
        args = self.rpc.call(self, Func='GETDTATTR', Attr=attr, Format=form)
        return args.get('Value',None)

    def get_dw1000_attr(self,attr):
        """Retrieves a DW1000 sysfs attribute from the remote device via RPC."""
        args = self.rpc.call(self, Func='GETDWATTR', Attr=attr)
        return args.get('Value',None)

    def set_dw1000_attr(self,attr,val):
        """
        Sets a DW1000 sysfs attribute on the remote device via RPC.
        Applies validation/conversion using CONV_ATTR if applicable.
        """
        if attr in self.CONV_ATTR:
            val = self.CONV_ATTR[attr](val)
        args = self.rpc.call(self, Func='SETDWATTR', Attr=attr, Value=val)
        return args.get('Value',None)

    def print_dw1000_attrs(self):
        """Prints the standard DW1000 attributes (from PRINT_ATTRS) for this device."""
        eprint('{} <{}>'.format(self.name,self.eui))
        for attr in DW1000.PRINT_ATTRS:
            val = self.get_dw1000_attr(attr)
            eprint('  {:20s}: {}'.format(attr,val))

    def distance_to(self,rem):
        """Calculates the Euclidean distance between this device and another DW1000 device."""
        D = self.coord - rem.coord
        return np.sqrt(np.dot(D,D.T))


    ##
    # ---------------------------
    # Static / Class Methods
    # ---------------------------

    @staticmethod
    def print_all_remote_attrs(remotes,summary=False):
        """
        Prints the attributes (from PRINT_ATTRS) for a list of remote DW1000 devices.

        Args:
            remotes (list): A list of DW1000 objects.
            summary (bool, optional): If True, prints a compact summary table.
                                      If False, prints detailed attributes per device.
                                      Defaults to False.
        """
        if summary:
            eprints(' {:24s}'.format('HOSTS'))
            for rem in remotes:
                eprints(' {:10s}'.format(rem.name[:9]))
            for attr in DW1000.PRINT_ATTRS:
                eprints('\n  {:20s}:  '.format(attr))
                for rem in remotes:
                    value = rem.get_dw1000_attr(attr)
                    eprints(' {:10s}'.format(str(value)))
            eprint()
        else:
            for rem in remotes:
                eprint('{} <{}>'.format(rem.name,rem.eui))
                for attr in DW1000.PRINT_ATTRS:
                    value = rem.get_dw1000_attr(attr)
                    eprint('  {:20s}: {}'.format(attr,value))

    @staticmethod
    def add_print_arguments(parser):
        """Adds command-line arguments to an argparse parser for printing device attributes."""
        parser.add_argument('--print-eui', action='store_true', default=False, help='Print EUI64 value')
        for attr in DW1000.PRINT_ATTRS:
            parser.add_argument('--print-'+attr, action='store_true', default=False, help='Print attribute <{}> value'.format(attr))

    @staticmethod
    def handle_print_arguments(args,remotes):
        """
        Handles the command-line arguments added by add_print_arguments.
        Prints the requested attributes for the given remotes.

        Args:
            args: The parsed arguments object from argparse.
            remotes (list): A list of DW1000 objects.

        Returns:
            bool: True if any attribute was printed, False otherwise.
        """
        ret = False
        for rem in remotes:
            if getattr(args, 'print_eui'):
                print(rem.eui)
            for attr in DW1000.PRINT_ATTRS:
                if getattr(args, 'print_'+attr):
                    val = rem.get_dw1000_attr(attr)
                    ret = True
                    print(val)
        return ret

    @staticmethod
    def add_device_arguments(parser):
        """Adds command-line arguments to an argparse parser for setting device attributes."""
        for attr in DW1000.DEV_ATTRS:
            parser.add_argument('--' + attr, type=str, default=None)

    @staticmethod
    def handle_device_arguments(args,remotes):
        """
        Handles the command-line arguments added by add_device_arguments.
        Sets the specified attributes on the given remote devices via RPC.

        Args:
            args: The parsed arguments object from argparse.
            remotes (list): A list of DW1000 objects.
        """
        for attr in DW1000.DEV_ATTRS:
            val = getattr(args,attr)
            if val is not None:
                for rem in remotes:
                    rem.set_dw1000_attr(attr, val)

    # ---------------------------
    # Attribute Conversions & Validation (Static Methods)
    # ---------------------------
    # These methods handle the conversion between different representations of DW1000 settings
    # (e.g., register values, human-readable strings, lists) and validate input values.

    @staticmethod
    def tx_power_code_to_list(code):
        """Converts a DW1000 TX power code (byte) into a list [coarse_dB, fine_dB]."""
        if isinstance(code,str):
            code = int(code,0)
        C = (code >> 5) & 0x07
        F = (code >> 0) & 0x1f
        c = (6 - C) * 3.0
        f = F * 0.5
        return [ c, f ]

    @staticmethod
    def tx_power_list_to_code(lst):
        """Converts a list [coarse_dB, fine_dB] into a DW1000 TX power code (byte)."""
        c = int(lst[0] / 3)
        d = int(lst[1] * 2)
        if c<0 or c>6:
            raise ValueError
        if d<0 or d>31:
            raise ValueError
        return (((6 - c) << 5) | d)

    @staticmethod
    def tx_power_string_to_code(txpwr):
        """
        Converts a TX power string (e.g., "15+10.5" or "0xAB") into a power code (byte).
        Handles both dB format and direct hex/int input.
        """
        T = txpwr.split('+')
        n = len(T)
        if n == 2:
            a = float(T[0])
            b = float(T[1])
            c = int(a / 3)
            d = int(b * 2)
            if c<0 or c>6:
                raise ValueError
            if a != 3*c:
                raise ValueError
            if d<0 or d>31:
                raise ValueError
            if b != d/2:
                raise ValueError
            return (((6 - c) << 5) | d)
        elif n == 1:
            a = int(txpwr,0)
            return a
        else:
            raise ValueError

    @staticmethod
    def tx_power_reg_to_list(reg):
        """Extracts the TX power code from the higher bytes of a register value and converts to list."""
        if isinstance(reg,str):
            reg = int(reg,0)
        return DW1000.tx_power_code_to_list(reg >> 16)

    @staticmethod
    def tx_power_to_reg(txpwr):
        """
        Converts various TX power representations (int, float, string "C+F", string "A:B:C:D", hex string)
        into the 32-bit register format (often repeating codes, e.g., 0xABABABAB).
        """
        if isinstance(txpwr,int):
            return txpwr
        
        elif isinstance(txpwr,float):
            return int(txpwr) ## convert to a+b?
            
        elif isinstance(txpwr,str):
            T = txpwr.split(':')
            n = len(T)
            if n == 4:
                A = DW1000.tx_power_string_to_code(T[0])
                B = DW1000.tx_power_string_to_code(T[1])
                C = DW1000.tx_power_string_to_code(T[2])
                D = DW1000.tx_power_string_to_code(T[3])
                if A<0 or B<0 or C<0 or D<0:
                    raise ValueError
                if A>255 or B>255 or C>255 or D>255:
                    raise ValueError
                return '0x{:02x}{:02x}{:02x}{:02x}'.format(A,B,C,D)
            elif n == 2:
                A = DW1000.tx_power_string_to_code(T[0])
                B = DW1000.tx_power_string_to_code(T[1])
                if A<0 or B<0:
                    raise ValueError
                if A>255 or B>255:
                    raise ValueError
                return '0x{:02x}{:02x}{:02x}{:02x}'.format(A,A,B,B)
            elif n == 1:
                A = DW1000.tx_power_string_to_code(T[0])
                if A<0:
                    raise ValueError
                if A<256:
                    return '0x{:02x}{:02x}{:02x}{:02x}'.format(A,A,A,A)
                else:
                    return '0x{:08x}'.format(A)
        raise ValueError

    @staticmethod
    def validate_txpsr(psr):
        """Validates the TX Preamble Symbol Repetitions (PSR) value."""
        psr = int(psr)
        if psr in (64,128,256,512,1024,2048,4096):
            return psr
        raise ValueError(f'Invalid PSR {psr}')

    @staticmethod
    def validate_rate(rate):
        """Validates the data rate (kbps)."""
        rate = int(rate)
        if rate in (110,850,6800):
            return rate
        raise ValueError(f'Invalid rate {rate}')

    @staticmethod
    def validate_prf(prf):
        """Validates the Pulse Repetition Frequency (MHz)."""
        prf = int(prf)
        if prf in (16,64):
            return prf
        raise ValueError(f'Invalid PRF {prf}')

    @staticmethod
    def validate_channel(ch):
        """Validates the UWB channel number."""
        ch = int(ch)
        if ch in (1,2,3,4,5,7):
            return ch
        raise ValueError(f'Invalid Channel {ch}')

    # Dictionary mapping attribute names to their validation/conversion functions
    CONV_ATTR = {
        'channel'   : validate_channel,
        'prf'       : validate_prf,
        'rate'      : validate_rate,
        'txpsr'     : validate_txpsr,
        'tx_power'  : tx_power_to_reg,
    }


class RPC:
    """
    Implements a simple RPC (Remote Procedure Call) mechanism.
    Listens for incoming TCP connections (from DW1000 devices) and UDP messages.
    Handles registration of devices and message handlers.
    Manages asynchronous function calls to remote devices and waits for responses.
    Runs its own listener thread.
    """
    def __init__(self, udp_port=9813):
        """
        Initializes the RPC server.

        Args:
            udp_port (int, optional): The UDP port to listen on for broadcasts/responses.
                                      Defaults to 9813.
        """
        self.running = False
        self.seqnum = 1
        self.pipes = {}
        self.calls = {}
        self.handler = {}
        self.socks = select.poll()
        self.uport = udp_port
        self.upipe = UDPTailPipe()
        self.upipe.bind('', udp_port)
        self.ufile = self.upipe.fileno()
        self.pipes[self.ufile] = self.upipe
        self.socks.register(self.upipe, select.POLLIN)
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        """The main loop for the RPC listener thread. Polls sockets and dispatches received messages."""
        self.running = True
        while self.running:
            for (fd,flags) in self.socks.poll(100):
                if flags & select.POLLIN:
                    if fd in self.pipes:
                        self.recvpipe(self.pipes[fd])

    def stop(self):
        """Stops the RPC listener thread."""
        self.running = False

    def sendudpmsgto(self,msg,addr):
        """Sends a UDP message to a specific address."""
        self.upipe.sendmsgto(msg,addr)

    def recvpipe(self,pipe):
        """Reads and processes all available messages from a given TailPipe."""
        try:
            pipe.fillbuf()
            while pipe.hasmsg():
                mesg = pipe.getmsg()
                self.recvmsg(mesg)
        except Exception as err:
            errhandler('RPC::recvpipe: Unable to decode', err)

    def recvmsg(self,mesg):
        """
        Parses a received JSON message string and dispatches it to the appropriate handler.
        Handles generic message types and specific RPC call returns.
        """
        try:
            data = json.loads(mesg)
            Type = data.get('Type')
            if Type in self.handler:
                self.handler[Type](data)
            elif Type == 'RPC':
                func = data.get('Func')
                seqn = data.get('Seqn')
                hand = f'RPC:{func}:{seqn}'
                if hand in self.handler:
                    self.handler[hand](data)

        except Exception as err:
            errhandler('RPC::recvmsg: Invalid message received: {}'.format(mesg), err)

    def add_device(self,dev):
        """Registers a connected DW1000 device (and its pipe) with the RPC server."""
        self.lock.acquire()
        self.socks.register(dev.pipe.sock, select.POLLIN)
        self.pipes[dev.pipe.fileno()] = dev.pipe
        self.lock.release()

    def del_device(self,dev):
        """Unregisters a DW1000 device from the RPC server."""
        self.lock.acquire()
        self.socks.unregister(dev.pipe.sock)
        self.pipes.pop(dev.pipe.fileno(),None)
        self.lock.release()

    def register(self,name,func):
        """Registers a handler function for a specific message type or RPC call."""
        self.lock.acquire()
        self.handler[name] = func
        self.lock.release()

    def unregister(self,name):
        """Unregisters a handler function."""
        self.lock.acquire()
        self.handler.pop(name,None)
        self.lock.release()

    def get_seqnum(self):
        """Generates a unique sequence number for an RPC call (thread-safe)."""
        self.lock.acquire()
        seqn = self.seqnum
        self.seqnum += 1
        self.lock.release()
        return seqn

    # ---------------------------
    # Remote Function Call Handling
    # ---------------------------

    def init_call(self,func,seqn):
        """Initializes tracking for an outgoing RPC call (creates wait event, registers handler)."""
        self.lock.acquire()
        self.calls[seqn] = {}
        self.calls[seqn]['data'] = {}
        self.calls[seqn]['wait'] = threading.Event()
        self.lock.release()
        self.register(f'RPC:{func}:{seqn}', self.handle_call_return)

    def finish_call(self,func,seqn):
        """Cleans up tracking for a completed/timed-out RPC call."""
        self.unregister(f'RPC:{func}:{seqn}')
        self.lock.acquire()
        self.calls.pop(seqn,None)
        self.lock.release()

    def wait_call_return(self,func,seqn,time=1.0):
        """
        Waits for the return message of a specific RPC call.

        Args:
            func (str): The function name of the call.
            seqn (int): The sequence number of the call.
            time (float, optional): Maximum time to wait in seconds. Defaults to 1.0.

        Returns:
            dict: The data received in the return message, or an empty dict if timed out.
        """
        data = {}
        if seqn in self.calls:
            self.calls[seqn]['wait'].wait(time)
        if seqn in self.calls:
            data = self.calls[seqn]['data']
        self.finish_call(func,seqn)
        return data

    def handle_call_return(self,data):
        """Handler for incoming RPC return messages. Stores data and signals the waiting thread."""
        seqn = data['Seqn']
        if seqn in self.calls:
            self.calls[seqn]['data'] = data
            self.calls[seqn]['wait'].set()

    def call(self,rem,Func,**kwargs):
        """
        Makes a synchronous RPC call to a remote device and waits for the response.

        Args:
            rem (DW1000): The remote device object.
            Func (str): The name of the function to call remotely.
            **kwargs: Arguments to pass to the remote function.

        Returns:
            dict: The 'Args' dictionary from the remote response, or {} on timeout/error.
        """
        Seqn = self.get_seqnum()
        self.init_call(Func,Seqn)
        rem.sendargs(Type='RPC',Func=Func,Seqn=Seqn,Args=kwargs)
        data = self.wait_call_return(Func,Seqn)
        return data.get('Args',{})

    def call_void(self,rem,Func,**kwargs):
        """
        Makes an asynchronous RPC call (fire-and-forget). Does not wait for a response.

        Args:
            rem (DW1000): The remote device object.
            Func (str): The name of the function to call remotely.
            **kwargs: Arguments to pass to the remote function.
        """
        seqn = self.get_seqnum()
        rem.sendargs(Type='RPC',Func=Func,Seqn=seqn,Args=kwargs)



class Blink():
    """
    Represents a single detected UWB blink (transmission) event recorded by an anchor.
    Stores metadata about the blink, including the anchor that received it,
    the beacon ID (bid), source address, the decoded frame, timestamps, and
    detailed timestamp/RF information.
    """
    def __init__(self,anchor,bid,src,frame,times,tinfo):
        """
        Initializes a Blink object.

        Args:
            anchor (str): EUI-64 of the anchor that recorded this blink.
            bid (int): Beacon ID extracted from the frame payload.
            src (str): Source address from the WPAN frame header.
            frame (TailFrame): The decoded TailFrame object.
            times (dict): Dictionary containing various timestamps (swts, hwts, hires).
            tinfo (dict): Dictionary containing detailed timestamp/RF info (rawts, lqi, snr, etc.).
        """
        self.anchor = anchor
        self.bid    = bid
        self.src    = src
        self.frame  = frame
        self.times  = times
        self.tinfo  = tinfo
        self.swts   = times['swts']
        self.hwts   = times['hwts']
        self.hires  = times['hires']
        self.rawts  = tinfo['rawts'] # Raw hardware timestamp

    def is_rx(self):
        """Returns True if this represents a received frame (LQI > 0)."""
        return (self.tinfo['lqi'] > 0)

    def timestamp(self):
        """Returns the high-resolution timestamp associated with the blink."""
        return self.times.hires

    def get_rx_level(self):
        """Calculates an estimated receive signal level based on CIR power and preamble count."""
        POW = self.tinfo['cir_pwr']
        RXP = self.tinfo['rxpacc']
        if POW>0 and RXP>0:
            level = (POW << 17) / (RXP*RXP)
            return level
        return None

    def get_fp_level(self):
        """Calculates an estimated first path signal level."""
        FP1 = self.tinfo['fp_ampl1']
        FP2 = self.tinfo['fp_ampl2']
        FP3 = self.tinfo['fp_ampl3']
        RXP = self.tinfo['rxpacc']
        if FP1>0 and FP2>0 and FP3>0 and RXP>0:
            level = (FP1*FP1 + FP2*FP2 + FP3*FP3) / (RXP*RXP)
            return level
        return None

    def get_xtal_ratio(self):
        """Calculates the crystal oscillator trim ratio (ttcko/ttcki), if available."""
        I = self.tinfo['ttcki']
        O = self.tinfo['ttcko']
        if O & 0x040000:
            O -= 0x080000
        if I:
            return O/I
        return None


class Blinks():
    """
    Manages collections of Blink objects received from multiple anchors.
    Uses an RPC instance to register handlers for TX/RX messages.
    Correlates blinks using a Beacon ID (bid).
    Provides methods for accessing blink data, managing blink lifetimes,
    and waiting for specific blinks to arrive.
    """
    def __init__(self,rpc):
        """
        Initializes the Blinks manager.

        Args:
            rpc (RPC): The RPC instance to use for registering message handlers.
        """
        self.rpc = rpc
        self.bid = 1
        self.blinks = {}
        self.timer = Timer()
        self.verbose = 0
        rpc.register('TX', self.handle_blink)
        rpc.register('RX', self.handle_blink) # Register handler for received blinks

    def stop(self):
        """Placeholder for potential cleanup (currently does nothing)."""
        pass

    def time(self):
        """Returns the current time from the internal Timer instance."""
        return self.timer.get()

    def nap(self,delay):
        """Advances the internal timer target and sleeps until that time."""
        return self.timer.nap(delay)

    def sync(self,delay=0):
        """Waits for a delay and then synchronizes the internal timer."""
        return self.timer.sync(delay)

    def get_euis_at(self,index,direction):
        """
        Retrieves a list of anchor EUIs that recorded a blink with the given index (bid)
        and transmission direction (currently unused/unreliable in Blink class?).

        Args:
            index (int): The Beacon ID (bid) of the blink.
            direction: The direction (likely intended 'TX' or 'RX', but not stored in Blink).

        Returns:
            list: A list of anchor EUI strings.
        """
        euis = []
        if index is not None:
            if index in self.blinks:
                for eui in self.blinks[index]['anchors']:
                    if self.blinks[index]['anchors'][eui]['dir'] == direction:
                        euis.append(eui)
        return euis

    def get_blink(self,index,rem):
        """
        Retrieves the Blink object for a specific beacon ID and remote anchor.

        Args:
            index (int): The Beacon ID (bid).
            rem (DW1000 or str): The remote anchor object or its EUI string.

        Returns:
            Blink: The corresponding Blink object.
        """
        if isinstance(rem,DW1000):
            rem = rem.eui
        return self.blinks[index]['anchors'][rem]

    def get_blink_time(self,index):
        """Gets the reception time (as recorded by Blinks manager) for a given beacon ID."""
        return self.blinks[index]['time']

    # --- Convenience Getters for Blink Data ---
    def get_times(self,index,rem):
        """Gets the 'times' dictionary from a specific Blink."""
        return self.get_blink(index,rem).times

    def get_swts(self,index,rem):
        """Gets the software timestamp (swts) from a specific Blink."""
        return self.get_blink(index,rem).swts

    def get_hwts(self,index,rem):
        """Gets the hardware timestamp (hwts) from a specific Blink."""
        return self.get_blink(index,rem).hwts

    def get_hires(self,index,rem):
        """Gets the high-resolution timestamp (hires) from a specific Blink."""
        return self.get_blink(index,rem).hires

    def get_timestamp(self,index,rem):
        """Gets the high-resolution timestamp (alias for get_hires) from a specific Blink."""
        return self.get_blink(index,rem).hires

    def get_rawts(self,index,rem):
        """Gets the raw hardware timestamp (rawts) from a specific Blink."""
        return self.get_blink(index,rem).rawts

    def get_tinfo(self,index,rem):
        """Gets the 'tinfo' dictionary (detailed timestamp info) from a specific Blink."""
        return self.get_blink(index,rem).tinfo

    def get_tinfo_attr(self,index,rem,attr):
        """Gets a specific attribute from the 'tinfo' dictionary of a Blink."""
        return self.get_blink(index,rem).tinfo[attr]

    def get_lqi(self,index,rem):
        """Gets the Link Quality Indicator (LQI) from a specific Blink."""
        return self.get_tinfo_attr(index,rem,'lqi')

    def get_snr(self,index,rem):
        """Gets the Signal-to-Noise Ratio (SNR) from a specific Blink."""
        return self.get_tinfo_attr(index,rem,'snr')

    def get_noise(self,index,rem):
        """Gets the noise level from a specific Blink."""
        return self.get_tinfo_attr(index,rem,'noise')

    def get_xtal_ratio(self,index,rem):
        """Gets the calculated crystal trim ratio from a specific Blink."""
        return self.get_blink(index,rem).get_xtal_ratio()

    def get_rx_level(self,index,rem):
        """Gets the estimated receive level from a specific Blink."""
        return self.get_blink(index,rem).get_rx_level()

    def get_fp_level(self,index,rem):
        """Gets the estimated first path level from a specific Blink."""
        return self.get_blink(index,rem).get_fp_level()

    def get_temp(self,index,rem):
        """Gets the temperature reading (in Celsius) from a specific Blink."""
        raw = self.get_tinfo_attr(index,rem,'temp')
        if raw > 32767:
            raw -= 65536
        return 0.01*raw

    def get_volt(self,index,rem):
        """Gets the voltage reading (in Volts) from a specific Blink."""
        raw = self.get_tinfo_attr(index,rem,'volt')
        return 0.001*raw

    # --- Blink Management ---
    def create_blink(self):
        """
        Creates a new entry in the internal blinks dictionary for a new beacon ID (bid).
        Initializes the anchor dictionary and a condition variable for waiting.

        Returns:
            int: The newly generated beacon ID (bid).
        """
        bid = self.bid
        self.bid += 1
        self.blinks[bid] = {
            'anchors' : {},
            'time'    : '{:.6f}'.format(self.time()),
            'wait'    : threading.Condition(),
        }
        return bid

    def purge_blink(self,bid):
        """Removes the entry for a given beacon ID from the internal dictionary."""
        self.blinks.pop(bid,None)

    def blink(self,rem):
        """
        Creates a new blink ID and sends a BEACON command via UDP to the specified remote device.

        Args:
            rem (DW1000): The remote device to send the beacon command to.

        Returns:
            int: The beacon ID (bid) used for the command.
        """
        bid = self.create_blink()
        rem.sendargsudp(Type='BEACON', Beacon=f'{bid:08x}', SubType=9)
        return bid

    def blink_bid(self,rem,bid):
        """Sends a BEACON command with a specific beacon ID to the remote device."""
        rem.sendargsudp(Type='BEACON', Beacon=f'{bid:08x}', SubType=9)

    def blinks_accounted_for(self,bids,ancs):
        """
        Checks if all specified anchors have reported receiving all specified beacon IDs.

        Args:
            bids (list): A list of beacon IDs to check.
            ancs (list): A list of DW1000 anchor objects to check against.

        Returns:
            int: 0 if all blinks are accounted for, otherwise the bid of the first
                 blink that is missing an anchor report.
        """
        for bid in bids:
            for anc in ancs:
                if anc.eui not in self.blinks[bid]['anchors']:
                    return bid
        return 0

    def wait_blinks(self,bids,ancs,wait=1.0):
        """
        Waits for all specified anchors to report receiving all specified beacon IDs,
        up to a maximum timeout. Uses condition variables for efficient waiting.

        Args:
            bids (list): A list of beacon IDs to wait for.
            ancs (list): A list of DW1000 anchor objects expected to report.
            wait (float, optional): Maximum time to wait in seconds. Defaults to 1.0.
        """
        until = time.time() + wait
        delay = (until - time.time()) / 2
        missing = 1
        while missing and delay > 0.0001:
            missing = self.blinks_accounted_for(bids,ancs)
            if missing:
                with self.blinks[missing]['wait']:
                    self.blinks[missing]['wait'].wait(delay)
            delay = (until - time.time()) / 2
        if self.verbose > 0:
            for bid in bids:
                for anc in ancs:
                    if anc.eui not in self.blinks[bid]['anchors']:
                        eprint(f'wait_blinks: ID:{bid} ANCHORS:{anc.name} missing')

    def handle_blink(self,data):
        """
        RPC handler for incoming 'TX' or 'RX' messages.
        Parses the message data, creates a Blink object, and stores it
        in the internal dictionary, keyed by beacon ID and anchor EUI.
        Notifies any threads waiting on the specific beacon ID.
        """
        if self.verbose > 1:
            eprint(f'handle_blink: {data}')
        eui = data.get('Anchor')
        src = data.get('Src')
        tms = data.get('Times')
        tsi = data.get('TSInfo')
        frd = data.get('Frame')
        frm = TailFrame(bytes.fromhex(frd))
        
        if frm.tail_beacon:
            (bid,) = struct.unpack('>Q', frm.tail_beacon)
        else:
            bid = None
        
        if bid in self.blinks:
            blk = Blink(eui,bid,src,frm,tms,tsi)
            with self.blinks[bid]['wait']:
                self.blinks[bid]['anchors'][eui] = blk
                self.blinks[bid]['wait'].notify_all()
