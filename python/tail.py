#!/usr/bin/python3
#
# tail.py	Tail python library
#
# This module provides core functionalities for the Tail UWB positioning system.
# It includes classes for network communication (TailPipe), interaction with
# DW1000 hardware via sysfs, definitions for timestamp structures, and classes
# for encoding/decoding IEEE 802.15.4 WPAN frames and the specific Tail protocol
# frames built upon them.
#

import os
import sys
import time
import math
import ctypes
import struct
import socket
import netifaces
import traceback

from ctypes import *


# ---------------------------------------------------------------------------
# Simple error printing helpers
# ---------------------------------------------------------------------------
def prints(*args, **kwargs):
    """Prints to stdout without a newline and flushes."""
    print(*args, end='', flush=True, **kwargs)

def eprint(*args, **kwargs):
    """Prints to stderr."""
    print(*args, file=sys.stderr, **kwargs)

def eprints(*args, **kwargs):
    """Prints to stderr without a newline and flushes."""
    print(*args, file=sys.stderr, end='', flush=True, **kwargs)

def errhandler(msg,err):
    """Prints an exception message and traceback to stderr."""
    eprint('\n*** EXCEPTION {}:\n{}***\n'.format(msg,traceback.format_exc()))



# ---------------------------------------------------------------------------
# Network pipe for transferring messages (typically JSON)
# ---------------------------------------------------------------------------
class TailPipe:
    """
    Base class for network communication pipes used in the Tail system.
    Provides a common interface for sending and receiving messages.
    Subclasses implement specific transport protocols (TCP, UDP).
    """
    def __init__(self,sock=None):
        """
        Initializes the TailPipe.

        Args:
            sock: An optional existing socket object to use. If None, subclasses
                  will typically create a new socket.
        """
        self.sock = sock
        self.local = None
        self.remote = None

    def fileno(self):
        """Returns the file descriptor of the underlying socket."""
        if self.sock is not None:
            return self.sock.fileno()
        return None

    def close(self):
        """Closes the underlying socket."""
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    @staticmethod
    def getsaddr(host, port, sock_type):
        """
        Resolves host and port to a socket address structure suitable for connect/bind.
        Prioritizes IPv6 if available for the given socket type.

        Args:
            host (str): The hostname or IP address.
            port (int): The port number.
            sock_type: The socket type (e.g., socket.SOCK_STREAM, socket.SOCK_DGRAM).

        Returns:
            The socket address tuple (e.g., ('::1', 1234, 0, 0) for IPv6, ('127.0.0.1', 1234) for IPv4)
            or None if resolution fails.
        """
        addrs = socket.getaddrinfo(host, port)
        # Prefer IPv6
        for addr in addrs:
            if addr[1] == sock_type and addr[0] == socket.AF_INET6:
                return addr[4]
        # Fallback to IPv4
        for addr in addrs:
            if addr[1] == sock_type and addr[0] == socket.AF_INET:
                if addr[0] == socket.AF_INET:
                    return addr[4]
        return None

# ---------------------------------------------------------------------------
# TCP Implementation of TailPipe
# ---------------------------------------------------------------------------
class TCPTailPipe(TailPipe):
    """
    TCP implementation of the TailPipe. Handles message framing using a
    delimiter (ASCII unit separator, 0x1f). Assumes IPv6 by default.
    """
    def __init__(self,sock=None):
        """
        Initializes the TCPTailPipe. Creates a new IPv6 TCP socket if none is provided.
        Sets SO_REUSEADDR and TCP_NODELAY options.
        """
        TailPipe.__init__(self,sock)
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buff = b'' # Internal buffer for received data

    @staticmethod
    def getsaddr(host,port):
        """Resolves host/port to a TCP socket address (IPv6 preferred)."""
        return TailPipe.getsaddr(host,port,socket.SOCK_STREAM)

    def close(self):
        """Closes the TCP socket and clears the buffer."""
        TailPipe.close(self)
        self.clear()

    def clear(self):
        """Clears the internal receive buffer."""
        self.buff = b''

    def recvraw(self):
        """Receives raw data from the socket (up to 4096 bytes)."""
        data = self.sock.recv(4096)
        if len(data) < 1:
            raise ConnectionResetError
        return data

    def fillbuf(self):
        """Reads data from the socket and appends it to the internal buffer."""
        self.buff += self.recvraw()

    def stripbuf(self):
        """Removes leading delimiters (0x1f) from the buffer."""
        while len(self.buff) > 0 and self.buff[0] == 31:
            self.buff = self.buff[1:] # Indent this line

    def hasmsg(self):
        """Checks if a complete message (ending with 0x1f) is in the buffer."""
        self.stripbuf()
        return (self.buff.find(31) > 0)

    def getmsg(self):
        """
        Extracts and returns the next complete message from the buffer if available.
        Returns None otherwise.
        """
        self.stripbuf()
        eom = self.buff.find(31)
        if eom > 0:
            msg = self.buff[0:eom]
            self.buff = self.buff[eom+1:]
            return msg.decode()
        return None

    def getmsgfrom(self):
        """Gets the next message and the remote address (for consistency with UDP)."""
        return (self.getmsg(),self.remote)

    def recvmsg(self):
        """Blocks until a complete message is received and returns it."""
        while not self.hasmsg():
            self.fillbuf()
        return self.getmsg()

    def recvmsgfrom(self):
        """Blocks until a complete message is received and returns it with the remote address."""
        return (self.recvmsg(),self.remote)

    def sendraw(self,data):
        """Sends raw bytes over the socket."""
        self.sock.sendall(data)

    def sendmsg(self,data):
        """Encodes a string message, appends the delimiter, and sends it."""
        self.sendraw(data.encode() + b'\x1f')

    def sendmsgto(self,data,addr):
        """Not applicable for TCP (use sendmsg). Raises TypeError."""
        raise TypeError

    def connect(self, host, port):
        """Connects the socket to the specified host and port."""
        self.remote = TCPTailPipe.getsaddr(host,port)
        self.sock.connect(self.remote)

    def bind(self,addr,port):
        """Not typically used for client TCP sockets. Raises TypeError."""
        raise TypeError

    def listen(self,addr,port):
        """Binds the socket to a local address/port and listens for incoming connections."""
        self.local = (addr,port)
        self.sock.bind(self.local)
        self.sock.listen()

    def accept(self):
        """
        Accepts an incoming connection.

        Returns:
            A new TCPTailPipe instance representing the connection to the client.
        """
        (csock,caddr) = self.sock.accept()
        pipe = TCPTailPipe(csock)
        pipe.local = self.local
        pipe.remote = caddr
        return pipe

# ---------------------------------------------------------------------------
# UDP Implementation of TailPipe
# ---------------------------------------------------------------------------
class UDPTailPipe(TailPipe):
    """
    UDP implementation of the TailPipe. Messages are datagrams.
    Assumes IPv6 by default.
    """
    def __init__(self,sock=None):
        """
        Initializes the UDPTailPipe. Creates a new IPv6 UDP socket if none is provided.
        Sets SO_REUSEADDR option.
        """
        TailPipe.__init__(self,sock)
        if self.sock is None:
            self.sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.buff = [] # Buffer for received (datagram, address) tuples

    @staticmethod
    def getsaddr(host,port):
        """Resolves host/port to a UDP socket address (IPv6 preferred)."""
        return TailPipe.getsaddr(host,port,socket.SOCK_DGRAM)

    @staticmethod
    def clone(parent):
        """Creates a new UDPTailPipe sharing the same socket as the parent."""
        pipe = UDPTailPipe(parent.sock)
        pipe.local = parent.local
        pipe.remote = parent.remote
        return pipe

    def close(self):
        """Closes the UDP socket and clears the buffer."""
        TailPipe.close(self)
        self.clear()

    def clear(self):
        """Clears the internal receive buffer."""
        self.buff = []

    def recvraw(self):
        """Receives a raw datagram and the sender's address."""
        (data,addr) = self.sock.recvfrom(4096)
        if len(data) < 1:
            raise ConnectionResetError
        return (data,addr)

    def fillbuf(self):
        """Receives one datagram and appends (data, addr) to the buffer."""
        self.buff.append(self.recvraw())

    def hasmsg(self):
        """Checks if there are any received datagrams in the buffer."""
        return bool(self.buff)

    def getmsg(self):
        """
        Retrieves the next datagram data from the buffer if available.
        Returns None otherwise.
        """
        if self.hasmsg():
            (data,addr) = self.buff.pop(0)
            return data.decode()
        return None

    def getmsgfrom(self):
        """
        Retrieves the next datagram data and sender address from the buffer.
        Returns None if the buffer is empty.
        """
        if self.hasmsg():
            (data,addr) = self.buff.pop(0)
            return (data.decode(),addr)
        return None

    def recvmsg(self):
        """Blocks until a datagram is received and returns its data."""
        while not self.hasmsg():
            self.fillbuf()
        return self.getmsg()

    def recvmsgfrom(self):
        """Blocks until a datagram is received and returns its data and sender address."""
        while not self.hasmsg():
            self.fillbuf()
        return self.getmsgfrom()

    def sendmsg(self,data):
        """Encodes and sends a string message to the default remote address."""
        self.sock.sendto(data.encode(),self.remote)

    def sendmsgto(self,data,addr):
        """Encodes and sends a string message to the specified address."""
        self.sock.sendto(data.encode(),addr)

    def connect(self,host,port):
        """Sets the default remote address for sendmsg(). Does not actually connect."""
        self.remote = UDPTailPipe.getsaddr(host,port)

    def bind(self,addr,port):
        """Binds the socket to a local address and port."""
        self.local = (addr,port)
        self.sock.bind(self.local)

    def listen(self,addr,port):
        """Alias for bind() for UDP sockets."""
        self.bind(addr,port)

    def accept(self):
        """Not applicable for UDP. Raises TypeError."""
        raise TypeError
    


# ---------------------------------------------------------------------------
# DW1000 Hardware Interaction (via sysfs and Device Tree)
# ---------------------------------------------------------------------------
# Path to the DW1000 sysfs attributes directory (specific to the hardware setup, likely Raspberry Pi)
DW1000_SYSFS = '/sys/devices/platform/soc/3f204000.spi/spi_master/spi0/spi0.0/dw1000/'

def SetDWAttr(attr, data):
    """Sets a DW1000 attribute via its sysfs file."""
    if os.path.isfile(DW1000_SYSFS + attr):
        with open(DW1000_SYSFS + attr, 'w') as f:
            f.write(str(data))

def GetDWAttr(attr):
    """Gets a DW1000 attribute value from its sysfs file."""
    if os.path.isfile(DW1000_SYSFS + attr):
        with open(DW1000_SYSFS + attr, 'r') as f:
            value = f.read()
        return value.rstrip()
    return None

# Path to the DW1000 device tree node attributes in sysfs
DW1000_SYSDT = '/sys/devices/platform/soc/3f204000.spi/spi_master/spi0/spi0.0/of_node/'

def GetDTAttrRaw(attr):
    """Reads a raw byte value from a device tree attribute file."""
    if os.path.isfile(DW1000_SYSDT + attr):
        with open(DW1000_SYSDT + attr, 'rb') as f:
            data = f.read()
        return data
    return None

def GetDTAttrStr(attr):
    """Reads a string value from a device tree attribute file, stripping trailing nulls/whitespace."""
    if os.path.isfile(DW1000_SYSDT + attr):
        with open(DW1000_SYSDT + attr, 'r') as f:
            data = f.read()
        return data.rstrip('\n\r\0')
    return None

def GetDTAttr(attr, form):
    """Reads a binary value from a device tree attribute file and unpacks it using a struct format."""
    if os.path.isfile(DW1000_SYSDT + attr):
        with open(DW1000_SYSDT + attr, 'rb') as f:
            data = f.read()
        return struct.unpack(form, data)
    return []


# ---------------------------------------------------------------------------
# Pretty printing helpers
# ---------------------------------------------------------------------------
# Default key length for formatted attribute printing
ATTR_KEY_LEN = 24

def fattr(key,val,ind=0,col=ATTR_KEY_LEN):
    """Formats a key-value pair with indentation and alignment."""
    return ind*' ' + key.ljust(col-ind) + ': ' + str(val).replace('\n', '\n'+ind*' ').replace(ind*' '+':',':')

def fattrnl(key,val,ind=0,col=ATTR_KEY_LEN):
    """Formats a key-value pair like fattr() and adds a newline."""
    return fattr(key,val,ind,col) + '\n'

def fnlattr(key,val,ind=0,col=ATTR_KEY_LEN):
    """Adds a newline before formatting a key-value pair like fattr()."""
    return '\n' + fattr(key,val,ind,col)

# ---------------------------------------------------------------------------
# Kernel Interface Data Structures (using ctypes)
# ---------------------------------------------------------------------------
class Timespec(Structure):
    """
    Represents the standard C timespec structure (seconds and nanoseconds).
    Used for software and legacy hardware timestamps.
    """
    _fields_ = [("tv_sec", c_long),  # Seconds
                ("tv_nsec", c_long)] # Nanoseconds
    # Removed duplicate ("tv_nsec", c_long) line above

    def __iter__(self):
        return ((x[0], getattr(self,x[0])) for x in self._fields_)

    def __int__(self):
        return (self.tv_sec * 1000000000 + self.tv_nsec)

    def __float__(self):
        return float(int(self))

    def __str__(self):
        return '0x{:x}'.format(int(self))

    def __bool__(self):
        """Returns True if the timestamp is non-zero."""
        return bool(self.tv_sec or self.tv_nsec)

class Timehires(Structure):
    """
    Represents a high-resolution timestamp format, likely specific to the
    DW1000 or associated hardware/driver. It seems to store nanoseconds
    and fractional nanoseconds (1/2^32 parts).
    """
    _fields_ = [
        ("tv_nsec", c_uint64), # Integer nanoseconds part
        ("tv_frac", c_uint32), # Fractional nanoseconds part (units of 2^-32 ns)
        ("__res", c_uint32) ]  # Reserved/padding

    def __iter__(self):
        return ((x[0], getattr(self,x[0])) for x in self._fields_)

    def __int__(self):
        return ((self.tv_nsec << 32) | self.tv_frac)

    def __float__(self):
        return (float(self.tv_nsec) + self.tv_frac/4294967296)

    def __str__(self):
        return '0x{:x}.{:08x}'.format(self.tv_nsec,self.tv_frac)

    def __bool__(self):
        """Returns True if the high-resolution timestamp is non-zero."""
        return bool(self.tv_nsec or self.tv_frac)

class TimestampInfo(Structure):
    """
    Detailed information associated with a hardware timestamp, likely from
    the DW1000 receiver, providing quality metrics and diagnostic data.
    """
    _fields_ = [
        ("rawts", c_uint64),    # Raw hardware timestamp counter value
        ("lqi", c_uint16),      # Link Quality Indicator
        ("snr", c_uint16),      # Signal-to-Noise Ratio
        ("fpr", c_uint16),      # First Path Rate? (Interpretation uncertain)
        ("noise", c_uint16),    # Noise level measurement
        ("rxpacc", c_uint16),   # RX Preamble Accumulation Count
        ("fp_index", c_uint16), # First Path Index (in CIR)
        ("fp_ampl1", c_uint16), # First Path Amplitude 1
        ("fp_ampl2", c_uint16), # First Path Amplitude 2
        ("fp_ampl3", c_uint16), # First Path Amplitude 3
        ("cir_pwr", c_uint32),  # Channel Impulse Response Power
        ("fp_pwr", c_uint32),   # First Path Power
        ("ttcko", c_uint32),    # Crystal Oscillator Trim (TX?)
        ("ttcki", c_uint32),    # Crystal Oscillator Trim (RX?)
        ("temp", c_int16),      # Temperature reading (device internal)
        ("volt", c_int16),      # Voltage reading (device internal)
    ]

    def __iter__(self):
        return ((x[0], getattr(self,x[0])) for x in self._fields_)

    def __str__(self):
        ret  = '[TimestampInfo]'
        for (key,val) in self:
            ret += fnlattr(key,val, 2)
        return ret

class Timestamp(Structure):
    """
    Comprehensive timestamp structure, combining software (kernel), legacy hardware,
    standard hardware (timespec), high-resolution hardware, and detailed info.
    Likely obtained via socket timestamping options (SO_TIMESTAMPING).
    """
    _fields_ = [
        ("sw", Timespec),           # Software timestamp (kernel)
        ("legacy", Timespec),       # Legacy hardware timestamp (if supported)
        ("hw", Timespec),           # Hardware timestamp (converted to timespec)
        ("hires", Timehires),       # High-resolution hardware timestamp
        ("tsinfo", TimestampInfo),  # Detailed timestamp info from hardware
    ]

    def __iter__(self):
        return ((x[0], getattr(self,x[0])) for x in self._fields_)

    def __str__(self):
        ret = '[Timestamp]'
        ret += fnlattr('sw', self.sw, 2)
        ret += fnlattr('hw', self.hw, 2)
        ret += fnlattr('hr', self.hires, 2)
        ret += fnlattr('ts', self.tsinfo, 2)
        return ret


# ---------------------------------------------------------------------------
# General Support Functions
# ---------------------------------------------------------------------------
def byteswap(data):
    """Reverses the byte order of a bytes object (e.g., for endianness conversion)."""
    return bytes(reversed(tuple(data)))

def bit(pos):
    """Returns an integer with the bit at the given position set (e.g., bit(3) -> 8)."""
    return (1<<pos)

def testbit(data,pos):
    """Tests if a specific bit is set in an integer."""
    return bool(data & bit(pos))

def getbits(data,pos,cnt):
    """Extracts a specified number of bits starting from a given position in an integer."""
    return (data>>pos) & ((1<<cnt)-1)

def makebits(data,pos,cnt):
    """Creates an integer by placing the lower 'cnt' bits of 'data' at 'pos'."""
    return (data & ((1<<cnt)-1)) << pos



# ---------------------------------------------------------------------------
# IEEE 802.15.4 Frame Format Handling
# ---------------------------------------------------------------------------
class WPANFrame:
    """
    Represents and handles encoding/decoding of basic IEEE 802.15.4 WPAN frames.
    Supports short (16-bit) and extended (64-bit EUI) addresses.
    """
    # Data Sequence Number (DSN) - automatically incremented for new frames
    DSN = 0

    # Address Modes Constants
    ADDR_NONE  = 0 # No address present
    ADDR_SHORT = 2 # 16-bit short address
    ADDR_EUI64 = 3 # 64-bit extended address (EUI-64)

    # Class variables to store the interface's own addresses (set via set_ifaddr)
    if_addr    = None # Interface EUI-64 address
    if_short   = None # Interface short address

    # Verbosity level for __str__ output
    verbosity  = 0

    def __init__(self, data=None, ancl=None):
        """
        Initializes a WPANFrame instance.

        Args:
            data (bytes, optional): Raw frame data to decode immediately. Defaults to None.
            ancl (list, optional): Ancillary data (e.g., from recvmsg) containing
                                   timestamp information. Defaults to None.
        """
        self.timestamp      = None
        self.frame          = None
        self.frame_len      = 0
        self.frame_control  = None
        self.frame_type     = 1
        self.frame_version  = 1
        self.frame_seqnum   = None
        self.header_len     = 0
        self.security       = False
        self.pending        = False
        self.ack_req        = False
        self.panid_comp     = True
        
        self.dst_mode       = 0
        self.dst_addr       = None
        self.dst_panid      = 0xffff
        
        self.src_mode       = 0
        self.src_addr       = None
        self.src_panid      = 0xffff # Source PAN ID

        if data is not None:
            self.decode(data)
        if ancl is not None:
            self.decode_ancl(ancl)

    @staticmethod
    def set_ifaddr(if_addr=None, if_short=None):
        """Sets the EUI-64 and short addresses for the local interface."""
        WPANFrame.if_addr  = if_addr
        WPANFrame.if_short = if_short

    @staticmethod
    def match_if(addr):
        """Checks if the given address matches the local interface address (short or EUI)."""
        return (addr == WPANFrame.if_addr) or (addr == WPANFrame.if_short)

    @staticmethod
    def match_bcast(addr):
        """Checks if the given address is a broadcast address (short or EUI)."""
        return (addr == 2 * b'\xff') or (addr == 8 * b'\xff')

    @staticmethod
    def match_local(addr):
        """Checks if the given address matches the local interface or is broadcast."""
        return WPANFrame.match_if(addr) or WPANFrame.match_bcast(addr)

    @staticmethod
    def is_eui(addr):
        """Checks if the given address is a valid EUI-64 address (bytes, 8 long, not broadcast)."""
        return (type(addr) is bytes) and (len(addr) == 8) and (addr != 8 * b'\xff')

    def get_src_eui(self):
        """Returns the source EUI-64 address as a hex string, or None if not present/EUI64."""
        if self.src_mode == WPANFrame.ADDR_EUI64:
            return self.src_addr.hex()
        return None

    def get_dst_eui(self):
        """Returns the destination EUI-64 address as a hex string, or None if not present/EUI64."""
        if self.dst_mode == WPANFrame.ADDR_EUI64:
            return self.dst_addr.hex()
        return None

    def get_peer_eui(self):
        """
        Attempts to identify the EUI-64 of the peer (non-local, non-broadcast) node.
        Returns the peer EUI-64 as hex string, or None if not applicable/found.
        """
        if WPANFrame.match_local(self.dst_addr) and WPANFrame.is_eui(self.src_addr):
            return self.src_addr.hex()
        if WPANFrame.match_local(self.src_addr) and WPANFrame.is_eui(self.dst_addr):
            return self.dst_addr.hex()
        return None

    def set_src_addr(self,addr):
        """
        Sets the source address and mode based on the input type.
        Accepts None, int (short), bytes (short/EUI64), or hex string (short/EUI64).
        """
        if addr is None:
            self.src_mode = WPANFrame.ADDR_NONE
            self.src_addr = None
        elif type(addr) is int:
            self.src_mode = WPANFrame.ADDR_SHORT
            self.src_addr = struct.pack('<H',addr)
        elif type(addr) is bytes:
            if len(addr) == 2:
                self.src_addr = addr
                self.src_mode = WPANFrame.ADDR_SHORT
            elif len(addr) == 8:
                self.src_addr = addr
                self.src_mode = WPANFrame.ADDR_EUI64
            else:
                raise ValueError
        elif type(addr) is str:
            if len(addr) == 4:
                self.src_addr = bytes.fromhex(addr)
                self.src_mode = WPANFrame.ADDR_SHORT
            elif len(addr) == 16:
                self.src_addr = bytes.fromhex(addr)
                self.src_mode = WPANFrame.ADDR_EUI64
            else:
                raise ValueError
        else:
            raise ValueError

    def set_src_panid(self,panid):
        """Sets the source PAN ID. Accepts int or 2-byte bytes."""
        if type(panid) is int:
            self.src_panid = panid
        elif type(panid) is bytes and len(addr) == 2:
            self.src_panid = struct.pack('<H',panid)
        else:
            raise ValueError

    def set_dst_addr(self,addr):
        """
        Sets the destination address and mode based on the input type.
        Accepts None, int (short), bytes (short/EUI64), or hex string (short/EUI64).
        """
        if addr is None:
            self.dst_mode = WPANFrame.ADDR_NONE
            self.dst_addr = None
        elif type(addr) is int:
            self.dst_mode = WPANFrame.ADDR_SHORT
            self.dst_addr = struct.pack('<H',addr)
        elif type(addr) is bytes:
            if len(addr) == 2:
                self.dst_addr = addr
                self.dst_mode = WPANFrame.ADDR_SHORT
            elif len(addr) == 8:
                self.dst_addr = addr
                self.dst_mode = WPANFrame.ADDR_EUI64
            else:
                raise ValueError
        elif type(addr) is str:
            if len(addr) == 4:
                self.dst_addr = bytes.fromhex(addr)
                self.dst_mode = WPANFrame.ADDR_SHORT
            elif len(addr) == 16:
                self.dst_addr = bytes.fromhex(addr)
                self.dst_mode = WPANFrame.ADDR_EUI64
            else:
                raise ValueError
        else:
            raise ValueError

    def set_dst_panid(self,panid):
        """Sets the destination PAN ID. Accepts int or 2-byte bytes."""
        if type(panid) is int:
            self.dst_panid = panid
        elif type(panid) is bytes and len(addr) == 2:
            self.dst_panid = struct.pack('<H',panid)
        else:
            raise ValueError

    def decode_ancl(self,ancl):
        """
        Decodes ancillary data (from recvmsg) to extract timestamp information.
        Looks for SOL_SOCKET/SO_TIMESTAMPING messages and populates self.timestamp.
        """
        for cmsg_level, cmsg_type, cmsg_data in ancl:
            #pr.debug('cmsg level={} type={} size={}\n'.format(cmsg_level,cmsg_type,len(cmsg_data)))
            if (cmsg_level == socket.SOL_SOCKET and cmsg_type == socket.SO_TIMESTAMPING):
                raw = cmsg_data.ljust(sizeof(Timestamp), b'\0')
                tss = Timestamp.from_buffer_copy(raw)
                self.timestamp = tss

    def decode(self,data):
        """
        Decodes a raw byte string into the WPAN frame fields.
        Parses Frame Control, Sequence Number, PAN IDs, and Addresses.
        Handles PAN ID compression. Does not handle security headers.

        Args:
            data (bytes): The raw frame data.

        Returns:
            int: The length of the parsed header.
        """
        ptr = 0
        self.frame = data
        self.frame_len = len(data)
        (fc,sq) = struct.unpack_from('<HB',data,ptr)
        ptr += 3
        self.frame_control = fc
        self.frame_seqnum = sq
        self.frame_type = getbits(fc,0,3)
        self.frame_version = getbits(fc,12,2)
        self.security = testbit(fc,3)
        self.pending = testbit(fc,4)
        self.ack_req = testbit(fc,5)
        self.dst_mode = getbits(fc,10,2)
        self.src_mode = getbits(fc,14,2)
        self.panid_comp = testbit(fc,6)
        if self.dst_mode != 0:
            (panid,) = struct.unpack_from('<H',data,ptr)
            self.dst_panid = panid
            ptr += 2
            if self.dst_mode == self.ADDR_SHORT:
                (addr,) = struct.unpack_from('2s',data,ptr)
                self.dst_addr = byteswap(addr)
                ptr += 2
            elif self.dst_mode == self.ADDR_EUI64:
                (addr,) = struct.unpack_from('8s',data,ptr)
                self.dst_addr = byteswap(addr)
                ptr += 8
        else:
            self.dst_panid = None
            self.dst_addr  = None
        if self.src_mode != 0:
            if self.panid_comp:
                self.src_panid = self.dst_panid
            else:
                (panid,) = struct.unpack_from('<H',data,ptr)
                self.src_panid = panid
                ptr += 2
            if self.src_mode == self.ADDR_SHORT:
                (addr,) = struct.unpack_from('2s',data,ptr)
                self.src_addr = byteswap(addr)
                ptr += 2
            elif self.src_mode == self.ADDR_EUI64:
                (addr,) = struct.unpack_from('8s',data,ptr)
                self.src_addr = byteswap(addr)
                ptr += 8
        else:
            self.src_panid = None
            self.src_addr  = None
        if self.security:
            raise NotImplementedError('decode WPAN security')
        self.header_len = ptr
        return ptr

    def encode(self):
        """
        Encodes the WPAN frame fields into a byte string.
        Constructs Frame Control based on flags and address modes.
        Assigns the next DSN if not already set.
        Handles PAN ID compression. Does not handle security headers.

        Returns:
            bytes: The encoded frame header. Payload must be appended separately.
        """
        if self.frame_control is None:
            fc = self.frame_type & 0x07
            if self.security:
                fc |= bit(3)
            if self.pending:
                fc |= bit(4)
            if self.ack_req:
                fc |= bit(5)
            if self.panid_comp and (self.src_panid == self.dst_panid):
                fc |= bit(6)
            fc |= makebits(self.dst_mode,10,2)
            fc |= makebits(self.src_mode,14,2)
            fc |= makebits(self.frame_version,12,2)
            self.frame_control = fc
        if self.frame_seqnum is None:
            self.frame_seqnum = WPANFrame.DSN
            WPANFrame.DSN = (WPANFrame.DSN + 1) & 0xff
        data = struct.pack('<HB', self.frame_control, self.frame_seqnum)
        if self.dst_mode != 0:
            data += struct.pack('<H',self.dst_panid)
            if self.dst_mode == self.ADDR_SHORT:
                data += struct.pack('2s',byteswap(self.dst_addr))
            elif self.dst_mode == self.ADDR_EUI64:
                data += struct.pack('8s',byteswap(self.dst_addr))
        if self.src_mode != 0:
            if not (self.panid_comp and (self.src_panid == self.dst_panid)):
                data += struct.pack('<H', self.src_panid)
            if self.src_mode == self.ADDR_SHORT:
                data += struct.pack('2s', byteswap(self.src_addr))
            elif self.src_mode == self.ADDR_EUI64:
                data += struct.pack('8s', byteswap(self.src_addr))
        if self.security:
            raise NotImplementedError('encode WPAN security')
        self.header_len = len(data)
        self.frame = data
        return data

    def __str__(self):
        """Returns a string representation of the WPAN frame."""
        if WPANFrame.verbosity == 0:
            ret = 'WPAN Frame'
            ret += ' size:{}'.format(self.frame_len)
            ret += ' seq:{}'.format(self.frame_seqnum)
            ret += ' src:{}'.format(self.src_addr.hex())
            ret += ' dst:{}'.format(self.dst_addr.hex())
        else:
            ret = 'WPAN Frame\n'
            if self.timestamp is not None:
                ret += fattrnl('Timestamp', self.timestamp, 2)
            ret += fattrnl('Length', self.frame_len, 2)
            ret += fattrnl('Control', '0x{:04x}'.format(self.frame_control), 2)
            ret += fattrnl('Type', self.frame_type, 4)
            ret += fattrnl('Version', self.frame_version, 4)
            ret += fattrnl('Security', self.security, 4)
            ret += fattrnl('Pending', self.pending, 4)
            ret += fattrnl('Ack.Req.', self.ack_req, 4)
            ret += fattrnl('Dst mode', self.dst_mode, 4)
            ret += fattrnl('Src mode', self.src_mode, 4)
            ret += fattrnl('PanID comp', self.panid_comp, 4)
            ret += fattrnl('SequenceNr', self.frame_seqnum, 2)
            ret += fattrnl('Src Addr', self.src_addr.hex(), 2)
            ret += fattrnl('Src PanID', '{:04x}'.format(self.src_panid), 2)
            ret += fattrnl('Dst Addr', self.dst_addr.hex(), 2)
            ret += fattrnl('Dst PanID', '{:04x}'.format(self.dst_panid), 2)

        return ret



# ---------------------------------------------------------------------------
# Tail Protocol Frame Format Handling (extends WPANFrame)
# ---------------------------------------------------------------------------

def int8(x):
    x = int(x) & 0xff
    if x > 127:
        x -= 256
    return x

def int16(x):
    x = int(x) & 0xffff
    if x > 32767:
        x -= 65536
    return x

class TailFrame(WPANFrame):
    """
    Represents and handles encoding/decoding of Tail-specific protocol frames,
    which are built on top of the basic WPAN frame structure.
    Defines different frame types (Blink, Beacon, Ranging, Config) and
    Information Elements (IEs).
    """
    # Tail Protocol Magic Numbers
    PROTO_1 = 0x37 # Original Tail protocol identifier
    PROTO_2 = 0x38 # Encrypted Tail protocol identifier (payload opaque here)

    # Information Element (IE) Keys (ID -> Human-readable name)
    IE_KEYS =  {
        0x00 : 'Batt',
        0x01 : 'Vreg',
        0x02 : 'Temp',
        0x40 : 'Vbatt',
        0x80 : 'Blinks',
        0xff : 'Debug', # Debug information
    }

    # Information Element (IE) Value Converters (ID -> lambda function)
    # These convert raw IE byte values into meaningful units.
    IE_CONV =  {
        0x01 : lambda x: round(int8(x)/173+3.300, 3),
        0x02 : lambda x: round(int8(x)/1.14+23.0, 2),
        0x40 : lambda x: round(x*5/32768, 3), # Vbatt (likely raw ADC to Volts)
    }

    def __init__(self, data=None, ancl=None, protocol=0):
        """
        Initializes a TailFrame instance.

        Args:
            data (bytes, optional): Raw frame data to decode immediately. Defaults to None.
            ancl (list, optional): Ancillary data (e.g., from recvmsg) containing
                                   timestamp information. Defaults to None.
            protocol (int, optional): Default Tail protocol version if creating a new frame.
                                      Defaults to 0 (meaning it will be determined during decode
                                      or set before encode).
        """
        WPANFrame.__init__(self)
        self.tail_protocol  = protocol
        self.tail_payload   = None
        self.tail_listen    = False
        self.tail_accel     = False
        self.tail_dcin      = False
        self.tail_salt      = False
        self.tail_timing    = False
        self.tail_frmtype   = None
        self.tail_subtype   = None
        self.tail_txtime    = None
        self.tail_rxtime    = None
        self.tail_rxtimes   = None
        self.tail_rxinfo    = None
        self.tail_rxinfos   = None
        self.tail_cookie    = None
        self.tail_beacon    = None
        self.tail_flags     = None
        self.tail_code      = None
        self.tail_test      = None
        self.tail_ies       = None
        self.tail_eies      = None
        self.tail_config    = None
        
        if data is not None:
            self.decode(data)
        if ancl is not None:
            self.decode_ancl(ancl)

    @staticmethod
    def tsdecode(data):
        """Decodes a 5-byte Tail timestamp into an integer (likely DW1000 units)."""
        times = struct.unpack_from('<Q', data.ljust(8, b'\0'))[0]
        return times

    @staticmethod
    def tsencode(times):
        """Encodes an integer timestamp into the 5-byte Tail format."""
        data = struct.pack('<Q',times)[0:5]
        return data

    def decode(self,data):
        """
        Decodes the Tail-specific payload portion of a WPAN frame.
        This method assumes WPANFrame.decode() has already been called.
        It parses the Tail protocol magic number and then dispatches
        to specific parsing logic based on the Tail frame type and subtype.

        Args:
            data (bytes): The full frame data (including WPAN header).
        """
        ptr = WPANFrame.decode(self,data)
        (magic,) = struct.unpack_from('<B',data,ptr)
        ptr += 1
        if magic == TailFrame.PROTO_1:
            self.tail_protocol = 1
            (frame,) = struct.unpack_from('<B',data,ptr)
            ptr += 1
            self.tail_frmtype = getbits(frame,4,4)
            self.tail_subtype = getbits(frame,0,4)
            if self.tail_frmtype == 0:
                self.tail_eies_present   = testbit(frame,1)
                self.tail_ies_present    = testbit(frame,2)
                self.tail_cookie_present = testbit(frame,3)
                (flags,) = struct.unpack_from('<B',data,ptr)
                ptr += 1
                self.tail_flags  = flags
                self.tail_listen = testbit(flags,7)
                self.tail_accel  = testbit(flags,6)
                self.tail_dcin   = testbit(flags,5)
                self.tail_salt   = testbit(flags,4)
                if self.tail_cookie_present:
                    (cookie,) = struct.unpack_from('16s',data,ptr)
                    ptr += 16
                    self.tail_cookie = cookie
                if self.tail_ies_present:
                    (iec,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_ies = {}
                    for i in range(iec):
                        (id,) = struct.unpack_from('<B',data,ptr)
                        ptr += 1
                        idf = getbits(id,6,2)
                        if idf == 0:
                            (val,) = struct.unpack_from('<B',data,ptr)
                            ptr += 1
                        elif idf == 1:
                            (val,) = struct.unpack_from('<H',data,ptr)
                            ptr += 2
                        elif idf == 2:
                            (val,) = struct.unpack_from('<I',data,ptr)
                            ptr += 4
                        else:
                            (val,) = struct.unpack_from('<p',data,ptr)
                            ptr += len(val) + 1
                        if id in TailFrame.IE_CONV:
                            val = TailFrame.IE_CONV[id](val)
                        if id in TailFrame.IE_KEYS:
                            self.tail_ies[TailFrame.IE_KEYS[id]] = val
                        else:
                            self.tail_ies['IE{:02X}'.format(id)] = val
                if self.tail_eies_present:
                    raise NotImplementedError('decode tail EIEs')
            elif self.tail_frmtype == 1:
                (flags,) = struct.unpack_from('<B',data,ptr)
                ptr += 1
                self.tail_flags = flags
                (id,) = struct.unpack_from('8s',data,ptr)
                self.tail_beacon = byteswap(id)
                ptr += 8
            elif self.tail_frmtype == 2:
                pass
                ## TBD
            elif self.tail_frmtype == 3:
                self.tail_owr = testbit(self.tail_subtype,3)
                (txtime,) = struct.unpack_from('5s',data,ptr)
                ptr += 5
                self.tail_txtime = TailFrame.tsdecode(txtime)
                if not self.tail_owr:
                    (cnt,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    bits = 0
                    self.tail_rxtimes = {}
                    for i in range(0,cnt,8):
                        (val,) = struct.unpack_from('<B',data,ptr)
                        ptr += 1
                        bits |= val << i
                    for i in range(cnt):
                        if testbit(bits,i):
                            (addr,) = struct.unpack_from('8s',data,ptr)
                            ptr += 8
                        else:
                            (addr,) = struct.unpack_from('2s',data,ptr)
                            ptr += 2
                        (rxdata,) = struct.unpack_from('5s',data,ptr)
                        ptr += 5
                        rxtime = TailFrame.tsdecode(rxdata)
                        self.tail_rxtimes[byteswap(addr)] = rxtime
                        if WPANFrame.match_if(byteswap(addr)):
                            self.tail_rxtime = rxtime
            elif self.tail_frmtype == 4:
                if self.tail_subtype == 0:
                    (magic,) = struct.unpack_from('<H',data,ptr)
                    ptr += 2
                    self.tail_reset_magic = magic
                elif self.tail_subtype == 1:
                    (iter,) = struct.unpack_from('<H',data,ptr)
                    ptr += 1
                    self.tail_iterator = iter
                elif self.tail_subtype == 2:
                    (cnt,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_config = {}
                    for i in range(cnt):
                        (key,) = struct.unpack_from('<H',data,ptr)
                        ptr += 2
                        self.tail_config[key] = None
                elif self.tail_subtype == 3:
                    (cnt,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_config = {}
                    for i in range(cnt):
                        (key,) = struct.unpack_from('<H',data,ptr)
                        ptr += 2
                        (val,) = struct.unpack_from('<p',data,ptr)
                        ptr += len(val) + 1
                        self.tail_config[key] = val
                elif self.tail_subtype == 4:
                    (cnt,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_config = {}
                    for i in range(cnt):
                        (key,) = struct.unpack_from('<H',data,ptr)
                        ptr += 2
                        self.tail_config[key] = None
                elif self.tail_subtype == 5:
                    (salt,) = struct.unpack_from('<16s',data,ptr)
                    ptr += 16
                    self.tail_salt = salt
                elif self.tailubtype == 15:
                    (test,) = struct.unpack_from('<p',data,ptr)
                    ptr += len(test) + 1
                    self.tail_test = test
                else:
                    raise NotImplementedError('decode config request: {}'.format(self.tail_subtype))
            elif self.tail_frmtype == 5:
                if self.tail_subtype == 0:
                    (magic,) = struct.unpack_from('<H',data,ptr)
                    ptr += 2
                elif self.tail_subtype == 1:
                    (iter,cnt,) = struct.unpack_from('<HB',data,ptr)
                    ptr += 3
                    self.tail_iterator = iter
                    self.tail_config = {}
                    for i in range(cnt):
                        (key,) = struct.unpack_from('<H',data,ptr)
                        ptr += 2
                        self.tail_config[key] = None
                elif self.tail_subtype == 2:
                    (cnt,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_config = {}
                    for i in range(cnt):
                        (key,val,) = struct.unpack_from('<Hs',data,ptr)
                        ptr += len(val) + 3
                        self.tail_config[key] = val
                elif self.tail_subtype == 3:
                    (code,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_code = code
                elif self.tail_subtype == 4:
                    (code,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    self.tail_code = code
                elif self.tail_subtype == 5:
                    (salt,) = struct.unpack_from('<16s',data,ptr)
                    ptr += 16
                    self.tail_salt = salt
                elif self.tail_subtype == 15:
                    (test,) = struct.unpack_from('<p',data,ptr)
                    ptr += len(test) + 1
                    self.tail_test = test
                else:
                    raise NotImplementedError('decode config response: {}'.format(self.tail_subtype))
            elif self.tail_frmtype == 15:
                self.tail_timing = testbit(self.tail_subtype,3)
                txtime = testbit(self.tail_subtype,2)
                rxtime = testbit(self.tail_subtype,1)
                rxinfo = testbit(self.tail_subtype,0)
                if txtime:
                    (tstamp,) = struct.unpack_from('5s',data,ptr)
                    ptr += 5
                    self.tail_txtime = TailFrame.tsdecode(tstamp)
                if rxtime:
                    self.tail_rxtimes = {}
                if rxinfo:
                    self.tail_rxinfos = {}
                if rxtime or rxinfo:
                    (cnt,) = struct.unpack_from('<B',data,ptr)
                    ptr += 1
                    bits = 0
                    for i in range(0,cnt,8):
                        (val,) = struct.unpack_from('<B',data,ptr)
                        ptr += 1
                        bits |= val << i
                    for i in range(cnt):
                        if testbit(bits,i):
                            (addr,) = struct.unpack_from('8s',data,ptr)
                            ptr += 8
                        else:
                            (addr,) = struct.unpack_from('2s',data,ptr)
                            ptr += 2
                        if rxtime:
                            (val,) = struct.unpack_from('5s',data,ptr)
                            ptr += 5
                            tstamp = TailFrame.tsdecode(val)
                            self.tail_rxtimes[byteswap(addr)] = tstamp
                            if WPANFrame.match_if(byteswap(addr)):
                                self.tail_rxtime = tstamp
                        if rxinfo:
                            rxinfo = struct.unpack_from('<4H',data,ptr)
                            ptr += 8
                            self.tail_rxinfos[byteswap(addr)] = rxinfo
                            if WPANFrame.match_if(byteswap(addr)):
                                self.tail_rxinfo = rxinfo
            else:
                raise NotImplementedError('decode tail frametype: {}'.format(self.tail_frmtype))
    ## Tail encrypted protocol
        elif magic == TailFrame.PROTO_2:
            self.tail_protocol = 2
            self.tail_payload = data[ptr:]
    ## Tail protocols end
        else:
            self.tail_protocol = 0
            self.tail_payload = data[ptr-1:]

    def encode(self):
        """
        Encodes the Tail-specific payload and appends it to the WPAN header.
        Calls WPANFrame.encode() first, then adds the Tail magic number
        and payload based on the frame type and subtype attributes set
        on the instance.

        Returns:
            bytes: The fully encoded Tail frame (WPAN header + Tail payload).
        """
        data = WPANFrame.encode(self)
        if self.tail_protocol == 1:
            data += struct.pack('<B',TailFrame.PROTO_1)
            if self.tail_frmtype == 0:
                self.tail_subtype = 0
                if self.tail_cookie is not None:
                    self.tail_subtype |= bit(3)
                if self.tail_ies is not None:
                    self.tail_subtype |= bit(2)
                if self.tail_eies is not None:
                    self.tail_subtype |= bit(1)
                frame = makebits(self.tail_frmtype,4,4) | makebits(self.tail_subtype,0,4)
                data += struct.pack('<B',frame)
                self.tail_flags = 0
                if self.tail_listen:
                    self.tail_flags |= bit(7)
                if self.tail_accel:
                    self.tail_flags |= bit(6)
                if self.tail_dcin:
                    self.tail_flags |= bit(5)
                if self.tail_salt:
                    self.tail_flags |= bit(4)
                data += struct.pack('<B',self.tail_flags)
                if self.tail_cookie is not None:
                    data += struct.pack('16s',self.tail_cookie)
                if self.tail_ies is not None:
                    data += struct.pack('<B',len(self.tail_ies))
                    for (id,val) in self.tail_ies.items():
                        data += struct.pack('<B', id)
                        idf = getbits(id,6,2)
                        if idf == 0:
                            data += struct.pack('<B', val)
                        elif idf == 1:
                            data += struct.pack('<H', val)
                        elif idf == 2:
                            data += struct.pack('<I', val)
                        else:
                            data += struct.pack('<p', val)
                if self.tail_eies is not None:
                    raise NotImplementedError('encode EIEs')
            elif self.tail_frmtype == 1:
                frame = makebits(self.tail_frmtype,4,4) | makebits(self.tail_subtype,0,4)
                flags = self.tail_flags
                data += struct.pack('<BB', frame, flags)
                data += struct.pack('8s', byteswap(self.tail_beacon))
            elif self.tail_frmtype == 2:
                frame = makebits(self.tail_frmtype,4,4) | makebits(self.tail_subtype,0,4)
                flags = self.tail_flags
                data += struct.pack('<BB',frame, flags)
            elif self.tail_frmtype == 3:
                self.tail_subtype = 0
                if self.tail_owr:
                    self.tail_subtype |= bit(3)
                frame = makebits(self.tail_frmtype,4,4) | makebits(self.tail_subtype,0,4)
                data += struct.pack('<B',frame)
                data += TailFrame.tsencode(self.tail_txtime)
                if not self.tail_owr:
                    cnt = len(self.tail_rxtimes)
                    data += struct.pack('<B', cnt)
                    mask = 1
                    bits = 0
                    for addr in self.tail_rxtimes:
                        if len(addr) == 8:
                            bits |= mask
                        mask <<= 1
                    for i in range(0,cnt,8):
                        data += struct.pack('<B', ((bits>>i) & 0xff))
                    for (addr,time) in self.tail_rxtimes.items():
                        if len(addr) == 8:
                            data += struct.pack('8s', byteswap(addr))
                        else:
                            data += struct.pack('2s', byteswap(addr))
                        data += TailFrame.tsencode(time)
            elif self.tail_frmtype == 4:
                if self.tail_subtype == 0:
                    data += struct.pack('<H',self.tail_reset_magic)
                elif self.tail_subtype == 1:
                    data += struct.pack('<H',self.tail_iterator)
                elif self.tail_subtype == 2:
                    data += struct.pack('<B',len(self.tail_config))
                    for key in tail_config:
                        data += struct.pack('<H',key)
                elif self.tail_subtype == 3:
                    data += struct.pack('<B',len(self.tail_config))
                    for (key,val) in tail_config.items():
                        data += struct.pack('<Hp',key,val)
                elif self.tail_subtype == 4:
                    data += struct.pack('<B',len(self.tail_config))
                    for key in tail_config:
                        data += struct.pack('<H',key)
                elif self.tail_subtype == 5:
                    data += struct.pack('<16s',self.tail_salt)
                elif self.tail_subtype == 15:
                    data += struct.pack('<16s',self.tail_test)
                else:
                    raise NotImplementedError('encode config request {}'.format(self.tail_subtype))
            elif self.tail_frmtype == 5: 
                if self.tail_subtype == 0:
                    data += struct.pack('<H',self.tail_reset_magic)
                elif self.tail_subtype == 1:
                    data += struct.pack('<H',self.tail_iterator)
                    data += struct.pack('<B',len(self.tail_config))
                    for key in tail_config:
                        data += struct.pack('<H',key)
                elif self.tail_subtype == 2:
                    data += struct.pack('<B',len(self.tail_config))
                    for (key,val) in tail_config.items():
                        data += struct.pack('<Hp',key,val)
                elif self.tail_subtype == 3:
                    data += struct.pack('<B',self.tail_code)
                elif self.tail_subtype == 4:
                    data += struct.pack('<B',self.tail_code)
                elif self.tail_subtype == 5:
                    data += struct.pack('<16s',self.tail_salt)
                elif self.tail_subtype == 15:
                    data += struct.pack('<16s',self.tail_test)
                else:
                    raise NotImplementedError('encode config response {}'.format(self.tail_subtype))
            elif self.tail_frmtype == 15:
                self.tail_subtype = 0
                if self.tail_timing:
                    self.tail_subtype |= bit(3)
                if self.tail_txtime:
                    self.tail_subtype |= bit(2)
                if self.tail_rxtimes:
                    self.tail_subtype |= bit(1)
                if self.tail_rxinfos:
                    self.tail_subtype |= bit(0)
                frame = makebits(self.tail_frmtype,4,4) | makebits(self.tail_subtype,0,4)
                data += struct.pack('<B',frame)
                if self.tail_txtime:
                    data += TailFrame.tsencode(self.tail_txtime)
                if self.tail_rxtimes:
                    addrs = self.tail_rxtimes.keys()
                elif self.tail_rxinfos:
                    addrs = self.tail_rxinfos.keys()
                if self.tail_rxtimes or self.tail_rxinfos:
                    cnt = len(addrs)
                    data += struct.pack('<B', cnt)
                    mask = 1
                    bits = 0
                    for addr in addrs:
                        if len(addr) == 8:
                            bits |= mask
                        mask <<= 1
                    for i in range(0,cnt,8):
                        data += struct.pack('<B', ((bits>>i) & 0xff))
                    for addr in addrs:
                        if len(addr) == 8:
                            data += struct.pack('8s', byteswap(addr))
                        else:
                            data += struct.pack('2s', byteswap(addr))
                        if self.tail_rxtimes:
                            data += TailFrame.tsencode(self.tail_rxtimes[addr])
                        if self.tail_rxinfos:
                            data += struct.pack('<4H', *self.tail_rxinfos[addr])
            else:
                raise NotImplementedError('encode tail frametype {}'.format(self.tail_frmtype))
        elif self.tail_protocol == 2:
            data += struct.pack('<B',TailFrame.PROTO_2)
            data += self.tail_payload
        else:
            data += self.tail_payload
        self.frame_len = len(data)
        self.frame_data = data
        return data

    def __str__(self):
        """Returns a string representation of the Tail frame (includes WPAN info)."""
        str = WPANFrame.__str__(self)
        if WPANFrame.verbosity == 0:
            str += ' TAIL'
            if self.tail_protocol == 1:
                if self.tail_frmtype == 0:
                    str += ' Tag Blink sub:{} flags:{}'.format(self.tail_subtype, self.tail_flags)
                elif self.tail_frmtype == 1:
                    str += ' Anchor Beacon sub:{} flags:{} ref:{}'.format(self.tail_subtype, self.tail_flags, self.tail_beacon.hex())
                elif self.tail_frmtype == 2:
                    str += ' Ranging Request'
                elif self.tail_frmtype == 3:
                    str += ' Ranging Response OWR:{}'.format(self.tail_owr)
                    if self.tail_rxtimes:
                        str += ' rxtimes:{}'.format(len(self.tail_rxtimes))
                elif self.tail_frmtype == 4:
                    str += ' Config Request'
                elif self.tail_frmtype == 5:
                    str += ' Config Response'
            elif self.tail_protocol == 2:
                str += ' Encrypted Frame'
        else:
            if self.tail_protocol == 1:
                str += fattrnl('TAIL Proto', '0x37',2)
                if self.tail_frmtype == 0:
                    str += fattrnl('Frame type', 'Tag Blink {}:{}'.format(self.tail_frmtype,self.tail_subtype), 4)
                    str += fattrnl('EIEs', testbit(self.tail_subtype,1), 4)
                    str += fattrnl('IEs', testbit(self.tail_subtype,2), 4)
                    str += fattrnl('Cookie', testbit(self.tail_subtype,3), 4)
                    str += fattrnl('Flags', '0x{:02x}'.format(self.tail_flags), 4)
                    str += fattrnl('Listen', testbit(self.tail_flags,7), 6)
                    str += fattrnl('Accel', testbit(self.tail_flags,6), 6)
                    str += fattrnl('DCin', testbit(self.tail_flags,5), 6)
                    str += fattrnl('Salt', testbit(self.tail_flags,4), 6)
                    if self.tail_cookie is not None:
                        str += fattrnl('Cookie', self.tail_cookie.hex(), 4)
                    if self.tail_ies is not None:
                        str += fattrnl('IEs', len(self.tail_ies), 4)
                        for (key,val) in self.tail_ies.items():
                            str += fattrnl(key,val,6)
                elif self.tail_frmtype == 1:
                    str += fattrnl('Frame type', 'Anchor Beacon {}:{}'.format(self.tail_frmtype,self.tail_subtype), 4)
                    str += fattrnl('Flags', '0x{:02x}'.format(self.tail_flags), 4)
                    str += fattrnl('Ref', self.tail_beacon.hex(), 4)
                elif self.tail_frmtype == 2:
                    str += fattrnl('Frame type', 'Ranging Request {}:{}'.format(self.tail_frmtype,self.tail_subtype), 4)
                elif self.tail_frmtype == 3:
                    str += fattrnl('Frame type', 'Ranging Response {}'.format(self.tail_frmtype), 4)
                    str += fattrnl('OWR', self.tail_owr, 6)
                    str += fattrnl('TxTime', self.tail_txtime, 4)
                    if self.tail_rxtimes:
                        str += fattrnl('RxTimes', len(self.tail_rxtimes), 4)
                        for (addr,time) in self.tail_rxtimes.items():
                            str += fattrnl(addr.hex(),time,6)
                elif self.tail_frmtype == 4:
                    str += fattrnl('Frame type', 'Config Request {}:{}\n'.format(self.tail_frmtype,self.tail_subtype), 4)
                    if self_tail_subtype == 0:
                        str += fattrnl('Config Req', 'RESET', 4)
                        str += fattrnl('Magic', self.tail_reset_magic, 6)
                    elif self_tail_subtype == 1:
                        str += fattrnl('Config Req', 'ENUMERATE', 4)
                        str += fattrnl('Iterator', self.tail_iterator, 6)
                    elif self_tail_subtype == 2:
                        str += fattrnl('Config Req', 'READ', 4)
                        str += fattrnl('Keys', '', 6)
                        for key in tail_config:
                            str += fattrnl(key,'',8)
                    elif self_tail_subtype == 3:
                        str += fattrnl('Config Req', 'WRITE', 4)
                        str += fattrnl('Keys', '', 6)
                        for (key,val) in tail_config.items():
                            str += fattrnl(key,val,8)
                    elif self_tail_subtype == 4:
                        str += fattrnl('Config Req', 'DELETE', 4)
                        str += fattrnl('Keys', '', 6)
                        for key in tail_config:
                            str += fattrnl(key,'',8)
                    elif self_tail_subtype == 5:
                        str += fattrnl('Config Req', 'SALT', 4)
                        str += fattrnl('Salt', self.tail_salt, 6)
                    elif self_tail_subtype == 15:
                        str += fattrnl('Config Req', 'TEST', 4)
                        str += fattrnl('Test', self.tail_test, 6)
                elif self.tail_frmtype == 5:
                    str += fattrnl('Frame type', 'Config Response {}:{}'.format(self.tail_frmtype,self.tail_subtype), 4)
                    if self_tail_subtype == 0:
                        str += fattrnl('Config Resp', 'RESET', 4)
                        str += fattrnl('Magic', self.tail_reset_magic, 6)
                    elif self_tail_subtype == 1:
                        str += fattrnl('Config Resp', 'ENUMERATE', 4)
                        str += fattrnl('Iterator', self.tail_iterator, 6)
                        for key in tail_config:
                            str += fattrnl(key,'',8)
                    elif self_tail_subtype == 2:
                        str += fattrnl('Config Resp', 'READ', 4)
                        str += fattrnl('Keys', '', 6)
                        for (key,val) in tail_config.items():
                            str += fattrnl(key,val,8)
                    elif self_tail_subtype == 3:
                        str += fattrnl('Config Resp', 'WRITE', 4)
                        str += fattrnl('Code', self.tail_code, 6)
                    elif self_tail_subtype == 4:
                        str += fattrnl('Config Resp', 'DELETE', 4)
                        str += fattrnl('Code', self.tail_code, 6)
                    elif self_tail_subtype == 5:
                        str += fattrnl('Config Resp', 'SALT', 4)
                        str += fattrnl('Salt', self.tail_salt, 6)
                    elif self_tail_subtype == 15:
                        str += fattrnl('Config Resp', 'TEST', 4)
                        str += fattrnl('Test', self.tail_test, 6)
                elif self.tail_frmtype == 15:
                    str += fattrnl('Frame type', 'Ranging Resp#2 {}'.format(self.tail_frmtype), 4)
                    str += fattrnl('Timing', bool(self.tail_timing), 6)
                    str += fattrnl('TxTime', bool(self.tail_txtime), 6)
                    str += fattrnl('RxTimes', bool(self.tail_rxtimes), 6)
                    str += fattrnl('RxInfos', bool(self.tail_rxinfos), 6)
                    if self.tail_txtime:
                        str += fattrnl('TxTime', self.tail_txtime, 4)
                    if self.tail_rxtimes:
                        str += fattrnl('RxTimes', len(self.tail_rxtimes), 4)
                        for (addr,time) in self.tail_rxtimes.items():
                            str += fattrnl(addr.hex(),time,6)
                    if self.tail_rxinfos is not None:
                        str += fattrnl('RxInfos', len(self.tail_rxinfos), 4)
                        for (addr,rxinfo) in self.tail_rxinfos.items():
                            str += fattrnl(addr.hex(),rxinfo,6)
            elif self.tail_protocol == 2:
                str += fattrnl('TAIL Proto', '0x38', 2)
                str += fattrnl('Payload', self.tail_payload.hex(), 4)
            elif self.tail_protocol == 0:
                str += fattrnl('Raw Payload', tail_payload.hex(), 2)
        return str

    
# ---------------------------------------------------------------------------
# Define Missing Socket Constants (for compatibility)
# ---------------------------------------------------------------------------
# This loop defines constants related to IEEE 802.15.4 sockets and timestamping
# if they are not already present in the standard `socket` module. This ensures
# the script works across different Python versions or environments where these
# might not be standard.
for name,value in (
        ('PROTO_IEEE802154', 0xf600),
        ('SO_TIMESTAMPING', 37),
        ('SOF_TIMESTAMPING_TX_HARDWARE',  (1<<0)),
        ('SOF_TIMESTAMPING_TX_SOFTWARE',  (1<<1)),
        ('SOF_TIMESTAMPING_RX_HARDWARE',  (1<<2)),
        ('SOF_TIMESTAMPING_RX_SOFTWARE',  (1<<3)),
        ('SOF_TIMESTAMPING_SOFTWARE',     (1<<4)),
        ('SOF_TIMESTAMPING_SYS_HARDWARE', (1<<5)),
        ('SOF_TIMESTAMPING_RAW_HARDWARE', (1<<6))):
    if not hasattr(socket, name):
        setattr(socket, name, value)
