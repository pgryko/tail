# -*- python -*-
#
# config.py - Configuration and Constants for Tail RTLS
#
# This file defines various constants, calibration data (splines, tables),
# and default/device-specific configurations used by the RTLS scripts.
# Note: This appears to be mostly hardcoded values rather than a dynamic config loader.
#
# --- Network ---
RPC_PORT = 61666 # Default RPC Port (Note: Different from calib/calibrate.py default)

# --- Physical Constants ---
C_ABS = 299792458 # Speed of light in vacuum (m/s)
C_AIR = 299705000 # Approx. speed of light in air (m/s)

# --- DW1000 Constants ---
DW1000_CLOCK_GHZ = 63.8976 # DW1000 clock frequency (GHz)
DW1000_CLOCK_HZ  = DW1000_CLOCK_GHZ * 1E9 # DW1000 clock frequency (Hz)

# Default Antenna Delays (likely superseded by DW1000_DEVICE_CALIB)
DW1000_64PRF_ANTD_NS = 514.4620 # Typical delay (ns) for 64 MHz PRF
DW1000_16PRF_ANTD_NS = 513.9067 # Typical delay (ns) for 16 MHz PRF
DW1000_64PRF_ANTD_DEFAULT = int(DW1000_64PRF_ANTD_NS * DW1000_CLOCK_GHZ / 2) # Default delay (ticks) for 64 MHz PRF
DW1000_16PRF_ANTD_DEFAULT = int(DW1000_16PRF_ANTD_NS * DW1000_CLOCK_GHZ / 2) # Default delay (ticks) for 16 MHz PRF

# --- Calibration Data ---

# DW1000 RX Power Table (Purpose unclear - mapping ADC values? Deprecated?)
# Format: Tuple of tuples, possibly (Input_Value, Ch1_Val, Ch2_Val, Ch3_Val) ?
DW1000_RX_POWER_TABLE = (
    (0, 0, 0, 0),
    (25, 25, 25, 25),
    (65, 65, 65, 65),
    (105, 105, 105, 105),
    (145, 145, 145, 145),
    (185, 185, 185, 185),
    (225, 225, 225, 225),
    (265, 265, 265, 265),
    (305, 305, 303, 305),
    (345, 345, 342, 345),
    (385, 385, 382, 385),
    (425, 420, 422, 425),
    (465, 460, 466, 465),
    (505, 502, 506, 505),
    (545, 542, 546, 545),
    (585, 576, 578, 576),
    (625, 612, 606, 622),
    (665, 644, 630, 658),
    (705, 668, 670, 695),
    (745, 686, 706, 730),
    (785, 710, 738, 765),
    (825, 716, 774, 795),
    (865, 735, 802, 810),
    (905, 752, 846, 840),
    (945, 763, 878, 865),
    (985, 775, 898, 888),
    (1025, 784, 921, 908),
    (1065, 796, 938, 928),
    (1105, 808, 954, 948),
    (1145, 816, 961, 966),
    (1185, 831, 975, 980),
    (1225, 843, 986, 1004),
    (1265, 854, 990, 1024),
    (1305, 866, 997, 1050),
    (1345, 883, 1006, 1070),
    (1385, 895, 1010, 1086),
    (1425, 904, 1018, 1098),
    (1465, 915, 1022, 1110),
    (1505, 924, 1026, 1118),
    (1545, 934, 1030, 1128),
    (1585, 944, 1034, 1140),
)

# Compensation Splines (Similar/identical to those in dwarf.py and calib/calibrate.py)
# Used to correct range measurements based on received signal power (dBm).
# Structure: {bandwidth_MHz: {PRF_MHz: ( ((dBm_min, dBm_max), (coeff_a, coeff_b, coeff_c)), ... )}}
DW1000_COMP_SPLINES = {
    500: {
        16: (
             ((-95.0, -89.0), (455.2684032227685, 9.851533848614533, 0.051993231111100435)),
             ((-89.0, -81.0), (622.1582642485359, 13.601867408230925, 0.07306251852581003)),
             ((-81.0, -75.0), (-206.63577569900568, -6.862181359285285, -0.05325876022235132)),
             ((-75.0, -69.0), (-14.553074442797616, -1.7399765082001553, -0.019110731344085252)),
             ((-69.0, -61.0), (-65.34665835419767, -3.212254801451526, -0.02977941823739272)),
        ),
        64: (
              ((-105.0, -91.0), (60.352219498834565, 1.259091772638299, 0.005614160222348175)),
              (( -91.0, -81.0), (182.24644860110973, 3.938085215997636, 0.020333901101900254)),
              (( -81.0, -75.0), (416.4655048521236, 9.72127075199398, 0.05603257081757784)),
              (( -75.0, -67.0), (-235.36598188854484, -7.660900913339617, -0.05984856478355027)),
              (( -67.0, -55.0), (-15.291050806896337, -1.0915006691080062, -0.010823194973097472)),
        ),
    },
    900: {
        16: (
             ((-95.0, -89.0), (183.67480691132866, 3.6158987280067723, 0.013391462077503746)),
             ((-89.0, -81.0), (909.6294873191769, 19.92948671225841, 0.10504083084258342)),
             ((-81.0, -75.0), (-79.62234302197686, -4.496483922533177, -0.0457367625993812)),
             ((-75.0, -69.0), (47.94037800579669, -1.094811223380935, -0.023058943654632458)),
             ((-69.0, -61.0), (-95.74259549796217, -5.259534749105479, -0.05323809715869743)),
        ),
        64: (
             ((-105.0, -95.0), (216.3308306122728, 4.1714254724783855, 0.01677363406513388)),
             (( -95.0, -83.0), (148.11352408935457, 2.7353156634156304, 0.009215393273090733)),
             (( -83.0, -75.0), (732.1532410581545, 16.808623222604062, 0.09399472336764236)),
             (( -75.0, -67.0), (-20.421159267426496, -3.260069209064098, -0.039796838878970675)),
             (( -67.0, -61.0), (-96.7161189002168, -5.537443600292282, -0.0567915127387808)),
             (( -61.0, -55.0), (5.839529695372041, -2.1750623191076377, -0.029231829906617435)),
        ),
    },
}

# --- Configuration Defaults & Device Specifics ---

# List of standard DW1000 attributes used for configuration/status display
DW1000_ATTRS = (
    'channel',
    'prf',
    'pcode',
    'txpsr',
    'rate',
    'smart_power',
    'tx_power',
    'xtalt',
    'antd',
    'snr_threshold',
    'fpr_threshold',
    'noise_threshold',
)

# List of attributes available within the detailed Timestamp Info (TSInfo) structure
DW1000_TSINFO_ATTRS = (
    'rawts',
    'lqi',
    'snr',
    'fpr',
    'noise',
    'rxpacc',
    'fp_index',
    'fp_ampl1',
    'fp_ampl2',
    'fp_ampl3',
    'cir_pwr',
    'fp_pwr',
    'ttcko',
    'ttcki',
    'temp',
    'volt',
)

# Default radio configuration parameters used if not overridden by device-specific config
DW1000_DEFAULT_CONFIG = {
    'channel'	      : 7,
    'pcode'	      : 20,
    'prf'	      : 64,
    'rate'	      : 6800,
    'txpsr'	      : 256,
    'smart_power'     : 0,
    'tx_power'        : 0xd1d1d1d1,
    'xtalt'	      : 15,
    'antd'            : DW1000_64PRF_ANTD_DEFAULT,
    'snr_threshold'   : 0,
    'fpr_threshold'   : 0,
    'noise_threshold' : 65535, # High threshold, likely disabling noise filtering
}

# Specific configuration profile presumably used during calibration procedures
DW1000_CALIB_CONFIG = {
    'channel'	      : 7,
    'pcode'	      : 20,
    'prf'	      : 64,
    'rate'	      : 850,
    'txpsr'	      : 1024,
    'smart_power'     : 0,
    'tx_power'        : 0xb1b1b1b1,
    'snr_threshold'   : 0,
    'fpr_threshold'   : 0,
    'noise_threshold' : 65535,
}

# Device-specific calibration values and metadata (coordinates, hostnames)
# Keyed by device EUI-64 address (lowercase hex string).
# This dictionary likely overrides defaults and potentially the ANTD constants above.
DW1000_DEVICE_CALIB = {
    '70b3d5b1e0000001': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000002': {
        'xtalt'	   : 17,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000003': {
        'xtalt'	   : 17,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000004': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000005': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000006': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000007': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000008': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000009': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e000000a': {
        'xtalt'	   : 17,
        'antd'     : 0x403b,
    },
    '70b3d5b1e000000b': {
        'xtalt'	   : 18,
        'antd'     : 0x403b,
    },
    '70b3d5b1e000000c': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e000000d': {
        'xtalt'	   : 15,
        'antd'     : 0x4018,
    },
    '70b3d5b1e000000e': {
        'xtalt'	   : 17,
        'antd'     : 0x403b,
    },
    '70b3d5b1e000000f': {
        'xtalt'	   : 16,
        'antd'     : 0x403b,
    },
    '70b3d5b1e0000010': {
        'xtalt'	   : 18,
        'antd'     : 0x4010,
    },
    '70b3d5b1e0000011': {
        'bss'      : 4,
        'host'     : 'bss5',
        'xtalt'	   : 17,
        'antd'     : 0x401e,
        'coord'    : (2.666, 0.185, 1.255),
    },
    '70b3d5b1e0000012': {
        'xtalt'	   : 23,		# Unstable
        'antd'     : 0x401f,
    },
    '70b3d5b1e0000013': {
        'bss'      : 5,
        'host'     : 'bss6',
        'xtalt'	   : 15,
        'antd'     : 0x402f,
        'coord'    : (6.177, 0.185, 1.255),
    },
    '70b3d5b1e0000014': {
        'bss'      : 0,
        'host'     : 'bss1',
        'xtalt'	   : 16,
        'antd'     : 0x4049,
        'coord'    : (0.150, 0.475, 0.035),
    },
    '70b3d5b1e0000015': {
        'bss'      : 1,
        'host'     : 'bss2',
        'xtalt'	   : 14,
        'antd'     : 0x4040,
        'coord'    : (8.545, 0.420, 0.035),
    },
    '70b3d5b1e0000016': {
        'bss'      : 2,
        'host'     : 'bss3',
        'xtalt'	   : 17,
        'antd'     : 0x404a,
        'coord'    : (8.567, 5.807, 0.035),
    },
    '70b3d5b1e0000017': {
        'bss'      : 3,
        'host'     : 'bss4',
        'xtalt'	   : 15,
        'antd'     : 0x4032,
        'coord'    : (0.175, 5.860, 0.035),
    },
    '70b3d5b1e0000018': {
        'bss'      : 6,
        'host'     : 'bss7',
        'xtalt'	   : 15,
        'antd'     : 0x4023,
        'coord'    : (6.102, 6.146, 1.265),
    },
    '70b3d5b1e0000019': {
        'bss'      : 7,
        'host'     : 'bss8',
        'xtalt'	   : 15,
        'antd'     : 0x4018,
        'coord'    : (2.299, 6.140, 1.270),
    },
    '70b3d5b1e000001b': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000001c': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000001d': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000001e': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000001f': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e0000025': {
        'xtalt'	   : 14,
        'antd'     : 0x4020,
    },
    '70b3d5b1e0000027': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000002a': { # Unstable?
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000002b': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000002c': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000002d': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000002e': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e000002f': {
        'xtalt'	   : 16,
        'antd'     : 0x4020,
    },
    '70b3d5b1e0000022': {
        'host'     : 'magpi0',
        'xtalt'	   : 17,
        'antd'     : 0x4055,
    },
    '70b3d5b1e0000020': {
        'host'     : 'magpi1',
        'xtalt'	   : 17,
        'antd'     : 0x4053,
    },
    '70b3d5b1e0000021': {
        'host'     : 'magpi2',
        'xtalt'	   : 17,
        'antd'     : 0x404D,
    },
    '70b3d5b1e0000024': {
        'host'     : 'magpi3',
        'xtalt'	   : 16,
        'antd'     : 0x4054,
    },
    '70b3d5b1e0000026': {
        'host'     : 'magpi4',
        'xtalt'	   : 17,
        'antd'     : 0x4055,
    },
    '70b3d5b1e0000029': {
        'host'     : 'magpi5',
        'xtalt'	   : 17,
        'antd'     : 0x4049,
    },
    '70b3d5b1e000001a': {
        'host'     : 'magpi06',
        'xtalt'	   : 16,
        'antd'     : 0x404E,
    },
    '70b3d5b1e0000023': {
        'host'     : 'magpi7',
        'xtalt'	   : 17,
        'antd'     : 0x404A,
    },
    '70b3d5b1e0000028': {
        'host'     : 'magpi8',
        'xtalt'	   : 18,
        'antd'     : 0x404A,
    },
}
