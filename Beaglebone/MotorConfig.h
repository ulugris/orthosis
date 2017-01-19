#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

// Common parameters

#define RS232 0         // Use RS232 interface (1) instead of USB (0)
#if RS232 == 1
    #ifdef __linux__
        #define RS232PORT "/dev/ttyS4"
    #else
        #define RS232PORT "COM3"
    #endif
#endif

#define ENCR4X 4096     // Encoder resolution (4X)
#define GEARRT 160      // Gear ratio

#define NOMCUR 3210     // Continuous current limit in mA
#define MAXCUR 7500     // Output current limit in mA
#define THCONS 296      // Thermal time constant winding (ms)
#define NUMPOL 8        // Pole pair number

#define MAXVEL 12500    // Maximum motor velocity and profile velocity in rpm
#define MAXACC 1000000  // Maximum motor acceleration and profile acceleration in rpm/s

#define HOMACC 5000     // Acceleration for homing profile
#define HOMSPS 5000     // Speed during search for switch
#define HOMSPI 5000     // Speed during search for index
#define HOMCUR 5000     // Current threshold
#define HOMTMO 10000    // Homing timeout in ms


// Right motor regulator parameters

#define RCURPGAIN 358   // Current regulator P-gain
#define RCURIGAIN 73    // Current regulator I-gain

#define RSPDPGAIN 4331  // Speed regulator P-gain
#define RSPDIGAIN 628   // Speed regulator I-gain

#define RSPDFFVEL 17281 // Velocity feedforward factor in speed regulator
#define RSPDFFACC 856   // Acceleration feedforward factor in speed regulator

#define RPOSPGAIN 942   // Position regulator P-gain
#define RPOSIGAIN 2429  // Position regulator I-gain
#define RPOSDGAIN 1733  // Position regulator D-gain

#define RPOSFFVEL 17281 // Velocity feedforward factor in position regulator
#define RPOSFFACC 856   // Acceleration feedforward factor in position regulator


// Left motor regulator parameters

#define LCURPGAIN 358   // Current regulator P-gain
#define LCURIGAIN 62    // Current regulator I-gain

#define LSPDPGAIN 4785  // Speed regulator P-gain
#define LSPDIGAIN 681   // Speed regulator I-gain

#define LSPDFFVEL 20244 // Velocity feedforward factor in speed regulator
#define LSPDFFACC 983   // Acceleration feedforward factor in speed regulator

#define LPOSPGAIN 1022  // Position regulator P-gain
#define LPOSIGAIN 2560  // Position regulator I-gain
#define LPOSDGAIN 1921  // Position regulator D-gain

#define LPOSFFVEL 20244 // Velocity feedforward factor in position regulator
#define LPOSFFACC 983   // Acceleration feedforward factor in position regulator

#endif // MOTORCONFIG_H

