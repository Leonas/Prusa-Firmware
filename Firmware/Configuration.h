#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "boards.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// Firmware version
#define FW_VERSION "3.2.1"
#define FW_COMMIT_NR   576
#define FW_DEV_VERSION FW_VERSION_UNKNOWN // FW_VERSION_UNKNOWN = unofficial build
#define FW_REPOSITORY "Unknown"
#define FW_VERSION_FULL FW_VERSION "-" STR(FW_COMMIT_NR)

// Debug version must have #define DEBUG_BUILD
#define FW_VERSION_DEBUG    6
#define FW_VERSION_DEVEL    5
#define FW_VERSION_ALPHA    4
#define FW_VERSION_BETA     3
#define FW_VERSION_RC       2
#define FW_VERSION_GOLD     1  //release version
#define FW_VERSION_UNKNOWN  0

#if FW_DEV_VERSION == FW_VERSION_DEBUG
#define DEBUG_BUILD
#else
#undef DEBUG_BUILD
#endif

#include "Configuration_prusa.h"

// Optional unique identifier for this printer, used by some programs to differentiate between machines.
// http://www.uuidgenerator.net/version4
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

#define FW_PRUSA3D_MAGIC "PRUSA3DFW"
#define FW_PRUSA3D_MAGIC_LEN 10

#define EEPROM_TOP 4096
#define EEPROM_SILENT 4095
#define EEPROM_LANG 4094
#define EEPROM_BABYSTEP_X 4092
#define EEPROM_BABYSTEP_Y 4090
#define EEPROM_BABYSTEP_Z 4088
#define EEPROM_CALIBRATION_STATUS 4087
#define EEPROM_BABYSTEP_Z0 4085
#define EEPROM_FILAMENTUSED 4081
#define EEPROM_TOTALTIME 4077

#define EEPROM_BED_CALIBRATION_CENTER     (EEPROM_TOTALTIME-2*4)
#define EEPROM_BED_CALIBRATION_VEC_X      (EEPROM_BED_CALIBRATION_CENTER-2*4)
#define EEPROM_BED_CALIBRATION_VEC_Y      (EEPROM_BED_CALIBRATION_VEC_X-2*4)

// Offsets of the Z heiths of the calibration points from the first point.
// The offsets are saved as 16bit signed int, scaled to tenths of microns.
#define EEPROM_BED_CALIBRATION_Z_JITTER   (EEPROM_BED_CALIBRATION_VEC_Y-2*8)
#define EEPROM_FARM_MODE (EEPROM_BED_CALIBRATION_Z_JITTER-1)
#define EEPROM_FARM_NUMBER (EEPROM_FARM_MODE-3)

// Correction of the bed leveling, in micrometers.
// Maximum 50 micrometers allowed.
// Bed correction is valid if set to 1. If set to zero or 255, the successive 4 bytes are invalid.
#define EEPROM_BED_CORRECTION_VALID (EEPROM_FARM_NUMBER-1)
#define EEPROM_BED_CORRECTION_LEFT  (EEPROM_BED_CORRECTION_VALID-1)
#define EEPROM_BED_CORRECTION_RIGHT (EEPROM_BED_CORRECTION_LEFT-1)
#define EEPROM_BED_CORRECTION_FRONT (EEPROM_BED_CORRECTION_RIGHT-1)
#define EEPROM_BED_CORRECTION_REAR  (EEPROM_BED_CORRECTION_FRONT-1)
#define EEPROM_TOSHIBA_FLASH_AIR_COMPATIBLITY (EEPROM_BED_CORRECTION_REAR-1)
#define EEPROM_PRINT_FLAG (EEPROM_TOSHIBA_FLASH_AIR_COMPATIBLITY-1)
#define EEPROM_PROBE_TEMP_SHIFT (EEPROM_PRINT_FLAG - 2*5) //5 x int for storing pinda probe temp shift relative to 50 C; unit: motor steps 
#define EEPROM_TEMP_CAL_ACTIVE (EEPROM_PROBE_TEMP_SHIFT - 1)
#define EEPROM_BOWDEN_LENGTH (EEPROM_TEMP_CAL_ACTIVE - 2*4) //4 x int for bowden lengths for multimaterial
#define EEPROM_CALIBRATION_STATUS_PINDA (EEPROM_BOWDEN_LENGTH - 1) //0 - not calibrated; 1 - calibrated
#define EEPROM_UVLO						(EEPROM_CALIBRATION_STATUS_PINDA - 1) //1 - uvlo during print
#define EEPROM_UVLO_CURRENT_POSITION	(EEPROM_UVLO-2*4) // 2 x float for current_position in X and Y axes
#define EEPROM_FILENAME (EEPROM_UVLO_CURRENT_POSITION - 8) //8chars to store filename without extension
#define EEPROM_FILE_POSITION (EEPROM_FILENAME - 4) //32 bit for uint32_t file position 
#define EEPROM_UVLO_CURRENT_POSITION_Z	(EEPROM_FILE_POSITION - 4) //float for current position in Z
#define EEPROM_UVLO_TARGET_HOTEND		(EEPROM_UVLO_CURRENT_POSITION_Z - 1)
#define EEPROM_UVLO_TARGET_BED			(EEPROM_UVLO_TARGET_HOTEND - 1)
#define EEPROM_UVLO_FEEDRATE			(EEPROM_UVLO_TARGET_BED - 2)
#define EEPROM_UVLO_FAN_SPEED			(EEPROM_UVLO_FEEDRATE - 1) 
#define EEPROM_FAN_CHECK_ENABLED		(EEPROM_UVLO_FAN_SPEED - 1)
#define EEPROM_UVLO_MESH_BED_LEVELING     (EEPROM_FAN_CHECK_ENABLED - 9*2)

#define EEPROM_UVLO_Z_MICROSTEPS     (EEPROM_UVLO_MESH_BED_LEVELING - 2)
#define EEPROM_UVLO_E_ABS            (EEPROM_UVLO_Z_MICROSTEPS - 1)
#define EEPROM_UVLO_CURRENT_POSITION_E	(EEPROM_UVLO_E_ABS - 4)                 //float for current position in E

// Crash detection mode EEPROM setting 
#define EEPROM_CRASH_DET         (EEPROM_UVLO_CURRENT_POSITION_E - 5)           // float (orig EEPROM_UVLO_MESH_BED_LEVELING-12) 
// Crash detection counter Y (last print)
#define EEPROM_CRASH_COUNT_Y       (EEPROM_CRASH_DET - 1)                       // uint8 (orig EEPROM_UVLO_MESH_BED_LEVELING-15)
// Filament sensor on/off EEPROM setting 
#define EEPROM_FSENSOR           (EEPROM_CRASH_COUNT_Y - 1)                     // uint8 (orig EEPROM_UVLO_MESH_BED_LEVELING-14) 
// Crash detection counter X (last print)
#define EEPROM_CRASH_COUNT_X       (EEPROM_FSENSOR - 1)                         // uint8 (orig EEPROM_UVLO_MESH_BED_LEVELING-15)
// Filament runout/error coutner (last print)
#define EEPROM_FERROR_COUNT      (EEPROM_CRASH_COUNT_X - 1)                     // uint8 (orig EEPROM_UVLO_MESH_BED_LEVELING-16)
// Power loss errors (last print)
#define EEPROM_POWER_COUNT       (EEPROM_FERROR_COUNT - 1)                      // uint8 (orig EEPROM_UVLO_MESH_BED_LEVELING-17)

#define EEPROM_XYZ_CAL_SKEW (EEPROM_POWER_COUNT - 4)                            // float for skew backup
#define EEPROM_WIZARD_ACTIVE (EEPROM_XYZ_CAL_SKEW - 1)
#define EEPROM_BELTSTATUS_X (EEPROM_WIZARD_ACTIVE - 2)                          // uint16
#define EEPROM_BELTSTATUS_Y (EEPROM_BELTSTATUS_X - 2)                           // uint16

#define EEPROM_DIR_DEPTH        (EEPROM_BELTSTATUS_Y-1)
#define EEPROM_DIRS  (EEPROM_DIR_DEPTH-80) //8 chars for each dir name, max 10 levels
#define EEPROM_SD_SORT (EEPROM_DIRS - 1) //0 -time, 1-alpha, 2-none
#define EEPROM_SECOND_SERIAL_ACTIVE (EEPROM_SD_SORT - 1)

#define EEPROM_FSENS_AUTOLOAD_ENABLED (EEPROM_SECOND_SERIAL_ACTIVE - 1)

// Crash detection counter X (total)
#define EEPROM_CRASH_COUNT_X_TOT       (EEPROM_FSENS_AUTOLOAD_ENABLED - 2)     // uint16
// Crash detection counter Y (total)
#define EEPROM_CRASH_COUNT_Y_TOT       (EEPROM_CRASH_COUNT_X_TOT - 2)          // uint16
// Filament runout/error coutner (total)
#define EEPROM_FERROR_COUNT_TOT      (EEPROM_CRASH_COUNT_Y_TOT - 2)            // uint16
// Power loss errors (total)
#define EEPROM_POWER_COUNT_TOT       (EEPROM_FERROR_COUNT_TOT - 2)             // uint16

////////////////////////////////////////
// TMC2130 Accurate sensorless homing 

// X-axis home origin (stepper phase in microsteps, 0..63 for 16ustep resolution)
#define EEPROM_TMC2130_HOME_X_ORIGIN           (EEPROM_POWER_COUNT_TOT - 1)                    // uint8
// X-axis home bsteps (number of microsteps backward)
#define EEPROM_TMC2130_HOME_X_BSTEPS           (EEPROM_TMC2130_HOME_X_ORIGIN - 1)              // uint8
// X-axis home fsteps (number of microsteps forward)
#define EEPROM_TMC2130_HOME_X_FSTEPS           (EEPROM_TMC2130_HOME_X_BSTEPS - 1)              // uint8
// Y-axis home origin (stepper phase in microsteps, 0..63 for 16ustep resolution)
#define EEPROM_TMC2130_HOME_Y_ORIGIN           (EEPROM_TMC2130_HOME_X_FSTEPS - 1)              // uint8
// X-axis home bsteps (number of microsteps backward)
#define EEPROM_TMC2130_HOME_Y_BSTEPS           (EEPROM_TMC2130_HOME_Y_ORIGIN - 1)              // uint8
// X-axis home fsteps (number of microsteps forward)
#define EEPROM_TMC2130_HOME_Y_FSTEPS           (EEPROM_TMC2130_HOME_Y_BSTEPS - 1)              // uint8
// Accurate homing enabled
#define EEPROM_TMC2130_HOME_ENABLED            (EEPROM_TMC2130_HOME_Y_FSTEPS - 1)              // uint8


////////////////////////////////////////
// TMC2130 uStep linearity correction
// Linearity correction factor (XYZE)
#define EEPROM_TMC2130_WAVE_X_FAC              (EEPROM_TMC2130_HOME_ENABLED - 1)               // uint8
#define EEPROM_TMC2130_WAVE_Y_FAC              (EEPROM_TMC2130_WAVE_X_FAC - 1)                 // uint8
#define EEPROM_TMC2130_WAVE_Z_FAC              (EEPROM_TMC2130_WAVE_Y_FAC - 1)                 // uint8
#define EEPROM_TMC2130_WAVE_E_FAC              (EEPROM_TMC2130_WAVE_Z_FAC - 1)                 // uint8


////////////////////////////////////////
// TMC2130 uStep resolution
// microstep resolution (XYZE): usteps = (256 >> mres)
#define EEPROM_TMC2130_X_MRES              (EEPROM_TMC2130_WAVE_E_FAC - 1)                     // uint8
#define EEPROM_TMC2130_Y_MRES              (EEPROM_TMC2130_X_MRES - 1)                         // uint8
#define EEPROM_TMC2130_Z_MRES              (EEPROM_TMC2130_Y_MRES - 1)                         // uint8
#define EEPROM_TMC2130_E_MRES              (EEPROM_TMC2130_Z_MRES - 1)                         // uint8

#define EEPROM_PRINTER_TYPE          (EEPROM_TMC2130_E_MRES - 2)                               // uint16
#define EEPROM_BOARD_TYPE            (EEPROM_PRINTER_TYPE - 2)                                 // uint16

//TMC2130 configuration
#define EEPROM_TMC_AXIS_SIZE  //axis configuration block size
#define EEPROM_TMC_X (EEPROM_TMC + 0 * EEPROM_TMC_AXIS_SIZE) //X axis configuration block
#define EEPROM_TMC_Y (EEPROM_TMC + 1 * EEPROM_TMC_AXIS_SIZE) //Y axis
#define EEPROM_TMC_Z (EEPROM_TMC + 2 * EEPROM_TMC_AXIS_SIZE) //Z axis
#define EEPROM_TMC_E (EEPROM_TMC + 3 * EEPROM_TMC_AXIS_SIZE) //E axis
//TMC2130 - X axis
#define EEPROM_TMC_X_USTEPS_INTPOL   (EEPROM_TMC_X +  0) // 1byte, bit 0..4 USTEPS, bit 7 INTPOL
#define EEPROM_TMC_X_PWM_AMPL        (EEPROM_TMC_X +  1) // 1byte (0..255)
#define EEPROM_TMC_X_PWM_GRAD_FREQ   (EEPROM_TMC_X +  2) // 1byte, bit 0..3 GRAD, bit 4..5 FREQ
#define EEPROM_TMC_X_TCOOLTHRS       (EEPROM_TMC_X +  3) // 2bytes (0..)
#define EEPROM_TMC_X_SG_THRS         (EEPROM_TMC_X +  5) // 1byte, (-64..+63)
#define EEPROM_TMC_X_CURRENT_H       (EEPROM_TMC_X +  6) // 1byte, (0..63)
#define EEPROM_TMC_X_CURRENT_R       (EEPROM_TMC_X +  7) // 1byte, (0..63)
#define EEPROM_TMC_X_HOME_SG_THRS    (EEPROM_TMC_X +  8) // 1byte, (-64..+63)
#define EEPROM_TMC_X_HOME_CURRENT_R  (EEPROM_TMC_X +  9) // 1byte, (-64..+63)
#define EEPROM_TMC_X_HOME_DTCOOLTHRS (EEPROM_TMC_X + 10) // 1byte (-128..+127)
#define EEPROM_TMC_X_DTCOOLTHRS_LOW  (EEPROM_TMC_X + 11) // 1byte (-128..+127)
#define EEPROM_TMC_X_DTCOOLTHRS_HIGH (EEPROM_TMC_X + 12) // 1byte (-128..+127)
#define EEPROM_TMC_X_SG_THRS_LOW     (EEPROM_TMC_X + 13) // 1byte, (-64..+63)
#define EEPROM_TMC_X_SG_THRS_HIGH    (EEPROM_TMC_X + 14) // 1byte, (-64..+63)

// Currently running firmware, each digit stored as uint16_t.
// The flavor differentiates a dev, alpha, beta, release candidate or a release version.
#define EEPROM_FIRMWARE_VERSION_END       (FW_PRUSA3D_MAGIC_LEN+8)
#define EEPROM_FIRMWARE_VERSION_FLAVOR    (FW_PRUSA3D_MAGIC_LEN+6)
#define EEPROM_FIRMWARE_VERSION_REVISION  (FW_PRUSA3D_MAGIC_LEN+4)
#define EEPROM_FIRMWARE_VERSION_MINOR     (FW_PRUSA3D_MAGIC_LEN+2)
#define EEPROM_FIRMWARE_VERSION_MAJOR     FW_PRUSA3D_MAGIC_LEN
// Magic string, indicating that the current or the previous firmware running was the Prusa3D firmware.
#define EEPROM_FIRMWARE_PRUSA_MAGIC 0

#define EEPROM_OFFSET 20 //offset for storing settings using M500

#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.

#define SERIAL_PORT 0
#define BAUDRATE 115200



#define POWER_SUPPLY 1

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 3  // (seconds)
#define TEMP_HYSTERESIS 5       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.


// PID settings
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX BANG_MAX // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#ifdef PIDTEMP
// If the temperature difference between the target temperature and the actual temperature
// is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_FUNCTIONAL_RANGE 10 
  #define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID
  #define PID_dT ((OVERSAMPLENR * 10.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine
#endif // PIDTEMP


//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#ifdef DEBUG_DISABLE_PREVENT_EXTRUDER
#undef PREVENT_DANGEROUS_EXTRUDE
#undef PREVENT_LENGTHY_EXTRUDE
#endif //DEBUG_DISABLE_PREVENT_EXTRUDER

#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.


//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

#define ENDSTOPPULLUPS 

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.

const bool X_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = true; // set to true to invert the logic of the endstop.

// Disable max endstops for compatibility with endstop checking routine
#if defined(COREXY) && !defined(DISABLE_MAX_ENDSTOPS)
  #define DISABLE_MAX_ENDSTOPS
#endif

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled


// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#ifdef DEBUG_DISABLE_SWLIMITS
#define min_software_endstops false
#define max_software_endstops false
#else
#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.
#endif //DEBUG_DISABLE_SWLIMITS


#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS) 
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

#define Z_HEIGHT_HIDE_LIVE_ADJUST_MENU 2.0f


#define DEFAULT_XJERK                10       // (mm/sec)
#define DEFAULT_YJERK                10       // (mm/sec)
#define DEFAULT_ZJERK                 0.4     // (mm/sec)
#define DEFAULT_EJERK                 2.5     // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// Custom M code points
#define CUSTOM_M_CODES
#ifdef CUSTOM_M_CODES
  #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
  #define Z_PROBE_OFFSET_RANGE_MIN -15
  #define Z_PROBE_OFFSET_RANGE_MAX -5
#endif


#define HOST_KEEPALIVE_FEATURE    // Disable this if your host doesn't like keepalive messages
#define HOST_KEEPALIVE_INTERVAL 2 // Number of seconds between "busy" messages. Set with M113.

//LCD and SD support
#define ULTRA_LCD  //general LCD support, also 16x2
#define SDSUPPORT // Enable SD Card Support in Hardware Console
#define SD_CHECK_AND_RETRY // Use CRC checks and retries on the SD communication
#define ENCODER_PULSES_PER_STEP 4 // Increase if you have a high resolution encoder
#define ENCODER_STEPS_PER_MENU_ITEM 1 // Set according to ENCODER_PULSES_PER_STEP or your liking

// http://reprap.org/wiki/RepRapDiscount_Smart_Controller
#define REPRAP_DISCOUNT_SMART_CONTROLLER


#if defined(ULTIMAKERCONTROLLER) || defined(REPRAP_DISCOUNT_SMART_CONTROLLER) || defined(G3D_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
#endif

#if defined(REPRAPWORLD_KEYPAD)
  #define NEWPANEL
  #define ULTIPANEL
#endif
#if defined(RA_CONTROL_PANEL)
 #define ULTIPANEL
 #define NEWPANEL
 #define LCD_I2C_TYPE_PCA8574
 #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
#endif


// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://bitbucket.org/fmalpartida/new-LiquidCrystal_Prusa/wiki/schematics#!shiftregister-connection 


#ifdef ULTIPANEL
//  #define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the DOG graphic display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif
#else //no panel but just LCD
  #ifdef ULTRA_LCD
  #ifdef DOGLCD // Change number of lines to match the 128x64 graphics display
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 5
  #else
    #define LCD_WIDTH 16
    #define LCD_HEIGHT 2
  #endif
  #endif
#endif


#define DEFAULT_NOMINAL_FILAMENT_DIA  1.75

// Calibration status of the machine, to be stored into the EEPROM,
enum CalibrationStatus
{
	CALIBRATION_STATUS_ASSEMBLED = 255,       // Freshly assembled, needs to peform a self-test and the XYZ calibration.
	CALIBRATION_STATUS_XYZ_CALIBRATION = 250, // For the wizard: self test has been performed, now the XYZ calibration is needed.
	CALIBRATION_STATUS_Z_CALIBRATION = 240,   // For the wizard: factory assembled, needs to run Z calibration.
	CALIBRATION_STATUS_LIVE_ADJUST = 230, 	  // The XYZ calibration has been performed, now it remains to run the V2Calibration.gcode.
  CALIBRATION_STATUS_CALIBRATED = 1,
  CALIBRATION_STATUS_UNKNOWN = 0,
};

#include "Configuration_adv.h"
#include "thermistortables.h"


#endif //__CONFIGURATION_H
