/*****************************************************************
 * EINSY Rambo Pin Assignments
 ******************************************************************/

#define ELECTRONICS "EINSY_RAMBO"

#define KNOWN_BOARD
#ifndef __AVR_ATmega2560__
  #error Oops!  Make sure you have 'Arduino Mega 2560 or Rambo' selected from the 'Tools -> Boards' menu.
#endif

#define TMC2130      // Trinamic Stepper Motor Drivers
#define UVLO_SUPPORT // Under Voltage Lockout Support


#define AMBIENT_THERMISTOR
#define PINDA_THERMISTOR


#define SWI2C                    // enable software i2c
#define SWI2C_A8                 // 8bit address functions

// #define PAT9125_SWI2C
// #define PAT9125_SWI2C_SDA      20 //SDA on P3
// #define PAT9125_SWI2C_SCL      21 //SCL on P3
// #define PAT9125_SWI2C_CFG    0xb1 //2us clock delay, 2048 cycles timeout

//#define PAT9125_HWI2C

#define X_TMC2130_CS           41
#define X_TMC2130_DIAG         64
#define X_STEP_PIN             37
#define X_DIR_PIN              49
#define X_MIN_PIN              12
//#define X_MAX_PIN              30
//#define X_MIN_PIN              X_TMC2130_DIAG
#define X_MAX_PIN              X_TMC2130_DIAG
#define X_ENABLE_PIN           29
#define X_MS1_PIN           -1
#define X_MS2_PIN           -1

#define Y_TMC2130_CS        39
#define Y_TMC2130_DIAG      69
#define Y_STEP_PIN          36
#define Y_DIR_PIN           48
#define Y_MIN_PIN           11
//#define Y_MAX_PIN           24
//#define Y_MIN_PIN           Y_TMC2130_DIAG
#define Y_MAX_PIN           Y_TMC2130_DIAG
#define Y_ENABLE_PIN        28
#define Y_MS1_PIN           -1
#define Y_MS2_PIN           -1

#define Z_TMC2130_CS        67
#define Z_TMC2130_DIAG      68
#define Z_STEP_PIN          35
#define Z_DIR_PIN           47
#define Z_MIN_PIN           10
#define Z_MAX_PIN           23
//#define Z_MAX_PIN           Z_TMC2130_DIAG
#define Z_ENABLE_PIN        27
#define Z_MS1_PIN           -1
#define Z_MS2_PIN           -1

#define HEATER_BED_PIN       4 //PG5
#define TEMP_BED_PIN         2 //A2

#define HEATER_0_PIN         3 //PE5
#define TEMP_0_PIN           0 //A0

#define HEATER_1_PIN        -1
#define TEMP_1_PIN           1 //A1

#define HEATER_2_PIN        -1
#define TEMP_2_PIN          -1

#define TEMP_AMBIENT_PIN     5 //A5

#define TEMP_PINDA_PIN       3 //A3

#define VOLT_PWR_PIN         4 //A4
#define VOLT_BED_PIN         9 //A9


#define E0_TMC2130_CS       66
#define E0_TMC2130_DIAG     65
#define E0_STEP_PIN         34
#define E0_DIR_PIN          43
#define E0_ENABLE_PIN       26
#define E0_MS1_PIN          -1
#define E0_MS2_PIN          -1

// Multiplexer Pins
#define E_MUX0_PIN         17  
#define E_MUX1_PIN         76
// #define E_MUX1_PIN         16  //issue with EINSY_RAMBO board (always +5V)

#define SDPOWER             -1
#define SDSS                77
#define LED_PIN             13
#define FAN_PIN              6
#define FAN_1_PIN           -1
#define PS_ON_PIN           -1
#define KILL_PIN            -1  // 80 with Smart Controller LCD
#define SUICIDE_PIN         -1  // PIN that has to be turned on right after start, to keep power flowing.

#ifdef ULTRA_LCD

//#define KILL_PIN            32

#ifdef NEWPANEL

//#define LCD_PWM_PIN         -1//32  // lcd backlight brightnes pwm control pin
//#define LCD_PWM_MAX         0x0f  // lcd pwm maximum value (0x07=64Hz, 0x0f=32Hz, 0x1f=16Hz)

#define BEEPER              84  // Beeper on AUX-4
#define LCD_PINS_RS         82
#define LCD_PINS_ENABLE     61
#define LCD_PINS_D4         59
#define LCD_PINS_D5         70
#define LCD_PINS_D6         85
#define LCD_PINS_D7         71

//buttons are directly attached using AUX-2
#define BTN_EN1                72
#define BTN_EN2                14
#define BTN_ENC                 9  // the click

#define SDCARDDETECT           15

#define TACH_0                 79
#define TACH_1                 80

#endif //NEWPANEL
#endif //ULTRA_LCD
