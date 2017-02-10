///////////////////////////////////////////////////////////////////////////////////
/// @file launchPad.ino
/// @brief Sketch for the Compressed Air Rocket Launch Pad.
/// This sketch monitors the communications with the Master Panel using Zigbee,
/// reads air pressure in the PVC chambers, controls the Air Compressor, and
/// controlls the air valves to launch the rockets.
/// @see LaunchPadPacket.h
///
///////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <XBee.h>
#include <LaunchDataPacket.h>

///////////////////////////////////////////////////////////////////////////////////
/// @def _SafetyLedBitMode_
/// @brief Define the Digital Input # for the safety LED.
///
/// @def _SafetyLedBitPort_
/// @brief Define the Digital Port the safety LED is connected to.
///
/// @def _SafetyLedBit_
/// @brief Define the Port Bit # for the safety LED.
///
/// @def _ErrorLedBitMode_
/// @brief Define the Digital Pin # for the Error LED.
/// LED will flash when there is a communication error.
///
/// @def _ErrorLedPort_
/// @brief Define the Digital Port the Error LED is connected to.
///
/// @def _ErrorLedBit_
/// @brief Define the Port Bit # for the Error LED.
///
/// @def _Pad1LaunchBitMode_
/// @brief Define the Digital Pin # for the 1st launch pad control.
/// LED will flash when there is a communication error.
///
/// @def _ErrorLedPort_
/// @brief Define the Digital Port the Error LED is connected to.
///
/// @def _ErrorLedBit_
/// @brief Define the Port Bit # for the Error LED.
///
/// In Port D, the Port Bit # is equal to the Digital Pin #.
/// In Port B, the Port Bit # is equal to the Digital Pin # minus 8.
///////////////////////////////////////////////////////////////////////////////////
#define _SafetyLedBitMode_     2
#define _SafetyLedPort_        PORTD
#define _SafetyLedBit_         2

#define _ErrorLedBitMode_      3
#define _ErrorLedPort_         PORTD
#define _ErrorLedBit_          3

#define _Pad1LaunchBitMode_    6
#define _Pad1LaunchPort_       PORTD
#define _Pad1LaunchBit_        6

#define _Pad2LaunchBitMode_    5
#define _Pad2LaunchPort_       PORTD
#define _Pad2LaunchBit_        5

#define _CompressorBitMode_    4
#define _CompressorPort_       PORTD
#define _CompressorBit_        4

///////////////////////////////////////////////////////////////////////////////////
/// @def _Pressure1Analog_
/// @brief Define the Analog Input # for the pressure sensor in Chamber #1.
/// @def _Pressure2Analog_
/// @brief Define the Analog Input # for the pressure sensor in Chamber #2.
///////////////////////////////////////////////////////////////////////////////////
#define _Pressure1Analog_      1
#define _Pressure2Analog_      2

///////////////////////////////////////////////////////////////////////////////////
/// @def _pulse_halfwidth_
/// @brief Define the 1/2 period of the pulse used to flash the LED in the launch
/// button.
///
/// @def _zigbee_timeout_
/// @brief Define the timeout for the zigbee communications.  Error occurs if
/// the time since the last communicaitons exceeds the timeout.
///////////////////////////////////////////////////////////////////////////////////
#define _pulse_halfwidth_       100
#define _zigbee_timeout_        1000

///////////////////////////////////////////////////////////////////////////////////
/// @brief Track the on/off state of the launch button LED when the LED is
/// flashing.
///////////////////////////////////////////////////////////////////////////////////
boolean pulse = false;                    // Is the launch button LED on or off.

///////////////////////////////////////////////////////////////////////////////////
/// @brief Time since the last toggle of the LED on/off status.  Used to control
/// the flashing of the LED.
///////////////////////////////////////////////////////////////////////////////////
long timeSinceLastPulse = 0;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Time since the last good Zigbee communication with the Master Panel.
/// Used to monitor a timeout error.
///////////////////////////////////////////////////////////////////////////////////
long timeSinceLastMasterSignal = 0;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Was a Rocket Launch Command given in the last Zigbee communications
/// received. Used to debounce the Rocket Launch Command.
///////////////////////////////////////////////////////////////////////////////////
boolean wasRocketLaunched = false;

///////////////////////////////////////////////////////////////////////////////////
/// @brief Instance of the Zigbee Class.  Only one instance created.  It is used
/// to control/perform handshakes and other communications with the other Zigbee
/// devices in the Zigbee wireless network.
///////////////////////////////////////////////////////////////////////////////////
XBee xbee = XBee();

///////////////////////////////////////////////////////////////////////////////////
/// @brief Instance of the LaunchDataPacket Class.  
/// Create an instance of the LaunchDataPacket Class.  Only one instance created.
/// This class decyphers all data being passed between the Master Panel, the
/// Remote Panel, and the Launch Pad.  The single instance of the Zigbee class
/// is passed to the LaunchDataPacket so the Master Panel can begin
/// communicate with the Remote Panel and the Launch Pad.
///////////////////////////////////////////////////////////////////////////////////
LaunchDataPacket data = LaunchDataPacket(xbee);




///////////////////////////////////////////////////////////////////////////////////
/// Supplies power to the Safety LED.
///
/// @param state -boolean- TRUE = turn on, FALSE = turn off.
///////////////////////////////////////////////////////////////////////////////////
void setSafetyLedState(boolean state){
  if (state) {
    bitSet(_SafetyLedPort_, _SafetyLedBit_);
    return;
  }
  bitClear(_SafetyLedPort_, _SafetyLedBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Supplies power to the Error LED.
///
/// @param state -boolean- TRUE = turn on, FALSE = turn off.
///////////////////////////////////////////////////////////////////////////////////
void setErrorLedState(boolean state){
  if (state) {
    bitSet(_ErrorLedPort_, _ErrorLedBit_);
    return;
  }
  bitClear(_ErrorLedPort_, _ErrorLedBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Checks if there is data waiting in the Zigbee.  If data is present, it reads 
/// the data and stores it in class variables and updates the LCD display acording 
/// to the data content.
///////////////////////////////////////////////////////////////////////////////////
void readZigbee() {
  data.readDataFromXbee();
}

///////////////////////////////////////////////////////////////////////////////////
/// Reads the current pressures, updates the data appropriate for the Launch Pad
/// and sends a data packet to the Master Controller via Zigbee.
/// All variables are stored in the class instance.
///////////////////////////////////////////////////////////////////////////////////
void writeZigbeeMaster() {
  data.sendDataToMaster();
}

///////////////////////////////////////////////////////////////////////////////////
/// Toggles the on/off state of the Error LED bit.
///////////////////////////////////////////////////////////////////////////////////
void blinkErrorLed() {
  if (pulse) {
    bitSet(_ErrorLedPort_, _ErrorLedBit_);
    return;
  }    
  bitClear(_ErrorLedPort_, _ErrorLedBit_);
}

///////////////////////////////////////////////////////////////////////////////////
/// Toggles the on/off state of the Error LED bit.
///////////////////////////////////////////////////////////////////////////////////
void blinkSafetyLed() {
  if (pulse) {
    bitSet(_SafetyLedPort_, _SafetyLedBit_);
    return;
  }    
  bitClear(_SafetyLedPort_, _SafetyLedBit_);
}


///////////////////////////////////////////////////////////////////////////////////
/// Reads the pressure as an uncalibrated value from the ADC.  The ADC value is
/// stored for transmission via Zigbee.
///////////////////////////////////////////////////////////////////////////////////
void readPressure() {
  data.setPressure1(analogRead(_Pressure1Analog_));
  data.setPressure2(analogRead(_Pressure2Analog_));
}

///////////////////////////////////////////////////////////////////////////////////
/// Launch the rocket(s).
///////////////////////////////////////////////////////////////////////////////////
void launchRocket() {
  if (wasRocketLaunched) {
    return;
  }
  wasRocketLaunched = true;
  bitClear(_CompressorPort_, _CompressorBit_);
  if (data.isPad1Selected()) {
    bitSet(_Pad1LaunchPort_, _Pad1LaunchBit_);
  }
  if (data.isPad2Selected()) {
    bitSet(_Pad2LaunchPort_, _Pad2LaunchBit_);
  }
  delay(100);
  bitClear(_Pad1LaunchPort_, _Pad1LaunchBit_);
  bitClear(_Pad2LaunchPort_, _Pad2LaunchBit_);

  data.setPressure1(analogRead(_Pressure1Analog_));
  data.setPressure2(analogRead(_Pressure2Analog_));
}

///////////////////////////////////////////////////////////////////////////////////
/// Reads the pressure of the chambers and writes the data do the LaunchDataPacket
/// instance, checks for communication errors, reads Zigbee commands into the
/// LaunchDataPacket instance, controls the air compressor status, launches
/// rockets, and sends the LaunchDataPacket instance over Zigbee.
///////////////////////////////////////////////////////////////////////////////////
void updateData() {
  // Read the chamber pressures and update the LaunchDataPacket instance.
  readPressure();

  // In order to blink an LED, we need to track when to turn the LED on and off.
  // pulseTime is the time since we last changed the state of the LEDs from on
  // to off or off to on.  pulse is the current state of the LEDs (on or off).
  long pulseTime = millis() - timeSinceLastPulse;
  if (pulseTime > _pulse_halfwidth_) {
    timeSinceLastPulse = millis();
    pulse = pulse ^ true;
    if (pulse) {
      // on the rising edge of pulse, send a data packet over Zigbee
      writeZigbeeMaster();
    }
  }
  // Check for data waiting on the Zigbee
  readZigbee();

  // Error if time since last Zigbee receive is more than 1 second.
  if (data.didMasterTimeout()) { // Communication Error
    data.compressorOff();
    data.masterArmIsOff();
    data.masterLaunchClear();
    blinkErrorLed();        // report error via LED
  } else { // timeout has not occured (yet).
    if (data.isMasterLaunchSet()) {
      launchRocket();
    } else {
      wasRocketLaunched = false;
      if (data.isCompressorOn()) {
        bitSet(_CompressorPort_, _CompressorBit_);
      } else {
        bitClear(_CompressorPort_, _CompressorBit_);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////
/// Arduino 'setup' function.  The 'setup' function is excecuted only once each
/// time the Atmel microcontroller is powered up and it executes after the arduino
/// boot loader finishes executing. \n
/// It will initialize the digital I/O pins, the serial port, and the Zigbee.
///////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Setup the input and output pins
  pinMode(_SafetyLedBitMode_, OUTPUT);
  pinMode(_ErrorLedBitMode_, OUTPUT);
  pinMode(_Pad1LaunchBitMode_, OUTPUT);
  pinMode(_Pad2LaunchBitMode_, OUTPUT);

  pinMode(_CompressorBitMode_, OUTPUT);
  pinMode(_Pressure1Analog_, INPUT);
  pinMode(_Pressure2Analog_, INPUT);

  // Initialize the pulse variable which keeps track if a flashing LED is on/off.
  pulse = false;

  // Initialize the serial port.
  Serial.begin(9600);

  // Initialize the Zigbee and define the Zigbee communication port as Serial
  xbee.setSerial(Serial);
}

///////////////////////////////////////////////////////////////////////////////////
/// Arduino 'loop' function.  The 'loop' function is excecuted over and over after
/// the 'setup' function finishes executing. \n
/// It will call the readPanel function in an infinite loop.
///////////////////////////////////////////////////////////////////////////////////
void loop() {
  updateData();  
}

