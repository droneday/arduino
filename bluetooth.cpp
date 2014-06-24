#include <Arduino.h>
#include <SoftwareSerial.h>
#include "bluetooth.h"

// Friendly name to use for Bluetooth ID:
#define BLUETOOTH_DRONE_NAME "DroneDayDrone"

// Pin numbers for I/O pins connected to the Bluetooth board.
// TODO: Configure these correctly.
int bluetoothKey = -1;
int bluetoothTx = 5;
int bluetoothRx = 6;

// Software UART for communicating with the Bluetooth board.
SoftwareSerial bluetooth(bluetoothRx, bluetoothTx);

// Input buffer for reading from the Bluetooth board.
String bluetoothBuffer = "";

// Current movement state received over Bluetooth.
int cmdPitch, cmdRoll, cmdYaw, cmdThrottle;

// Setup function to initialize Bluetooth receiver.
void bluetoothSetup() {

  if (bluetoothKey >= 0) {
    // Bluetooth board starts in passthrough mode.
    pinMode(bluetoothKey, OUTPUT);
    digitalWrite(bluetoothKey, LOW);
  }

  // Set up serial. 9600 is default baud.
  // TODO: See if this is fast enough, and potentially raise
  // to a higher baud rate through AT commands.
  bluetooth.begin(9600);

  // Default drone name,
  bluetoothATCommand("AT+NAME=" BLUETOOTH_DRONE_NAME);
}

// Read a line of text from the Bluetooth device, stripping
// the newline character.
String bluetoothReadLine() {
  String result = "";
  for (;;) {
    if (bluetooth.available()) {
      char c = bluetooth.read();
      if (c == '\r' || c == '\n') {
        break;
      } else {
        result += c;
      }
    }
  }
  return result;
}

// Execute an AT command on the Bluetooth board.
String bluetoothATCommand(String command) {

  if (bluetoothKey < 0) {
    return "Not supported.";
  }

  // Switch into AT command mode by toggling the KEY pin high.
  digitalWrite(bluetoothKey, HIGH);
  delay(250);  // TODO: Check if delay is needed.

  // Send command to board and read the response.
  bluetooth.println(command);
  String response = bluetoothReadLine();

  // Toggle back into normal passthrough mode.
  digitalWrite(bluetoothKey, LOW);
  delay(250);  // TODO: Check if delay is needed.

  return response;
}

// Dump current configured control state over Bluetooth channel.
void bluetoothDumpState() {
  bluetooth.print("Pitch: ");
  bluetooth.print(cmdPitch);
  bluetooth.print(" Roll: ");
  bluetooth.print(cmdRoll);
  bluetooth.print(" Yaw: ");
  bluetooth.print(cmdYaw);
  bluetooth.print(" Throttle: ");
  bluetooth.print(cmdThrottle);
  bluetooth.println();
  // TODO: Dump other useful info here.
}

// Parse and execute a complete command read from the Bluetooth
// module.
void bluetoothCommand() {
  if (bluetoothBuffer.length() < 1) {
    return;
  }
  switch (toupper(bluetoothBuffer[0])) {

    // Normal flight commands: Adjust position and turn on
    // axis.
    case 'P':
      cmdPitch = bluetoothBuffer.substring(1).toInt();
      break;
    case 'R':
      cmdRoll = bluetoothBuffer.substring(1).toInt();
      break;
    case 'Y':
      cmdYaw = bluetoothBuffer.substring(1).toInt();
      break;
    case 'T':
      cmdThrottle = bluetoothBuffer.substring(1).toInt();
      break;

    // For debugging.
    case 'D':
      bluetoothDumpState();
      break;

    // Send AT command to Bluetooth board and read back the
    // response. Useful if we want to do something like change
    // the Bluetooth display name.
    case '#':
      bluetooth.println(bluetoothATCommand(bluetoothBuffer.substring(1)));
      break;

    default:
      bluetooth.println("Unknown command type!");
      break;
  }
}

void bluetoothPoll() {
  // Check for new characters received over Bluetooth.
  // When a complete line has been read, process it as a command.
  while (bluetooth.available()) {
    char c = bluetooth.read();
    if (c == '\r' || c == '\n') {
      bluetoothCommand();
      bluetoothBuffer = "";
    } else {
      bluetoothBuffer += c;
    }
  }
}
