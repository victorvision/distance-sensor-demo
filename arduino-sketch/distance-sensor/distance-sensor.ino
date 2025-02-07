#include "LumenProtocol.h"

extern "C" void lumen_write_bytes(uint8_t* data, uint32_t length) {
  Serial.write(data, length);
}

extern "C" uint16_t lumen_get_byte() {
  if (Serial.available()) {
    return Serial.read();
  }
  return DATA_NULL;
}

// Definition of addresses and packets
const uint16_t DISTANCE_ADDRESS = 500;
const uint16_t ALARM_DIST_MAX_ADDRESS = 520;
const uint16_t ALARM_DIST_MIN_ADDRESS = 530;
const uint16_t CHANGE_BACKGROUND_COLOR_ADDRESS = 124;

// Packets for the measured distance and alarms
lumen_packet_t Distance = {DISTANCE_ADDRESS, kS32}; 
lumen_packet_t AlarmDistMax = {ALARM_DIST_MAX_ADDRESS, kS32};
lumen_packet_t AlarmDistMin = {ALARM_DIST_MIN_ADDRESS, kS32};
lumen_packet_t ChangeBackgroundColorPacket = {CHANGE_BACKGROUND_COLOR_ADDRESS, kBool};

// Distance sensor variables
const int TRIG_PIN = 3;
const int ECHO_PIN = 2;
float distMaxCm = 10.0;         // Maximum distance limit in cm
float distMinCm = 0.0;          // Minimum distance limit in cm
float currentDistance = 0.0;    // Variable for the measured distance

// General variables
const uint16_t PERIOD = 50;   // Sensor update time
unsigned long timeNow = 0;
const int GREEN_LED_PIN = 9;
const int RED_LED_PIN = 10;

// Initial setup
void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT); 
    pinMode(ECHO_PIN, INPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    delay(3000);  // Initialization delay
}

// Main program (loop)
void loop() {
    // Updates the distance sensor reading periodically
    if (millis() > timeNow + PERIOD) {
        timeNow = millis();

        // Request the distance in centimeters
        currentDistance = RequestDist();

        // Check if distance limits are exceeded and update the background
        if (currentDistance > distMaxCm || currentDistance < distMinCm) {
            activateRedBackground();  // Out of limits
        } else {
            activateGreenBackground();  // Within limits
        }

        // Send the measured distance to the display
        sendDistance(currentDistance);
    }

    // Process received packets
    processReceivedPackets();
}

// Processes the received packets and updates as needed
void processReceivedPackets() {
    while (lumen_available() > 0) {
        lumen_packet_t* currentPacket = lumen_get_first_packet();

        // Update the maximum distance value
        if (currentPacket->address == ALARM_DIST_MAX_ADDRESS) {
            updateAlarmDistanceMax(currentPacket->data._s32);
        }

        // Update the minimum distance value
        if (currentPacket->address == ALARM_DIST_MIN_ADDRESS) {
            updateAlarmDistanceMin(currentPacket->data._s32);
        }
    }
}

// Updates the maximum distance value and sends it
void updateAlarmDistanceMax(float valueAlarm) {
    distMaxCm = valueAlarm;  // Update directly in cm

    // Update and send the maximum distance packet
    sendAlarmDistanceMax();
}

// Updates the minimum distance value and sends it
void updateAlarmDistanceMin(float valueAlarm) {
    distMinCm = valueAlarm;  // Update directly in cm

    // Update and send the minimum distance packet
    sendAlarmDistanceMin();
}

// Sends the updated maximum distance
void sendAlarmDistanceMax() {
    AlarmDistMax.data._s32 = distMaxCm;
    lumen_write_packet(&AlarmDistMax);  // Send the maximum distance packet
}

// Sends the updated minimum distance
void sendAlarmDistanceMin() {
    AlarmDistMin.data._s32 = distMinCm;
    lumen_write_packet(&AlarmDistMin);  // Send the minimum distance packet
}

// Sends the measured distance
void sendDistance(float value) {
    Distance.data._s32 = value;  // Write the distance into the packet
    lumen_write_packet(&Distance);  // Send the distance packet
}

// Function to read the distance from the ultrasonic sensor (in cm)
float RequestDist() {
    long duration;  // ECHO time
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);  // Time the Echo pin was HIGH

    return duration / 29.0 / 2.0;  // Convert to centimeters
}

// Activates the green background (within limits)
void activateGreenBackground() {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    ChangeBackgroundColorPacket.data._bool = false;
    lumen_write_packet(&ChangeBackgroundColorPacket);
}

// Activates the red background (out of bounds)
void activateRedBackground() {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    ChangeBackgroundColorPacket.data._bool = true;
    lumen_write_packet(&ChangeBackgroundColorPacket);
}
