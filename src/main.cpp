#include "Arduino.h"
#include "SinghNetwork.h"
#include "AutoPilot.h"
#include "ArduinoJson.h"
#include "WiFiUdp.h"
#include "Pilot.h"

SinghNetwork singhNetwork;
AutoPilot autoPilot;

unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];
char replyBuffer[UDP_TX_PACKET_MAX_SIZE + 1];

WiFiUDP Udp;

void setup() {
    // Setup logging
    Serial.begin(115200);
    delay(100);

    // Setup indicators
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_BUILTIN_AUX, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN_AUX, HIGH);

    // Networking
    singhNetwork.connect(true);
    Udp.begin(localPort);

    // Pilot
    delay(100);
    autoPilot.init();
    delay(1000);
}
/**
 * Send data about current state.
 */
void sendData(const IPAddress &remoteIP, uint16_t remotePort) {
    StaticJsonDocument<200> jsonDocument;
    jsonDocument[DIRECTION] = FORWARD;

    serializeJson(jsonDocument, replyBuffer);
    Udp.beginPacket(remoteIP, remotePort);
    Udp.write(replyBuffer);
    Udp.endPacket();
}

/**
 * Blink wifi indicator without delay.
 */
unsigned long previousMillis = 0;
unsigned short interval = 1000;
uint8_t ledState = LOW;

void blinkIfWifiIsConnected() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        if (ledState == HIGH) {
            ledState = LOW;
        } else {
            ledState = HIGH;
        }

        digitalWrite(LED_BUILTIN, ledState);
        digitalWrite(LED_BUILTIN_AUX, ledState);
    }
}

void loop() {
    autoPilot.update();

    // Begin remote pilot section
    singhNetwork.update();

    // If there's data available, read a packet
    int packetSize = Udp.parsePacket();
    if (packetSize) {
        Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
                      packetSize,
                      Udp.remoteIP().toString().c_str(), Udp.remotePort(),
                      Udp.destinationIP().toString().c_str(), Udp.localPort(),
                      ESP.getFreeHeap());

        // read the packet into packetBuffer
        int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
        packetBuffer[n] = 0;
        Serial.println("Contents:");
        Serial.println(packetBuffer);

        // Apply instructions
        autoPilot.applyPilotUpdates(packetBuffer, packetSize);

        // Send a reply, to the IP address and port that sent us the packet we received
        sendData(Udp.remoteIP(), Udp.remotePort());
    }
    blinkIfWifiIsConnected();
}