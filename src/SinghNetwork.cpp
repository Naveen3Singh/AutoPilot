#include "ESP8266WiFi.h"
#include <SinghNetwork.h>
#include "ESP8266HTTPUpdateServer.h"
#include "ESP8266WebServer.h"
#include "ESP8266mDNS.h"

ESP8266WebServer webServer(80);
ESP8266HTTPUpdateServer updateServer;

SinghNetwork::SinghNetwork() {
    WiFi.mode(WIFI_AP_STA);
}

void SinghNetwork::connect(bool autoConnect) {
    // Setup IoT Network
    WiFi.softAP(AP_SSID, PASSWORD);

    // Connect to Home
    WiFi.begin(STATION_SSID, PASSWORD);
    WiFi.setAutoReconnect(autoConnect);
    WiFi.setAutoConnect(autoConnect);
    MDNS.begin(HOST);
    MDNS.addService("http", "tcp", 80);

    updateServer.setup(&webServer);
    webServer.begin();
    delay(1000);
}

void SinghNetwork::update() {
    webServer.handleClient();
    MDNS.update();
}

bool SinghNetwork::isConnected() {
    return WiFi.isConnected() || WiFi.status() == WL_CONNECTED;
}
