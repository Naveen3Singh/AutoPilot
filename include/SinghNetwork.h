#ifndef AUTOPILOT_SINGH_NETWORK_H
#define AUTOPILOT_SINGH_NETWORK_H

#define AP_SSID "Singh Robotics Network"
#define STATION_SSID "Singh Network"
#define PASSWORD "bringMeThanos"
#define HOST "roomba"

class SinghNetwork {
public:
    SinghNetwork();
    void connect(bool autoConnect);
    void update();

    bool isConnected();
};

#endif //AUTOPILOT_SINGH_NETWORK_H