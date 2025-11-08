#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

class PestoLinkAgent
{
public:
    PestoLinkAgent(const char *name);

    bool is_connected() const;
    float get_axis(uint8_t axis);
    bool get_button(uint8_t button);
    void telemetryPrint(const String &telemetry, const String &hex_code);
    void telemetryPrintBatteryVoltage(float voltage);

private:
    static constexpr const char *SERVICE_UUID = "27df26c5-83f4-4964-bae0-d7b7cb0a1f54";
    static constexpr const char *TX_UUID = "266d9d74-3e10-4fcd-88d2-cb63b5324d0c";
    static constexpr const char *RX_UUID = "452af57e-ad27-422c-88ae-76805ea641a9";

    NimBLEServer *server;
    NimBLECharacteristic *txChar;
    NimBLECharacteristic *rxChar;
    bool connected;
    uint8_t byteList[20];
    unsigned long lastTelemetryMs;

    void send(uint8_t *data, size_t len);
};
