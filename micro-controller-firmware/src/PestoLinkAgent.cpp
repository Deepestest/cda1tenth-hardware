#ifndef PESTOLINKAGENT_INLINE_H
#define PESTOLINKAGENT_INLINE_H

#include <NimBLEDevice.h>
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// BLE UUIDs used by the implementation (use your real UUIDs if different)
#define SERVICE_UUID "27df26c5-83f4-4964-bae0-d7b7cb0a1f54"
#define TX_UUID "266d9d74-3e10-4fcd-88d2-cb63b5324d0c"
#define RX_UUID "452af57e-ad27-422c-88ae-76805ea641a9"

class PestoLinkAgent
{
public:
    bool connected;
    unsigned long lastTelemetryMs;
    uint8_t byteList[20];
    NimBLEServer *server;
    NimBLECharacteristic *txChar;
    NimBLECharacteristic *rxChar;

    PestoLinkAgent(const char *name);
    bool is_connected() const;
    void send(uint8_t *data, size_t len);
    float get_axis(uint8_t axis);
    bool get_button(uint8_t button);
    void telemetryPrint(const String &telemetry, const String &hex_code);
    void telemetryPrintBatteryVoltage(float voltage);
};

#endif

class ServerCallbacks : public NimBLEServerCallbacks
{
    PestoLinkAgent *parent;

public:
    ServerCallbacks(PestoLinkAgent *p) : parent(p) {}
    void onConnect(NimBLEServer *) { parent->connected = true; }
    void onDisconnect(NimBLEServer *)
    {
        parent->connected = false;
        NimBLEDevice::startAdvertising();
    }
};

class RXCallback : public NimBLECharacteristicCallbacks
{
    PestoLinkAgent *parent;

public:
    RXCallback(PestoLinkAgent *p) : parent(p) {}
    void onWrite(NimBLECharacteristic *c)
    {
        std::string value = c->getValue();
        if (value.size() == 0)
            return;

        const uint8_t *data = (const uint8_t *)value.data();
        if (data[0] == 0x01)
            memcpy(parent->byteList, data, min(value.size(), sizeof(parent->byteList)));
        else
        {
            // reset to neutral
            uint8_t neutral[20] = {1, 127, 127, 127, 127, 0};
            memcpy(parent->byteList, neutral, sizeof(neutral));
        }
    }
};

PestoLinkAgent::PestoLinkAgent(const char *name)
    : connected(false), lastTelemetryMs(0)
{
    memset(byteList, 127, sizeof(byteList));
    byteList[0] = 1;

    String shortName = String(name).substring(0, 8);
    NimBLEDevice::init(shortName.c_str());
    // ESP_PWR_LVL_P7 may not be defined in this build; use numeric TX power level (P7)
    NimBLEDevice::setPower(7);

    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks(this));

    NimBLEService *service = server->createService(SERVICE_UUID);

    txChar = service->createCharacteristic(TX_UUID,
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    rxChar = service->createCharacteristic(RX_UUID,
                                           NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxChar->setCallbacks(new RXCallback(this));

    service->start();
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    Serial.begin(115200);
    delay(10);
    Serial.println("PestoLinkAgent: configuring advertising...");
    adv->addServiceUUID(SERVICE_UUID);
    adv->start();
    Serial.println("PestoLinkAgent: advertising started");
}

bool PestoLinkAgent::is_connected() const
{
    return connected;
}

void PestoLinkAgent::send(uint8_t *data, size_t len)
{
    if (connected)
    {
        txChar->setValue(data, len);
        txChar->notify();
    }
}

float PestoLinkAgent::get_axis(uint8_t axis)
{
    if (axis > 3)
        return 0;
    uint8_t raw = byteList[1 + axis];
    if (raw == 127)
        return 0.0f;
    return (raw / 127.5f) - 1.0f;
}

bool PestoLinkAgent::get_button(uint8_t button)
{
    uint16_t raw_buttons = (byteList[6] << 8) | byteList[5];
    return (raw_buttons >> button) & 0x01;
}

void PestoLinkAgent::telemetryPrint(const String &telemetry, const String &hex_code)
{
    if (millis() - lastTelemetryMs < 500)
        return;

    uint8_t msg[11] = {0};
    for (int i = 0; i < 8 && i < telemetry.length(); ++i)
        msg[i] = telemetry[i];

    uint32_t color = strtoul(hex_code.c_str(), nullptr, 16);
    msg[8] = (color >> 16) & 0xFF;
    msg[9] = (color >> 8) & 0xFF;
    msg[10] = color & 0xFF;

    send(msg, sizeof(msg));
    lastTelemetryMs = millis();
}

void PestoLinkAgent::telemetryPrintBatteryVoltage(float voltage)
{
    String voltageStr = String(voltage, 2) + "V";
    if (voltage >= 7.6)
        telemetryPrint(voltageStr, "00FF00");
    else if (voltage >= 7.0)
        telemetryPrint(voltageStr, "FFFF00");
    else
        telemetryPrint(voltageStr, "FF0000");
}
