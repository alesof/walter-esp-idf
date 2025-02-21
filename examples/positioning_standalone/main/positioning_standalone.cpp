#include <esp_mac.h>
#include <esp_log.h>
#include <inttypes.h>
#include <cstring>
#include <esp_system.h>
#include <driver/uart.h>
#include <driver/temp_sensor.h>
#include "WalterModem.h"

#define MAX_GNSS_CONFIDENCE 100.0       // Confidence threshold for a valid fix
#define PACKET_SIZE 18                  // Buffer size
#define GNSS_RETRY_COUNT 5              // Maximum GNSS fix attempts
#define GNSS_TIMEOUT 300                // Timeout loops (each 500ms)

WalterModem modem;                      // Modem instance
volatile bool fixRcvd = false;          // Flag for fix received
WalterModemGNSSFix posFix = {};         // Last GNSS fix

/**
 * @brief Callback function when GNSS fix is received.
 */
void fixHandler(const WalterModemGNSSFix *fix, void *args) {
    memcpy(&posFix, fix, sizeof(WalterModemGNSSFix));
    fixRcvd = true;
}

extern "C" void app_main(void) {
    ESP_LOGI("positioning", "Walter Positioning GNSS Only v1.0");
    WalterModemRsp rsp = {};

    /* Get MAC address */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI("positioning", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* Initialize Modem */
    if (WalterModem::begin(UART_NUM_1)) {
        ESP_LOGI("positioning", "Modem initialized successfully");
    } else {
        ESP_LOGE("positioning", "Failed to initialize modem");
        return;
    }

    /* Configure GNSS */
    if (!modem.configGNSS()) {
        ESP_LOGE("positioning", "Failed to configure GNSS");
        return;
    }

    if (!modem.setClock("\"24/02/21,12:26:00+04\"")) {
        ESP_LOGE("positioning", "Failed to set clock");
        return;
    }

    if(!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
        ESP_LOGI("positioning", "Could not set operational state to FULL");
        return;
    }

    if(!modem.setApproxPos()) {
       ESP_LOGI("positioning", "Could not set approximate position");
       return;
    }

    /* Set fix handler */
    modem.setGNSSfixHandler(fixHandler);

    /* GNSS Fix Loop */
    for(;;) {
        for (int i = 0; i < GNSS_RETRY_COUNT; ++i) {
            fixRcvd = false;

            ESP_LOGI("positioning", "Starting GNSS fix attempt %d", i + 1);

            if (!modem.performGNSSAction()) {
                ESP_LOGE("positioning", "Failed to start GNSS fix attempt");
                return;
            }

            ESP_LOGI("positioning", "Requesting GNSS fix...");

            /* Wait for GNSS fix or timeout */
            int j = 0;
            while (!fixRcvd) {
                ESP_LOGI("positioning", "Waiting for GNSS fix... %d/%d", j, GNSS_TIMEOUT);
                if (j >= GNSS_TIMEOUT) {
                    ESP_LOGE("positioning", "GNSS fix timed out");
                    return;
                }
                j++;
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            /* Validate GNSS fix confidence */
            if (posFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE) {
                break;  // Acceptable fix received
            }
        }

        /* Count strong satellite signals */
        uint8_t strongSatellites = 0;
        for (int i = 0; i < posFix.satCount; ++i) {
            if (posFix.sats[i].signalStrength >= 30) {
                strongSatellites++;
            }
        }

        /* Print GNSS Fix Information */
        ESP_LOGI("positioning", "GNSS Fix Received:"
                 " Confidence: %.02f"
                 " Latitude: %.06f"
                 " Longitude: %.06f"
                 " Satellites: %d"
                 " Strong Signals: %d",
                 posFix.estimatedConfidence,
                 posFix.latitude,
                 posFix.longitude,
                 posFix.satCount,
                 strongSatellites);

        /* Validate GNSS fix */
        float lat = posFix.latitude;
        float lon = posFix.longitude;

        if (posFix.estimatedConfidence > MAX_GNSS_CONFIDENCE) {
            lat = 0.0;
            lon = 0.0;
            ESP_LOGE("positioning", "Invalid GNSS fix (Low confidence)");
        }

        /* Sleep before the next fix attempt */
        ESP_LOGI("positioning", "Sleeping for 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

}
