#include <SPI.h>

#include "payload.h"

#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <TFT_eSPI.h>

/* --- HARDWARE DEFINITIONS --- */
#define NRF_CE   27
#define NRF_CSN  25
#define NRF_SCK  14
#define NRF_MISO 12
#define NRF_MOSI 13

#define JOY_L_X  32 // Yaw
#define JOY_L_Y  33 // Throttle
#define JOY_R_X  34 // Roll
#define JOY_R_Y  35 // Pitch

/* --- SYSTEM INSTANCES --- */
TFT_eSPI tft = TFT_eSPI(); 
SPIClass *hspi = NULL;
RF24 radio(NRF_CE, NRF_CSN);

/* --- SHARED MEMORY & MULTITHREADING --- */
rc_payload_t global_payload;
SemaphoreHandle_t payload_mutex;
const byte address[6] = "PASIL"; // 5-byte pipe address

/* --- FREE_RTOS TASK PROTOTYPES --- */
void TaskKinematics(void *pvParameters);
void TaskCockpit(void *pvParameters);


void setup() {
    Serial.begin(115200);
    
    /* 1. Initialize Thread Safety */
    payload_mutex = xSemaphoreCreateMutex();
    global_payload.magic = 0x5A;
    global_payload.state = 0;

    /* 2. Arm the Visuals (Core 0 / VSPI) */
    tft.init();
    tft.setRotation(1); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("VANGUARD GCS BOOTING...", 10, 10, 4);

    /* 3. Arm the RF Link (Core 1 / HSPI) */
    hspi = new SPIClass(HSPI);
    hspi->begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
    
    if (!radio.begin(hspi)) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("FATAL: NRF24 OFFLINE", 10, 40, 4);
        while (1); // Halt
    }
    
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX); // Push LNA to maximum range
    radio.setDataRate(RF24_2MBPS); // 2Mbps for lowest latency
    radio.stopListening();         // Set as Transmitter

    /* 4. Spawn FreeRTOS Tasks onto specific silicon cores */
    xTaskCreatePinnedToCore(TaskKinematics, "Task_RF", 4096, NULL, 2, NULL, 1); // Core 1 (High Priority)
    xTaskCreatePinnedToCore(TaskCockpit, "Task_UI", 4096, NULL, 1, NULL, 0);    // Core 0 (Low Priority)
}

void loop() {
    /* FreeRTOS handles the execution. The Arduino loop() is deprecated in this architecture. */
    vTaskDelete(NULL); 
}

/* =========================================================================
 * CORE 1: KINEMATICS & RF TELEMETRY (50Hz Deterministic Loop)
 * ========================================================================= */
void TaskKinematics(void *pvParameters) {
    rc_payload_t local_tx;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz

    for (;;) {
        /* 1. Sample the Hall Effect Sensors */
        int raw_yaw   = analogRead(JOY_L_X);
        int raw_thr   = analogRead(JOY_L_Y);
        int raw_roll  = analogRead(JOY_R_X);
        int raw_pitch = analogRead(JOY_R_Y);

        /* 2. Normalize and Apply Deadbands (Assume 12-bit ADC: 0-4095, Center ~2048) */
        // NOTE: You must calibrate these mapping values based on your physical sticks!
        local_tx.throttle = map(raw_thr, 0, 4095, 0, 1000);
        local_tx.yaw      = map(raw_yaw, 0, 4095, -1000, 1000);
        local_tx.pitch    = map(raw_pitch, 0, 4095, -1000, 1000);
        local_tx.roll     = map(raw_roll, 0, 4095, -1000, 1000);
        
        local_tx.magic = 0x5A;
        local_tx.state = 1; // Armed
        local_tx.checksum = local_tx.throttle + local_tx.pitch + local_tx.roll + local_tx.yaw;

        /* 3. Blast the payload into the air */
        radio.write(&local_tx, sizeof(rc_payload_t));

        /* 4. Update the shared memory for the display core */
        if (xSemaphoreTake(payload_mutex, (TickType_t)5) == pdTRUE) {
            global_payload = local_tx;
            xSemaphoreGive(payload_mutex);
        }

        /* 5. Sleep exactly until the next 20ms boundary */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =========================================================================
 * CORE 0: THE GLASS COCKPIT (10Hz UI Loop)
 * ========================================================================= */
void TaskCockpit(void *pvParameters) {
    rc_payload_t display_data;
    char buffer[32];

    tft.fillScreen(TFT_BLACK);

    for (;;) {
        /* 1. Safely grab the latest telemetry */
        if (xSemaphoreTake(payload_mutex, (TickType_t)10) == pdTRUE) {
            display_data = global_payload;
            xSemaphoreGive(payload_mutex);
        }

        /* 2. Render UI (Only refreshing numbers to prevent flicker) */
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        
        sprintf(buffer, "THR: %4d", display_data.throttle);
        tft.drawString(buffer, 10, 10, 4);
        
        sprintf(buffer, "PIT: %4d", display_data.pitch);
        tft.drawString(buffer, 10, 40, 4);
        
        sprintf(buffer, "ROL: %4d", display_data.roll);
        tft.drawString(buffer, 10, 70, 4);

        sprintf(buffer, "YAW: %4d", display_data.yaw);
        tft.drawString(buffer, 10, 100, 4);

        /* 3. 10Hz Refresh Target */
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}