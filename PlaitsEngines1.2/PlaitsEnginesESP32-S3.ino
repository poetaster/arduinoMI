/* 
  This program cycles through MI Plaits 1.2 engines on ESP32-S3 with IS2 DAC.

  Copyright (c) 2025 Vadims Maksimovs 
  https://github.com/ledlaux
  MIT licence
  --------------------------------------------------------------------------------
  Arduino Port 
  --------------------------------------------------------------------------------
  This code is part of the porting project of Mutable Instruments eurorack modules to Arduino by Mark Washeim
  Main repository: https://github.com/poetaster/arduinoMI
  --------------------------------------------------------------------------------
  Original Mutable Instruments Code
  --------------------------------------------------------------------------------
  Copyright (c) 2020 Emilie O. Gillet 
  stmlib, Plaits, Clouds libraries
  MIT licence
  --------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include "driver/i2s.h"
#include <STMLIB.h>
#include <PLAITS.h>

// ================= CONFIG =================
#define SAMPLE_RATE        48000
#define ENGINE_SWITCH_MS   3000
#define WORKSPACE_SIZE     49152   
#define AUDIO_STACK_SIZE   16384   
#define PLAITS_ENGINES      24      

constexpr int AUDIO_BLOCK = plaits::kBlockSize;

#define I2S_BCLK_PIN   5
#define I2S_LRCLK_PIN  7
#define I2S_DATA_PIN   6

// ================= GLOBAL STATE =================
volatile int engine_idx = 1;     
float master_volume = 0.3f;      
float current_pitch = 48.0f;     

TaskHandle_t audioTaskHandle = NULL;

plaits::Voice voice;
plaits::Patch patch;
plaits::Modulations modulations;
plaits::Voice::Frame out_buffer[AUDIO_BLOCK];
uint8_t* plaits_workspace = nullptr; 

static int16_t stereo[AUDIO_BLOCK * 2];

// ================= AUDIO ENGINE =================
void renderAudio() {
    bool triggerActive = (ulTaskNotifyTake(pdTRUE, 0) > 0);

    patch.engine = engine_idx - 1; 
    patch.note = current_pitch;
    
    patch.harmonics = 0.4f;
    patch.timbre = 0.4f;
    patch.morph = 0.4f;

    if (engine_idx == 23) { // String Machine
        patch.harmonics = 0.5f; 
        patch.timbre = 0.2f;    
        patch.morph = 0.7f;     
        patch.decay = 0.9f;     
        modulations.trigger = triggerActive ? 1.0f : 0.0f;
        modulations.trigger_patched = true;
    } 
    else if (engine_idx == 24 || (engine_idx >= 14 && engine_idx <= 16)) { // Chiptune and Drums
        patch.decay = (engine_idx >= 14 && engine_idx <= 16) ? 0.6f : 0.7f;
        modulations.trigger = triggerActive ? 1.0f : 0.0f;
        modulations.trigger_patched = true;
    } 
    else { // Melodic Engines
        patch.decay = 0.2f;
        modulations.trigger = 1.0f;
        modulations.trigger_patched = false;
    }
    
    modulations.level = 1.0f;
    modulations.level_patched = true;

    voice.Render(patch, modulations, out_buffer, AUDIO_BLOCK);

    size_t bytes_written;
    for (int i = 0; i < AUDIO_BLOCK; i++) {
        float s_main = (out_buffer[i].out / 32768.0f) * master_volume;
        float s_aux  = (out_buffer[i].aux / 32768.0f) * master_volume;
        
        if (s_main > 1.0f) s_main = 1.0f;
        if (s_main < -1.0f) s_main = -1.0f;
        if (s_aux > 1.0f) s_aux = 1.0f;
        if (s_aux < -1.0f) s_aux = -1.0f;

        stereo[i * 2]     = (int16_t)(s_main * 32767.0f);
        stereo[i * 2 + 1] = (int16_t)(s_aux * 32767.0f);  
    }
    i2s_write(I2S_NUM_0, stereo, sizeof(stereo), &bytes_written, portMAX_DELAY);
}

void audioTask(void* pvParameters) {
    while(true) {
        renderAudio();
    }
}

void demoTask(void* pvParameters) {
    uint32_t last_engine_switch = millis(); 
    uint32_t last_trigger = 0;
    
    while(true) {
        uint32_t now = millis();

        if (now - last_engine_switch >= ENGINE_SWITCH_MS) {
            last_engine_switch = now;
            engine_idx++;
            if (engine_idx > PLAITS_ENGINES) {
                engine_idx = 1;
            }
            Serial.printf("Engine: %d\n", engine_idx);
        }

        if ((engine_idx >= 14 && engine_idx <= 16) || (engine_idx == 23) || (engine_idx == 24)) {
            if (now - last_trigger >= 500) { 
                last_trigger = now;
                if (audioTaskHandle != NULL) {
                    xTaskNotifyGive(audioTaskHandle);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000); 

    plaits_workspace = (uint8_t*) heap_caps_malloc(WORKSPACE_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!plaits_workspace) plaits_workspace = (uint8_t*) malloc(WORKSPACE_SIZE);

    stmlib::BufferAllocator alloc(plaits_workspace, WORKSPACE_SIZE);
    voice.Init(&alloc);

    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256,
        .use_apll = false,
        .tx_desc_auto_clear = true
    };

    i2s_pin_config_t pins = {
        .bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_LRCLK_PIN,
        .data_out_num = I2S_DATA_PIN, .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pins);

    xTaskCreatePinnedToCore(audioTask, "audio_loop", AUDIO_STACK_SIZE, NULL, 24, &audioTaskHandle, 1);
    xTaskCreatePinnedToCore(demoTask, "demo_loop", 4096, NULL, 1, NULL, 0);

    Serial.printf("System Ready. Engine: %d\n", engine_idx);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
