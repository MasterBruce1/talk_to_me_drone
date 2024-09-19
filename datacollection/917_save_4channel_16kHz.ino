#include <driver/adc.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "SD.h"
#define WAV_HEADER_SIZE 44
int file_counter = 0;  
const unsigned long bufferSize = 16000; //16000

#define SAMPLES_PER_SECOND 16000   // 16000
#define READ_LEN 1024              //
#define NUM_CHANNELS 4             // 
#define SAMPLE_BITS 16

//
static adc_channel_t channel[NUM_CHANNELS] = {
    ADC_CHANNEL_0,  //
    ADC_CHANNEL_1,  // 
    ADC_CHANNEL_2,  //
    ADC_CHANNEL_3   //
};
static TaskHandle_t s_task_handle;
static SemaphoreHandle_t dma_semaphore;
adc_continuous_handle_t handle = NULL;
unsigned long start_time = 0;  // 
unsigned long end_time = 0;    // 
// 
uint16_t buffer1[SAMPLES_PER_SECOND];
uint16_t buffer2[SAMPLES_PER_SECOND];
uint16_t buffer3[SAMPLES_PER_SECOND];
uint16_t buffer4[SAMPLES_PER_SECOND];
uint16_t sample_index1 = 0, sample_index2 = 0, sample_index3 = 0, sample_index4 = 0;  // 

#define PCM_BUFFER_SIZE SAMPLES_PER_SECOND  //
int16_t pcmBuffer1[PCM_BUFFER_SIZE];  // 
int16_t pcmBuffer2[PCM_BUFFER_SIZE];
int16_t pcmBuffer3[PCM_BUFFER_SIZE];
int16_t pcmBuffer4[PCM_BUFFER_SIZE];

//
static bool IRAM_ATTR adcComplete(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

static void adc_init(adc_channel_t *channel, uint8_t channel_num) {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 8192,   // 
        .conv_frame_size = READ_LEN,  //
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLES_PER_SECOND * NUM_CHANNELS,  
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,  
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, 
    };
    adc_digi_pattern_config_t adc_pattern[NUM_CHANNELS] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = channel[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
}


void stop_adc_and_clear_dma() {

    ESP_ERROR_CHECK(adc_continuous_stop(handle));

    uint8_t temp_buffer[READ_LEN];
    uint32_t ret_num = 0;

    while (adc_continuous_read(handle, temp_buffer, READ_LEN, &ret_num, 0) == ESP_OK) {
        // Do nothing, just clearing DMA buffer
    }
}

void restart_adc() {
    ESP_ERROR_CHECK(adc_continuous_start(handle));
}


void save_to_wav(String fileName, int16_t *dataBuffer1, int16_t *dataBuffer2, int16_t *dataBuffer3, int16_t *dataBuffer4) {
    uint32_t sample_size = bufferSize * sizeof(uint16_t) * NUM_CHANNELS;  
    uint32_t record_size = bufferSize * sizeof(uint16_t) * NUM_CHANNELS;
    uint8_t wav_header[WAV_HEADER_SIZE];


    generate_wav_header(wav_header, record_size, SAMPLES_PER_SECOND);

    File file = SD.open(fileName.c_str(), FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }


    file.write(wav_header, WAV_HEADER_SIZE);


    for (int i = 0; i < SAMPLES_PER_SECOND; i++) {
        file.write((uint8_t*) &dataBuffer1[i], sizeof(int16_t));  
        file.write((uint8_t*) &dataBuffer2[i], sizeof(int16_t));  
        file.write((uint8_t*) &dataBuffer3[i], sizeof(int16_t));  
        file.write((uint8_t*) &dataBuffer4[i], sizeof(int16_t));  
    }

    file.close();
    Serial.printf("Recording complete: %s\n", fileName.c_str());
}


void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate) {
    uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
    uint32_t byte_rate = SAMPLES_PER_SECOND * NUM_CHANNELS * SAMPLE_BITS / 8;
    const uint8_t set_wav_header[] = {
        'R', 'I', 'F', 'F', // ChunkID
        file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
        'W', 'A', 'V', 'E', // Format
        'f', 'm', 't', ' ', // Subchunk1ID
        0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
        0x01, 0x00, // AudioFormat (1 for PCM)
        0x04, 0x00, // NumChannels (4 channels)
        sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
        byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
        0x08, 0x00, // BlockAlign
        0x10, 0x00, // BitsPerSample (16 bits)
        'd', 'a', 't', 'a', // Subchunk2ID
        wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
    };
    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}


int16_t map_adc_to_pcm(uint16_t adc_value) {
    
    return int16_t((adc_value / 4095.0) * 65535 - 32768);
}

void setup() {
    Serial.begin(115200);

    if (!SD.begin(21)) {
        Serial.println("Failed to mount SD Card!");
        while (1);
    }


    s_task_handle = xTaskGetCurrentTaskHandle();
    dma_semaphore = xSemaphoreCreateBinary();


    adc_init(channel, NUM_CHANNELS);


    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adcComplete,  
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));

    
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    start_time = micros();  
}

void loop() {
    uint8_t result[READ_LEN] = {0};  
    uint32_t ret_num = 0;
    while (1) {
        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        while (adc_continuous_read(handle, result, READ_LEN, &ret_num, 0) == ESP_OK) {
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                
                switch (p->type2.channel) {
                    case ADC_CHANNEL_0:
                        buffer1[sample_index1++] = p->type2.data;
                        break;
                    case ADC_CHANNEL_1:
                        buffer2[sample_index2++] = p->type2.data;
                        break;
                    case ADC_CHANNEL_2:
                        buffer3[sample_index3++] = p->type2.data;
                        break;
                    case ADC_CHANNEL_3:
                        buffer4[sample_index4++] = p->type2.data;
                        break;
                }

                
                if (sample_index1 >= SAMPLES_PER_SECOND && sample_index2 >= SAMPLES_PER_SECOND &&
                    sample_index3 >= SAMPLES_PER_SECOND && sample_index4 >= SAMPLES_PER_SECOND) {
                    
                    end_time = micros();  
                    unsigned long duration = end_time - start_time;
                    Serial.printf("Time to fill all buffers: %lu microseconds\n", duration);

                    
                    stop_adc_and_clear_dma();

                    
                    for (int j = 0; j < SAMPLES_PER_SECOND; j++) {
                        pcmBuffer1[j] = map_adc_to_pcm(buffer1[j]);
                        pcmBuffer2[j] = map_adc_to_pcm(buffer2[j]);
                        pcmBuffer3[j] = map_adc_to_pcm(buffer3[j]);
                        pcmBuffer4[j] = map_adc_to_pcm(buffer4[j]);
                    }

                    
                    String fileName = "/yueyuan_voice_minifly_" + String(file_counter) + ".wav";

                    
                    save_to_wav(fileName, pcmBuffer1, pcmBuffer2, pcmBuffer3, pcmBuffer4);

                    file_counter++;
                    
                    
                    sample_index1 = sample_index2 = sample_index3 = sample_index4 = 0;
                    
                    restart_adc();
                    start_time = micros();
                }
            }
        }
    }
}
