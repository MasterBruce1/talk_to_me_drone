#include "FS.h"
#include "SPI.h"
#include <I2S.h>
#include "SD.h"
#define MIC_PIN1 A0  // ADC pin connected to microphone
#define MIC_PIN2 A1
#define MIC_PIN3 A2
#define MIC_PIN4 A3
#define sampleRate 4000 
#define SAMPLE_RATE 4000
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44

#include "arduinoMFCC.h"

//...............................data_record_................
hw_timer_t *timer = NULL;
#define I2S_SAMPLE_RATE 16000U
const unsigned long bufferSize = 16000;  // Buffer for 4 microphones, 4000 samples each
int bufferIndex = 0;
uint16_t packetId = 0;
int16_t dataBuffer[bufferSize];  // Buffer for 4 microphones
unsigned long lastReportTime = 0;
int sampleCount = 0;  // Counter for samples per second
#define record_size_i2s  (I2S_SAMPLE_RATE * SAMPLE_BITS / 8) * 1

uint8_t i2s_data[record_size_i2s];
int16_t pcm_data[record_size_i2s / 2];
float pcm_float_data[record_size_i2s / 2];  
float dataBuffer_float_data[bufferSize];    
float mfccFeatures_i2s[255];  
float mfccFeatures_adc[507];
float frameBuffer[512];  // Frame buffer

TaskHandle_t i2sTaskHandle = NULL;  

uint32_t recorded_sample_size = 0;
volatile bool dataReadyI2S = false;  
volatile bool dataReadyADC = false;  
volatile bool stopTimer = false; 
String baseFileName = "330_NOISY_15cm";//0_VOICE_90cm,0_noisy_90cm
int fileNumber = 1;
//..........................
int16_t map_adc_to_pcm(uint16_t adc_value) {
    return int16_t((adc_value / 4095.0) * 65535 - 32768);
}
void timer_interrupt() {
  if (stopTimer) {
    return; // 
  }
  unsigned long currentTime = millis();
  for (int i = 0; i < 4; i++) {
    uint16_t adc_value = analogRead(MIC_PIN1 + i);
    int16_t pcm_value = map_adc_to_pcm(adc_value);
    if (i == 0) {
      dataBuffer[bufferIndex] = pcm_value;          // Microphone 1
    } else if (i == 1) {
      dataBuffer[bufferIndex + 4000] = pcm_value;   // Microphone 2
    } else if (i == 2) {
      dataBuffer[bufferIndex + 8000] = pcm_value;   // Microphone 3
    } else if (i == 3) {
      dataBuffer[bufferIndex + 12000] = pcm_value;  // Microphone 4
    }
  }
  bufferIndex++;
  sampleCount++;  // Increment the sample count
  if (sampleCount == 4000) {
    sampleCount = 0;
    Serial.print("how many milli second: ");
    Serial.println(currentTime - lastReportTime);
  }
  if (bufferIndex >= 4000) {
    timerAlarmDisable(timer);
    stopTimer = true;  // 
    dataReadyADC = true;
    bufferIndex = 0;
    Serial.println("buffer ready to go!");
  }
}

void startRecording() {
  lastReportTime = millis();
  Serial.println("Recording started...");
}

void i2stask(void *pvParameters) {
    while(1) {
        size_t bytes_read = 0;
        int total_bytes_read = 0;  

        // 持续读取直到填满 i2s_data
        while (total_bytes_read < record_size_i2s) {
            esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, &i2s_data[total_bytes_read], record_size_i2s - total_bytes_read, &bytes_read, portMAX_DELAY);
            if (bytes_read > 0) {
                total_bytes_read += bytes_read;
            } else {
                
                Serial.println("Failed to read data or no data available!");
                vTaskDelay(10 / portTICK_PERIOD_MS);  
            }
        }
        
        if (total_bytes_read == record_size_i2s) {
            recorded_sample_size = total_bytes_read;
            dataReadyI2S = true;

            // uint8_t to int16_t PCM
            for (size_t i = 0; i < recorded_sample_size; i += 2) {
                //  uint8_t to int16_t
                pcm_data[i / 2] = (int16_t)((uint16_t)i2s_data[i] | ((uint16_t)i2s_data[i + 1] << 8));
            }
        }
        
        vTaskSuspend(NULL);
        while (!dataReadyADC) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
    }
}

void process_data_buffers() {

}

void setup() {
  Serial.begin(115200);

  if (!SD.begin(21)) {
    Serial.println("Failed to mount SD Card!");
    while (1);
  }

  I2S.setAllPins(-1, 42, 41, -1, -1);
  if (!I2S.begin(PDM_MONO_MODE, I2S_SAMPLE_RATE, SAMPLE_BITS)) {
    Serial.println("Failed to initialize I2S!");
    while (1) ;
  }
  Serial.println("Starting recording...");
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, timer_interrupt, true);
  timerAlarmWrite(timer, 1000000 / sampleRate, true);  // 10Mhz/4000
  startRecording();
  xTaskCreatePinnedToCore(
      i2stask,          // task 
      "i2s read task",      // task name
      4096,                 // buffer
      NULL,                 // 
      1,                    // 
      &i2sTaskHandle,   // 
      0                     // 
  );
  timerAlarmEnable(timer);
  vTaskResume(i2sTaskHandle);  
}

void save_i2s_wav(String i2sfileName){
  File file = SD.open(i2sfileName.c_str(), FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing.");
        return;
    }
    // Generate WAV header and write it
    uint8_t wav_header[WAV_HEADER_SIZE];
    generate_wav_header(wav_header, recorded_sample_size, I2S_SAMPLE_RATE);
    file.write(wav_header, WAV_HEADER_SIZE);
    // Increase volume (if needed)
    for (uint32_t i = 0; i < recorded_sample_size; i += 2) {
        int16_t* sample = (int16_t*)(i2s_data + i);
        *sample = *sample << 1;  // Increase volume by 6dB
    }
    // Write data to the file
    file.write(i2s_data, recorded_sample_size);
    file.close();
    Serial.printf("WAV file saved: %s\n", i2sfileName.c_str());
}
void save_to_wav(String fileName) {
  uint32_t sample_size = bufferSize * sizeof(uint16_t);
  uint32_t record_size = bufferSize * sizeof(uint16_t);
  uint8_t wav_header[WAV_HEADER_SIZE];
  // Write the header to the WAV file
  generate_wav_header(wav_header, record_size, SAMPLE_RATE);
  File file = SD.open(fileName.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.write(wav_header, WAV_HEADER_SIZE);
  file.write((uint8_t*)dataBuffer, sample_size);
  file.close();
  Serial.printf("Recording complete: %s\n", fileName.c_str());
}
void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate) {
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    0x01, 0x00, // NumChannels (1 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

// MFCC feature
void calculateMFCC(float* inputSignal, int signalLength, int mfccSize, int dctMfccSize, int frameSize, float samplingRate, float frameLength, float frameStride, float* outputMFCC) {
  
    arduinoMFCC mymfcc(mfccSize, dctMfccSize, frameSize, samplingRate);


    mymfcc.create_hamming_window();
    mymfcc.create_mel_filter_bank(); 
    mymfcc.create_dct_matrix();


    int numSamplesPerFrame = frameLength * samplingRate; 
    int numSamplesStride = frameStride * samplingRate;   


    int numFrames = (signalLength - numSamplesPerFrame) / numSamplesStride + 1;


    for (int i = 0; i < numFrames; i++) {
        int frameStartIndex = i * numSamplesStride;


        if (frameStartIndex + numSamplesPerFrame <= signalLength) {
           
            for (int j = 0; j < frameSize; j++) {
                frameBuffer[j] = inputSignal[frameStartIndex + j];
            }          
        }
        mymfcc.computeWithDCT(frameBuffer, outputMFCC + i * dctMfccSize);
      }
}


void save_mfcc_to_file(String fileName, float* mfccFeatures, int numCoefficients) {
  File file = SD.open(fileName, FILE_WRITE);
  if (file) {
    for (int i = 0; i < numCoefficients; i++) {
      file.print(mfccFeatures[i]);
      if (i < numCoefficients - 1) {
        file.print(", ");
      }
    }
    file.println();
    file.close();
    Serial.println("MFCC features saved to " + fileName);
  } else {
    Serial.println("Failed to open file for writing: " + fileName);
  }
}


void loop() {
  if (dataReadyI2S) {
    vTaskSuspend(i2sTaskHandle);

    
    while (!dataReadyADC) {
      Serial.println("11111111111111111.");
      delay(10);
    }
    //timerAlarmDisable(timer);
    String fileName = "/" + baseFileName + String(fileNumber) + ".wav";
    String i2sfileName = "/" + baseFileName + "_i2s_" + String(fileNumber) + ".wav";
    String mfccI2SFileName = "/" + baseFileName + "_mfcc_i2s_" + String(fileNumber) + ".txt";
    String mfccADCFileName = "/" + baseFileName + "_mfcc_adc_" + String(fileNumber) + ".txt";
    fileNumber++;
    save_to_wav(fileName);
    save_i2s_wav(i2sfileName);
    for (size_t i = 0; i < recorded_sample_size / 2; i++) {
      pcm_float_data[i] = pcm_data[i] / 32768.0f;  // 
    }
    calculateMFCC(pcm_float_data, 16000, 32, 15, 512, 16000, 0.2, 0.05,mfccFeatures_i2s);
    save_mfcc_to_file(mfccI2SFileName, mfccFeatures_i2s, 255);  // save MFCC
    for (size_t i = 0; i < bufferSize; i++) {
      dataBuffer_float_data[i] = dataBuffer[i] / 32768.0f;
    }
    calculateMFCC(dataBuffer_float_data, 16000, 32, 13, 512, 4000, 0.2, 0.1,mfccFeatures_adc);
    save_mfcc_to_file(mfccADCFileName, mfccFeatures_adc, 507);  // save MFCC

    
    dataReadyI2S = false;
    dataReadyADC = false;
    stopTimer = false; 
    
    vTaskResume(i2sTaskHandle);
    startRecording();
    timerAlarmEnable(timer);
  }
}