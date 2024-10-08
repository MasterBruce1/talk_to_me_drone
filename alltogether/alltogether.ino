#include "FS.h"
#include "SPI.h"
#include <I2S.h>
#include "arduinoMFCC.h"
#include <Chirale_TensorFlowLite.h>
#include <math.h>
// include static array definition of pre-trained model
#include "model_combined_tflite.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

#define MIC_PIN1 A0  // ADC pin connected to microphone
#define MIC_PIN2 A1
#define MIC_PIN3 A2
#define MIC_PIN4 A3
#define sampleRate 4000  // Adjusted sample rate to 3022Hz, THE REAL TIME SAMPLE RATE IS AROUND 3000
#define SAMPLE_RATE 4000
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44

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
// model input
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
TfLiteTensor* input2 = nullptr;
TfLiteTensor* output2 = nullptr;
constexpr int kTensorArenaSize = 16 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
//model_keywords_output
int8_t output_data[3];
int8_t output_data2[1];

int8_t input_data_int8[255];
int8_t input_data2_int8[507];
float input_scale = 0.024330588057637215;  
int input_zero_point = 10;                 
float input_scale2 = 0.024489019066095352;  
int input_zero_point2 = 7;                 

//..........................
int16_t map_adc_to_pcm(uint16_t adc_value) {
    return int16_t((adc_value / 4095.0) * 65535 - 32768);
}
void timer_interrupt() {
  if (stopTimer) {
    return; 
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
    stopTimer = true;  
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

            
            for (size_t i = 0; i < recorded_sample_size; i += 2) {
                
                pcm_data[i / 2] = (int16_t)((uint16_t)i2s_data[i] | ((uint16_t)i2s_data[i + 1] << 8));
            }
        }
        
        vTaskSuspend(NULL);
        while (!dataReadyADC) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
    }
}



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

float wrapTo360(float angle) {
    float result = fmod(angle, 360.0);  
    if (result < 0) {
        result += 360.0;  
    }
    return result;
}

void process_data_buffers() {
  unsigned long startTime1 = millis();  
    
  for (size_t i = 0; i < recorded_sample_size / 2; i++) {
      pcm_float_data[i] = pcm_data[i] / 32768.0f;  
  }
  calculateMFCC(pcm_float_data, 16000, 32, 15, 512, 16000, 0.2, 0.05,mfccFeatures_i2s);
  for (int i = 0; i < 255; i++) {
    input_data_int8[i] = (int8_t)(mfccFeatures_i2s[i] / input_scale) + input_zero_point;
  }
  
  for (size_t i = 0; i < bufferSize; i++) {
      dataBuffer_float_data[i] = dataBuffer[i] / 32768.0f;
  }

  calculateMFCC(dataBuffer_float_data, 16000, 32, 13, 512, 4000, 0.2, 0.1,mfccFeatures_adc);
  for (int i = 0; i < 255; i++) {
    input_data2_int8[i] = (int8_t)(mfccFeatures_adc[i] / input_scale2) + input_zero_point2;
  }

  // Place the value in the model's input tensor
    for (int i = 0; i < 255; i++) {
    input->data.int8[i] = input_data_int8[i];
    }
    for (int i = 0; i < 507; i++) {
    input2->data.int8[i] = input_data2_int8[i];
    }
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      Serial.println("Invoke failed!");
      return;
    }
    for (int i = 0; i < 3; i++) {
    output_data[i] = output->data.int8[i];
    Serial.print("Output ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(output_data[i]);
    }
    for (int i = 0; i < 1; i++) {  
    //output_data2[i] = output2->data.int8[i];
    float output_float = (output2->data.int8[i] - input_zero_point2) * input_scale2;
    
    float wrapped_output = wrapTo360(output_float);
    Serial.print("Output2 ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(wrapped_output);
    }
  

  Serial.println("Data has been normalized to the range [-1, 1].");
  memset(i2s_data, 0, sizeof(i2s_data));
  memset(pcm_data, 0, sizeof(pcm_data));

  unsigned long endTime1 = millis();  
  unsigned long elapsedTime1 = endTime1 - startTime1;  

    
    Serial.print("Process buffer time (ms): ");
    Serial.println(elapsedTime1);
}

void setup() {
  Serial.begin(115200);
  //model init....................
  model = tflite::GetModel(combined_model_int8_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model provided and schema version are not equal!");
    while(true); // stop program here
  }
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;
  
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    while(true); // stop program here
  }

  input = interpreter->input(0);
  output = interpreter->output(0);
  input2 = interpreter->input(1);
  output2 = interpreter->output(1);
  Serial.println("Initialization done.");
  //...................model init...........

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
      i2stask,          
      "i2s read task",      
      4096,                 
      NULL,                
      1,                    
      &i2sTaskHandle,  
      0                     
  );
  timerAlarmEnable(timer);
  vTaskResume(i2sTaskHandle);  
}
void loop() {
  if (dataReadyI2S) {
    vTaskSuspend(i2sTaskHandle);

    
    while (!dataReadyADC) {
      Serial.println("11111111111111111.");
      delay(10);
    }
    //timerAlarmDisable(timer);
    
    process_data_buffers();

    
    dataReadyI2S = false;
    dataReadyADC = false;
    stopTimer = false;  
    
    vTaskResume(i2sTaskHandle);
    startRecording();
    timerAlarmEnable(timer);
  }
}