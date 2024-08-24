# talk_to_me_drone
1.data_collection
  * add the Arduino_MFCC.zip to arduinno lib.
  * run datacollection.ino, change baseFileName and fileNumber for different groups of data.
  * both ADC and I2S microphone's .wav audios and MFCC features will be saved.(you could use the .wav files for training on edge impulse, or directly use MFCC features to train the model in jupyternotebook)
2. model
  * in the model folder, you can use the recorded MFCC features to train both the keywords spotting model and localization model and save them as .h5 file (in keywords_train.ipynb and localization_train.ipynb). Or you can download the trained .h5 model from edge impulse as well for the next step.
  * Then use get_model_weights.ipynb to save weights from .h5 models.
  * Use model_combinedto1.ipynb to load weights from both models and then combined into one .tflite model.
  * Use tfite_xxd.py to convert .tflite model to .h file.
  * Load the .h file in alltogether.ino and runs it on ESP32.


   
