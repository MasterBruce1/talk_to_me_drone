# talk_to_me_drone
1.data_collection
  * add the Arduino_MFCC.zip to arduinno lib.
  * run datacollection.ino, change baseFileName and fileNumber for different groups of data.
  * both ADC and I2S microphone's .wav audios and MFCC features will be saved.(you could use the .wav files for edge impulse, or directly use MFCC features to train the model in jupyternotebook)
2. model
  * in the model folder, you can use the recorded MFCC features to train both the keywords spotting model and localization model and save them as .h file.
  * Then use get_weight_model.jp

   
