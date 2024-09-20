# -*- coding: utf-8 -*-
import numpy as np
import wave
import os

def calculate_snr(signal, noise):
    assert len(signal) == len(noise), "Signal and noise must be of the same length"
    # remove DC component
    signal = signal - np.mean(signal)
    noise = noise - np.mean(noise)

    signal = np.asarray(signal, dtype=np.float64)
    noise = np.asarray(noise, dtype=np.float64)
    
    signal_power = np.mean(signal ** 2)
    noise_power = np.mean(noise ** 2)
    
    if signal_power < 0 or noise_power < 0:
        raise ValueError("Signal or noise power is negative. This should not happen.")

    if noise_power == 0:
        return float('inf')  

    snr = 10 * np.log10(signal_power / noise_power)
    return snr

def read_wav(file_path):
    with wave.open(file_path, 'rb') as wav_file:
        # load wav
        params = wav_file.getparams()
        n_channels, sampwidth, framerate, n_frames = params[:4]

        # load frames
        frames = wav_file.readframes(n_frames)
        
        
        audio_data = np.frombuffer(frames, dtype=np.int16)

    
        if n_channels > 1:
            audio_data = np.reshape(audio_data, (-1, n_channels))

        return audio_data

# pure_voice path
signal_file = 'B:/72-91/2024.9.1_2024.12.31/SELD/dataset/16KHz/Yueyuan/minifly/voice/0/yueyuan_voice_minifly_14.wav'
# pure_noise path
noise_folder = 'B:/72-91/2024.9.1_2024.12.31/SELD/dataset/16KHz/pure_noise'

# load pure_voice
signal = read_wav(signal_file)
snrs_list = []

# calculate SNR for each noise file 
for noise_filename in os.listdir(noise_folder):
    if noise_filename.endswith('.wav'):
        noise_file = os.path.join(noise_folder, noise_filename)
        noise = read_wav(noise_file)

        # make sure same length
        min_length = min(signal.shape[0], noise.shape[0])
        trimmed_signal = signal[:min_length, :]
        trimmed_noise = noise[:min_length, :]

        # makesure same channel
        if trimmed_signal.shape[1] != trimmed_noise.shape[1]:
            raise ValueError(f"Signal and noise must have the same number of channels. Got {trimmed_signal.shape[1]} and {trimmed_noise.shape[1]}")

        # calculate SNR for each channel
        channel_snrs = []
        num_channels = trimmed_signal.shape[1]

        for ch in range(num_channels):
            signal_channel = trimmed_signal[:, ch]
            noise_channel = trimmed_noise[:, ch]
            snr = calculate_snr(signal_channel, noise_channel)
            channel_snrs.append(snr)
        
        # print SNR for each channel
        print(f"SNRs for {noise_filename} by channel: {channel_snrs}")

        # calculate average SNR
        average_snr = np.mean(channel_snrs)
        snrs_list.append(average_snr)
        print(f"Average SNR for {noise_filename}: {average_snr:.2f} dB")
        
# print all average SNRs
print(f"All Average SNRs: {snrs_list}")

# print overall average SNR
overall_average_snr = np.mean(snrs_list)
print(f"Overall Average SNR: {overall_average_snr:.2f} dB")
