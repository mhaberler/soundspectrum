# soundspectrum

read I2S PDM microphone
run fft
determine frequency peak

pioarduino 3.13
m5stick-c, cplus 1.1, cplus2

size 256 window: 16000/256 = 62.5 windows/sec = 16mS/Window
FFT time = 1.7mS

size 2048 window: = 7.8 windows/sec = 128mS/window
FFT time = 14mS

# noise detection 
Method: Spectral Flatness Analysis with Thresholding

# spectral slope analysis
add Method: Linear Regression on Log-Magnitude Spectrum

# grok3 discourse
https://x.com/i/grok/share/qcpPDmrsrhZ0wuTwgElHRvtKq

# Sound level meter  (branch sound_level_dba)

https://github.com/blissend/sound_level_dba/

any M5Stick-C

# related projects
https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Unit/PDM/PDM.ino
https://github.com/eggfly/CloveCardComputer/blob/2059ddbabd9a7ba951b062c7270596089fca97ca/CloveV2/FactoryTest/src/PDM_SPM1423.cpp
https://github.com/m5stack/M5StickC/blob/202fddddf05d878381f3b4d5bfa6b6e70031a290/examples/Basics/Micophone/Micophone.ino
https://github.com/esikora/M5StickC_AudioVisLed


https://github.com/enriqqqq/guardiband?tab=readme-ov-file
https://github.com/enriqqqq/cough_detection






