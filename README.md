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

# similar projects
https://github.com/m5stack/M5-ProductExampleCodes/blob/master/Unit/PDM/PDM.ino






