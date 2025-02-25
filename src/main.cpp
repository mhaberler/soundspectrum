#include <Arduino.h>     // M5Unified for M5AtomS3U support
// #include <driver/i2s.h>    // ESP32 I2S driver
#include "ESP_I2S.h"
#include "arduinoFFT.h"    // arduinoFFT library by kosme
#include "TimerStats.hpp"
#include <math.h>          // For log and exp

// I2S pin definitions https://docs.m5stack.com/en/core/AtomS3U
#if defined(ARDUINO_M5Stick_C)
    #define I2S_SD  34         // Data (DOUT from codec)
    #define I2S_SCK 0         // Bit Clock (BCLK)
#endif
#if defined(ARDUINO_ATOMS3U)
    #define I2S_SD  38          // Data (DOUT from codec)
    #define I2S_SCK 39         // Bit Clock (BCLK)
#endif

#define SAMPLE_RATE 16000  // 16 kHz sample rate (ES7210 supports up to 48 kHz)
#define SAMPLES 256*8        // FFT sample size (power of 2)
#define BUFFER_SIZE (SAMPLES * 2) // 16-bit samples, mono
char buffer[BUFFER_SIZE];
size_t bytes_read;

TimerStats tsFft;

I2SClass i2s;

// FFT setup with float
ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vReal[SAMPLES];
float vImag[SAMPLES];
float samplingPeriod = 1.0 / SAMPLE_RATE;

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("M5AtomS3U FFT Demo");
    log_w("Arduino Version: %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
    log_w("ESP-IDF Version: %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);


    Serial.println("Initializing I2S bus...");
    i2s.setPinsPdmRx(I2S_SCK, I2S_SD);

    // Initialize the I2S bus in standard mode
    if (!i2s.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("Failed to initialize I2S bus!");
        return;
    }
    Serial.println("I2S bus initialized.");

    delay(100); // Stabilize hardware
}

float calculateSFM(float* spectrum, int numBins) {
    float geomMeanLog = 0.0;
    float arithMean = 0.0;
    const float epsilon = 1e-6; // Avoid log(0)

    for (int i = 0; i < numBins; i++) {
        float val = spectrum[i] + epsilon;
        geomMeanLog += log(val); // Sum of logs for geometric mean
        arithMean += val;        // Sum for arithmetic mean
    }

    geomMeanLog /= numBins;     // Average log
    arithMean /= numBins;       // Average magnitude

    return exp(geomMeanLog - log(arithMean)); // SFM
}

float calculateSpectralSlope(float* spectrum, int numBins) {
    float xSum = 0.0, ySum = 0.0, xySum = 0.0, x2Sum = 0.0;
    const float epsilon = 1e-6; // Avoid log(0)
    int n = numBins; // 128 bins (0-8 kHz)

    // Precompute x sums (bin indices)
    for (int i = 0; i < n; i++) {
        float x = (float)i;
        xSum += x;
        x2Sum += x * x;
    }

    // Compute y (log-magnitude) and cross terms
    for (int i = 0; i < n; i++) {
        float y = log10(spectrum[i] + epsilon);
        ySum += y;
        xySum += (float)i * y;
    }

    // Slope via least squares
    float numerator = n * xySum - xSum * ySum;
    float denominator = n * x2Sum - xSum * xSum;
    return numerator / denominator; // Slope in log10/bin units
}

void loop() {


    // Capture audio
    bytes_read = i2s.readBytes(buffer, BUFFER_SIZE);
    if (bytes_read == 0) {
        Serial.println("Failed to read I2S data");
        return;
    }

    // Convert to float for FFT
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (float)((int16_t)(buffer[i * 2] | (buffer[i * 2 + 1] << 8)));
        //vReal[i] = buffer[i] / 32768.0 * 128;  // Normalize to [-128, 127]
        vImag[i] = 0.0;
    }
    tsFft.Start();
    // Perform FFT
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);
    tsFft.Stop();

    // Get peak frequency
    float peakFreq = FFT.majorPeak(vReal, SAMPLES, SAMPLE_RATE);

    float sfm = calculateSFM(vReal, SAMPLES / 2); // Use first half (0-8 kHz)
    float slope = calculateSpectralSlope(vReal, SAMPLES / 2); // First 128 bins

    // Output to Serial console
    Serial.printf("Peak Frequency: %.1f Hz, SFM: %.2f slope = %f", peakFreq, sfm, slope); // tsFft.Mean());
    // Classify based on slope
    if (fabs(slope) < 0.005) {
        Serial.println(" - White Noise (flat)");
    } else if (slope < -0.01 && slope > -0.05) {
        Serial.println(" - Pink Noise (~3 dB/octave)");
    } else if (slope < -0.05) {
        Serial.println(" - Brown Noise (~6 dB/octave)");
    } else {
        Serial.println(" - Likely Signal (variable slope)");
    }
    delay(1); // Control update rate
}