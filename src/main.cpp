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
#define SAMPLES 256 *8      // FFT sample size (power of 2)
#define BUFFER_SIZE (SAMPLES * 2) // 16-bit samples, mono

#define NUM_BINS (SAMPLES / 2) // 128 bins (0-8 kHz)

// Button pin for M5Stick-C
#define BUTTON_PIN 37

char buffer[BUFFER_SIZE];
size_t bytes_read;

TimerStats tsFft;

I2SClass i2s;

// Buffer to store recorded sample spectrum
float recordedSample[NUM_BINS] = {0};
bool sampleRecorded = false;

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

    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Active LOW on M5Stick-C

    Serial.println("Initializing I2S bus...");
    i2s.setPinsPdmRx(I2S_SCK, I2S_SD);

    // Initialize the I2S bus in standard mode
    if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
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


// Record a sample into the buffer
void recordSample() {
    char buffer[BUFFER_SIZE];
    size_t bytes_read;

    Serial.println("Recording sample...");

    // i2s_read(I2S_NUM_0, buffer, BUFFER_SIZE, &bytes_read, portMAX_DELAY);

    bytes_read = i2s.readBytes(buffer, BUFFER_SIZE);

    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (float)((int16_t)(buffer[i * 2] | (buffer[i * 2 + 1] << 8)));
        vImag[i] = 0.0;
    }

    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    memcpy(recordedSample, vReal, NUM_BINS * sizeof(float));
    sampleRecorded = true;

    Serial.println("Sample recorded.");
}

// Spectral Angle Mapping (returns angle in radians)
float spectralAngle(float* spectrum1, float* spectrum2, int len) {
    float dotProduct = 0.0, norm1 = 0.0, norm2 = 0.0;
    const float epsilon = 1e-6;

    for (int i = 0; i < len; i++) {
        dotProduct += spectrum1[i] * spectrum2[i];
        norm1 += spectrum1[i] * spectrum1[i];
        norm2 += spectrum2[i] * spectrum2[i];
    }

    norm1 = sqrt(norm1);
    norm2 = sqrt(norm2);
    float cosTheta = (norm1 > epsilon && norm2 > epsilon) ? dotProduct / (norm1 * norm2) : 0.0;

    if (cosTheta > 1.0) cosTheta = 1.0;
    if (cosTheta < -1.0) cosTheta = -1.0;

    return acos(cosTheta);
}

float gaussianWeight(float x, float lowerBound, float upperBound) {
    // Calculate the center (mean)
    float mu = (lowerBound + upperBound) / 2.0;

    // Calculate k (distance from center to bounds)
    float k = upperBound - mu; // Assumes upperBound > lowerBound

    // // Calculate sigma based on w(L) = w(U) = 0.01
    // float sigma = k / sqrt(2.0 * 4.60517018599); // ln(0.01) = -4.605...

    // Calculate sigma based on w(L) = w(U) = 0.1
    double sigma = k / sqrt(2.0 * 2.30258509299); // ln(0.1) = -2.302585...



    // Calculate the Gaussian weight: exp(-((x - mu)^2) / (2 * sigma^2))
    float exponent = -pow(x - mu, 2) / (2.0 * pow(sigma, 2));
    float weight = exp(exponent);

    return weight;
}

void loop() {

    // Check button press to record sample
    if (digitalRead(BUTTON_PIN) == LOW && !sampleRecorded) {
        recordSample();
        delay(100); // Debounce
    }

    // Capture audio
    bytes_read = i2s.readBytes(buffer, BUFFER_SIZE);
    if (bytes_read == 0) {
        Serial.println("Failed to read I2S data");
        return;
    }
    tsFft.Start();
    // Convert to float for FFT
    for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (float)((int16_t)(buffer[i * 2] | (buffer[i * 2 + 1] << 8)));
        //vReal[i] = buffer[i] / 32768.0 * 128;  // Normalize to [-128, 127]
        vImag[i] = 0.0;
    }

    // Perform FFT
    FFT.dcRemoval();
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);


    // Get peak frequency
    float peakFreq = FFT.majorPeak(vReal, SAMPLES, SAMPLE_RATE);

    float sfm = calculateSFM(vReal, SAMPLES / 2); // Use first half (0-8 kHz)

    tsFft.Stop();

    // SAM comparison with recorded sample (if available)
    float angle = sampleRecorded ? spectralAngle(vReal, recordedSample, NUM_BINS) : M_PI;

    Serial.printf(">peak_freq:%.1f§Hz\n", peakFreq);
    // Serial.printf(">slope:%f\n", slope);
    Serial.printf(">sfm:%f\n", sfm);
    Serial.printf(">sam:%f\n", angle);
    Serial.printf(">fft_time:%f\n", tsFft.Mean());

#define FREQ_LOW 150
#define FREQ_HIGH 250

#define SFM_LOW 0.45
#define SFM_HIGH 0.7

#define SAM_LOW 0.7
#define SAM_HIGH 1.0

    float peak_freq_weight = gaussianWeight(peakFreq, FREQ_LOW,FREQ_HIGH );
    float sfm_weight = gaussianWeight(sfm, SFM_LOW, SFM_HIGH);
    float sam_weight = gaussianWeight(angle, SAM_LOW, SAM_HIGH);
    
    float weight_product = peak_freq_weight * sfm_weight * sam_weight;
    float weight_sum = peak_freq_weight + sfm_weight + sam_weight;

    Serial.printf(">peak_freq_weight:%f\n", peak_freq_weight);
    Serial.printf(">sfm_weight:%f\n", sfm_weight);
    Serial.printf(">sam_weight:%f\n", sam_weight);
    Serial.printf(">weight_product:%f\n", weight_product);
    Serial.printf(">weight_sum:%f\n", weight_sum);


    // bool p1 = (peakFreq > FREQ_LOW) && (peakFreq < FREQ_HIGH);
    // bool p2 = (sfm > SFM_LOW) && (sfm < SFM_HIGH);
    // bool burner = p1 && p2;

    // Serial.printf(">p1:%d\n", p1);
    // Serial.printf(">p2:%d\n", p2);
    // Serial.printf(">p3:%d\n", burner);

    // peak 150-250
    // sfm 0.45 - 0.7
    // // Output to Serial console
    // Serial.printf("Peak Frequency: %.1f Hz, SFM: %.2f slope = %f", peakFreq, sfm, slope); // tsFft.Mean());
    // // Classify based on slope
    // if (fabs(slope) < 0.005) {
    //     Serial.println(" - White Noise (flat)");
    // } else if (slope < -0.01 && slope > -0.05) {
    //     Serial.println(" - Pink Noise (~3 dB/octave)");
    // } else if (slope < -0.05) {
    //     Serial.println(" - Brown Noise (~6 dB/octave)");
    // } else {
    //     Serial.println(" - Likely Signal (variable slope)");
    // }
    delay(1); // Control update rate
}