#include <Arduino.h>       // M5Unified for M5AtomS3U support
#include <driver/i2s_pdm.h> // Modern ESP-IDF I2S PDM driver
#include <freertos/FreeRTOS.h> // FreeRTOS for tasks
#include <ringbuffer.hpp>

#include "arduinoFFT.h"    // arduinoFFT library by kosme
#include "TimerStats.hpp"
#include <math.h>          // For log and exp

#if defined(WS2812_LED_PIN)
    #include <FastLED.h>
    #define NUM_LEDS    1   // Number of LEDs (adjust as needed)
    #define LED_TYPE    WS2812
    #define COLOR_ORDER GRB
    CRGB leds[NUM_LEDS];
#endif

// I2S pin definitions https://docs.m5stack.com/en/core/AtomS3U
#if defined(ARDUINO_M5Stick_C)
    #define I2S_SD  GPIO_NUM_34 // Data (DOUT from codec)
    #define I2S_SCK GPIO_NUM_0  // Bit Clock (BCLK)
    #define BUTTON_PIN GPIO_NUM_37
#endif
#if defined(ARDUINO_ATOMS3U)
    #define I2S_SD  GPIO_NUM_38 // Data (DOUT from codec)
    #define I2S_SCK GPIO_NUM_39 // Bit Clock (BCLK)
    #define BUTTON_PIN GPIO_NUM_41
#endif

#define SAMPLE_RATE 16000  // 16 kHz sample rate (ES7210 supports up to 48 kHz)
#define SAMPLES 256 * 8    // FFT sample size (power of 2)
#define BUFFER_SIZE (SAMPLES * 2) // 16-bit samples, mono
#define NUM_BINS (SAMPLES / 2) // 128 bins (0-8 kHz)
#define I2S_PORT I2S_NUM_0 // Use I2S port 0
#define RING_BUFFER_SIZE (BUFFER_SIZE * 4) // 4x buffer size for ring buffer

// Global variables
static i2s_chan_handle_t rx_handle = NULL; // I2S channel handle
static espidf::RingBuffer audioRingBuffer; // Ring buffer instance
static TaskHandle_t i2sTaskHandle = NULL; // I2S reader task handle

TimerStats tsFft;

// Buffer to store recorded sample spectrum
float recordedSample[NUM_BINS] = {0};
bool sampleRecorded = false;

// FFT setup with float
ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vReal[SAMPLES];
float vImag[SAMPLES];
float samplingPeriod = 1.0 / SAMPLE_RATE;

// queue error counters
uint32_t queue_full, acquire_fail;

#if defined(WS2812_LED_PIN)
void setRainbowColor(float value) {
    uint8_t hue = map(value * 100, 0, 300, 0, 255); // Scale 0-3 to 0-255 hue
    leds[0] = CHSV(hue, 255, 255); // Full saturation and brightness
}

void led_setup() {
    FastLED.addLeds<LED_TYPE, WS2812_LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(50); // Set brightness (0-255, adjust as needed)
}

void led_update(float value) {
    setRainbowColor(value);
    FastLED.show();
}
#else
void led_setup() {}
void led_update(float v) {}
#endif

void i2s_setup() {
    // PDM RX channel configuration
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

    // PDM RX specific configuration
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = I2S_SCK,         // PDM clock pin
            .din = I2S_SD,          // PDM data in pin
            .invert_flags = {
                .clk_inv = false    // No inversion for PDM clock
            }
        }
    };

    // Initialize the RX channel
    if (i2s_new_channel(&chan_cfg, NULL, &rx_handle) != ESP_OK) {
        Serial.println("Failed to create I2S channel!");
        return;
    }

    // Initialize PDM RX mode
    if (i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg) != ESP_OK) {
        Serial.println("Failed to initialize PDM RX mode!");
        return;
    }

    // Enable the RX channel
    if (i2s_channel_enable(rx_handle) != ESP_OK) {
        Serial.println("Failed to enable I2S RX channel!");
        return;
    }

    Serial.println("I2S PDM RX bus initialized.");
}

// I2S reader task
void i2sReaderTask(void *pvParameters) {
    size_t bytes_read;
    uint8_t *i2s_buffer;

    while (1) {
        if (audioRingBuffer.send_acquire((void **)&i2s_buffer, BUFFER_SIZE, 0) != pdTRUE) {
            acquire_fail++;
            log_e("Failed to acquire queue item");
            return;
        }
        esp_err_t ret = i2s_channel_read(rx_handle, i2s_buffer, BUFFER_SIZE, &bytes_read, portMAX_DELAY);
        if (ret == ESP_OK && bytes_read > 0) {
            // Send to ring buffer
            BaseType_t sent =  audioRingBuffer.send_complete(i2s_buffer);
            if (sent != pdTRUE) {
                queue_full++;
                log_e("Failed to send queue item");
            }
        } else {
            log_e("I2S read failed in task: ret=%d bytes_read=%u", ret, bytes_read);
            vTaskDelay(1);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);

    Serial.println("M5AtomS3U FFT Demo");
    log_w("Arduino Version: %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
    log_w("ESP-IDF Version: %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);

    // Initialize button
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Active LOW on M5Stick-C

    led_setup();

    Serial.println("Initializing I2S bus...");
    i2s_setup();

    // Initialize ring buffer
    audioRingBuffer.create(RING_BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (!audioRingBuffer) { // Implicit conversion to RingbufHandle_t
        Serial.println("Failed to create ring buffer!");
        while (1); // Halt on failure
    }

    // Create I2S reader task
    BaseType_t taskCreated = xTaskCreate(
                                 i2sReaderTask,       // Task function
                                 "I2SReader",         // Task name
                                 4096,                // Stack size
                                 NULL,                // Parameters
                                 2,                   // Priority (higher than main loop)
                                 &i2sTaskHandle       // Task handle
                             );
    if (taskCreated != pdPASS) {
        Serial.println("Failed to create I2S reader task!");
        while (1); // Halt on failure
    }

    delay(100); // Stabilize hardware
}

float calculateSFM(float* spectrum, int numBins) {
    float geomMeanLog = 0.0;
    float arithMean = 0.0;
    const float epsilon = 1e-6; // Avoid log(0)

    for (int i = 1; i < numBins; i++) {
        float val = spectrum[i] + epsilon;
        geomMeanLog += log(val); // Sum of logs for geometric mean
        arithMean += val;        // Sum for arithmetic mean
    }

    geomMeanLog /= numBins;     // Average log
    arithMean /= numBins;       // Average magnitude

    return exp(geomMeanLog - log(arithMean)); // SFM
}

void recordSample() {
    Serial.println("Recording sample...");

    size_t bytes_received;
    char *data = (char *)audioRingBuffer.receive(&bytes_received, portMAX_DELAY);
    if (data != NULL && bytes_received == BUFFER_SIZE) {
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = (float)((int16_t)(data[i * 2] | (data[i * 2 + 1] << 8)));
            vImag[i] = 0.0;
        }

        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);

        memcpy(recordedSample, vReal, NUM_BINS * sizeof(float));
        sampleRecorded = true;

        Serial.println("Sample recorded.");
    } else {
        Serial.println("Failed to receive sample from ring buffer");
    }

    // Return the buffer
    if (data != NULL) {
        audioRingBuffer.return_item(data);
    }
}

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
    float mu = (lowerBound + upperBound) / 2.0;
    float k = upperBound - mu;
    double sigma = k / sqrt(2.0 * 2.30258509299); // ln(0.1) = -2.302585...
    float exponent = -pow(x - mu, 2) / (2.0 * pow(sigma, 2));
    return exp(exponent);
}

void loop() {
    // Check button press to record sample
    if (digitalRead(BUTTON_PIN) == LOW) {
        recordSample();
        delay(100); // Debounce
    }

    // Capture audio from ring buffer
    size_t bytes_received;
    char *data = (char *)audioRingBuffer.receive(&bytes_received, pdMS_TO_TICKS(10));
    if (data != NULL && bytes_received == BUFFER_SIZE) {
        tsFft.Start();
        // Convert to float for FFT
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = (float)((int16_t)(data[i * 2] | (data[i * 2 + 1] << 8)));
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
        Serial.printf(">sfm:%f\n", sfm);
        Serial.printf(">sam:%f\n", angle);
        Serial.printf(">fft_time:%f\n", tsFft.Mean());

#define FREQ_LOW 150
#define FREQ_HIGH 250
#define SFM_LOW 0.45
#define SFM_HIGH 0.7
#define SAM_LOW 0.7
#define SAM_HIGH 1.0

        float peak_freq_weight = gaussianWeight(peakFreq, FREQ_LOW, FREQ_HIGH);
        float sfm_weight = gaussianWeight(sfm, SFM_LOW, SFM_HIGH);
        float sam_weight = gaussianWeight(angle, SAM_LOW, SAM_HIGH);

        float weight_product = peak_freq_weight * sfm_weight * sam_weight;
        float weight_sum = peak_freq_weight + sfm_weight + sam_weight;

        Serial.printf(">peak_freq_weight:%f\n", peak_freq_weight);
        Serial.printf(">sfm_weight:%f\n", sfm_weight);
        Serial.printf(">sam_weight:%f\n", sam_weight);
        Serial.printf(">weight_product:%f\n", weight_product);
        Serial.printf(">weight_sum:%f\n", weight_sum);

        led_update(sfm * 10);

        // Return the buffer
        audioRingBuffer.return_item(data);
    } else {
        // No data available, skip processing
        delay(1); // Prevent tight loop
    }
}