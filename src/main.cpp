#include <M5Unified.h>     // M5Unified for M5AtomS3U support
#include <driver/i2s.h>    // ESP32 I2S driver
#include "arduinoFFT.h"    // arduinoFFT library by kosme

// I2S pin definitions for M5AtomS3U's ES7210 mic
#define I2S_WS  5          // Word Select (LRCLK)
#define I2S_SD  7          // Data (DOUT from codec)
#define I2S_SCK 6          // Bit Clock (BCLK)
#define SAMPLE_RATE 16000  // 16 kHz sample rate (ES7210 supports up to 48 kHz)
#define SAMPLES 256        // FFT sample size (power of 2)
#define BUFFER_SIZE (SAMPLES * 2) // 16-bit samples, mono

// FFT setup with float
ArduinoFFT<float> FFT = ArduinoFFT<float>();
float vReal[SAMPLES];
float vImag[SAMPLES];
float samplingPeriod = 1.0 / SAMPLE_RATE;

void setup() {
  // Initialize M5Unified
  auto cfg = M5.config();
  M5.begin(cfg); // Starts Serial and initializes hardware (e.g., codec)

  // Serial setup
  Serial.begin(115200);
  delay(3000);
//   while (!Serial) delay(10); // Wait for Serial to connect
  Serial.println("M5AtomS3U FFT Demo");
  log_w("Arduino Version: %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  log_w("ESP-IDF Version: %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);

  // I2S configuration for ES7210 mic input
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Mono from mic
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  // Set I2S pins for M5AtomS3U
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE, // No output needed
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_NUM_0, &pin_config);

  delay(100); // Stabilize hardware
}

void loop() {
  uint8_t buffer[BUFFER_SIZE];
  size_t bytes_read;

  // Capture audio
  i2s_read(I2S_NUM_0, buffer, BUFFER_SIZE, &bytes_read, portMAX_DELAY);

  // Convert to float for FFT
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (float)((int16_t)(buffer[i * 2] | (buffer[i * 2 + 1] << 8)));
    vImag[i] = 0.0;
  }

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Get peak frequency
  float peakFreq = FFT.majorPeak(vReal, SAMPLES, SAMPLE_RATE);

  // Output to Serial console
  Serial.print("Peak Frequency: ");
  Serial.print(peakFreq);
  Serial.println(" Hz");

  delay(100); // Control update rate
}