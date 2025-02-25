#include <M5StickCPlus2.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "arduinoFFT.h"

// Copyright 2024 Adam Carlin

const uint16_t samples = 1024;
const double samplingFrequency = 44100;

double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// A-weighting filter coefficients
const double aWeightingCoefficients[] = {
    -85.4, -77.4, -69.4, -61.4, -53.4, -45.4, -37.4, -29.4, -21.4, -13.4,
    -5.4, 2.6, 10.6, 18.6, 26.6, 34.6, 42.6, 50.6, 58.6, 66.6,
    74.6, 82.6, 90.6, 98.6, 106.6, 114.6, 122.6, 130.6, 138.6, 146.6,
    154.6, 162.6, 170.6, 178.6, 186.6, 194.6, 202.6, 210.6, 218.6, 226.6,
    234.6, 242.6, 250.6, 258.6, 266.6, 274.6, 282.6, 290.6, 298.6, 306.6,
    314.6, 322.6, 330.6, 338.6, 346.6, 354.6, 362.6, 370.6, 378.6, 386.6,
    394.6, 402.6, 410.6, 418.6, 426.6, 434.6, 442.6, 450.6, 458.6, 466.6,
    474.6, 482.6, 490.6, 498.6, 506.6, 514.6, 522.6, 530.6, 538.6, 546.6,
    554.6, 562.6, 570.6, 578.6, 586.6, 594.6, 602.6, 610.6, 618.6, 626.6,
    634.6, 642.6, 650.6, 658.6, 666.6, 674.6, 682.6, 690.6, 698.6, 706.6,
    714.6, 722.6, 730.6, 738.6, 746.6, 754.6, 762.6, 770.6, 778.6, 786.6,
    794.6, 802.6, 810.6, 818.6, 826.6, 834.6, 842.6, 850.6, 858.6, 866.6,
    874.6, 882.6, 890.6, 898.6, 906.6, 914.6, 922.6, 930.6, 938.6, 946.6,
    954.6, 962.6, 970.6, 978.6, 986.6, 994.6, 1002.6, 1010.6, 1018.6, 1026.6
};

int ambientNoiseAdjustment = 45; // Arduino is noisy, maybe other things too, used to calibrate

// Define constants for LCD dimensions
const int LCD_WIDTH = 240;
const int LCD_HEIGHT = 135;

// Define ST7789 display commands
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29

// Prometheus endpoint configuration
const char* prometheusEndpoint = "http://<your_ip:port_maybe>/metrics/job/sound_level_dba/instance/<room_location>";

// Wi-Fi configuration
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

void setup() {
    auto cfg = M5.config();
    cfg.internal_mic = true;
    M5.begin(cfg);

    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(2);

    Serial.begin(115200);
    while (!Serial) continue;
    Serial.println("Ready");
#if 0
    // StickCP2.Mic.begin();
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
#endif
}

void loop() {
    M5.update();

    static int16_t micData[samples];
    static int16_t dbAHistory[LCD_WIDTH];
    static int dbAHistoryIndex = 0;
    static uint64_t lastPushTime = 0;

    // Read microphone data
    if (StickCP2.Mic.record(micData, samples, samplingFrequency)) {
        for (int i = 0; i < samples; i++) {
            vReal[i] = micData[i];
            vImag[i] = 0;
        }

        FFT.dcRemoval();
        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();

        // Apply A-weighting filter
        for (int i = 0; i < samples / 2; i++) {
            double frequency = (i * 1.0 * samplingFrequency) / samples;
            int index = static_cast<int>(frequency / (samplingFrequency / 2) * 120);
            if (index >= 0 && index < 120) {
                vReal[i] += aWeightingCoefficients[index];
            }
        }

        // Find the maximum magnitude after applying A-weighting
        double maxMagnitude = 0;
        for (int i = 0; i < samples / 2; i++) {
            if (vReal[i] > maxMagnitude) {
                maxMagnitude = vReal[i];
            }
        }

        double decibels = abs((20 * log10(maxMagnitude)) - ambientNoiseAdjustment);

        if (isnan(decibels)) {
            decibels = 0;
        }

        // Update dBA history buffer
        dbAHistory[dbAHistoryIndex++] = decibels;
        if (dbAHistoryIndex >= LCD_WIDTH) {
            dbAHistoryIndex = 0;
        }

        // Display dBA and AN values on a single line
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("dBA: %.2f | AN: %d", decibels, ambientNoiseAdjustment);

        // Define maximum graph dimensions
        const int maxGraphWidth = 240; // Adjusted for rotated display width
        const int maxGraphHeight = 135; // Adjusted for rotated display height

        // Calculate margins dynamically to fit within bounds
        const int bottomMargin = 25; // Space for x-axis labels, reduced to fit
        const int topMargin = 20; // Space for top margin, keeping consistent
        const int effectiveGraphWidth = maxGraphWidth;
        const int effectiveGraphHeight = maxGraphHeight - bottomMargin - topMargin;

        // Draw X-axis line
        M5.Lcd.drawLine(0, maxGraphHeight - bottomMargin, maxGraphWidth, maxGraphHeight - bottomMargin, WHITE);

        // Center "Time" label beneath the graph
        String timeLabel = "Time";
        int timeLabelWidth = 6 * timeLabel.length(); // Approximate width of the label, assuming 6 pixels per character
        M5.Lcd.setCursor((maxGraphWidth - timeLabelWidth) / 2, maxGraphHeight - bottomMargin + 5); // Centering the label
        M5.Lcd.print(timeLabel);

        // Draw line graph of dBA history
        for (int i = 0; i < effectiveGraphWidth; i++) {
            int index = (dbAHistoryIndex + i) % effectiveGraphWidth;
            int dBAValue = constrain(dbAHistory[index], 0, 140);
            int y = map(dBAValue, 0, 140, maxGraphHeight - bottomMargin - 1, topMargin + 1);
            M5.Lcd.drawPixel(i, y, WHITE);
        }

        Serial.print("dBA: ");
        Serial.print(decibels);
        Serial.print(", AN: ");
        Serial.println(ambientNoiseAdjustment);

#if 0
        // Send dBA to Prometheus Pushgateway every 1 second
        if (millis() - lastPushTime >= 1000) {
            if (WiFi.status() == WL_CONNECTED) {
                HTTPClient http;
                http.begin(prometheusEndpoint);
                http.addHeader("Content-Type", "text/plain");

                String payload = "sound_level_dba " + String(decibels) + "\n";
                int httpResponseCode = http.POST(payload);

                if (httpResponseCode == 200) {
                    Serial.println("Data " + String(decibels) + " pushed to Prometheus Pushgateway");
                } else {
                    Serial.print("Error pushing data to Prometheus Pushgateway. Response code: ");
                    Serial.println(httpResponseCode);
                }

                http.end();
            }
            lastPushTime = millis();
        }
#endif
    }

    // Adjust ambient noise factor using buttons
    if (M5.BtnA.wasPressed() && !M5.BtnA.wasDoubleClicked()) {
        ambientNoiseAdjustment--;
    } else if (M5.BtnB.wasPressed()) {
        ambientNoiseAdjustment++;
    }

    // Turn off display on double click of button A
    if (M5.BtnA.wasDoubleClicked()) {
        M5.Lcd.writecommand(ST7789_DISPOFF);
    }

    // Turn on display on any button press
    if (M5.BtnA.wasPressed() || M5.BtnB.wasPressed()) {
        M5.Lcd.writecommand(ST7789_DISPON);
    }

    delay(100);
}