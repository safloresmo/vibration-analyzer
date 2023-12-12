#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "RTClib.h"
#include <arduinoFFT.h>

// Definiciones para la pantalla ILI9341
#define TFT_CS     15
#define TFT_RST    4
#define TFT_DC     2
#define BUTTON_PIN 13 // Pin para el botón de cambio de modo

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
RTC_DS3231 rtc;
arduinoFFT FFT = arduinoFFT();

// Variables para FFT
#define SAMPLES 128             // Número de muestras (potencia de 2)
#define SAMPLING_FREQUENCY 1000 // Frecuencia de muestreo en Hz
#define BUFFER_SIZE (SAMPLES / 2 * 2) // Tamaño del buffer para dos mediciones
double vReal[SAMPLES];
double vImag[SAMPLES];
double fftBuffer[BUFFER_SIZE] = {0}; // Buffer para almacenar los datos de FFT
unsigned long microseconds;
unsigned int sampling_period_us;

// Variables para osciloscopio
const int GRAPH_WIDTH = 320; // Ancho de la gráfica en píxeles
int graphPoints[GRAPH_WIDTH]; // Almacena los puntos de la gráfica

// Variables para controlar el desplazamiento de la gráfica
unsigned long lastUpdateTime = 0;
const int pixelsPerSecond = 15; // Desplazamiento por segundo (ajustable)
const int updateInterval = 1000 / pixelsPerSecond; // Intervalo de actualización en milisegundos

// Constantes para el escalado de la gráfica
const float accelerationToVelocityScale = 9.81 * 1000; // g a mm/s
const float maxVelocity = 100000.0; // Máxima velocidad en mm/s que queremos mostrar
const float velocityToPixelScale = tft.height() / maxVelocity;

// Modo actual (true para FFT, false para osciloscopio)
bool modeFFT = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  tft.begin();
  tft.setRotation(0); // Ajusta para la orientación de tu pantalla

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Configura el botón

  // Inicialización y configuración del sensor ADXL345
  if (!accel.begin()) {
    Serial.println("Ocurrió un error con el sensor ADXL345");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  // Inicialización y configuración del RTC
  if (!rtc.begin()) {
    Serial.println("Ocurrió un error con el RTC DS3231");
    while (1);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  tft.fillScreen(ILI9341_BLACK);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop() {
  // Cambio de modo con el botón
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    modeFFT = !modeFFT; // Cambia el modo
    tft.fillScreen(ILI9341_BLACK); // Limpia la pantalla al cambiar de modo
    delay(200); // Anti-rebote
  }
  lastButtonState = currentButtonState;

  if (modeFFT) {
    runFFTMode();
  } else {
    runOscilloscopeMode();
  }
}

void runFFTMode() {
  // FFT Mode
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();
    sensors_event_t event;
    accel.getEvent(&event);
    vReal[i] = event.acceleration.x;
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us)) {
      // Espere hasta la siguiente muestra
    }
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // Desplazar los datos en el buffer
  for (int i = 0; i < BUFFER_SIZE - SAMPLES / 2; i++) {
    fftBuffer[i] = fftBuffer[i + SAMPLES / 2];
  }

  // Almacenar nuevos datos en el buffer
  for (int i = 0; i < SAMPLES / 2; i++) {
    fftBuffer[BUFFER_SIZE - SAMPLES / 2 + i] = (vReal[i] / SAMPLES) * tft.height();
  }

  tft.fillScreen(ILI9341_BLACK);
  drawGrid();

  // Dibujar desde el buffer
  int lastX = 0, lastY = tft.height() / 2;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int newX = map(i, 0, BUFFER_SIZE, 0, tft.width());
    int newY = tft.height() / 2 - (int)fftBuffer[i];

    tft.drawLine(lastX, lastY, newX, newY, ILI9341_RED);
    lastX = newX;
    lastY = newY;
  }

  delay(1000);
}

void runOscilloscopeMode() {
  // Oscilloscope Mode
  sensors_event_t event;
  accel.getEvent(&event);

  // Estimar la velocidad en mm/s
  float velocity = event.acceleration.y * accelerationToVelocityScale; // Asumiendo que y es la dirección de interés

  // Escalar la velocidad para la representación en píxeles
  int yVal = map(velocity, -maxVelocity, maxVelocity, 0, tft.height());

  // Actualizar la gráfica basándose en el tiempo
  if (millis() - lastUpdateTime > updateInterval) {
    lastUpdateTime = millis();
    
    // Desplaza los puntos de la gráfica
    for (int i = 1; i < GRAPH_WIDTH; i++) {
      graphPoints[i - 1] = graphPoints[i];
    }
    graphPoints[GRAPH_WIDTH - 1] = yVal;

    tft.fillScreen(ILI9341_BLACK);
    drawGrid();

    // Dibuja la gráfica
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
      tft.drawLine(i, graphPoints[i], i + 1, graphPoints[i + 1], ILI9341_BLUE);
    }
  }
}

void drawGrid() {
  uint16_t gridColor = ILI9341_DARKGREY;
  
  for (int i = 0; i < tft.width(); i += 15) {
    tft.drawFastVLine(i, 0, tft.height(), gridColor);
  }
  
  for (int i = 0; i < tft.height(); i += 15) {
    tft.drawFastHLine(0, i, tft.width(), gridColor);
  }
}
