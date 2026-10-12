// Analizador de vibraciones: muestrea un eje del acelerómetro ADXL345 y lo
// muestra en una pantalla ILI9341, como espectro de frecuencias o como forma
// de onda. Un botón cambia entre los dos modos.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <arduinoFFT.h>

#include "config.h"

// Zona de la pantalla (320 x 240, apaisada) donde va la gráfica.
const int16_t GRAFICA_X = 32;
const int16_t GRAFICA_Y = 28;
const int16_t GRAFICA_ANCHO = 256;
const int16_t GRAFICA_ALTO = 176;
const int16_t PIE_Y = 222;

const int16_t PX_POR_BARRA = GRAFICA_ANCHO / (MUESTRAS / 2);
static_assert(PX_POR_BARRA >= 1 && GRAFICA_ANCHO % (MUESTRAS / 2) == 0,
              "Las barras del espectro deben caber justas en la gráfica");
static_assert(MUESTRAS % GRAFICA_ANCHO == 0, "La forma de onda dibuja un punto por columna");

// La ventana de Hamming atenúa la señal por este factor.
const double GANANCIA_HAMMING = 0.54;

enum Modo { ESPECTRO, FORMA_DE_ONDA };

static Adafruit_ILI9341 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
static Adafruit_ADXL345_Unified acelerometro(12345);
static RTC_DS3231 reloj;
static bool hayReloj = false;

static double vReal[MUESTRAS];
static double vImag[MUESTRAS];
static ArduinoFFT<double> fft(vReal, vImag, MUESTRAS, FRECUENCIA_MUESTREO);

static Modo modo = ESPECTRO;
static volatile bool botonPulsado = false;
static volatile unsigned long ultimaPulsacion = 0;

static void IRAM_ATTR alPulsar() {
  unsigned long ahora = millis();
  if (ahora - ultimaPulsacion > REBOTE_MS) {
    ultimaPulsacion = ahora;
    botonPulsado = true;
  }
}

// Aceleración del eje elegido, en m/s².
static float leerAceleracion() {
  int16_t cuentas = EJE == 'X'   ? acelerometro.getX()
                    : EJE == 'Y' ? acelerometro.getY()
                                 : acelerometro.getZ();
  return cuentas * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
}

// Llena vReal con MUESTRAS lecturas tomadas a ritmo fijo y les quita la media,
// que es la parte de la gravedad que cae sobre el eje.
static void capturar() {
  const unsigned long periodo = 1000000UL / FRECUENCIA_MUESTREO;
  unsigned long siguiente = micros();
  double suma = 0;

  for (uint16_t i = 0; i < MUESTRAS; i++) {
    while ((long)(micros() - siguiente) < 0) {
    }
    siguiente += periodo;
    vReal[i] = leerAceleracion();
    vImag[i] = 0;
    suma += vReal[i];
  }

  double media = suma / MUESTRAS;
  for (uint16_t i = 0; i < MUESTRAS; i++) {
    vReal[i] -= media;
  }
}

static void escribir(int16_t x, int16_t y, const char *texto, uint16_t color = ILI9341_WHITE) {
  tft.setCursor(x, y);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.print(texto);
}

// Título y rótulos de los ejes: lo que no cambia mientras dura el modo.
static void dibujarMarco() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  escribir(4, 4, modo == ESPECTRO ? "ESPECTRO" : "FORMA DE ONDA", ILI9341_YELLOW);
  tft.setTextSize(1);

  char rotulo[12];
  for (int i = 0; i <= 4; i++) {
    int16_t x = GRAFICA_X + i * GRAFICA_ANCHO / 4;
    if (modo == ESPECTRO) {
      snprintf(rotulo, sizeof rotulo, "%d", i * FRECUENCIA_MUESTREO / 8);
    } else {
      snprintf(rotulo, sizeof rotulo, "%d", i * 250 * MUESTRAS / FRECUENCIA_MUESTREO);
    }
    escribir(x - 3 * strlen(rotulo), GRAFICA_Y + GRAFICA_ALTO + 4, rotulo, ILI9341_LIGHTGREY);
  }
  escribir(GRAFICA_X + GRAFICA_ANCHO + 6, GRAFICA_Y + GRAFICA_ALTO + 4,
           modo == ESPECTRO ? "Hz" : "ms", ILI9341_LIGHTGREY);
}

static void limpiarGrafica() {
  tft.fillRect(GRAFICA_X, GRAFICA_Y, GRAFICA_ANCHO + 1, GRAFICA_ALTO + 1, ILI9341_BLACK);
  for (int i = 0; i <= 4; i++) {
    tft.drawFastVLine(GRAFICA_X + i * GRAFICA_ANCHO / 4, GRAFICA_Y, GRAFICA_ALTO, ILI9341_DARKGREY);
    tft.drawFastHLine(GRAFICA_X, GRAFICA_Y + i * GRAFICA_ALTO / 4, GRAFICA_ANCHO, ILI9341_DARKGREY);
  }
}

static void escribirPie(const char *texto) {
  char linea[41];
  snprintf(linea, sizeof linea, "%-40s", texto);
  escribir(GRAFICA_X, PIE_Y, linea);
}

// Una barra por frecuencia; su altura es la amplitud de esa componente. La
// escala se ajusta sola a la barra más alta.
static void mostrarEspectro() {
  fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  const double aAmplitud = 2.0 / (MUESTRAS * GANANCIA_HAMMING);
  double maximo = 0;
  uint16_t barraMaxima = 1;
  for (uint16_t i = 1; i < MUESTRAS / 2; i++) {
    vReal[i] *= aAmplitud;
    if (vReal[i] > maximo) {
      maximo = vReal[i];
      barraMaxima = i;
    }
  }
  double tope = max(maximo, (double)ESPECTRO_MINIMO);

  limpiarGrafica();
  for (uint16_t i = 1; i < MUESTRAS / 2; i++) {
    int16_t alto = vReal[i] / tope * GRAFICA_ALTO;
    tft.fillRect(GRAFICA_X + i * PX_POR_BARRA, GRAFICA_Y + GRAFICA_ALTO - alto, PX_POR_BARRA, alto,
                 ILI9341_RED);
  }

  char texto[41];
  snprintf(texto, sizeof texto, "Pico: %.1f Hz   %.2f m/s2",
           barraMaxima * (double)FRECUENCIA_MUESTREO / MUESTRAS, maximo);
  escribirPie(texto);
}

// La señal tal como se midió, centrada en cero y con la escala ajustada a su
// valor más alto.
static void mostrarFormaDeOnda() {
  double maximo = 0, cuadrados = 0;
  for (uint16_t i = 0; i < MUESTRAS; i++) {
    maximo = max(maximo, fabs(vReal[i]));
    cuadrados += vReal[i] * vReal[i];
  }
  double tope = max(maximo, (double)SENAL_MINIMA);

  limpiarGrafica();
  const uint16_t salto = MUESTRAS / GRAFICA_ANCHO;
  const int16_t centro = GRAFICA_Y + GRAFICA_ALTO / 2;
  int16_t previo = centro;
  for (int16_t x = 0; x < GRAFICA_ANCHO; x++) {
    int16_t y = centro - vReal[x * salto] / tope * (GRAFICA_ALTO / 2 - 1);
    if (x > 0) {
      tft.drawLine(GRAFICA_X + x - 1, previo, GRAFICA_X + x, y, ILI9341_CYAN);
    }
    previo = y;
  }

  char texto[41];
  snprintf(texto, sizeof texto, "Escala: +-%.2f m/s2   RMS: %.2f", tope, sqrt(cuadrados / MUESTRAS));
  escribirPie(texto);
}

static void mostrarHora() {
  if (!hayReloj) {
    return;
  }
  DateTime ahora = reloj.now();
  char texto[9];
  snprintf(texto, sizeof texto, "%02d:%02d:%02d", ahora.hour(), ahora.minute(), ahora.second());
  escribir(268, 8, texto, ILI9341_LIGHTGREY);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  if (!acelerometro.begin()) {
    Serial.println("No se encuentra el ADXL345");
    escribir(8, 8, "No se encuentra el ADXL345", ILI9341_RED);
    while (true) {
      delay(1000);
    }
  }
  acelerometro.setRange(ADXL345_RANGE_16_G);
  acelerometro.setDataRate(ADXL345_DATARATE_800_HZ);
  static_assert(FRECUENCIA_MUESTREO == 800, "Cambia también la velocidad que se pide al sensor");

  // El reloj solo pone la hora en la pantalla: sin él todo lo demás funciona.
  hayReloj = reloj.begin();
  if (hayReloj && reloj.lostPower()) {
    reloj.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), alPulsar, FALLING);

  dibujarMarco();
}

void loop() {
  if (botonPulsado) {
    botonPulsado = false;
    modo = modo == ESPECTRO ? FORMA_DE_ONDA : ESPECTRO;
    dibujarMarco();
  }

  capturar();
  if (modo == ESPECTRO) {
    mostrarEspectro();
  } else {
    mostrarFormaDeOnda();
  }
  mostrarHora();
}
