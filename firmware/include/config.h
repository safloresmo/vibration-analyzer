#pragma once

#include <Arduino.h>

// Pantalla ILI9341 por SPI. MOSI, MISO y SCK van en los pines de fábrica del
// ESP32: 23, 19 y 18.
const uint8_t PIN_TFT_CS = 15;
const uint8_t PIN_TFT_DC = 2;
const uint8_t PIN_TFT_RST = 4;

// Botón que cambia de modo, entre el pin y GND.
const uint8_t PIN_BOTON = 13;
const unsigned long REBOTE_MS = 250;

// Eje del acelerómetro que se analiza: 'X', 'Y' o 'Z'.
const char EJE = 'X';

// Muestreo. El espectro llega hasta la mitad de la frecuencia de muestreo y
// cada barra abarca FRECUENCIA_MUESTREO / MUESTRAS hercios.
const uint16_t MUESTRAS = 256;            // Potencia de 2
const uint16_t FRECUENCIA_MUESTREO = 800; // Hz; la misma que se pide al sensor

// Por debajo de estos valores la escala deja de ampliarse, para no llenar la
// pantalla con el ruido del sensor en reposo. En m/s².
const float ESPECTRO_MINIMO = 0.5;
const float SENAL_MINIMA = 1.0;
