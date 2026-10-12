"""Dibuja los diagramas de conexiones del analizador.

Coloca las piezas de `piezas/` y traza los cables entre sus pines. Las
ilustraciones del acelerómetro, del reloj y del botón son de la biblioteca de
piezas de Fritzing (CC BY-SA 3.0).

    python docs/conexiones/generar.py
"""

import re
from pathlib import Path

AQUI = Path(__file__).parent

ROJO, NEGRO = "#d8262c", "#3a3a3a"
AMARILLO, VERDE, AZUL = "#f2c200", "#2e9e4f", "#2f6fd0"
NARANJA, MORADO, GRIS, ROSA = "#ee7f1a", "#8b4fc0", "#9a9a9a", "#e66aa9"

# Pines de la placa, de izquierda a derecha, con el USB a la derecha.
ARRIBA = "D23 D22 TX0 RX0 D21 D19 D18 D5 TX2 RX2 D4 D2 D15 GND 3V3".split()
ABAJO = "EN VP VN D34 D35 D32 D33 D25 D26 D27 D14 D12 D13 GND VIN".split()
PANTALLA = "SDO LED SCK SDI DC RESET CS GND VCC".split()
ADXL345 = "GND VCC CS INT1 INT2 SDO SDA SCL".split()
DS3231 = {"SCL": 28.9, "SDA": 36.1, "VCC": 43.3, "GND": 50.5}


def pieza(nombre, x, y):
    texto = (AQUI / "piezas" / f"{nombre}.svg").read_text()
    cuerpo = texto[texto.index(">", texto.index("<svg")) + 1 : texto.rindex("</svg>")]
    return f'<g transform="translate({x} {y})">{cuerpo}</g>'


def placa(x, y):
    """La placa y dos funciones que dan el centro de cada pin."""
    arriba = lambda pin: (x + 12 + 7.2 * ARRIBA.index(pin), y + 5)
    abajo = lambda pin: (x + 12 + 7.2 * ABAJO.index(pin), y + 75)
    return pieza("esp32", x, y), arriba, abajo


def cable(color, *puntos):
    trazo = " ".join(f"{px:.2f},{py:.2f}" for px, py in puntos)
    comun = 'fill="none" stroke-linecap="round" stroke-linejoin="round"'
    return (
        f'<polyline points="{trazo}" {comun} stroke="#000000" stroke-opacity="0.45" stroke-width="3.4"/>'
        f'<polyline points="{trazo}" {comun} stroke="{color}" stroke-width="2.4"/>'
    )


def escalon(color, origen, destino, y):
    """Baja desde `origen` hasta la altura `y`, cruza y baja a `destino`."""
    return cable(color, origen, (origen[0], y), (destino[0], y), destino)


def guardar(nombre, ancho, alto, *partes):
    svg = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{ancho * 4}" height="{alto * 4}" '
        f'viewBox="0 0 {ancho} {alto}">'
        f'<rect width="{ancho}" height="{alto}" fill="#ffffff"/>' + "".join(partes) + "</svg>\n"
    )
    (AQUI / f"{nombre}.svg").write_text(svg)


def pantalla():
    esp32, arriba, _ = placa(20, 168)
    pin = lambda nombre: (42 + 26 + 7.2 * PANTALLA.index(nombre), 10 + 90)
    led, vcc = pin("LED"), pin("VCC")
    guardar(
        "pantalla",
        176,
        260,
        pieza("pantalla", 42, 10),
        esp32,
        # La luz de fondo se alimenta del propio pin VCC de la pantalla.
        cable(ROSA, led, (led[0], 106), (vcc[0] - 4, 106), vcc),
        cable(MORADO, pin("SDO"), arriba("D19")),
        cable(GRIS, pin("RESET"), arriba("D4")),
        escalon(AMARILLO, pin("SCK"), arriba("D18"), 147),
        escalon(VERDE, pin("SDI"), arriba("D23"), 156),
        escalon(AZUL, pin("DC"), arriba("D2"), 137),
        escalon(NARANJA, pin("CS"), arriba("D15"), 129),
        escalon(NEGRO, pin("GND"), arriba("GND"), 121),
        escalon(ROJO, vcc, arriba("3V3"), 113),
    )


def acelerometro():
    esp32, arriba, _ = placa(20, 120)
    pin = lambda nombre: (122 + 3.6 + 7.2 * ADXL345.index(nombre), 20 + 39.6)
    cs, vcc = pin("CS"), pin("VCC")
    guardar(
        "acelerometro",
        196,
        212,
        pieza("adxl345", 122, 20),
        esp32,
        cable(NEGRO, pin("GND"), arriba("GND")),
        cable(ROJO, vcc, arriba("3V3")),
        # CS a VCC deja el sensor en modo I2C.
        cable(ROJO, cs, (cs[0], cs[1] + 7), (vcc[0] + 4, cs[1] + 7), vcc),
        escalon(VERDE, pin("SDA"), arriba("D21"), 95),
        escalon(AMARILLO, pin("SCL"), arriba("D22"), 107),
    )


def reloj():
    esp32, arriba, _ = placa(20, 120)
    pin = lambda nombre: (165 + 4.3, 30 + DS3231[nombre])
    codo = lambda nombre, destino: (pin(nombre), (arriba(destino)[0], pin(nombre)[1]), arriba(destino))
    guardar(
        "reloj",
        284,
        212,
        pieza("ds3231", 165, 30),
        esp32,
        cable(AMARILLO, *codo("SCL", "D22")),
        cable(VERDE, *codo("SDA", "D21")),
        cable(ROJO, *codo("VCC", "3V3")),
        cable(NEGRO, *codo("GND", "GND")),
    )


def boton():
    esp32, _, abajo = placa(20, 12)
    x, y = 116.3, 112
    d13, gnd = abajo("D13"), abajo("GND")
    # Dos patas en esquinas opuestas: el botón las une al pulsarlo.
    guardar(
        "boton",
        176,
        166,
        esp32,
        pieza("boton", x, y),
        cable(AZUL, d13, (x + 2.1, y + 1.5)),
        cable(NEGRO, gnd, (gnd[0], 98), (148, 98), (148, 152), (x + 22.1, 152), (x + 22.1, y + 31.5)),
    )


if __name__ == "__main__":
    pantalla()
    acelerometro()
    reloj()
    boton()
