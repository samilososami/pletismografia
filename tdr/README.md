# Sketch Arduino del TDR

## Abrir y compilar

1. Instala en Arduino IDE 2 el paquete de placas **esp32 by Espressif Systems**.
2. Instala estas bibliotecas desde el gestor de bibliotecas:
   - `Adafruit GFX Library`
   - `Adafruit SSD1306`
3. Abre `tdr.ino` desde esta misma carpeta. No copies el archivo fuera de
   `tdr/`, porque necesita los módulos de `src/`.
4. Selecciona **ESP32 Dev Module** como placa.
5. Selecciona el puerto serie de la Keyestudio ESP32 Plus y pulsa **Subir**.

Compilación equivalente mediante Arduino CLI:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 tdr
```

## Distribución del código

```text
tdr/
├── tdr.ino                  Firmware, PPG, OLED, Wi-Fi y API
└── src/
    └── web/
        ├── web_page.h       Interfaz pública del módulo web
        └── web_page.cpp     HTML, CSS y JavaScript almacenados en flash
```

Arduino compila recursivamente todo lo que se encuentra dentro de `src/`, por
lo que no hay que añadir manualmente `web_page.cpp` al proyecto.

## Pines configurados

| Elemento | Pin ESP32 |
|---|---:|
| LDR / señal analógica | GPIO34 |
| Motor, entrada del módulo | GPIO25 |
| OLED SDA | GPIO21 |
| OLED SCL/SCK | GPIO22 |

La OLED esperada es SSD1306 de 128 × 32 píxeles. El firmware prueba las
direcciones I²C `0x3C` y `0x3D`.

## Red local

- SSID: `fotopletismografia`
- Contraseña: `12345678`
- Dirección habitual: `http://192.168.4.1/`

## Advertencia

El sistema con LED y LDR es un prototipo educativo. Los BPM, la respiración y
el estrés son estimaciones experimentales y no deben utilizarse para tomar
decisiones médicas.
