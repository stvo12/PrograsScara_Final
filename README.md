# PrograsScara_Final

Programas de un robot SCARA para recogida de piezas sobre una banda transportadora, con seguimiento por encoder, visión HuskyLens y control mediante mando y OLED.

Esta copia conserva los archivos locales disponibles el **26 de octubre de 2026**. Los programas originales de trabajo no se modificaron al preparar el repositorio. No se incluyen modelos CAD, imágenes, vídeos, compilaciones ni credenciales.

## Programas

| Carpeta | Equipo | Función |
|---|---|---|
| `PORTENTA_SCARA_Ordenado` | Portenta con Machine Control | Calibración, movimiento SCARA, encoder y secuencia automática de agarre/descarga. |
| `ESP_CamaraI2C` | ESP32 conectado a HuskyLens | Calibración visual, detección y envío de datos a la PMC por I2C, dirección `0x29`. |
| `ESP_WROOM_32_CONTROL_OLED` | ESP32-WROOM-32 | Mando Bluetooth, OLED y servos de apertura/giro, dirección I2C `0x28`. |
| `libraries/DFRobot_HuskylensV2` | Dependencia Arduino | Copia de la biblioteca instalada para la cámara, versión declarada 1.0.9. Incluye su licencia MIT y archivos originales de documentación. |

Los documentos dentro de las carpetas de firmware son notas históricas; para los valores vigentes prevalece el código de esta copia.

## Configuración de esta copia

- Eslabones: L1 = 120.5 mm; L2 = 120 mm.
- Transmisión Z: polea del motor de 50 dientes y polea del tornillo de 39 dientes, una sola etapa.
- Auto reconoce exactamente el nombre `pieza5` (distingue mayúsculas/minúsculas).
- `pieza5` utiliza la descarga que antes correspondía a Pieza3: X = -105.081, Y = -12.370, Z = 125 mm.
- Las otras dos posiciones de descarga permanecen definidas.
- Hover: 20 mm; Z de pick: 0 mm.
- Correcciones de posición: X = -25 mm, Y = -15 mm.
- Estimado de descenso: horizontal = 0.80 s; vertical = 1.30 s.
- Anticipo de cierre: horizontal = 35 mm; vertical = 55 mm.
- Espera posterior al comando de cierre: 500 ms.

La configuración principal está en `PORTENTA_SCARA_Ordenado/PORTENTA_SCARA_Ordenado.ino`. Los límites y ángulos de los servos también se configuran en el programa del ESP de control. Se conservaron los valores existentes de cada equipo sin unificarlos.

## Carga y dependencias

1. Instalar Arduino IDE y el soporte de las placas utilizadas.
2. Para la PMC, instalar `Arduino_PortentaMachineControl` (versión local observada: 1.0.5). El entorno utilizado dispone del core Arduino Mbed Portenta 4.5.0.
3. Para el ESP de cámara, instalar la biblioteca incluida en `libraries/DFRobot_HuskylensV2` en la carpeta de bibliotecas de Arduino. Conservar una copia de cualquier instalación previa antes de reemplazarla.
4. Para el ESP de control, instalar el soporte ESP32 con Bluepad32 y las bibliotecas `Adafruit_GFX`, `Adafruit_SSD1306` y `ESP32Servo` (versión local observada de ESP32Servo: 3.2.0).
5. Abrir el `.ino` dentro de cada carpeta y seleccionar la placa y puerto correspondientes. Compilar y cargar cada programa en su equipo, no los tres en la misma placa.
6. Verificar cableado, direcciones I2C y parámetros mecánicos. Ejecutar la calibración del robot y la calibración visual antes de operar Auto.

No se ha realizado una compilación integral nueva como parte de la publicación de esta copia. La instalación y entrenamiento del modelo de HuskyLens son externos al repositorio.

## Resultados de agarre reportados

15 intentos por orientación, con el ajuste de velocidad de banda indicado por el usuario como **30** (sin unidad especificada).

| Orientación | Éxitos | Fallos | Tasa de éxito |
|---|---:|---:|---:|
| Horizontal | 13 | 2 | 86.7 % |
| Vertical | 10 | 5 | 66.7 % |
| Total | 23 | 7 | 76.7 % |

- Horizontal: O O O X O X O O O O O O O O O.
- Vertical: O O O X O X O X O X O O X O O.

O = agarre exitoso; X = fallo. Estos resultados cuantifican éxito de agarre, no repetibilidad posicional en milímetros. No se dispone de una captura identificada del firmware cargado durante esos ensayos para verificar que coincida byte a byte con esta copia.

## Operación y seguridad

Los parámetros son específicos del montaje físico. No operar el robot sin verificar finales de carrera, sentido de giro, límites y zona libre. Un mensaje de movimiento completado indica que el programa terminó de emitir los pulsos; no confirma la posición física de los motores. Tras una pérdida de referencia, recalibrar antes de continuar. Alimentar los servos con una fuente adecuada y mantener tierra común según el cableado del sistema.

## Licencias

La biblioteca de DFRobot conserva su licencia en `libraries/DFRobot_HuskylensV2/LICENSE.txt`. No se asigna una licencia adicional al código propio en esta publicación.
