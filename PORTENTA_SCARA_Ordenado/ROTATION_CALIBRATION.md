# Calibracion del giro de la garra

Configuracion en `PORTENTA_SCARA_Ordenado.ino`:

- `AUTO_ROTATION_COMPENSATION_ENABLED`: `true`, con la calibracion medida.
  Con `false`, Auto vuelve a comandos fijos vertical=40 y horizontal=100.
- `AUTO_ROTATION_S0_DEG`: `92.0`, medido con Y de la garra paralelo al ultimo
  eslabon y M1/M2 quietos.
- `AUTO_ROTATION_ZERO_X_OFFSET_DEG`: `-90.0`, porque el cero se midio sobre Y.
  Si se mide sobre X, usar `0.0` y el nuevo s0 correspondiente. Los sentidos
  opuestos de un mismo eje son equivalentes para estas condiciones paralelas.
- `AUTO_ROTATION_GAIN`: grados fisicos de la garra por grado de comando del
  servo, con signo. Valor `-1.0`: acople directo y giro horario al aumentar
  el comando, visto desde arriba. Confirmar ese punto de vista en la prueba.
- `AUTO_ROTATION_CAMERA_Y_ROBOT_DEG`: `90.0` para el mapa actual, donde Y de
  camara sigue la direccion +Y del robot.
- `AUTO_ROTATION_MIN_ANGLE` y `AUTO_ROTATION_MAX_ANGLE`: 0 y 180, limites del
  protocolo actual. Reducirlos si el recorrido mecanico util es menor.

Los valores medidos estan incorporados y la compensacion esta activa en el
codigo. Cargar la PMC para aplicarlos al robot.
La camara y el ESP de servos conservan el protocolo existente.

El calculo usa q1+q2 de la misma solucion IK elegida para la preposicion,
incluyendo las correcciones X/Y del pick. Para ancho>alto se exige Yg paralelo
a Yc; para alto>ancho se exige Xg paralelo a Yc. Se elige una solucion modulo
180 grados dentro del rango configurado y proxima al ultimo comando Auto.
Esto considera equivalentes los sentidos opuestos de un mismo eje de agarre.

Modelo en grados: theta_Xg = q1 + q2 + ZERO_X_OFFSET + gain * (servo - s0).
El comando despejado es s0 + (theta_Xg_deseado - q1 - q2 - ZERO_X_OFFSET) / gain,
con las alternativas de paralelismo modulo 180 grados.

La orden se envia antes de la preposicion, de modo que el giro pueda asentarse
durante el movimiento. Antes del descenso final se exige que haya transcurrido
`GRIPPER_SETTLE_MS` desde esa orden. Esta implementacion controla la orientacion
final de recogida; no mantiene orientacion constante durante toda la trayectoria
ni durante el traslado a place.

Diagnostico serial:

- `AUTO_ROTATION_COMPENSATED`: orientacion detectada, angulo del ultimo eslabon,
  orientacion deseada de Xg y comando calculado del servo.
- `AUTO_ROTATION_CALIBRATION_REQUIRED`: se activo sin configurar s0/gain.
- `AUTO_ROTATION_UNREACHABLE`: ninguna orientacion equivalente cabe en el rango.
- `AUTO_ROTATION_TARGET_INVALID`: orientacion desconocida o IK no disponible.

La orientacion sigue limitada a horizontal/vertical por ancho y altura;
las piezas inclinadas requieren una medicion adicional de su angulo.
