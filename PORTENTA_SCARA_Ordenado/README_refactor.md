# Refactor PMC propuesto

Este paquete reorganiza la progra de la PMC para que el `.ino` quede más limpio y la lógica quede distribuida por módulos.

## Idea general

- La **configuración editable** se quedó en `PORTENTA_SCARA_I2C_CONTROL.ino`.
- La lógica pesada se movió a:
  - `esp_interface.*`
  - `motion_service.*`
  - `calibration_service.*`
  - `command_dispatcher.*`
  - `status_report.*`
  - `gui_protocol.*`
  - `sequence_runner.*`
  - `robot_state.*`

## Qué se reutilizó

Se conservaron:
- `ScaraCalibrationCore.h`
- `kinematics.*`
- `motor_map.*`
- `safety.*`
- `scara_pose.*`

## Notas importantes

1. El módulo `gui_protocol.*` ya deja un canal Serial básico para futura GUI.
2. `sequence_runner.*` quedó como base para pick and place, pero aún tiene placeholders de gripper.
