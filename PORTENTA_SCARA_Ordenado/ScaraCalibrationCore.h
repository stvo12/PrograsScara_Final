#pragma once

#include <Arduino.h>
#include <Arduino_PortentaMachineControl.h>

/* ============================================================
   SCARA Sync v6+
   Control de 3 motores paso a paso (DM542T) con Portenta
   Modos:
     - Modo 1 (Single): Un ciclo con A1/A2/A3 actuales.
     - Modo 2 (Repeat): Asistente para N repeticiones con compensación en M3.
     - Modo 3 (Points): Secuencia P1..P4 -> retorno a 0 con pausa entre puntos.
   Interacción por monitor serial (115200 bps). Paro de emergencia: 'S'.
   ============================================================ */

/* =================== ADVERTENCIA MECÁNICA ===================
   Mantener una separación mínima de ~4 cm entre el brazo y la mesa
   para evitar colisiones durante movimientos de prueba.
   ============================================================ */

/* =================== Parámetros rápidos ===================
   Variables de tiempos en milisegundos que pueden ajustarse por comandos:
   - M3_HOLD_MS : espera en posición objetivo de M3 antes de regresar.
   - PRE_M3_DELAY_MS : retardo tras mover M1+M2 y antes de iniciar M3.
   - REP_PAUSE_MS : pausa entre repeticiones (Modo 2) y entre P1..P4 (Modo 3).
   ============================================================ */
uint16_t           M3_HOLD_MS       = 6000;   // Espera en A3 antes de regresar (ajustable con H3)
uint16_t           PRE_M3_DELAY_MS  = 3000;   // Retardo previo a mover M3 (ajustable con D3)
uint16_t           REP_PAUSE_MS     = 3000;   // Pausa entre ciclos/puntos (ajustable con RP)

/* =================== Asignación de pines (Portenta MC: J6) ===================
   Cada motor usa un canal STEP (pulso) y DIR (dirección).
   ENA_CH es común para los tres drivers DM542T (entrada ENA+).
   Nota: el DM542T usa entradas optoacopladas y ENA es activo en nivel bajo.
   ============================================================================ */
// M1
const uint8_t STEP1_CH = 0;  // CH0 -> PUL+ M1
const uint8_t DIR1_CH  = 1;  // CH1 -> DIR+ M1
// M2
const uint8_t STEP2_CH = 2;  // CH2 -> PUL+ M2
const uint8_t DIR2_CH  = 3;  // CH3 -> DIR+ M2
// M3
const uint8_t STEP3_CH = 4;  // CH4 -> PUL+ M3
const uint8_t DIR3_CH  = 5;  // CH5 -> DIR+ M3
// ENA común (DM542T activo-bajo)
const uint8_t ENA_CH   = 6;  // CH6 -> ENA+

/* ============== Resolución y temporización de pasos ==============
   - STEPS_PER_REV: pasos por revolución del motor a la resolución de micro‐paso
     configurada por DIP (debe coincidir con el driver, p.ej. 1600 = 1/8 micro‐paso).
   - g_PULSE*_US: semiperiodo del pulso STEP en microsegundos. Periodo = 2*valor.
     Valores mayores -> menor velocidad (más lento y más torque disponible).
   - DIR_SETUP_US: tiempo mínimo tras cambiar DIR antes del primer pulso STEP.
   ============================================================== */
constexpr long   STEPS_PER_REV   = 1600;  // Debe corresponder al micro‐paso DIP del DM542T
unsigned int    g_PULSE12_US     = 500;   // M1 & M2: semiperiodo de STEP en us
unsigned int    g_PULSE3_US      = 450;   // M3: semiperiodo de STEP en us
constexpr unsigned int DIR_SETUP_US = 30; // Guard time tras cambiar DIR (us)

/* ============== Lógica de habilitación (ENA activo-bajo) ==============
   - ENA_LEVEL_ENABLE : nivel lógico para habilitar drivers (LOW en DM542T).
   - ENA_LEVEL_DISABLE: nivel lógico para deshabilitar (HIGH).
   ============================================================== */
const PinStatus ENA_LEVEL_ENABLE  = LOW;   // 0 = habilitado
const PinStatus ENA_LEVEL_DISABLE = HIGH;  // 1 = deshabilitado

/* ============== Convención de dirección por motor ==============
   DIR*_LEVEL_NEG define el nivel de DIR que se considera "negativo".
   Si un eje invierte su sentido real vs. esperado, ajustar estos niveles.
   ============================================================== */
const PinStatus DIR1_LEVEL_NEG = LOW;
const PinStatus DIR2_LEVEL_NEG = LOW;
const PinStatus DIR3_LEVEL_NEG = LOW;

/* ============== Estado global ==============
   - g_stop: bandera de paro de emergencia (forzada con comando 'S').
   - g_ena : último estado escrito a la salida ENA (habilitado/deshabilitado).
   ============================================================== */
volatile bool g_stop = false;
PinStatus     g_ena  = ENA_LEVEL_DISABLE;

/* ============== Objetivos de articulación (grados) y límites ==============
   A1/A2/A3 son ángulos de articulación (no del motor). Sus rangos son
   validados con clampAngleForMotor() antes de mover. R1/R2/R3 son las
   reducciones (grados de motor necesarios por cada grado de articulación).
   ============================================================== */
float g_A1 = 400.0f;  // M1: 0..600
float g_A2 = 300.0f;  // M2: 0..850
float g_A3 = -100.0f; // M3: -6800..1700

float g_R1 = 1.0f;    // Grados de motor por 1° de articulación en M1
float g_R2 = 1.0f;    // "" M2
float g_R3 = 1.0f;    // "" M3

/* ============== Compensación en Modo 2 ==============
   g_M3CompDeg: incremento acumulativo aplicado a A3 por repetición,
   útil para compensar deriva mecánica/sag en ciclos repetidos.
   ============================================================== */
float g_M3CompDeg = 1.0f;

/* ============== Buffer de línea para comandos por Serial ============== */
char   buf[96]; 
size_t blen = 0;

/* ============== Selección de modo de trabajo ==============
   MODE_SINGLE  (1): ciclo único con A1/A2/A3.
   MODE_REPEAT  (2): asistente para N repeticiones.
   MODE_POINTS  (3): ejecuta P1..P4 con retorno y pausa.
   ============================================================== */
enum Mode : uint8_t { MODE_SINGLE=1, MODE_REPEAT=2, MODE_POINTS=3 };
Mode g_mode = MODE_SINGLE;

/* ============== Estado del asistente del Modo 2 ==============
   RS_NEED_N: espera el número de repeticiones.
   RS_NEED_ANGLES: espera A1 A2 A3.
   RS_READY: preparado para ejecutar (Enter).
   ============================================================== */
enum RepStage : uint8_t { RS_IDLE=0, RS_NEED_N, RS_NEED_ANGLES, RS_READY };
RepStage g_repStage = RS_IDLE;
uint32_t g_repN = 0;
float    g_repA1=0, g_repA2=0, g_repA3=0;

/* ============== Modo 3 (cuatro puntos P1..P4) ==============
   - Estructura Pose con A1/A2/A3.
   - g_P[] contiene los cuatro puntos. g_P_set[] indica cuál está definido.
   - g_pStage controla la carga guiada de P1..P4 desde el serial.
   ============================================================== */
struct Pose { float A1, A2, A3; };
Pose g_P[4] = {
  {  0.f,   500.f, -6800.f },
  {  200.f, 400.f, -6800.f },   // Valores iniciales de ejemplo
  {  200.f, 850.f, -6800.f },
  {  400.f, 250.f, -6800.f }
};
bool  g_P_set[4] = { true, true, true, true };
enum PStage : uint8_t { PS_IDLE=0, PS_NEED_P1, PS_NEED_P2, PS_NEED_P3, PS_NEED_P4, PS_READY };
PStage g_pStage = PS_IDLE;

/* =================== Utilitarios numéricos ===================
   - motorDegToSteps: convierte grados de motor a pasos (según STEPS_PER_REV).
   - jointDegToSteps: convierte grados de articulación a pasos via relación R*.
   - setENA/toggleENA: controlan el pin ENA común y reportan estado por serial.
   - stepHigh/stepLow: helpers para emitir flanco en canal STEP.
   - quickSerialPoll: lee carácter 'S' para PARO inmediato y deshabilita ENA.
   - delayWithPoll: espera no bloqueante (sondeo de 'S' durante la espera).
   ============================================================ */
inline long motorDegToSteps(double motorDeg) {
  return (long) lround((motorDeg / 360.0) * (double)STEPS_PER_REV);
}
inline long jointDegToSteps(double jointDeg, double ratioMotorPerJoint) {
  return motorDegToSteps(jointDeg * ratioMotorPerJoint);
}
inline void setENA(PinStatus level) {
  g_ena = level;
  MachineControl_DigitalOutputs.write(ENA_CH, g_ena);
  if (g_ena == ENA_LEVEL_ENABLE) g_stop = false; // Al habilitar, limpiar paro.
  Serial.print(F("ENA = "));
  Serial.println((g_ena == ENA_LEVEL_ENABLE) ? F("0 (HABILITADO)") : F("1 (DESHABILITADO)"));
}
inline void toggleENA() { setENA((g_ena==ENA_LEVEL_ENABLE)?ENA_LEVEL_DISABLE:ENA_LEVEL_ENABLE); }

inline void stepHigh(uint8_t stepCh) { MachineControl_DigitalOutputs.write(stepCh, HIGH); }
inline void stepLow (uint8_t stepCh) { MachineControl_DigitalOutputs.write(stepCh, LOW);  }

inline void quickSerialPoll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='S' || c=='s') {              // Paro de emergencia por consola
      g_stop = true; 
      setENA(ENA_LEVEL_DISABLE);         // Desenergizar inmediatamente
      Serial.println(F(">>> PARO: ENA=1"));
    }
  }
}

/* =================== Espera con sondeo (no bloqueante) ===================
   Mantiene lazo de espera en ms pero monitorea 'S' y buffer serial.
   Retorna false si se activó g_stop durante la espera.
   ======================================================================= */
bool delayWithPoll(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {
    if (g_stop) return false;
    quickSerialPoll();
    delay(1);
  }
  return true;
}

/* =================== Movimiento de un solo eje ===================
   Genera 'steps' pulsos en stepCh, con DIR en dirLevel y semiperiodo pulseUs.
   Respeta g_stop y realiza sondeo de seguridad durante el lazo.
   Retorna la cantidad de pasos efectivamente emitidos.
   ================================================================= */
long moveFixedSingle(uint8_t stepCh, uint8_t dirCh, PinStatus dirLevel, long steps, unsigned int pulseUs) {
  if (steps <= 0) return 0;
  MachineControl_DigitalOutputs.write(stepCh, LOW);
  delayMicroseconds(5);
  MachineControl_DigitalOutputs.write(dirCh, dirLevel);
  delayMicroseconds(DIR_SETUP_US);

  long done = 0;
  for (long i = 0; i < steps; i++) {
    if (g_stop) break;
    stepHigh(stepCh);
    delayMicroseconds(pulseUs);
    stepLow(stepCh);
    delayMicroseconds(pulseUs);
    done++;
    quickSerialPoll();
  }
  return done;
}

/* =================== Movimiento sincronizado de dos ejes (M1 y M2) ===================
   Implementación tipo DDA/Bresenham:
   - Sincroniza la emisión de pasos para que ambos ejes terminen al mismo tiempo.
   - stepsA/B: número total de pasos requeridos en cada eje.
   - dirLevA/B: niveles de dirección.
   - pulseUs: semiperiodo de STEP compartido.
   - doneA/doneB: pasos efectivos ejecutados (salida).
   ================================================================================ */
void movePairSync(uint8_t stepA, uint8_t dirA, PinStatus dirLevA, long stepsA,
                  uint8_t stepB, uint8_t dirB, PinStatus dirLevB, long stepsB,
                  long &doneA, long &doneB, unsigned int pulseUs)
{
  doneA = 0; doneB = 0;
  if (stepsA <= 0 && stepsB <= 0) return;

  MachineControl_DigitalOutputs.write(stepA, LOW);
  MachineControl_DigitalOutputs.write(stepB, LOW);
  delayMicroseconds(5);
  MachineControl_DigitalOutputs.write(dirA, dirLevA);
  MachineControl_DigitalOutputs.write(dirB, dirLevB);
  delayMicroseconds(DIR_SETUP_US);

  long ticks = max(stepsA, stepsB);
  long accA = 0, accB = 0;

  for (long i = 0; i < ticks; i++) {
    if (g_stop) break;
    bool doA=false, doB=false;

    accA += stepsA; if (accA >= ticks) { doA = true; accA -= ticks; }
    accB += stepsB; if (accB >= ticks) { doB = true; accB -= ticks; }

    if (doA || doB) {
      if (doA) stepHigh(stepA);
      if (doB) stepHigh(stepB);
      delayMicroseconds(pulseUs);
      if (doA) { stepLow(stepA); doneA++; }
      if (doB) { stepLow(stepB); doneB++; }
      delayMicroseconds(pulseUs);
    } else {
      // Si ningún eje requiere paso en este tick, mantener timing.
      delayMicroseconds(pulseUs*2);
    }
    quickSerialPoll();
  }
}

/* =================== Validación de límites por motor ===================
   Limita el valor angular de articulación 'v' a los rangos seguros por eje.
   ===================================================================== */
float clampAngleForMotor(uint8_t id, float v) {
  if      (id == 1) { if (v < 0.f) v = 0.f;     if (v > 600.f)  v = 600.f;  }
  else if (id == 2) { if (v < 0.f) v = 0.f;     if (v > 850.f)  v = 850.f;  }
  else if (id == 3) { if (v < -6800.f) v=-6800.f; if (v > 1700.f) v = 1700.f; }
  return v;
}

/* =================== Ciclo completo con ángulos dados ===================
   Secuencia:
     1) M1 y M2 en sincronía: 0 -> A1/A2
     2) Retardo PRE_M3_DELAY_MS
     3) M3: 0 -> A3
     4) Espera M3_HOLD_MS en la posición de A3
     5) M3: A3 -> 0
     6) M1 y M2 sincronizados: A1/A2 -> 0
   Condiciones:
     - Requiere ENA habilitado (E0).
     - Respeta paro de emergencia en todo momento.
   ====================================================================== */
void runCycleSyncAngles(float A1, float A2, float A3) {
  if (g_ena != ENA_LEVEL_ENABLE) {
    Serial.println(F("No puedo mover: ENA=1 (OFF). Usa 'E' o 'E0' para habilitar."));
    return;
  }
  g_stop = false;

  // Clampeo de seguridad a rangos por eje
  A1 = clampAngleForMotor(1, A1);
  A2 = clampAngleForMotor(2, A2);
  A3 = clampAngleForMotor(3, A3);

  // Conversión a pasos según reducción
  long s1 = labs(jointDegToSteps((double)A1, (double)g_R1));
  long s2 = labs(jointDegToSteps((double)A2, (double)g_R2));
  long s3 = labs(jointDegToSteps((double)A3, (double)g_R3));

  // Determinar niveles DIR de ida y regreso
  PinStatus dir1_go = (A1 < 0.0f) ? DIR1_LEVEL_NEG : (DIR1_LEVEL_NEG==HIGH?LOW:HIGH);
  PinStatus dir2_go = (A2 < 0.0f) ? DIR2_LEVEL_NEG : (DIR2_LEVEL_NEG==HIGH?LOW:HIGH);
  PinStatus dir3_go = (A3 < 0.0f) ? DIR3_LEVEL_NEG : (DIR3_LEVEL_NEG==HIGH?LOW:HIGH);

  PinStatus dir1_back = (dir1_go==HIGH?LOW:HIGH);
  PinStatus dir2_back = (dir2_go==HIGH?LOW:HIGH);
  PinStatus dir3_back = (dir3_go==HIGH?LOW:HIGH);

  // 1) M1 y M2 sincronizados hasta A1/A2
  long out1=0, out2=0;
  movePairSync(STEP1_CH, DIR1_CH, dir1_go, s1,
               STEP2_CH, DIR2_CH, dir2_go, s2,
               out1, out2, g_PULSE12_US);
  if (g_stop) { Serial.println(F("Abortado en etapa 1.")); return; }

  // 2) Espera antes de mover M3
  if (!delayWithPoll(PRE_M3_DELAY_MS)) { Serial.println(F("Abortado en espera pre-M3.")); return; }

  // 3) M3 hacia A3
  long out3 = moveFixedSingle(STEP3_CH, DIR3_CH, dir3_go, s3, g_PULSE3_US);
  if (g_stop) { Serial.println(F("Abortado en etapa 2.")); return; }

  // 4) Hold en A3
  if (!delayWithPoll(M3_HOLD_MS)) { Serial.println(F("Abortado en hold.")); return; }

  // 5) Retorno M3 a 0
  long back3 = moveFixedSingle(STEP3_CH, DIR3_CH, dir3_back, out3, g_PULSE3_US);
  if (g_stop) { Serial.println(F("Abortado en etapa 4.")); return; }

  // 6) Retorno M1 y M2 a 0 en sincronía
  long b1=0, b2=0;
  movePairSync(STEP1_CH, DIR1_CH, dir1_back, out1,
               STEP2_CH, DIR2_CH, dir2_back, out2,
               b1, b2, g_PULSE12_US);
  if (g_stop) { Serial.println(F("Abortado en etapa 5.")); return; }

  Serial.println(F("Ciclo OK."));
}

/* =================== Modo 1 (Single) ===================
   Ejecuta un ciclo usando los valores actuales g_A1, g_A2, g_A3.
   ====================================================== */
void runCycleSync() { runCycleSyncAngles(g_A1, g_A2, g_A3); }

/* =================== Parseo de argumentos por Serial ===================
   - parseFloatAfter : lee un float tras espacios/igual.
   - parseUIntAfter_u32/u16 : lee enteros sin signo (32/16 bits).
   - parse3floats : lee tres floats separados por espacios (A1 A2 A3).
   ====================================================================== */
bool parseFloatAfter(const char* s, float &val) {
  while (*s==' '||*s=='\t'||*s=='=') s++;
  char* endp=nullptr;
  val = strtof(s,&endp);
  return (endp!=s);
}
bool parseUIntAfter_u32(const char* s, uint32_t &val) {
  while (*s==' '||*s=='\t'||*s=='=') s++;
  char* endp=nullptr;
  long tmp = strtol(s,&endp,10);
  if (endp==s || tmp<=0) return false;
  val = (uint32_t)tmp;
  return true;
}
bool parseUIntAfter_u16(const char* s, uint16_t &val) {
  uint32_t tmp;
  if (!parseUIntAfter_u32(s, tmp)) return false;
  if (tmp > 65535) tmp = 65535;
  val = (uint16_t)tmp;
  return true;
}
bool parse3floats(const char* s, float &a, float &b, float &c) {
  char* endp=nullptr;
  while (*s==' '||*s=='\t') s++;
  a = strtof(s,&endp); if (endp==s) return false; s=endp;
  while (*s==' '||*s=='\t') s++;
  b = strtof(s,&endp); if (endp==s) return false; s=endp;
  while (*s==' '||*s=='\t') s++;
  c = strtof(s,&endp); if (endp==s) return false;
  return true;
}

/* =================== Estado & Ayuda por Serial ===================
   - printPositions : imprime P1..P4 en formato A1/A2/A3.
   - printStatus : imprime modo, ENA, A*, R*, velocidades, tiempos, etc.
   - help : lista de comandos disponibles y breve guía de modos.
   ================================================================ */
void printPositions() {
  for (int i=0;i<4;i++){
    Serial.print(F("P")); Serial.print(i+1); Serial.print(F(": "));
    Serial.print(g_P[i].A1); Serial.print(F("/"));
    Serial.print(g_P[i].A2); Serial.print(F("/"));
    Serial.println(g_P[i].A3);
  }
}
void printStatus() {
  Serial.println(F("=== Estado ==="));
  Serial.print(F("Modo: ")); 
  if      (g_mode==MODE_SINGLE)  Serial.println(F("1 (Single)"));
  else if (g_mode==MODE_REPEAT)  Serial.println(F("2 (Repeticiones)"));
  else                           Serial.println(F("3 (Puntos)"));
  Serial.print(F("ENA: ")); Serial.println((g_ena==ENA_LEVEL_ENABLE)?F("ON(0)"):F("OFF(1)"));
  Serial.print(F("A1=")); Serial.print(g_A1); Serial.print(F("°  R1=")); Serial.println(g_R1,4);
  Serial.print(F("A2=")); Serial.print(g_A2); Serial.print(F("°  R2=")); Serial.println(g_R2,4);
  Serial.print(F("A3=")); Serial.print(g_A3); Serial.print(F("°  R3=")); Serial.println(g_R3,4);
  Serial.print(F("V12(us)=")); Serial.print(g_PULSE12_US);
  Serial.print(F("  V3(us)=")); Serial.println(g_PULSE3_US);
  Serial.print(F("HoldM3(ms)=")); Serial.print(M3_HOLD_MS);
  Serial.print(F("  PreM3(ms)=")); Serial.print(PRE_M3_DELAY_MS);
  Serial.print(F("  RepPause(ms)=")); Serial.println(REP_PAUSE_MS);
  Serial.print(F("CompM3(deg/rep)=")); Serial.println(g_M3CompDeg,4);
  if (g_mode==MODE_REPEAT) {
    Serial.print(F("RepStage=")); Serial.print((int)g_repStage);
    Serial.print(F("  N=")); Serial.print(g_repN);
    Serial.print(F("  A1/A2/A3=")); Serial.print(g_repA1); Serial.print('/');
    Serial.print(g_repA2); Serial.print('/'); Serial.println(g_repA3);
  }
  if (g_mode==MODE_POINTS) {
    Serial.println(F("Posiciones P1..P4 (A1/A2/A3):"));
    printPositions();
    Serial.print(F("PStage=")); Serial.println((int)g_pStage);
  }
}
void help() {
  Serial.println(F("Comandos:"));
  Serial.println(F("  Enter     : Modo 1 -> un ciclo;  Modo 2 -> según asistente;  Modo 3 -> ejecuta P1..P4 si READY"));
  Serial.println(F("  M1 / M2 / M3 : Cambiar a Modo 1 (Single) / 2 (Repeticiones) / 3 (Puntos)"));
  Serial.println(F("  A1/A2/A3 <deg> : (Modo 1) Cambia ángulo (M1:0..600, M2:0..850, M3:-6800..1700)"));
  Serial.println(F("  R1/R2/R3 <ratio> : Grados motor / 1° articulación (>0)"));
  Serial.println(F("  V12 <us>  : Velocidad M1/M2 (mitad período en us)"));
  Serial.println(F("  V3  <us>  : Velocidad M3 (mitad período en us)"));
  Serial.println(F("  D3  <ms>  : Delay antes de mover M3 (default 3000 ms)"));
  Serial.println(F("  H3  <ms>  : Hold sobre A3 antes de regresar (default 600 ms)"));
  Serial.println(F("  RP  <ms>  : Pausa entre repeticiones (M2) y entre puntos (M3)"));
  Serial.println(F("  AC  <deg> : Compensación M3 por repetición en Modo 2 (deg/rep).  AC? muestra valor"));
  Serial.println(F("  P1/P2/P3/P4 <A1> <A2> <A3>  : Define una pose;  P1? / P? para ver"));
  Serial.println(F("  E / E0 / E1 : ENA toggle / ON / OFF"));
  Serial.println(F("  S        : PARO de emergencia (desenergiza ENA)"));
  Serial.println(F("  ? / h    : Estado / Ayuda"));
  Serial.println(F(""));
  Serial.println(F("Modo 2: al entrar, ingresa N y luego A1 A2 A3; Enter ejecuta."));
  Serial.println(F("Modo 3: al entrar, ingresa P1..P4 (si hace falta); Enter ejecuta P1->0, P2->0, P3->0, P4->0 con pausa RP."));
}

/* =================== Modo 2 (Repeticiones) ===================
   Flujo:
     - RS_NEED_N      : ingresar N.
     - RS_NEED_ANGLES : ingresar A1 A2 A3.
     - RS_READY       : Enter ejecuta N ciclos, con A3 compensado por AC*k.
   ============================================================ */
void enterMode2() {
  g_mode = MODE_REPEAT;
  g_repStage = RS_NEED_N;
  g_repN = 0;
  g_repA1 = g_repA2 = g_repA3 = 0.0f;
  Serial.println(F("MODO 2: Repeticiones."));
  Serial.println(F("Ingresa N (repeticiones) y presiona Enter."));
}
void runCycleSyncAngles(float A1, float A2, float A3); // forward declaration

void handleMode2Line() {
  if (g_repStage == RS_NEED_N) {
    uint32_t n;
    if (parseUIntAfter_u32(buf, n)) {
      g_repN = n;
      g_repStage = RS_NEED_ANGLES;
      Serial.print(F("N=")); Serial.println(g_repN);
      Serial.println(F("Ingresa A1 A2 A3 (grados, separados por espacio) y presiona Enter."));
    } else {
      Serial.println(F("Valor inválido. Ingresa un entero > 0 para N."));
    }
    return;
  }
  if (g_repStage == RS_NEED_ANGLES) {
    float a1,a2,a3;
    if (parse3floats(buf, a1,a2,a3)) {
      g_repA1 = clampAngleForMotor(1, a1);
      g_repA2 = clampAngleForMotor(2, a2);
      g_repA3 = clampAngleForMotor(3, a3);
      g_repStage = RS_READY;
      Serial.print(F("A1/A2/A3 = ")); Serial.print(g_repA1); Serial.print('/');
      Serial.print(g_repA2); Serial.print('/'); Serial.println(g_repA3);
      Serial.println(F("READY: Enter ejecuta; 'N' reingresa N; 'A' reingresa ángulos."));
    } else {
      Serial.println(F("Formato inválido. Ej: 20 35 -15"));
    }
    return;
  }
  if (g_repStage == RS_READY) {
    if (blen==0) {
      if (g_ena != ENA_LEVEL_ENABLE) { Serial.println(F("ENA=1 (OFF). Usa 'E0' para habilitar.")); return; }
      Serial.print(F("Ejecutando ")); Serial.print(g_repN); Serial.println(F(" repetición(es)..."));
      for (uint32_t k=1; k<=g_repN; ++k) {
        float A3k = clampAngleForMotor(3, g_repA3 + (float)k * g_M3CompDeg);
        Serial.print(F("  Repetición ")); Serial.print(k); Serial.print(F("/")); Serial.print(g_repN);
        Serial.print(F("  | A3k=")); Serial.println(A3k);
        runCycleSyncAngles(g_repA1, g_repA2, A3k);
        if (g_stop) { Serial.println(F(">>> PARO durante repeticiones.")); break; }
        if (k < g_repN) { 
          if (!delayWithPoll(REP_PAUSE_MS)) { Serial.println(F("Abortado en pausa entre repeticiones.")); break; } 
        }
      }
      Serial.println(F("Fin de repeticiones. (Enter=otra vez; 'N'/'A' para cambiar)"));
      return;
    } else if ((blen==1) && (buf[0]=='N' || buf[0]=='n')) {
      g_repStage = RS_NEED_N;
      Serial.println(F("Ingresa N (repeticiones) y presiona Enter."));
      return;
    } else if ((blen==1) && (buf[0]=='A' || buf[0]=='a')) {
      g_repStage = RS_NEED_ANGLES;
      Serial.println(F("Ingresa A1 A2 A3 (grados) y presiona Enter."));
      return;
    } else {
      Serial.println(F("READY: Enter ejecuta, 'N' cambia N, 'A' cambia ángulos."));
      return;
    }
  }
  Serial.println(F("Modo 2: estado inválido. Escribe M2 para reiniciar el asistente."));
}

/* =================== Modo 3 (Puntos: P1..P4) ===================
   - enterMode3: si P1..P4 están definidos, queda READY, de lo contrario
     solicita cada punto secuencialmente.
   - handleMode3Line: en READY, Enter ejecuta P1->0, P2->0, P3->0, P4->0
     con pausa REP_PAUSE_MS entre puntos.
   =============================================================== */
void enterMode3() {
  g_mode = MODE_POINTS;
  bool allset = g_P_set[0] && g_P_set[1] && g_P_set[2] && g_P_set[3];
  if (allset) {
    g_pStage = PS_READY;
    Serial.println(F("MODO 3: Puntos (P1..P4 listos). Enter ejecuta secuencia. Usa P1/P2/P3/P4 <A1 A2 A3> para cambiar."));
  } else {
    g_pStage = PS_NEED_P1;
    Serial.println(F("MODO 3: Define P1 <A1 A2 A3> y presiona Enter."));
  }
}
void handleMode3Line() {
  // Función lambda para parsear una línea como Pn <A1 A2 A3>
  auto parsePoseLine = [&](int idx)->bool{
    float a1,a2,a3;
    if (!parse3floats(buf,a1,a2,a3)) return false;
    g_P[idx].A1 = clampAngleForMotor(1,a1);
    g_P[idx].A2 = clampAngleForMotor(2,a2);
    g_P[idx].A3 = clampAngleForMotor(3,a3);
    g_P_set[idx] = true;
    return true;
  };

  // Carga guiada de P1..P4
  if (g_pStage == PS_NEED_P1) {
    if (parsePoseLine(0)) { Serial.println(F("P1 ok. Ahora P2 <A1 A2 A3> y Enter.")); g_pStage = PS_NEED_P2; }
    else Serial.println(F("Formato inválido. Ej: 20 10 -5"));
    return;
  }
  if (g_pStage == PS_NEED_P2) {
    if (parsePoseLine(1)) { Serial.println(F("P2 ok. Ahora P3 <A1 A2 A3> y Enter.")); g_pStage = PS_NEED_P3; }
    else Serial.println(F("Formato inválido. Ej: 30 12 -8"));
    return;
  }
  if (g_pStage == PS_NEED_P3) {
    if (parsePoseLine(2)) { Serial.println(F("P3 ok. Ahora P4 <A1 A2 A3> y Enter.")); g_pStage = PS_NEED_P4; }
    else Serial.println(F("Formato inválido. Ej: 40 15 -10"));
    return;
  }
  if (g_pStage == PS_NEED_P4) {
    if (parsePoseLine(3)) { 
      g_pStage = PS_READY; 
      Serial.println(F("P4 ok. READY. Enter ejecuta P1->0, P2->0, P3->0, P4->0 (pausa RP)."));
    } else Serial.println(F("Formato inválido. Ej: 50 18 -12"));
    return;
  }

  // Ejecución en READY con Enter
  if (g_pStage == PS_READY) {
    if (blen==0) {
      if (g_ena != ENA_LEVEL_ENABLE) { Serial.println(F("ENA=1 (OFF). Usa 'E0' para habilitar.")); return; }
      Serial.println(F("Ejecutando secuencia P1..P4:"));
      for (int i=0;i<4;i++){
        Serial.print(F("  P")); Serial.print(i+1); Serial.print(F(" -> 0  | A1/A2/A3="));
        Serial.print(g_P[i].A1); Serial.print(F("/"));
        Serial.print(g_P[i].A2); Serial.print(F("/"));
        Serial.println(g_P[i].A3);
        runCycleSyncAngles(g_P[i].A1, g_P[i].A2, g_P[i].A3);
        if (g_stop) { Serial.println(F(">>> PARO durante Modo 3.")); return; }
        if (i<3) { 
          if (!delayWithPoll(REP_PAUSE_MS)) { Serial.println(F("Abortado en pausa entre puntos.")); return; } 
        }
      }
      Serial.println(F("Fin secuencia Modo 3. (Enter=otra vez; usa P1..P4 para cambiar)"));
      return;
    } else {
      Serial.println(F("READY: Enter ejecuta. Usa P1..P4 <A1 A2 A3> para actualizar o P? para ver todo."));
      return;
    }
  }

  Serial.println(F("Modo 3: estado inválido. Escribe M3 para reiniciar el asistente."));
}

/* =================== Comandos P1..P4 (globales) ===================
   Permiten:
     - 'P?'        : listar P1..P4.
     - 'Pn?'       : ver pose individual.
     - 'Pn a b c'  : definir pose (A1=a, A2=b, A3=c).
   Pueden usarse en cualquier modo.
   ================================================================ */
bool handlePCommandInline() {
  if (blen==1 && (buf[0]=='P'||buf[0]=='p')) { // Ayuda corta
    Serial.println(F("Uso: P1/P2/P3/P4 <A1 A2 A3>  |  P1?  |  P?"));
    return true;
  }
  if ((blen==2) && (buf[0]=='P'||buf[0]=='p') && buf[1]=='?') {
    printPositions(); 
    return true;
  }
  if ((blen==3) && (buf[0]=='P'||buf[0]=='p') && (buf[1]>='1'&&buf[1]<='4') && buf[2]=='?') {
    int idx = buf[1]-'1';
    Serial.print(F("P")); Serial.print(idx+1); Serial.print(F(" = "));
    Serial.print(g_P[idx].A1); Serial.print('/');
    Serial.print(g_P[idx].A2); Serial.print('/');
    Serial.println(g_P[idx].A3);
    return true;
  }
  if ((blen>=2) && (buf[0]=='P'||buf[0]=='p') && (buf[1]>='1'&&buf[1]<='4')) {
    float a1,a2,a3;
    if (parse3floats(buf+2, a1,a2,a3)) {
      int idx = buf[1]-'1';
      g_P[idx].A1 = clampAngleForMotor(1,a1);
      g_P[idx].A2 = clampAngleForMotor(2,a2);
      g_P[idx].A3 = clampAngleForMotor(3,a3);
      g_P_set[idx] = true;
      Serial.print(F("P")); Serial.print(idx+1); Serial.print(F(" actualizado a "));
      Serial.print(g_P[idx].A1); Serial.print('/');
      Serial.print(g_P[idx].A2); Serial.print('/');
      Serial.println(g_P[idx].A3);
    } else {
      Serial.println(F("Uso: Pn <A1 A2 A3>   (ej. P1 20 10 -5)"));
    }
    return true;
  }
  return false;
}

/* =================== Procesamiento de línea de comandos ===================
   Orden de evaluación:
     - Paro ('S'), cambio de modo ('M1/M2/M3'), ENA ('E/E0/E1').
     - Velocidad ('V12', 'V3'), tiempos ('D3', 'H3', 'RP'), compensación ('AC').
     - Ángulos (A1/A2/A3) en Modo 1, reducciones (R1/R2/R3).
     - P1..P4 (en cualquier modo).
     - Estado ('?') y ayuda ('h').
     - Enter: ejecuta según modo (Single/Repeat/Points).
   Si el comando no coincide, muestra lista resumida.
   ======================================================================= */
void processLine() {
  buf[blen] = '\0';

  // Paro de emergencia
  if ((blen==1) && (buf[0]=='S'||buf[0]=='s')) {
    g_stop = true; setENA(ENA_LEVEL_DISABLE);
    Serial.println(F(">>> PARO: ENA=1, movimiento abortado."));
    blen = 0; return;
  }

  // Cambiar modo
  if ((blen==2) && (buf[0]=='M'||buf[0]=='m') && (buf[1]=='1')) { 
    g_mode = MODE_SINGLE; g_repStage = RS_IDLE; g_pStage = PS_IDLE; 
    Serial.println(F("Modo 1 (Single). Enter ejecuta un ciclo.")); 
    blen=0; return; 
  }
  if ((blen==2) && (buf[0]=='M'||buf[0]=='m') && (buf[1]=='2')) { enterMode2(); blen=0; return; }
  if ((blen==2) && (buf[0]=='M'||buf[0]=='m') && (buf[1]=='3')) { enterMode3(); blen=0; return; }

  // ENA (toggle/ON/OFF)
  if ((blen==1) && (buf[0]=='E'||buf[0]=='e')) { toggleENA(); blen=0; return; }
  if ((blen==2) && (buf[0]=='E'||buf[0]=='e') && (buf[1]=='0'||buf[1]=='1')) { 
    setENA((buf[1]=='0') ? ENA_LEVEL_ENABLE : ENA_LEVEL_DISABLE); blen=0; return; 
  }

  // Velocidades (V12/V3)
  if ((blen>=4) && (buf[0]=='V'||buf[0]=='v')) {
    if ((buf[1]=='1' && buf[2]=='2') || buf[1]=='3') {
      uint32_t v32;
      const char* p = (buf[1]=='3') ? buf+2 : buf+3;
      if (parseUIntAfter_u32(p, v32)) {
        if (v32 < 50) v32 = 50; if (v32 > 10000) v32 = 10000;
        if (buf[1]=='3') { g_PULSE3_US  = (unsigned int)v32;  Serial.print(F("Vel M3 (V3) = "));  Serial.print(g_PULSE3_US);  Serial.println(F(" us")); }
        else             { g_PULSE12_US = (unsigned int)v32;  Serial.print(F("Vel M1/M2 (V12) = ")); Serial.print(g_PULSE12_US); Serial.println(F(" us")); }
      } else {
        Serial.println(F("Uso: V12 <us>  |  V3 <us>"));
      }
      blen=0; return;
    }
  }

  // Tiempos (D3, RP)
  if ((blen>=2) && (buf[0]=='D'||buf[0]=='d') && (buf[1]=='3')) {
    uint16_t ms;
    if (parseUIntAfter_u16(buf+2, ms)) {
      PRE_M3_DELAY_MS = ms;
      Serial.print(F("PreM3 delay = ")); Serial.print(PRE_M3_DELAY_MS); Serial.println(F(" ms"));
    } else {
      Serial.println(F("Uso: D3 <ms>"));
    }
    blen=0; return;
  }
  if ((blen>=2) && (buf[0]=='R'||buf[0]=='r') && (buf[1]=='P')) {
    uint16_t ms;
    if (parseUIntAfter_u16(buf+2, ms)) {
      REP_PAUSE_MS = ms;
      Serial.print(F("Pausa (RP) = ")); Serial.print(REP_PAUSE_MS); Serial.println(F(" ms"));
    } else {
      Serial.println(F("Uso: RP <ms>"));
    }
    blen=0; return;
  }

  // Hold M3 (H3)
  if ((blen>=2) && (buf[0]=='H'||buf[0]=='h') && (buf[1]=='3')) {
    if (blen==3 && buf[2]=='?') {
      Serial.print(F("HoldM3(ms) = ")); Serial.println(M3_HOLD_MS);
      blen=0; return;
    }
    uint16_t ms;
    if (parseUIntAfter_u16(buf+2, ms)) {
      M3_HOLD_MS = ms;
      Serial.print(F("HoldM3 = ")); Serial.print(M3_HOLD_MS); Serial.println(F(" ms"));
    } else {
      Serial.println(F("Uso: H3 <ms>   (ej. H3 1200)   o H3?"));
    }
    blen=0; return;
  }

  // Compensación M3 (AC)
  if ((blen>=2) && (buf[0]=='A' || buf[0]=='a') && (buf[1]=='C' || buf[1]=='c')) {
    if (blen==2) {
      Serial.print(F("CompM3(deg/rep) actual = ")); Serial.println(g_M3CompDeg,4);
      Serial.println(F("Uso: AC <deg>   (ej. AC 1.0)   o AC?"));
      blen=0; return;
    }
    if (blen==3 && buf[2]=='?') {
      Serial.print(F("CompM3(deg/rep) = ")); Serial.println(g_M3CompDeg,4);
      blen=0; return;
    }
    float v;
    if (parseFloatAfter(buf+2, v)) {
      g_M3CompDeg = v;
      Serial.print(F("CompM3(deg/rep) actualizado a ")); Serial.println(g_M3CompDeg,4);
    } else {
      Serial.println(F("Uso: AC <deg>   (ej. AC 1.0)   o AC?"));
    }
    blen=0; return;
  }

  // Ángulos (A1/A2/A3) sólo en Modo 1
  if ((blen>=2) && (buf[0]=='A'||buf[0]=='a')) {
    if (g_mode!=MODE_SINGLE) { Serial.println(F("Estás en Modo 2/3. Usa el asistente o cambia a M1.")); blen=0; return; }
    if (buf[1]>='1' && buf[1]<='3') {
      float v;
      if (parseFloatAfter(buf+2, v)) {
        uint8_t id = buf[1]-'0';
        if      (id==1){ g_A1 = clampAngleForMotor(1,v); Serial.print(F("M1: A1=")); Serial.print(g_A1); Serial.println(F("° (0..600)")); }
        else if (id==2){ g_A2 = clampAngleForMotor(2,v); Serial.print(F("M2: A2=")); Serial.print(g_A2); Serial.println(F("° (0..850)")); }
        else            { g_A3 = clampAngleForMotor(3,v); Serial.print(F("M3: A3=")); Serial.print(g_A3); Serial.println(F("° (-6800..1700)")); }
      } else {
        Serial.println(F("Uso: A1/A2/A3 <deg>"));
      }
      blen=0; return;
    }
  }

  // Reducciones (R1/R2/R3)
  if ((blen>=2) && (buf[0]=='R'||buf[0]=='r')) {
    if (buf[1]>='1' && buf[1]<='3') {
      float v;
      if (parseFloatAfter(buf+2, v) && v > 0.f) {
        uint8_t id = buf[1]-'0';
        if      (id==1){ g_R1 = v; Serial.print(F("M1: R1=")); Serial.println(g_R1,6); }
        else if (id==2){ g_R2 = v; Serial.print(F("M2: R2=")); Serial.println(g_R2,6); }
        else            { g_R3 = v; Serial.print(F("M3: R3=")); Serial.println(g_R3,6); }
      } else {
        Serial.println(F("Uso: R1/R2/R3 <ratio> (>0)"));
      }
      blen=0; return;
    }
  }

  // P1..P4 (siempre disponible)
  if (handlePCommandInline()) { blen=0; return; }

  // Estado / Ayuda
  if ((blen==1) && (buf[0]=='?' ))              { printStatus(); blen=0; return; }
  if ((blen==1) && (buf[0]=='H'||buf[0]=='h'))  { help();        blen=0; return; }

  // Enter: ejecuta según modo
  if (blen==0) {
    if (g_mode==MODE_SINGLE)      { runCycleSync(); return; }
    else if (g_mode==MODE_REPEAT) { handleMode2Line(); return; }
    else if (g_mode==MODE_POINTS) { handleMode3Line(); return; }
  }

  // Si nada coincidió, ayuda resumida
  if (g_mode==MODE_REPEAT) { handleMode2Line(); blen=0; return; }
  if (g_mode==MODE_POINTS) { handleMode3Line(); blen=0; return; }

  Serial.println(F("Cmd inválido. Usa M1/M2/M3, Enter, A1/A2/A3, R1/R2/R3, V12/V3, D3, H3, RP, AC, P1..P4, E/E0/E1, S, ?, h"));
  blen = 0;
}

/* =================== Setup ===================
   - Inicializa Serial a 115200 bps.
   - Configura salidas digitales del Portenta MC (J6) en LOW.
   - Deshabilita ENA por seguridad (drivers OFF).
   - Imprime banner, ayuda y estado inicial.
   ============================================ */
void scara_init() {
  //Serial.begin(115200);
  //while (!Serial) {}

  MachineControl_DigitalOutputs.begin(true);
  MachineControl_DigitalOutputs.writeAll(0);

  MachineControl_DigitalOutputs.write(STEP1_CH, LOW);
  MachineControl_DigitalOutputs.write(DIR1_CH,  LOW);
  MachineControl_DigitalOutputs.write(STEP2_CH, LOW);
  MachineControl_DigitalOutputs.write(DIR2_CH,  LOW);
  MachineControl_DigitalOutputs.write(STEP3_CH, LOW);
  MachineControl_DigitalOutputs.write(DIR3_CH,  LOW);

  setENA(ENA_LEVEL_DISABLE);  // Inicio seguro (drivers deshabilitados)

  Serial.println(F("SCARA Sync v6+: modos 1/2/3, P1..P4 con RP, pre-M3 (D3), hold M3 (H3), AC Modo2, paro S, ENA por comandos."));
  //help(); 
  //printStatus();
}

/* =================== Loop principal ===================
   Lector de consola:
     - Acumula caracteres en 'buf' hasta CR/LF.
     - Al recibir CR/LF, llama a processLine() y limpia el buffer.
   ===================================================== */


void scara_serial_loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\r' || c=='\n') processLine();
    else if (blen < sizeof(buf)-1) buf[blen++] = c;
  }
}
