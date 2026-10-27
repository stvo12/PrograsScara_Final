#pragma once

#include <Arduino.h>

// =====================================================
// ESTADO DE UNA PIEZA EN LA BANDA
// =====================================================

enum PieceOrientation : uint8_t {
  PIECE_ORIENTATION_UNKNOWN = 0,
  PIECE_ORIENTATION_HORIZONTAL,
  PIECE_ORIENTATION_VERTICAL
};

struct TrackedPiece {

  bool active = false;

  uint8_t id = 0;

  // Posicion inicial detectada por la camara
  // expresada en coordenadas fisicas de la banda.
  float initialLongitudinalMm = 0.0f;
  float transverseMm = 0.0f;
  float cameraXmm = 0.0f;
  float cameraYmm = 0.0f;
  int16_t widthPx = 0;
  int16_t heightPx = 0;
  PieceOrientation orientation = PIECE_ORIENTATION_UNKNOWN;

  // Encoder al momento de deteccion.
  int32_t encoderCountAtDetection = 0;

  // Tiempo de deteccion.
  uint32_t detectionTimeMs = 0;
};

enum TeachMapStatus : uint8_t {
  TEACH_MAP_OK = 0,
  TEACH_MAP_NO_PIECE,
  TEACH_MAP_INVALID_COORDS,
  TEACH_MAP_OUT_LEFT,
  TEACH_MAP_OUT_RIGHT,
  TEACH_MAP_OUT_LONGITUDINAL
};


// =====================================================
// API
// =====================================================

void beltTrackingInit();

void updateBeltEstimator();

void registerDetectedPiece(
    uint8_t id,
    float cameraXmm,
    float cameraYmm,
    int16_t widthPx,
    int16_t heightPx
);

bool hasTrackedPiece();

uint8_t getTrackedPieceId();
PieceOrientation getTrackedPieceOrientation();

void clearTrackedPiece();

float getTrackedPieceCurrentLongitudinalMm();

float getTrackedPieceTransverseMm();

float getBeltSpeedMmS();
int32_t getBeltEncoderCount();
int32_t getBeltEncoderRevolutions();
float getBeltTraveledMm();
bool isBeltEncoderMoving();
float getTrackedPieceDistanceToPickMm();
float getPickLongitudinalTargetMm();

float predictTrackedPieceLongitudinalMm(
    float futureTimeSeconds
);

bool trackedPieceToRobotMock(
    float& robotX,
    float& robotY,
    long& mockM1Steps,
    long& mockM2Steps
);

bool trackedPieceToRobotTaught(
    float& robotX,
    float& robotY
);
TeachMapStatus getTeachMapStatus();
const char* teachMapStatusText(TeachMapStatus status);

bool trackedPieceToRobot(
    float futureTimeSeconds,
    float& robotX,
    float& robotY
);

// Punto fijo de intercepcion con correcciones XY aplicadas exactamente una vez.
bool trackedPiecePickTargetToRobot(
    float& robotX,
    float& robotY
);

uint32_t getTrackedPieceAgeMs();
