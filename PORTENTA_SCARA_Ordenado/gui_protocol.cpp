#include "gui_protocol.h"

#include <stdio.h>
#include <string.h>

namespace {
char g_guiBuf[128];
size_t g_guiLen = 0;
GuiCommand g_pending;
}

void guiProtocolInit() {
  g_guiLen = 0;
  g_pending = GuiCommand();
}

static void clearLine() {
  g_guiLen = 0;
  g_guiBuf[0] = '\0';
}

static bool startsWith(const char* line, const char* prefix) {
  return strncmp(line, prefix, strlen(prefix)) == 0;
}

static void parseLine(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;

  GuiCommand cmd;
  cmd.pending = true;

  if (strcmp(line, "GET_STATUS") == 0) {
    cmd.type = GUI_CMD_GET_STATUS;
  } else if (strcmp(line, "HOME") == 0) {
    cmd.type = GUI_CMD_HOME;
  } else if (strcmp(line, "HOME_M3") == 0) {
    cmd.type = GUI_CMD_HOME_M3;
  } else if (strcmp(line, "ENABLE") == 0) {
    cmd.type = GUI_CMD_ENABLE;
  } else if (strcmp(line, "DISABLE") == 0) {
    cmd.type = GUI_CMD_DISABLE;
  } else if (strcmp(line, "ESTOP") == 0) {
    cmd.type = GUI_CMD_ESTOP;
  } else if (strcmp(line, "RESET_ESTOP") == 0) {
    cmd.type = GUI_CMD_RESET_ESTOP;
  } else if (startsWith(line, "GOTO_XYZ")) {
    cmd.type = GUI_CMD_GOTO_XYZ;
    if (sscanf(line, "GOTO_XYZ %f %f %f", &cmd.a, &cmd.b, &cmd.c) != 3) {
      Serial.println("ERR BAD_GOTO_XYZ");
      return;
    }
  } else if (startsWith(line, "MOVE_JOINTS")) {
    cmd.type = GUI_CMD_MOVE_JOINTS;
    if (sscanf(line, "MOVE_JOINTS %f %f %f", &cmd.a, &cmd.b, &cmd.c) != 3) {
      Serial.println("ERR BAD_MOVE_JOINTS");
      return;
    }
  } else if (startsWith(line, "RUN_PICK ")) {
    cmd.type = GUI_CMD_RUN_PICK;
    int pieceType = 0;
    if (sscanf(line, "RUN_PICK %f %f %d", &cmd.a, &cmd.b, &pieceType) != 3 ||
        (pieceType != 1 && pieceType != 2 && pieceType != 5)) {
      Serial.println("ERR BAD_RUN_PICK USE: RUN_PICK X Y PIEZA(1,2,5)");
      return;
    }
    cmd.c = (float)pieceType;
  } else if (strcmp(line, "RUN_PICKPLACE") == 0) {
    cmd.type = GUI_CMD_RUN_PICKPLACE;
  } else {
    Serial.print("ERR UNKNOWN_CMD ");
    Serial.println(line);
    return;
  }

  g_pending = cmd;
}

void updateGuiProtocol() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      g_guiBuf[g_guiLen] = '\0';
      parseLine(g_guiBuf);
      clearLine();
      continue;
    }

    if (g_guiLen + 1 < sizeof(g_guiBuf)) {
      g_guiBuf[g_guiLen++] = c;
    } else {
      clearLine();
      Serial.println("ERR LINE_TOO_LONG");
    }
  }
}

bool guiHasPendingCommand() {
  return g_pending.pending;
}

GuiCommand guiTakePendingCommand() {
  GuiCommand out = g_pending;
  g_pending = GuiCommand();
  return out;
}
