#pragma once

// ── Simulator I/O API (MMS stdin/stdout protocol) ────────────────────────────

int  API_mazeWidth();
int  API_mazeHeight();

int  API_wallFront();
int  API_wallRight();
int  API_wallLeft();

int  API_moveForward();   // Returns 1 on success, 0 if wall collision
void API_turnRight();
void API_turnLeft();

void API_setWall   (int x, int y, char direction);
void API_clearWall (int x, int y, char direction);

void API_setColor    (int x, int y, char color);
void API_clearColor  (int x, int y);
void API_clearAllColor();

void API_setText    (int x, int y, const char* text);
void API_clearText  (int x, int y);
void API_clearAllText();

int  API_wasReset();
void API_ackReset();
