// API.cpp  –  MMS stdin/stdout protocol implementation

#include "API.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

constexpr int BUFFER_SIZE = 64;

int getInteger(const char* cmd) {
    printf("%s\n", cmd);
    fflush(stdout);
    char buf[BUFFER_SIZE];
    (void)fgets(buf, BUFFER_SIZE, stdin);
    return atoi(buf);
}

bool getBoolean(const char* cmd) {
    printf("%s\n", cmd);
    fflush(stdout);
    char buf[BUFFER_SIZE];
    (void)fgets(buf, BUFFER_SIZE, stdin);
    return (strncmp(buf, "true", 4) == 0);
}

bool getAck(const char* cmd) {
    printf("%s\n", cmd);
    fflush(stdout);
    char buf[BUFFER_SIZE];
    (void)fgets(buf, BUFFER_SIZE, stdin);
    return (strncmp(buf, "ack", 3) == 0);
}

} // anonymous namespace

int  API_mazeWidth()  { return getInteger("mazeWidth");  }
int  API_mazeHeight() { return getInteger("mazeHeight"); }

int  API_wallFront()  { return getBoolean("wallFront") ? 1 : 0; }
int  API_wallRight()  { return getBoolean("wallRight") ? 1 : 0; }
int  API_wallLeft()   { return getBoolean("wallLeft")  ? 1 : 0; }

int  API_moveForward()  { return getAck("moveForward") ? 1 : 0; }
void API_turnRight()    { getAck("turnRight"); }
void API_turnLeft()     { getAck("turnLeft");  }

void API_setWall  (int x, int y, char d) { printf("setWall %d %d %c\n",   x, y, d); fflush(stdout); }
void API_clearWall(int x, int y, char d) { printf("clearWall %d %d %c\n", x, y, d); fflush(stdout); }

void API_setColor   (int x, int y, char c) { printf("setColor %d %d %c\n",  x, y, c); fflush(stdout); }
void API_clearColor (int x, int y)         { printf("clearColor %d %d\n",   x, y);    fflush(stdout); }
void API_clearAllColor()                   { printf("clearAllColor\n");                fflush(stdout); }

void API_setText   (int x, int y, const char* t) { printf("setText %d %d %s\n",  x, y, t); fflush(stdout); }
void API_clearText (int x, int y)                { printf("clearText %d %d\n",   x, y);    fflush(stdout); }
void API_clearAllText()                          { printf("clearAllText\n");               fflush(stdout); }

int  API_wasReset() { return getBoolean("wasReset") ? 1 : 0; }
void API_ackReset() { getAck("ackReset"); }
