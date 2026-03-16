// main.cpp  –  Micromouse floodfill controller
// Loop: sense → reflood → choose → move

#include "Maze.h"
#include "Interface.h"
#include "API.h"

int main() {
    Maze maze;          // constructor initialises all cells & flood values
    Pose pose;          // starts at MMS (0,0) facing NORTH

    while (true) {
        // ── 1. Read walls, update model, mark visited ─────────────────────
        iface_sense_and_update(maze, pose);

        // ── 2. Re-propagate flood values from current cell ────────────────
        iface_reflood_from_current(maze, pose, false);

        // ── 3. Choose best next direction ─────────────────────────────────
        Direction next_dir = iface_choose_next_dir(maze, pose);

        // ── 4. Rotate (optimally) + move ──────────────────────────────────
        bool moved = iface_execute_step(pose, next_dir);

        // ── 5. If blocked by an unexpected wall, sense again and retry ────
        if (!moved) {
            iface_sense_and_update(maze, pose);
            iface_reflood_from_current(maze, pose, false);
            next_dir = iface_choose_next_dir(maze, pose);
            iface_execute_step(pose, next_dir);
        }
    }

    return 0;
}
