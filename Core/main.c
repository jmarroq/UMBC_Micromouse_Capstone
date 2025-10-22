#include "system_init.h"
#include "motion_control.h"
#include "map_data.h"
#include "solver.h"

int main(void) {
    System_Init();
    Motion_Init();
    Map_Init();
    Solver_Init();

    while (1) {
        Map_Update();
        Path_Step();
        Motion_Update();
    }
}
