#include <stdbool.h>   // bool, true, false
#include <stdio.h>     // snprintf
#include <stdlib.h>    // rand, srand, calloc, free
#include <math.h>      // cosf, sinf, sqrtf, atan2f, fmaxf
#include <time.h>      // time (for srand seed)
#include "raylib.h"    // Camera2D, Draw*, BeginMode2D, etc.
#include "pufferball.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


int main(void) {
    pufferball env = {0};
    init(&env);

    // Ensure window is created before first WindowShouldClose() call
    c_render(&env);

    while (!WindowShouldClose()) {
        c_step(&env);
        c_render(&env);
    }

    c_close(&env);
    return 0;
}
