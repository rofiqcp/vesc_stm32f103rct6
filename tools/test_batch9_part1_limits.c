#include <math.h>
#include <stdio.h>
#include "motor/mc_math.h"

static int closef(float a, float b, float tol) { return fabsf(a - b) <= tol; }
static int fail(const char *s) { fprintf(stderr, "FAIL: %s\n", s); return 1; }

int main(void) {
    /* Symmetric absolute thermal envelope: 20 A -> 10 A halfway -> 0 A. */
    if (!closef(mc_math_thermal_current_limit(20.0f, 70.0f, 80.0f, 100.0f), 20.0f, 1e-4f))
        return fail("thermal below-start");
    if (!closef(mc_math_thermal_current_limit(20.0f, 90.0f, 80.0f, 100.0f), 10.0f, 1e-4f))
        return fail("thermal midpoint");
    if (!closef(mc_math_thermal_current_limit(20.0f, 100.0f, 80.0f, 100.0f), 0.0f, 1e-4f))
        return fail("thermal end");

    /* l_temp_accel_dec=0.15 moves 80/100 C to 71.75/88.75 C. */
    if (!closef(mc_math_thermal_accel_limit(20.0f, 71.0f, 80.0f, 100.0f, 0.15f), 20.0f, 1e-4f))
        return fail("accel thermal below-start");
    if (!closef(mc_math_thermal_accel_limit(20.0f, 80.25f, 80.0f, 100.0f, 0.15f), 10.0f, 2e-3f))
        return fail("accel thermal midpoint");
    if (!closef(mc_math_thermal_accel_limit(20.0f, 89.0f, 80.0f, 100.0f, 0.15f), 0.0f, 1e-4f))
        return fail("accel thermal end");

    /* foc_start_curr_dec follows VESC: at 0 rpm fraction*Imax, linear to full. */
    if (!closef(mc_math_start_current_limit(20.0f, 0.0f, 0.25f, 2500.0f), 5.0f, 1e-4f))
        return fail("start current zero rpm");
    if (!closef(mc_math_start_current_limit(20.0f, 1250.0f, 0.25f, 2500.0f), 12.5f, 1e-4f))
        return fail("start current midpoint");
    if (!closef(mc_math_start_current_limit(20.0f, 2500.0f, 0.25f, 2500.0f), 20.0f, 1e-4f))
        return fail("start current threshold");
    if (!closef(mc_math_start_current_limit(20.0f, 500.0f, 1.0f, 2500.0f), 20.0f, 1e-4f))
        return fail("default start fraction disables reduction");

    puts("ALL BATCH 9 PART-1 LIMIT NUMERICS: PASS");
    return 0;
}
