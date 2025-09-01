/* cik.h - v0.1 - public domain data structures - nickscha 2025

A C89 standard compliant, single header, nostdlib (no C Standard Library) Computational Inverse Kinematics (CIK).

This Test class defines cases to verify that we don't break the excepted behaviours in the future upon changes.

LICENSE

  Placed in the public domain and also MIT licensed.
  See end of file for detailed license information.

*/
#include "../cik.h" /* Computational Inverse Kinematics */

#include "../deps/test.h" /* Simple Testing framework    */
#include "../deps/perf.h" /* Simple Performance profiler */

#include <stdio.h>

void cik_test_fabrik_solver_robot_arm(void)
{
  /* ---- Arm Setup ---- */
  int joint_count = 3;
  v3 positions[CIK_MAX_JOINTS];
  v3 hinge_axes[CIK_MAX_JOINTS];
  int hinge_types[CIK_MAX_JOINTS];
  float max_angles[CIK_MAX_JOINTS];
  float hinge_min[CIK_MAX_JOINTS];
  float hinge_max[CIK_MAX_JOINTS];

  /* ---- Animation Setup ---- */
  v3 target_start = cik_v3(2.0f, 1.0f, 0.0f);
  v3 target_end = cik_v3(1.0f, -1.0f, 0.0f);
  v3 target = target_start;

  float speed = 0.5f; /* units per second */
  int forward = 1;
  int step = 0;
  float tolerance = 1e-5f; /* tolerance */
  int max_iterations = 16;

  /* Initial positions (straight arm) */
  positions[0] = cik_v3(0.0f, 0.0f, 0.0f);
  positions[1] = cik_v3(1.5f, 0.1f, 0.0f);
  positions[2] = cik_v3(3.0f, 0.0f, 0.0f);

  /* Joint constraints */
  max_angles[0] = CIK_PI; /* allow full freedom */
  max_angles[1] = CIK_PI;

  hinge_types[0] = 0; /* spherical joint */

  hinge_types[1] = 0;                       /* hinge joint */
  hinge_axes[1] = cik_v3(0.0f, 0.0f, 1.0f); /* Z hinge */
  hinge_min[1] = -CIK_PI;                   /* allow full swing -180° */
  hinge_max[1] = CIK_PI;                    /* allow full swing +180° */

  printf("[cik][fabrik] start simulation\n");

  while (1)
  {
    float dt = 0.16f; /* 60 FPS */

    v3 dir;
    float dist;
    int i;
    int reached;

    /* Move target smoothly up and down */
    if (forward)
    {
      dir = cik_v3_sub(target_end, target);
    }
    else
    {
      dir = cik_v3_sub(target_start, target);
    }

    dist = cik_v3_length(dir);

    if (dist < 0.01f)
    {
      forward = !forward; /* reverse direction */
    }
    else
    {
      v3 step;

      dir = cik_v3_normalize(dir);
      step = cik_v3_scale(dir, speed * dt);
      target = cik_v3_add(target, step);
    }

    /* Solve IK */
    PERF_PROFILE_WITH_NAME({ reached = cik_fabrik_solve(
                                 positions,
                                 joint_count,
                                 target,
                                 max_angles,
                                 hinge_types,
                                 hinge_axes,
                                 hinge_min,
                                 hinge_max,
                                 tolerance,     /* tolerance */
                                 max_iterations /* max iterations */
                             ); }, "cik_fabrik_solve");

    if (reached == 3)
    {
      printf("[cik][fabrik][%3i] target unreachable, clamped at max reach\n", step);
      break;
    }
    else if (reached == 2)
    {
      printf("[cik][fabrik][%3i] invalid input (n < 2 or exceeds CIK_MAX_JOINTS or degenerate lengths)\n", step);
      break;
    }
    else if (reached == 1)
    {
      printf("[cik][fabrik][%3i] max_iter reached (did not converge)\n", step);
      break;
    }
    else if (reached == 0)
    {
      /* Print joint positions */
      printf("[cik][fabrik][%3i]  target: (%6.2f, %6.2f, %6.2f)\n", step, (double)target.x, (double)target.y, (double)target.z);

      for (i = 0; i < joint_count; ++i)
      {
        printf("[cik][fabrik][%3i] joint %d: (%6.2f, %6.2f, %6.2f)\n", step, i, (double)positions[i].x, (double)positions[i].y, (double)positions[i].z);
      }

      break;
    }

    step++;
  }

  printf("[cik][fabrik] finished simulation\n");
}

int main(void)
{
  cik_test_fabrik_solver_robot_arm();

  return 0;
}

/*
   -----------------------------------------------------------------------------
   This software is available under 2 licenses -- choose whichever you prefer.
   ------------------------------------------------------------------------------
   ALTERNATIVE A - MIT License
   Copyright (c) 2025 nickscha
   Permission is hereby granted, free of charge, to any person obtaining a copy of
   this software and associated documentation files (the "Software"), to deal in
   the Software without restriction, including without limitation the rights to
   use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
   of the Software, and to permit persons to whom the Software is furnished to do
   so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   ------------------------------------------------------------------------------
   ALTERNATIVE B - Public Domain (www.unlicense.org)
   This is free and unencumbered software released into the public domain.
   Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
   software, either in source code form or as a compiled binary, for any purpose,
   commercial or non-commercial, and by any means.
   In jurisdictions that recognize copyright laws, the author or authors of this
   software dedicate any and all copyright interest in the software to the public
   domain. We make this dedication for the benefit of the public at large and to
   the detriment of our heirs and successors. We intend this dedication to be an
   overt act of relinquishment in perpetuity of all present and future rights to
   this software under copyright law.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
   WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ------------------------------------------------------------------------------
*/
