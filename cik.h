/* cik.h - v0.1 - public domain data structures - nickscha 2025

A C89 standard compliant, single header, nostdlib (no C Standard Library) Computational Inverse Kinematics (CIK).

LICENSE

  Placed in the public domain and also MIT licensed.
  See end of file for detailed license information.

*/
#ifndef CIK_H
#define CIK_H

/* #############################################################################
 * # COMPILER SETTINGS
 * #############################################################################
 */
/* Check if using C99 or later (inline is supported) */
#if __STDC_VERSION__ >= 199901L
#define CIK_INLINE inline
#define CIK_API static
#elif defined(__GNUC__) || defined(__clang__)
#define CIK_INLINE __inline__
#define CIK_API static
#elif defined(_MSC_VER)
#define CIK_INLINE __inline
#define CIK_API static
#else
#define CIK_INLINE
#define CIK_API static
#endif

#ifndef CIK_MAX_JOINTS
#define CIK_MAX_JOINTS 128
#endif

#define CIK_PI_DOUBLED 6.28318530717958647692f
#define CIK_PI 3.14159265358979323846f
#define CIK_PI_HALF 1.57079632679489661923f
#define CIK_PI_QUARTER 0.7853981633974483f

#ifndef VM_H
typedef struct v3
{
  float x;
  float y;
  float z;

} v3;
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wuninitialized"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4699) /* MSVC-specific aliasing warning */
#endif
CIK_API CIK_INLINE float cik_invsqrt(float number)
{
  union
  {
    float f;
    long i;
  } conv;

  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  conv.f = number;
  conv.i = 0x5f3759df - (conv.i >> 1); /* Magic number for approximation */
  y = conv.f;
  y = y * (threehalfs - (x2 * y * y)); /* One iteration of Newton's method */

  return (y);
}
#ifdef __GNUC__
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif

CIK_API CIK_INLINE float cik_sqrtf(float x)
{
  return (x * cik_invsqrt(x));
}

#define CIK_LUT_SIZE 256
#define CIK_LUT_MASK (CIK_LUT_SIZE - 1)

static const float cik_lut[CIK_LUT_SIZE] = {
    0.0000f, 0.0245f, 0.0491f, 0.0736f, 0.0980f, 0.1224f, 0.1467f, 0.1710f,
    0.1951f, 0.2191f, 0.2430f, 0.2667f, 0.2903f, 0.3137f, 0.3369f, 0.3599f,
    0.3827f, 0.4052f, 0.4276f, 0.4496f, 0.4714f, 0.4929f, 0.5141f, 0.5350f,
    0.5556f, 0.5758f, 0.5957f, 0.6152f, 0.6344f, 0.6532f, 0.6716f, 0.6895f,
    0.7071f, 0.7242f, 0.7409f, 0.7572f, 0.7730f, 0.7883f, 0.8032f, 0.8176f,
    0.8315f, 0.8449f, 0.8577f, 0.8701f, 0.8819f, 0.8932f, 0.9040f, 0.9142f,
    0.9239f, 0.9330f, 0.9415f, 0.9495f, 0.9569f, 0.9638f, 0.9700f, 0.9757f,
    0.9808f, 0.9853f, 0.9892f, 0.9925f, 0.9952f, 0.9973f, 0.9988f, 0.9997f,
    1.0000f, 0.9997f, 0.9988f, 0.9973f, 0.9952f, 0.9925f, 0.9892f, 0.9853f,
    0.9808f, 0.9757f, 0.9700f, 0.9638f, 0.9569f, 0.9495f, 0.9415f, 0.9330f,
    0.9239f, 0.9142f, 0.9040f, 0.8932f, 0.8819f, 0.8701f, 0.8577f, 0.8449f,
    0.8315f, 0.8176f, 0.8032f, 0.7883f, 0.7730f, 0.7572f, 0.7409f, 0.7242f,
    0.7071f, 0.6895f, 0.6716f, 0.6532f, 0.6344f, 0.6152f, 0.5957f, 0.5758f,
    0.5556f, 0.5350f, 0.5141f, 0.4929f, 0.4714f, 0.4496f, 0.4276f, 0.4052f,
    0.3827f, 0.3599f, 0.3369f, 0.3137f, 0.2903f, 0.2667f, 0.2430f, 0.2191f,
    0.1951f, 0.1710f, 0.1467f, 0.1224f, 0.0980f, 0.0736f, 0.0491f, 0.0245f,
    0.0000f, -0.0245f, -0.0491f, -0.0736f, -0.0980f, -0.1224f, -0.1467f, -0.1710f,
    -0.1951f, -0.2191f, -0.2430f, -0.2667f, -0.2903f, -0.3137f, -0.3369f, -0.3599f,
    -0.3827f, -0.4052f, -0.4276f, -0.4496f, -0.4714f, -0.4929f, -0.5141f, -0.5350f,
    -0.5556f, -0.5758f, -0.5957f, -0.6152f, -0.6344f, -0.6532f, -0.6716f, -0.6895f,
    -0.7071f, -0.7242f, -0.7409f, -0.7572f, -0.7730f, -0.7883f, -0.8032f, -0.8176f,
    -0.8315f, -0.8449f, -0.8577f, -0.8701f, -0.8819f, -0.8932f, -0.9040f, -0.9142f,
    -0.9239f, -0.9330f, -0.9415f, -0.9495f, -0.9569f, -0.9638f, -0.9700f, -0.9757f,
    -0.9808f, -0.9853f, -0.9892f, -0.9925f, -0.9952f, -0.9973f, -0.9988f, -0.9997f,
    -1.0000f, -0.9997f, -0.9988f, -0.9973f, -0.9952f, -0.9925f, -0.9892f, -0.9853f,
    -0.9808f, -0.9757f, -0.9700f, -0.9638f, -0.9569f, -0.9495f, -0.9415f, -0.9330f,
    -0.9239f, -0.9142f, -0.9040f, -0.8932f, -0.8819f, -0.8701f, -0.8577f, -0.8449f,
    -0.8315f, -0.8176f, -0.8032f, -0.7883f, -0.7730f, -0.7572f, -0.7409f, -0.7242f,
    -0.7071f, -0.6895f, -0.6716f, -0.6532f, -0.6344f, -0.6152f, -0.5957f, -0.5758f,
    -0.5556f, -0.5350f, -0.5141f, -0.4929f, -0.4714f, -0.4496f, -0.4276f, -0.4052f,
    -0.3827f, -0.3599f, -0.3369f, -0.3137f, -0.2903f, -0.2667f, -0.2430f, -0.2191f,
    -0.1951f, -0.1710f, -0.1467f, -0.1224f, -0.0980f, -0.0736f, -0.0491f, -0.0245f};

CIK_API CIK_INLINE float cik_sinf(float x)
{
  float index, frac;
  int i, i2;

  x -= CIK_PI_DOUBLED * (float)((int)(x * (1.0f / CIK_PI_DOUBLED)));

  if (x < 0)
  {
    x += CIK_PI_DOUBLED;
  }

  index = x * (CIK_LUT_SIZE / CIK_PI_DOUBLED);
  i = (int)index;
  frac = index - (float)i;

  i &= (CIK_LUT_SIZE - 1);
  i2 = (i + 1) & (CIK_LUT_SIZE - 1);

  return (cik_lut[i] + frac * (cik_lut[i2] - cik_lut[i]));
}

CIK_API CIK_INLINE float cik_cosf(float x)
{
  return (cik_sinf(x + CIK_PI_HALF));
}

CIK_API CIK_INLINE float cik_atan2f(float y, float x)
{
  float abs_y = (y < 0) ? -y : y;
  float angle, r;

  if (x > 0.0f)
  {
    r = (x - abs_y) / (x + abs_y);
    angle = CIK_PI_QUARTER - CIK_PI_QUARTER * r;
  }
  else if (x < 0.0f)
  {
    r = (x + abs_y) / (abs_y - x);
    angle = 3.0f * CIK_PI_QUARTER - CIK_PI_QUARTER * r;
  }
  else /* x == 0 */
  {
    angle = CIK_PI_HALF;
  }

  return (y < 0) ? -angle : angle;
}

CIK_API CIK_INLINE float cik_fabsf(float x)
{
  return x < 0.0f ? -x : x;
}

CIK_API CIK_INLINE v3 cik_v3(float x, float y, float z)
{
  v3 r;
  r.x = x;
  r.y = y;
  r.z = z;

  return r;
}

CIK_API CIK_INLINE v3 cik_v3_add(v3 a, v3 b)
{
  return cik_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

CIK_API CIK_INLINE v3 cik_v3_sub(v3 a, v3 b)
{
  return cik_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

CIK_API CIK_INLINE v3 cik_v3_scale(v3 a, float s)
{
  return cik_v3(a.x * s, a.y * s, a.z * s);
}

CIK_API CIK_INLINE float cik_v3_dot(v3 a, v3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

CIK_API CIK_INLINE v3 cik_v3_cross(v3 a, v3 b)
{
  v3 r;
  r.x = a.y * b.z - a.z * b.y;
  r.y = a.z * b.x - a.x * b.z;
  r.z = a.x * b.y - a.y * b.x;
  return r;
}

CIK_API CIK_INLINE float cik_v3_length_2(v3 a)
{
  return a.x * a.x + a.y * a.y + a.z * a.z;
}

CIK_API CIK_INLINE float cik_v3_length(v3 a)
{
  return cik_sqrtf(cik_v3_length_2(a));
}

CIK_API CIK_INLINE v3 cik_v3_normalize(v3 a)
{
  float l = cik_v3_length(a);
  v3 zero = {0, 0, 0};

  if (l > 1e-9f)
  {
    float inv = 1.0f / l;
    return cik_v3(a.x * inv, a.y * inv, a.z * inv);
  }

  return zero;
}

/* ---------------------- Constraint Enforcers ---------------------- */

/* Spherical cone constraint */
CIK_API CIK_INLINE void cik_fabrik_enforce_spherical_cone(
    v3 parent,
    v3 *child,
    v3 rest_dir,
    float max_angle)
{
  v3 dir = cik_v3_normalize(cik_v3_sub(*child, parent));
  float cosang = cik_v3_dot(rest_dir, dir);
  float cosmax = cik_cosf(max_angle);

  if (cosang < cosmax)
  {
    /* Clamp to cone */
    v3 axis = cik_v3_normalize(cik_v3_cross(rest_dir, dir));
    v3 ortho = cik_v3_normalize(cik_v3_cross(axis, rest_dir));

    float c = cosmax;
    float s = cik_sinf(max_angle);
    v3 newdir;

    float d;

    newdir.x = rest_dir.x * c + ortho.x * s;
    newdir.y = rest_dir.y * c + ortho.y * s;
    newdir.z = rest_dir.z * c + ortho.z * s;

    d = cik_v3_length(cik_v3_sub(*child, parent));

    *child = cik_v3_add(parent, cik_v3_scale(newdir, d));
  }
}

/* Hinge joint constraint */
/* ---------------------- Correct Hinge Enforcement ---------------------- */
/* parent       = position of parent joint
 * child        = pointer to child joint position
 * axis         = hinge axis (unit vector)
 * min_angle    = min angle in radians relative to rest_dir
 * max_angle    = max angle in radians relative to rest_dir
 * rest_dir     = rest direction of the bone (normalized)
 */
CIK_API CIK_INLINE void cik_fabrik_enforce_hinge(
    v3 parent,
    v3 *child,
    v3 axis,
    float min_angle,
    float max_angle,
    v3 rest_dir)
{
  v3 bone = cik_v3_sub(*child, parent);
  float len = cik_v3_length(bone);
  v3 dir;
  float dot_ax;
  v3 proj;
  float proj_len;
  v3 rest_proj;
  float cos_ang;
  float sin_ang;
  float angle;

  if (len < 1e-8f)
  {
    return;
  }

  /* Normalize bone */
  dir = cik_v3_scale(bone, 1.0f / len);

  /* Project bone onto hinge plane */
  dot_ax = cik_v3_dot(dir, axis);
  proj = cik_v3_sub(dir, cik_v3_scale(axis, dot_ax));
  proj_len = cik_v3_length(proj);

  /* If projection is degenerate, the bone is aligned with the hinge axis. */
  /* We will use the rest_dir to define its orientation on the plane. */
  if (proj_len < 1e-8f)
  {
    proj = cik_v3_sub(rest_dir, cik_v3_scale(axis, cik_v3_dot(rest_dir, axis)));

    /* Check if rest_dir is also aligned with the hinge axis.
     * If so, we must generate an arbitrary valid direction on the hinge plane.
     */
    if (cik_v3_length_2(proj) < 1e-8f)
    {
      v3 up = {0.0f, 1.0f, 0.0f};
      /* If axis is aligned with 'up', use a different vector for the cross product */
      if (cik_fabsf(cik_v3_dot(axis, up)) > 0.99f)
      {
        up.x = 1.0f;
        up.y = 0.0f;
        up.z = 0.0f;
      }
      proj = cik_v3_normalize(cik_v3_cross(axis, up));
    }
    else
    {
      proj = cik_v3_normalize(proj);
    }
  }
  else
  {
    proj = cik_v3_scale(proj, 1.0f / proj_len);
  }

  /* Compute angle relative to rest_dir in hinge plane */
  rest_proj = cik_v3_sub(rest_dir, cik_v3_scale(axis, cik_v3_dot(rest_dir, axis)));

  /* Apply the same robust fallback for the rest_proj calculation */
  if (cik_v3_length_2(rest_proj) < 1e-8f)
  {
    v3 up = {0.0f, 1.0f, 0.0f};
    if (cik_fabsf(cik_v3_dot(axis, up)) > 0.99f)
    {
      up.x = 1.0f;
      up.y = 0.0f;
      up.z = 0.0f;
    }
    rest_proj = cik_v3_normalize(cik_v3_cross(axis, up));
  }
  else
  {
    rest_proj = cik_v3_normalize(rest_proj);
  }

  cos_ang = cik_v3_dot(rest_proj, proj);
  sin_ang = cik_v3_dot(cik_v3_cross(rest_proj, proj), axis);
  angle = cik_atan2f(sin_ang, cos_ang);

  /* Clamp to min/max */
  if (angle < min_angle)
  {
    angle = min_angle;
  }

  if (angle > max_angle)
  {
    angle = max_angle;
  }

  /* Rotate rest_proj by clamped angle around axis */
  {
    float cos_theta = cik_cosf(angle);
    float sin_theta = cik_sinf(angle);

    v3 cross = cik_v3_cross(axis, rest_proj);

    v3 new_dir;
    /* Simplified Rodrigues' formula because axis and rest_proj are orthogonal */
    new_dir.x = rest_proj.x * cos_theta + cross.x * sin_theta;
    new_dir.y = rest_proj.y * cos_theta + cross.y * sin_theta;
    new_dir.z = rest_proj.z * cos_theta + cross.z * sin_theta;

    *child = cik_v3_add(parent, cik_v3_scale(new_dir, len));
  }
}

/* ---------------------- FABRIK Solver ---------------------- */
/* 0 = converged within tolerance
 * 1 = max_iter reached (did not converge)
 * 2 = invalid input (n < 2 or exceeds CIK_MAX_JOINTS or degenerate lengths)
 * 3 = target unreachable, clamped at max reach
 */
CIK_API CIK_INLINE int cik_fabrik_solve(
    v3 *pos,          /* [n] joint positions (in/out) */
    int n,            /* number of joints */
    v3 target,        /* target position */
    float *max_angle, /* spherical limits [n-1] */
    int *hinge_type,  /* 0 = spherical, 1 = hinge */
    v3 *hinge_axis,   /* hinge axes */
    float *hinge_min, /* hinge min angles */
    float *hinge_max, /* hinge max angles */
    float tolerance,
    int max_iter)
{
  float lengths[CIK_MAX_JOINTS];
  v3 rest_dirs[CIK_MAX_JOINTS];
  v3 root = pos[0];
  float total_len = 0.0f;
  int i, iter;

  v3 root_to_target;
  float dist2;

  if (n < 2 || n > CIK_MAX_JOINTS)
  {
    return 2;
  }

  /* Precompute lengths and rest dirs */
  for (i = 0; i < n - 1; i++)
  {
    v3 diff = cik_v3_sub(pos[i + 1], pos[i]);
    lengths[i] = cik_v3_length(diff);

    if (lengths[i] < 1e-10f)
    {
      return 2;
    }

    total_len += lengths[i];
    rest_dirs[i] = cik_v3_normalize(diff);
  }

  /* Check reachability */
  root_to_target = cik_v3_sub(target, root);
  dist2 = cik_v3_length_2(root_to_target);

  if (dist2 > total_len * total_len)
  {
    /* Target is unreachable — stretch arm toward it */
    v3 dir = cik_v3_normalize(root_to_target);

    for (i = 1; i < n; ++i)
    {
      pos[i] = cik_v3_add(pos[i - 1], cik_v3_scale(dir, lengths[i - 1]));
    }

    return 3;
  }

  /* Iteration loop */
  for (iter = 0; iter < max_iter; ++iter)
  {
    v3 diff;

    /* Forward reaching */
    pos[n - 1] = target;

    for (i = n - 2; i >= 0; --i)
    {
      v3 dir = cik_v3_normalize(cik_v3_sub(pos[i], pos[i + 1]));
      pos[i] = cik_v3_add(pos[i + 1], cik_v3_scale(dir, lengths[i]));
    }

    /* Backward reaching */
    pos[0] = root;

    for (i = 0; i < n - 1; ++i)
    {
      v3 dir = cik_v3_normalize(cik_v3_sub(pos[i + 1], pos[i]));
      pos[i + 1] = cik_v3_add(pos[i], cik_v3_scale(dir, lengths[i]));

      /* Apply constraints */
      if (hinge_type[i] == 0)
      {
        cik_fabrik_enforce_spherical_cone(pos[i], &pos[i + 1], rest_dirs[i], max_angle[i]);
      }
      else
      {
        cik_fabrik_enforce_hinge(pos[i], &pos[i + 1], hinge_axis[i], hinge_min[i], hinge_max[i], rest_dirs[i]);
      }
    }

    /* Check convergence */
    diff = cik_v3_sub(pos[n - 1], target);

    if (cik_v3_length_2(diff) <= tolerance * tolerance)
    {
      return 0;
    }
  }

  return 1;
}

/*
 * Calculates the current angle of a hinge joint.
 * bone_parent_pos: Position of the parent joint.
 * bone_child_pos: Position of the child joint.
 * axis: The hinge axis.
 * rest_dir: The bone's initial, resting direction.
 * Returns the angle in radians.
 */
CIK_API CIK_INLINE float cik_calculate_hinge_angle(v3 bone_parent_pos, v3 bone_child_pos, v3 axis, v3 rest_dir)
{
  v3 bone = cik_v3_sub(bone_child_pos, bone_parent_pos);
  v3 dir = cik_v3_normalize(bone);

  /* Project bone and rest_dir onto the hinge plane */
  v3 proj = cik_v3_normalize(cik_v3_sub(dir, cik_v3_scale(axis, cik_v3_dot(dir, axis))));
  v3 rest_proj = cik_v3_normalize(cik_v3_sub(rest_dir, cik_v3_scale(axis, cik_v3_dot(rest_dir, axis))));

  /* Find the signed angle between them */
  float cos_ang = cik_v3_dot(rest_proj, proj);
  float sin_ang = cik_v3_dot(cik_v3_cross(rest_proj, proj), axis);

  return cik_atan2f(sin_ang, cos_ang);
}

#endif /* CIK_H */

/*
   ------------------------------------------------------------------------------
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
