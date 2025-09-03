/* cik.h - v0.1 - public domain data structures - nickscha 2025

A C89 standard compliant, single header, nostdlib (no C Standard Library) Computational Inverse Kinematics (CIK).

This example demonstrates a win32 program using cik.h without using and linking to the C standard library.

The resulting executable from the build file has only linking to kernel32 (for VirtualAlloc) and nothing else :)

It also does not uses "windows.h" but defines the windows functions prototypes directly since windows.h is
massivly bloated and reduces the build time significantly.

This example tested with clang and gcc.

Please read build.bat file to see the compiler flags and their description.

LICENSE

  Placed in the public domain and also MIT licensed.
  See end of file for detailed license information.

*/
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wbuiltin-declaration-mismatch"
#endif

#if defined(_MSC_VER) && !defined(__clang__)
#pragma function(memset)
#endif
void *memset(void *dest, int c, unsigned int count)
{
  char *bytes = (char *)dest;
  while (count--)
  {
    *bytes++ = (char)c;
  }
  return dest;
}

#if defined(_MSC_VER) && !defined(__clang__)
#pragma function(memcpy)
#endif
void *memcpy(void *dest, const void *src, unsigned int count)
{
  char *dest8 = (char *)dest;
  const char *src8 = (const char *)src;
  while (count--)
  {
    *dest8++ = *src8++;
  }
  return dest;
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include "../deps/pmem.h" /* Platform memory provider         */
#include "../deps/pio.h"  /* Platform io provider             */
#include "../deps/csr.h"  /* C Software Renderer              */
#include "../deps/vm.h"   /* Linear Algebra Library           */
#include "../deps/sb.h"   /* String Builder                   */
#include "../deps/mvx.h"  /* Mesh Voxelizer                   */
#include "../cik.h"       /* Computational Inverse Kinematics */

/* Vertex data */
static float cube_vertices[] = {
    /* Position x,y,z */
    -0.5f, -0.5f, 0.5f,  /* */
    0.5f, -0.5f, 0.5f,   /* */
    0.5f, 0.5f, 0.5f,    /* */
    -0.5f, 0.5f, 0.5f,   /* */
    -0.5f, -0.5f, -0.5f, /* */
    0.5f, -0.5f, -0.5f,  /* */
    0.5f, 0.5f, -0.5f,   /* */
    -0.5f, 0.5f, -0.5f,  /* */
};

/* Index data counterclockwise to form the triangles of a cube.  */
static int cube_indices[] = {
    0, 3, 2, 0, 2, 1, /* Front face (+z normal, facing camera)   */
    4, 5, 6, 4, 6, 7, /* Back face (-z normal, away from camera) */
    3, 7, 6, 3, 6, 2, /* Top face (+y normal)                    */
    0, 1, 5, 0, 5, 4, /* Bottom face (-y normal)                 */
    1, 2, 6, 1, 6, 5, /* Right face (+x normal)                  */
    0, 4, 7, 0, 7, 3  /* Left face (-x normal)                   */
};

static unsigned long cube_vertices_size = sizeof(cube_vertices) / sizeof(cube_vertices[0]);
static unsigned long cube_indices_size = sizeof(cube_indices) / sizeof(cube_indices[0]);

static void *cik_binary_memcpy(void *dest, void *src, unsigned long count)
{
  char *dest8 = (char *)dest;
  char *src8 = (char *)src;
  while (count--)
  {
    *dest8++ = *src8++;
  }

  return dest;
}

static void cik_write_ppm(pmem *memory_io, csr_context *context, char *name, int frame)
{
  unsigned char *ptr = memory_io->memory;

  char buf[128];
  char buf_fn[64];

  sb sb, sbfn;

  /* PPM Header */
  sb_init(&sb, buf, sizeof(buf));
  sb_append_cstr(&sb, "P6\n");
  sb_append_ulong_direct(&sb, (unsigned long)context->width);
  sb_append_cstr(&sb, " ");
  sb_append_ulong_direct(&sb, (unsigned long)context->height);
  sb_append_cstr(&sb, "\n255\n");
  sb_term(&sb);

  /* File name test_{frame}.ppm */
  sb_init(&sbfn, buf_fn, sizeof(buf_fn));
  sb_append_cstr(&sbfn, name);
  sb_append_cstr(&sbfn, "_");
  sb_append_ulong_direct(&sbfn, (unsigned long)frame);
  sb_append_cstr(&sbfn, ".ppm");
  sb_term(&sbfn);

  cik_binary_memcpy(ptr, buf, (unsigned long)sb.len);
  ptr += sb.len;
  cik_binary_memcpy(ptr, context->framebuffer, (unsigned long)(context->width * context->height * (int)sizeof(csr_color)));

  pio_write(sbfn.buf, memory_io->memory, (unsigned long)(sb.len + context->width * context->height * (int)sizeof(csr_color)));
}

static v3 cik_v3_lerp(v3 a, v3 b, float t)
{
  return cik_v3_add(a, cik_v3_scale(cik_v3_sub(b, a), t));
}

static void run_fabrik_arm(csr_context *context, pmem *memory_io)
{
  csr_color clear_color = {40, 40, 40};

  v3 world_up = vm_v3(0.0f, 1.0f, 0.0f);
  v3 cam_position = vm_v3(0.0f, 1.0f, 3.0f);
  v3 cam_look_at_pos = vm_v3(1.0f, 0.0f, 0.0f);
  float cam_fov = 90.0f;

  m4x4 projection = vm_m4x4_perspective(vm_radf(cam_fov), (float)context->width / (float)context->height, 0.1f, 1000.0f);
  m4x4 view = vm_m4x4_lookAt(cam_position, cam_look_at_pos, world_up);
  m4x4 projection_view = vm_m4x4_mul(projection, view);

  int frame;
  int frame_count = 225;

  /* ---- Arm Setup ---- */
  int joint_count = 3;
  v3 positions[CIK_MAX_JOINTS];
  v3 hinge_axes[CIK_MAX_JOINTS];
  int hinge_types[CIK_MAX_JOINTS];
  float max_angles[CIK_MAX_JOINTS];
  float hinge_min[CIK_MAX_JOINTS];
  float hinge_max[CIK_MAX_JOINTS];

  /* ---- Animation Setup ---- */
  v3 target = cik_v3(2.0f, 1.0f, 0.0f);
  v3 current_target = cik_v3(3.0f, 0.0f, 0.0f);
  float tolerance = 1e-5f; /* tolerance */
  int max_iterations = 16;

  float dt = 0.16f; /* 60 FPS */
  float speed = 2.5f;
  int grapped = 0;
  int grapped_count = 0;
  v3 grapped_positions[64];

  /* Initial positions (straight arm) */
  positions[0] = cik_v3(0.0f, 0.0f, 0.0f);
  positions[1] = cik_v3(1.5f, 0.0f, 0.0f);
  positions[2] = cik_v3(3.0f, 0.0f, 0.0f);

  /* Joint constraints */
  max_angles[0] = CIK_PI; /* allow full freedom */
  max_angles[1] = CIK_PI;

  hinge_types[0] = 0; /* spherical joint */

  hinge_types[1] = 0;                       /* hinge joint */
  hinge_axes[1] = cik_v3(0.0f, 0.0f, 1.0f); /* Z hinge */
  hinge_min[1] = -CIK_PI;                   /* allow full swing -180° */
  hinge_max[1] = CIK_PI;                    /* allow full swing +180° */

  for (frame = 0; frame < frame_count; ++frame)
  {
    int i;

    /* Clear Screen Frame and Depth Buffer */
    csr_render_clear_screen(context, clear_color);

    if (frame > 0)
    {
      int solved;

      /* When current target is reached we set a new target */
      if (cik_v3_length(cik_v3_sub(current_target, target)) < 0.05f)
      {
        if (!grapped)
        {
          grapped = 1;
          target = vm_v3(-1.0f + (0.25f * (float)grapped_count), -0.5f, 1.0f);
          grapped_count++;
        }
        else
        {
          grapped = 0;
          grapped_positions[grapped_count - 1] = target;

          /* Once we reach the target, pick a new random target */
          target = cik_v3(
              ((float)(vm_randi() % 400) / 100.0f) - 2.0f, /* x: -2.0 .. +2.0 */
              ((float)(vm_randi() % 200) / 100.0f),        /* y:  0.0 .. +2.0 */
              ((float)(vm_randi() % 400) / 100.0f) - 2.0f  /* z: -2.0 .. +2.0 */
          );
        }
      }

      /* Slowly move the target towards the final target position for smooth animation */
      current_target = cik_v3_lerp(current_target, target, speed * dt);

      solved = cik_fabrik_solve(
          positions,
          joint_count,
          current_target,
          max_angles,
          hinge_types,
          hinge_axes,
          hinge_min,
          hinge_max,
          tolerance,     /* tolerance */
          max_iterations /* max iterations */
      );

      /* Target unreachable */
      if (solved == 3)
      {
        break;
      }
    }

    for (i = 0; i < joint_count; ++i)
    {
      m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, positions[i]), 0.25f);
      m4x4 model_view_projection = vm_m4x4_mul(projection_view, model);

      /* Render arm position */
      csr_render(
          context,
          CSR_RENDER_SOLID,
          CSR_CULLING_CCW_BACKFACE, 3,
          cube_vertices, cube_vertices_size,
          cube_indices, cube_indices_size,
          model_view_projection.e, csr_init_color(0, 255, 65));

      /* Render lines connecting arms */
      if (i <= 1)
      {
        v3 position = positions[i];
        v3 position_next = positions[i + 1];

        model = vm_m4x4_from_to_scaled(position, position_next, 0.05f, 0.05f);
        model_view_projection = vm_m4x4_mul(projection_view, model);

        csr_render(
            context,
            CSR_RENDER_SOLID,
            CSR_CULLING_CCW_BACKFACE, 3,
            cube_vertices, cube_vertices_size,
            cube_indices, cube_indices_size,
            model_view_projection.e, csr_init_color(0, 143, 17));
      }
    }

    /* Render target */
    {
      m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, grapped ? current_target : target), 0.25f);
      m4x4 model_view_projection = vm_m4x4_mul(projection_view, model);

      csr_render(
          context,
          CSR_RENDER_SOLID,
          CSR_CULLING_CCW_BACKFACE, 3,
          cube_vertices, cube_vertices_size,
          cube_indices, cube_indices_size,
          model_view_projection.e, csr_init_color(255, 0, 0));
    }

    /* Render placed targets */
    {
      int i;

      for (i = 0; i < grapped_count; ++i)
      {
        m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, grapped_positions[i]), 0.25f);
        m4x4 model_view_projection = vm_m4x4_mul(projection_view, model);

        csr_render(
            context,
            CSR_RENDER_SOLID,
            CSR_CULLING_CCW_BACKFACE, 3,
            cube_vertices, cube_vertices_size,
            cube_indices, cube_indices_size,
            model_view_projection.e, csr_init_color(220, 220, 220));
      }
    }

    /* Write framebuffer to ppm file */
    cik_write_ppm(memory_io, context, "arm", frame);
  }
}

#include "head.h" /* Head Obj Model Vertices and Indices */

static void run_fabrik_mesh_builder(csr_context *context, pmem *memory_io)
{
#define grid_x 101
#define grid_y 101
#define grid_z 101
  unsigned char voxels[grid_x * grid_y * grid_z];

  csr_color clear_color = {40, 40, 40};

  v3 world_up = vm_v3(0.0f, 1.0f, 0.0f);
  v3 cam_position = vm_v3(grid_x * 0.5f, grid_y, grid_z * 1.25f);
  v3 cam_look_at_pos = vm_v3(grid_x * 0.5f, grid_y * 0.5f, 0.0f);
  float cam_fov = 90.0f;

  m4x4 projection = vm_m4x4_perspective(vm_radf(cam_fov), (float)context->width / (float)context->height, 0.1f, 1000.0f);
  m4x4 view = vm_m4x4_lookAt(cam_position, cam_look_at_pos, world_up);
  m4x4 projection_view = vm_m4x4_mul(projection, view);

  int frame;
  int frame_count = 1;

  /* Voxelize the mesh with 1 cell padding */
  if (!mvx_voxelize_mesh(
          head_vertices, head_vertices_size, /* Mesh Vertices     */
          head_indices, head_indices_size,   /* Mesh Indices      */
          grid_x, grid_y, grid_z,            /* Grid Size         */
          1, 1, 1,                           /* Grid Cell Padding */
          voxels                             /* Output Voxels     */
          ))
  {
    return;
  }

  /* Access voxels */
  for (frame = 0; frame < frame_count; ++frame)
  {
    int x, y, z;

    /* Clear Screen Frame and Depth Buffer */
    csr_render_clear_screen(context, clear_color);

    for (z = 0; z < grid_z; ++z)
    {
      for (y = grid_y - 1; y >= 0; --y)
      {
        for (x = 0; x < grid_x; ++x)
        {
          long idx = x + y * grid_x + z * grid_x * grid_y;

          if (voxels[idx])
          {
            /* Voxel is set */
            v3 voxel_position = vm_v3((float)x, (float)y, (float)z);
            m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, voxel_position), 0.5f);
            m4x4 model_view_projection = vm_m4x4_mul(projection_view, model);

            csr_color head_color = csr_init_color(
                (unsigned char)vm_randf_range(0.0f, 200.0f),
                (unsigned char)vm_randf_range(0.0f, 200.0f),
                (unsigned char)vm_randf_range(0.0f, 200.0f));

            csr_render(
                context,
                CSR_RENDER_SOLID,
                CSR_CULLING_CCW_BACKFACE, 3,
                cube_vertices, cube_vertices_size,
                cube_indices, cube_indices_size,
                model_view_projection.e, head_color);
          }
        }
      }
    }

    /* Write framebuffer to ppm file */
    cik_write_ppm(memory_io, context, "mesh_builder", frame);
  }
}

#ifdef __clang__
#elif __GNUC__
__attribute((externally_visible))
#endif
#ifdef __i686__
__attribute((force_align_arg_pointer))
#endif
int
mainCRTStartup(void)
{
  csr_context context = {0};

  /* Define the render area */
  int window_width = 800;
  int window_height = 600;

  pmem memory_graphics = {0};
  pmem memory_io = {0};

  memory_graphics.memory_size = (unsigned long)((window_width * window_height * (int)sizeof(csr_color)) + (window_width * window_height * (int)sizeof(float)));
  memory_io.memory_size = (unsigned long)((window_width * window_height * (int)sizeof(csr_color) + 512));

  if (!pmem_allocate(&memory_graphics) ||
      !pmem_allocate(&memory_io) ||
      !csr_init_model(&context, memory_graphics.memory, memory_graphics.memory_size, window_width, window_height))
  {
    return 1;
  }

  run_fabrik_arm(&context, &memory_io);
  run_fabrik_mesh_builder(&context, &memory_io);

  if (!pmem_free(&memory_io) ||
      !pmem_free(&memory_graphics))
  {
    return 1;
  }

  pio_print("[cik] finished\n");

  return 0;
}

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
