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

#include "../deps/pmem.h" /* Platform memory provider */
#include "../deps/pio.h"  /* Platform io provider             */
#include "../deps/csr.h"  /* C Software Renderer              */
#include "../deps/vm.h"   /* Linear Algebra Library           */
#include "../deps/sb.h"   /* String Builder                   */

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

static void cik_write_ppm(pmem *memory_io, csr_context *context, int frame)
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
  sb_append_cstr(&sbfn, "test_");
  sb_append_ulong_direct(&sbfn, (unsigned long)frame);
  sb_append_cstr(&sbfn, ".ppm");
  sb_term(&sbfn);

  cik_binary_memcpy(ptr, buf, (unsigned long)sb.len);
  ptr += sb.len;
  cik_binary_memcpy(ptr, context->framebuffer, (unsigned long)(context->width * context->height * (int)sizeof(csr_color)));

  pio_write(sbfn.buf, memory_io->memory, (unsigned long)(sb.len + context->width * context->height * (int)sizeof(csr_color)));
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
  csr_color clear_color = {40, 40, 40};
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

  {
    v3 world_up = vm_v3(0.0f, 1.0f, 0.0f);
    v3 cam_position = vm_v3(0.0f, 1.0f, 2.0f);
    v3 cam_look_at_pos = vm_v3_zero;
    float cam_fov = 90.0f;

    m4x4 projection = vm_m4x4_perspective(vm_radf(cam_fov), (float)context.width / (float)context.height, 0.1f, 1000.0f);
    m4x4 view = vm_m4x4_lookAt(cam_position, cam_look_at_pos, world_up);
    m4x4 projection_view = vm_m4x4_mul(projection, view);

    m4x4 model_base = vm_m4x4_translate(vm_m4x4_identity, vm_v3_zero);

    int frame;

    for (frame = 0; frame < 1; ++frame)
    {
      /* Rotate the cube around the model_rotation axis */
      m4x4 model_view_projection = vm_m4x4_mul(projection_view, model_base);

      /* Clear Screen Frame and Depth Buffer */
      csr_render_clear_screen(&context, clear_color);

      /* Render cube */
      csr_render(
          &context,
          CSR_RENDER_SOLID,
          CSR_CULLING_CCW_BACKFACE, 3,
          cube_vertices, cube_vertices_size,
          cube_indices, cube_indices_size,
          model_view_projection.e);

      /* Write framebuffer to ppm file */
      cik_write_ppm(&memory_io, &context, frame);
    }
  }

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
