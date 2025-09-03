/* cik.h - v0.2 - public domain data structures - nickscha 2025

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
#define grid_x 25
#define grid_y 25
#define grid_z 25

  csr_color clear_color = {40, 40, 40};

  v3 world_up = vm_v3(0.0f, 1.0f, 0.0f);
  v3 cam_position = vm_v3(grid_x * 0.5f, grid_y, grid_z * 1.25f);
  v3 cam_look_at_pos = vm_v3(grid_x * 0.5f, grid_y * 0.5f, 0.0f);
  float cam_fov = 90.0f;

  m4x4 projection = vm_m4x4_perspective(vm_radf(cam_fov), (float)context->width / (float)context->height, 0.1f, 1000.0f);
  m4x4 view = vm_m4x4_lookAt(cam_position, cam_look_at_pos, world_up);
  m4x4 projection_view = vm_m4x4_mul(projection, view);

  int frame;
  int frame_count = 250;

  /* ---- Arm Setup ---- */
  int joint_count = 4;
  v3 positions[CIK_MAX_JOINTS];
  v3 hinge_axes[CIK_MAX_JOINTS];
  int hinge_types[CIK_MAX_JOINTS];
  float max_angles[CIK_MAX_JOINTS];
  float hinge_min[CIK_MAX_JOINTS];
  float hinge_max[CIK_MAX_JOINTS];

  /* ---- Animation Setup ---- */
  v3 target = cik_v3(-10.0f, 0.0f, 0.0f);
  v3 current_target = cik_v3(0.0f, (grid_y - 1) * 2.0f, 0.0f);
  float tolerance = 0.1f; /* tolerance */
  int max_iterations = 32;

  float dt = 0.16f; /* 60 FPS */
  float speed = 6.0f;

  long last_voxel_placed_idx = 0;
  int grapped = 0;
  int grapped_count = 0;

  unsigned char *voxels;
  v3 *grapped_positions;

  pmem memory_voxels = {0};
  pmem memory_positions = {0};

  memory_voxels.memory_size = grid_x * grid_y * grid_z;
  memory_positions.memory_size = grid_x * grid_y * grid_z * (unsigned long)sizeof(v3);

  /* Initial positions (straight arm) */
  positions[0] = cik_v3(0.0f, 0.0f, 0.0f);
  positions[1] = cik_v3(0.0f, (grid_y - 1) * 2.0f * 0.25f, 0.0f);
  positions[2] = cik_v3(0.0f, (grid_y - 1) * 2.0f * 0.5f, 0.0f);
  positions[3] = cik_v3(0.0f, (grid_y - 1) * 2.0f, 0.0f);

  /* Joint constraints */
  max_angles[0] = CIK_PI; /* allow full freedom */
  max_angles[1] = CIK_PI; /* allow full freedom */
  max_angles[2] = CIK_PI;

  hinge_types[0] = 0;                       /* spherical joint */
  hinge_types[1] = 0;                       /* spherical joint */
  hinge_types[2] = 0;                       /* hinge joint */
  hinge_axes[2] = cik_v3(0.0f, 0.0f, 1.0f); /* Z hinge */
  hinge_min[2] = -CIK_PI;                   /* allow full swing -180° */
  hinge_max[2] = CIK_PI;                    /* allow full swing +180° */

  pmem_allocate(&memory_voxels);
  pmem_allocate(&memory_positions);

  voxels = memory_voxels.memory;
  grapped_positions = memory_positions.memory;

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
    int solved;
    int i;

    /* Clear Screen Frame and Depth Buffer */
    csr_render_clear_screen(context, clear_color);

    if (frame > 0)
    {
      /* When current target is reached we set a new target */
      if (cik_v3_length(cik_v3_sub(current_target, target)) < 0.1f)
      {
        if (!grapped)
        {
          int x, y, z;
          int found = 0;

          grapped = 1;

          for (z = 0; z < grid_z; ++z)
          {
            if (found)
            {
              break;
            }
            for (y = grid_y - 1; y >= 0; --y)
            {
              if (found)
              {
                break;
              }
              for (x = 0; x < grid_x; ++x)
              {
                long idx = x + y * grid_x + z * grid_x * grid_y;

                if (voxels[idx] && idx > last_voxel_placed_idx)
                {
                  /* Voxel is set */
                  v3 voxel_position = vm_v3((float)x, (float)y, (float)z);

                  target = voxel_position;

                  last_voxel_placed_idx = idx;
                  found = 1;
                  break;
                }
              }
            }
          }

          grapped_count++;
        }
        else
        {
          grapped = 0;
          grapped_positions[grapped_count - 1] = target;
          target = cik_v3(-10.0f, 0.0f, 0.0f);
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
        pio_print("[cik][fabrik] target unreachable, clamped at max reach\n");
        break;
      }
      else if (solved == 2)
      {
        pio_print("[cik][fabrik] invalid input (n < 2 or exceeds CIK_MAX_JOINTS or degenerate lengths)\n");
        break;
      }
      else if (solved == 1)
      {
        pio_print("[cik][fabrik] max_iter reached (did not converge)\n");
        break;
      }
    }

    for (i = 0; i < joint_count; ++i)
    {
      m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, positions[i]), 1.0f);
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
      if (i <= joint_count - 2)
      {
        v3 position = positions[i];
        v3 position_next = positions[i + 1];

        model = vm_m4x4_from_to_scaled(position, position_next, 1.0f, 1.0f);
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
      m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, grapped ? current_target : target), 1.0f);
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

      vm_seed_lcg = 1234;

      for (i = 0; i < grapped_count; ++i)
      {
        m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, grapped_positions[i]), 1.0f);
        m4x4 model_view_projection = vm_m4x4_mul(projection_view, model);

        csr_color c = csr_init_color(
            (unsigned char)vm_randf_range(0.0f, 255.0f),
            (unsigned char)vm_randf_range(0.0f, 255.0f),
            (unsigned char)vm_randf_range(0.0f, 255.0f));

        csr_render(
            context,
            CSR_RENDER_SOLID,
            CSR_CULLING_CCW_BACKFACE, 3,
            cube_vertices, cube_vertices_size,
            cube_indices, cube_indices_size,
            model_view_projection.e, c);
      }
    }

    /* Write framebuffer to ppm file */
    cik_write_ppm(memory_io, context, "mesh_builder", frame);
  }
}

static void run_fabrik_excavator(csr_context *context, pmem *memory_io)
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
  int frame_count = 2;

#define NUM_JOINTS 4

  v3 pos[NUM_JOINTS] = {
      {0.0f, 0.0f, 0.0f}, /* Joint 0: Base (Root) */
      {1.0f, 1.0f, 0.0f}, /* Joint 1: Boom Pivot */
      {3.0f, 0.5f, 0.0f}, /* Joint 2: Stick Pivot */
      {4.0f, 0.0f, 0.0f}  /* Joint 3: Bucket Pivot */
  };

  int hinge_type[NUM_JOINTS - 1];
  v3 hinge_axis[NUM_JOINTS - 1];
  float hinge_min[NUM_JOINTS - 1];
  float hinge_max[NUM_JOINTS - 1];
  float max_angle[NUM_JOINTS - 1]; /* Unused, but the function needs it */

  int i;
  v3 rest_dirs[NUM_JOINTS - 1];

  /* Example: User wants to control only the Stick/Elbow (joint index 1) */
  int controlled_joint_index = 0; /* 0=Boom, 1=Stick, 2=Bucket, -1=Full IK */

  /* Store the original, full-range limits */
  float original_hinge_min[3] = {-CIK_PI_QUARTER, 0.0f, -CIK_PI_HALF};
  float original_hinge_max[3] = {CIK_PI_QUARTER, CIK_PI * 0.75f, CIK_PI_HALF};

  /* --- DYNAMICALLY SET CONSTRAINTS BEFORE SOLVING --- */
  float current_angle;
  const float epsilon = 0.001f; /* A very small angle to create a tight lock */

  /* Setup excavator arm */
  {
    /* All joints are hinges rotating around the X-axis */
    v3 axis = {0.0f, 0.0f, 1.0f};

    for (i = 0; i < NUM_JOINTS - 1; ++i)
    {
      hinge_type[i] = 1; /* 1 = Hinge Joint */
      hinge_axis[i] = axis;
    }

    /* Set plausible angle limits for each joint (in radians) */
    /* Bone 0 (Base -> Boom) */
    hinge_min[0] = -CIK_PI_QUARTER; /* -45 degrees */
    hinge_max[0] = CIK_PI_QUARTER;  /* +45 degrees */

    /* Bone 1 (Boom -> Stick) */
    hinge_min[1] = 0.0f;
    hinge_max[1] = CIK_PI * 0.75f; /* 135 degrees */

    /* Bone 2 (Stick -> Bucket) */
    hinge_min[2] = -CIK_PI_HALF; /* -90 degrees */
    hinge_max[2] = CIK_PI_HALF;  /* +90 degrees */
  }

  /* First, pre-calculate the rest directions from the initial setup */
  for (i = 0; i < NUM_JOINTS - 1; ++i)
  {
    rest_dirs[i] = cik_v3_normalize(cik_v3_sub(pos[i + 1], pos[i]));
  }

  for (i = 0; i < NUM_JOINTS - 1; ++i)
  {
    if (controlled_joint_index == -1 || i == controlled_joint_index)
    {
      /* This joint is active, so give it full range of motion */
      hinge_min[i] = original_hinge_min[i];
      hinge_max[i] = original_hinge_max[i];
    }
    else
    {
      /* This joint is locked. Calculate its current angle. */
      current_angle = cik_calculate_hinge_angle(pos[i], pos[i + 1], hinge_axis[i], rest_dirs[i]);

      /* Set min/max to the current angle to lock it in place */
      hinge_min[i] = current_angle - epsilon;
      hinge_max[i] = current_angle + epsilon;
    }
  }

  for (frame = 0; frame < frame_count; ++frame)
  {
    v3 target = vm_v3(3.2f, 0.4f, 0.0f);

    /* Clear Screen Frame and Depth Buffer */
    csr_render_clear_screen(context, clear_color);

    if (frame > 0)
    {
      int solved = cik_fabrik_solve(
          pos,
          NUM_JOINTS,
          target,    /* Now, you can provide a target and solve. The IK will only be able to meaningfully move the one unlocked joint. */
          max_angle, /* Unused for hinges */
          hinge_type,
          hinge_axis,
          hinge_min, /* The dynamically adjusted limits */
          hinge_max, /* The dynamically adjusted limits */
          0.05f,     /* Tolerance */
          12         /* Max iterations */
      );

      /* Target unreachable */
      if (solved == 3)
      {
        pio_print("[cik][fabrik] target unreachable, clamped at max reach\n");
        break;
      }
      else if (solved == 2)
      {
        pio_print("[cik][fabrik] invalid input (n < 2 or exceeds CIK_MAX_JOINTS or degenerate lengths)\n");
        break;
      }
      else if (solved == 1)
      {
        pio_print("[cik][fabrik] max_iter reached (did not converge)\n");
      }
    }

    for (i = 0; i < NUM_JOINTS; ++i)
    {
      m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, pos[i]), 0.25f);
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
      if (i <= NUM_JOINTS - 2)
      {
        v3 position = pos[i];
        v3 position_next = pos[i + 1];

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
      m4x4 model = vm_m4x4_scalef(vm_m4x4_translate(vm_m4x4_identity, target), 0.25f);
      m4x4 model_view_projection = vm_m4x4_mul(projection_view, model);

      csr_render(
          context,
          CSR_RENDER_SOLID,
          CSR_CULLING_CCW_BACKFACE, 3,
          cube_vertices, cube_vertices_size,
          cube_indices, cube_indices_size,
          model_view_projection.e, csr_init_color(255, 0, 0));
    }

    /* Write framebuffer to ppm file */
    cik_write_ppm(memory_io, context, "excavator", frame);
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
  run_fabrik_excavator(&context, &memory_io);

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
