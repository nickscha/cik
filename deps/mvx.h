/* mvx.h - v0.2 - public domain data structures - nickscha 2025

A C89 standard compliant, single header, nostdlib (no C Standard Library) mesh voxelizer (MVX).

LICENSE

  Placed in the public domain and also MIT licensed.
  See end of file for detailed license information.

*/
#ifndef MVX_H
#define MVX_H

/* #############################################################################
 * # COMPILER SETTINGS
 * #############################################################################
 */
#if __STDC_VERSION__ >= 199901L
#define MVX_INLINE inline
#define MVX_API extern
#elif defined(__GNUC__) || defined(__clang__)
#define MVX_INLINE __inline__
#define MVX_API static
#elif defined(_MSC_VER)
#define MVX_INLINE __inline
#define MVX_API static
#else
#define MVX_INLINE
#define MVX_API static
#endif

/* #############################################################################
 * # Linear Algebra & Math functions
 * #############################################################################
 */
typedef struct mvx_v3
{
  float x;
  float y;
  float z;

} mvx_v3;

typedef struct mvx_v3i
{
  int x;
  int y;
  int z;

} mvx_v3i;

MVX_API MVX_INLINE float mvx_absf(float a)
{
  return (a < 0.0f) ? -a : a;
}

MVX_API MVX_INLINE float mvx_minf(float a, float b)
{
  return (a < b) ? a : b;
}

MVX_API MVX_INLINE float mvx_maxf(float a, float b)
{
  return (a > b) ? a : b;
}

MVX_API MVX_INLINE float mvx_clampf(float v, float min, float max)
{
  return mvx_minf(max, mvx_maxf(v, min));
}

MVX_API MVX_INLINE int mvx_mini(int a, int b)
{
  return (a < b) ? a : b;
}

MVX_API MVX_INLINE int mvx_maxi(int a, int b)
{
  return (a > b) ? a : b;
}

MVX_API MVX_INLINE int mvx_floorf(float v)
{
  int i = (int)v;
  if (v < 0.0f && (float)i != v)
  {
    return i - 1;
  }
  return i;
}

MVX_API MVX_INLINE int mvx_ceilf(float v)
{
  return -mvx_floorf(-v);
}

MVX_API MVX_INLINE int mvx_clampi(int v, int min, int max)
{
  if (v < min)
  {
    return min;
  }
  if (v > max)
  {
    return max;
  }
  return v;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_init(float x, float y, float z)
{
  mvx_v3 result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_sub(mvx_v3 a, mvx_v3 b)
{
  mvx_v3 result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  result.z = a.z - b.z;
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_add(mvx_v3 a, mvx_v3 b)
{
  mvx_v3 result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_cross(mvx_v3 a, mvx_v3 b)
{
  mvx_v3 result;
  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;
  return result;
}

MVX_API MVX_INLINE float mvx_v3_dot(mvx_v3 a, mvx_v3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_scale(mvx_v3 a, float s)
{
  mvx_v3 result;
  result.x = a.x * s;
  result.y = a.y * s;
  result.z = a.z * s;
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_abs(mvx_v3 a)
{
  mvx_v3 result;
  result.x = mvx_absf(a.x);
  result.y = mvx_absf(a.y);
  result.z = mvx_absf(a.z);
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_min(mvx_v3 a, mvx_v3 b)
{
  mvx_v3 result;
  result.x = mvx_minf(a.x, b.x);
  result.y = mvx_minf(a.y, b.y);
  result.z = mvx_minf(a.z, b.z);
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_max(mvx_v3 a, mvx_v3 b)
{
  mvx_v3 result;
  result.x = mvx_maxf(a.x, b.x);
  result.y = mvx_maxf(a.y, b.y);
  result.z = mvx_maxf(a.z, b.z);
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_clamp(mvx_v3 v, mvx_v3 min, mvx_v3 max)
{
  mvx_v3 result;
  result.x = mvx_clampf(v.x, min.x, max.x);
  result.y = mvx_clampf(v.y, min.y, max.y);
  result.z = mvx_clampf(v.z, min.z, max.z);
  return result;
}

MVX_API MVX_INLINE mvx_v3 mvx_v3_max_scalar(mvx_v3 v, float s)
{
  mvx_v3 result;
  result.x = mvx_maxf(v.x, s);
  result.y = mvx_maxf(v.y, s);
  result.z = mvx_maxf(v.z, s);
  return result;
}

MVX_API MVX_INLINE float mvx_v3_min_element(mvx_v3 v)
{
  return mvx_minf(mvx_minf(v.x, v.y), v.z);
}

MVX_API MVX_INLINE float mvx_v3_max_element(mvx_v3 v)
{
  return mvx_maxf(mvx_maxf(v.x, v.y), v.z);
}

MVX_API MVX_INLINE mvx_v3i mvx_v3i_init(int x, int y, int z)
{
  mvx_v3i result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

MVX_API MVX_INLINE mvx_v3i mvx_v3i_min(mvx_v3i a, mvx_v3i b)
{
  mvx_v3i result;
  result.x = mvx_mini(a.x, b.x);
  result.y = mvx_mini(a.y, b.y);
  result.z = mvx_mini(a.z, b.z);
  return result;
}

MVX_API MVX_INLINE mvx_v3i mvx_v3i_max(mvx_v3i a, mvx_v3i b)
{
  mvx_v3i result;
  result.x = mvx_maxi(a.x, b.x);
  result.y = mvx_maxi(a.y, b.y);
  result.z = mvx_maxi(a.z, b.z);
  return result;
}

/* point inside axis-aligned box (with epsilon) in world space */
MVX_API MVX_INLINE int mvx_point_in_box_eps_world(mvx_v3 p, mvx_v3 c, mvx_v3 h, float eps)
{
  if (p.x < c.x - h.x - eps || p.x > c.x + h.x + eps)
  {
    return 0;
  }
  if (p.y < c.y - h.y - eps || p.y > c.y + h.y + eps)
  {
    return 0;
  }
  if (p.z < c.z - h.z - eps || p.z > c.z + h.z + eps)
  {
    return 0;
  }
  return 1;
}

/* #############################################################################
 * # Mesh Voxelization Algorythm
 * #############################################################################
 */

/* Akenine-Moeller triangle-box overlap test */
MVX_API MVX_INLINE int mvx_triangle_box_overlap(
    mvx_v3 boxcenter, mvx_v3 boxhalf,
    mvx_v3 triv0, mvx_v3 triv1, mvx_v3 triv2)
{
  float SLOP = 1e-6f; /* absolute, world-space */

  /* move triangle to box space */
  mvx_v3 v0 = mvx_v3_sub(triv0, boxcenter);
  mvx_v3 v1 = mvx_v3_sub(triv1, boxcenter);
  mvx_v3 v2 = mvx_v3_sub(triv2, boxcenter);

  mvx_v3 e0 = mvx_v3_sub(v1, v0);
  mvx_v3 e1 = mvx_v3_sub(v2, v1);
  mvx_v3 e2 = mvx_v3_sub(v0, v2);

  /* 9 axis tests */
  mvx_v3 fexyz[3];
  fexyz[0] = mvx_v3_abs(e0);
  fexyz[1] = mvx_v3_abs(e1);
  fexyz[2] = mvx_v3_abs(e2);

  {
    int i;
    for (i = 0; i < 3; ++i)
    {
      mvx_v3 f = fexyz[i];
      mvx_v3 edge = (i == 0) ? e0 : (i == 1) ? e1
                                             : e2;

      /* X-axis family */
      {
        float p0 = v0.z * edge.y - v0.y * edge.z;
        float p1 = v1.z * edge.y - v1.y * edge.z;
        float p2 = v2.z * edge.y - v2.y * edge.z;
        float pmax = mvx_maxf(mvx_maxf(p0, p1), p2);
        float pmin = mvx_minf(mvx_minf(p0, p1), p2);
        float rad = boxhalf.y * f.z + boxhalf.z * f.y + SLOP;

        if (pmax < -rad || pmin > rad)
        {
          return 0;
        }
      }

      /* Y-axis family */
      {
        float p0 = v0.x * edge.z - v0.z * edge.x;
        float p1 = v1.x * edge.z - v1.z * edge.x;
        float p2 = v2.x * edge.z - v2.z * edge.x;
        float pmax = mvx_maxf(mvx_maxf(p0, p1), p2);
        float pmin = mvx_minf(mvx_minf(p0, p1), p2);
        float rad = boxhalf.x * f.z + boxhalf.z * f.x + SLOP;

        if (pmax < -rad || pmin > rad)
        {
          return 0;
        }
      }
      /* Z-axis family */
      {
        float p0 = v0.y * edge.x - v0.x * edge.y;
        float p1 = v1.y * edge.x - v1.x * edge.y;
        float p2 = v2.y * edge.x - v2.x * edge.y;
        float pmax = mvx_maxf(mvx_maxf(p0, p1), p2);
        float pmin = mvx_minf(mvx_minf(p0, p1), p2);
        float rad = boxhalf.x * f.y + boxhalf.y * f.x + SLOP;

        if (pmax < -rad || pmin > rad)
        {
          return 0;
        }
      }
    }
  }

  /* AABB vs AABB */
  {
    mvx_v3 tri_min = mvx_v3_min(v0, mvx_v3_min(v1, v2));
    mvx_v3 tri_max = mvx_v3_max(v0, mvx_v3_max(v1, v2));
    mvx_v3 bh = mvx_v3_add(boxhalf, mvx_v3_init(SLOP, SLOP, SLOP));

    if (tri_max.x < -bh.x || tri_min.x > bh.x)
    {
      return 0;
    }

    if (tri_max.y < -bh.y || tri_min.y > bh.y)
    {
      return 0;
    }

    if (tri_max.z < -bh.z || tri_min.z > bh.z)
    {
      return 0;
    }
  }

  /* plane-box overlap (treat touching as overlap) */
  {
    mvx_v3 n = mvx_v3_cross(e0, e1);
    float d = -mvx_v3_dot(n, v0);
    float rad = mvx_v3_dot(boxhalf, mvx_v3_abs(n)) + SLOP;
    if (d > rad || d < -rad)
      return 0;
  }

  return 1;
}

/* Aspect-ratio preserving, centered voxelizer with padding */
MVX_API MVX_INLINE int mvx_voxelize_mesh(
    float *vertices,              /* The array of vertex positions (x, y, z) for the mesh. */
    unsigned long vertices_size,  /* The number of floats in the vertices array. This should be 3 times the number of vertices. */
    int *indices,                 /* The array of triangle indices. Each triplet of indices forms a triangle. */
    unsigned long indices_size,   /* The number of integers in the indices array. This should be 3 times the number of triangles. */
    int grid_x,                   /* The total number of voxels along the x-axis of the grid. */
    int grid_y,                   /* The total number of voxels along the y-axis of the grid. */
    int grid_z,                   /* The total number of voxels along the z-axis of the grid. */
    int grid_pad_x,               /* The number of empty voxels to pad on both the left and right sides of the grid. */
    int grid_pad_y,               /* The number of empty voxels to pad on both the bottom and top sides of the grid. */
    int grid_pad_z,               /* The number of empty voxels to pad on both the front and back sides of the grid. */
    unsigned char *output_voxels) /* The output array of unsigned characters where the voxelized mesh will be stored. A value of 1 means the voxel is occupied. */
{
  unsigned long vcount = vertices_size / 3;
  unsigned long tricount = indices_size / 3;

  mvx_v3 min_b = mvx_v3_init(0, 0, 0);
  mvx_v3 max_b = mvx_v3_init(0, 0, 0);

  int first = 1;
  long total = (long)grid_x * (long)grid_y * (long)grid_z;
  long q;
  unsigned long t;
  float vxsize;

  mvx_v3 size;
  mvx_v3 effective_grid;
  mvx_v3 need_v;
  mvx_v3i margin;

  int ia, ib, ic;

  mvx_v3 v0, v1, v2;
  mvx_v3 tmin_b, tmax_b;
  mvx_v3i i_min, i_max;

  int x, y, z;

  if (!vertices || !indices || vcount == 0 || tricount == 0 || grid_x <= 0 || grid_y <= 0 || grid_z <= 0)
  {
    return 0;
  }

  /* clear voxels */
  for (q = 0; q < total; ++q)
  {
    output_voxels[q] = 0;
  }

  /* mesh bounds */
  for (t = 0; t < vcount; ++t)
  {
    mvx_v3 current_v = mvx_v3_init(vertices[3 * t + 0], vertices[3 * t + 1], vertices[3 * t + 2]);

    if (first)
    {
      min_b = max_b = current_v;
      first = 0;
    }
    else
    {
      min_b = mvx_v3_min(min_b, current_v);
      max_b = mvx_v3_max(max_b, current_v);
    }
  }
  if (max_b.x <= min_b.x)
  {
    max_b.x = min_b.x + 1e-6f;
  }
  if (max_b.y <= min_b.y)
  {
    max_b.y = min_b.y + 1e-6f;
  }
  if (max_b.z <= min_b.z)
  {
    max_b.z = min_b.z + 1e-6f;
  }

  /* uniform voxel size (keep aspect ratio) */
  size = mvx_v3_sub(max_b, min_b);

  /* effective grid size after accounting for the gap */
  effective_grid = mvx_v3_init(
      (float)(grid_x - grid_pad_x * 2),
      (float)(grid_y - grid_pad_y * 2),
      (float)(grid_z - grid_pad_z * 2));

  effective_grid = mvx_v3_max_scalar(effective_grid, 1.0f);

  vxsize = mvx_maxf(size.x / effective_grid.x,
                    mvx_maxf(size.y / effective_grid.y,
                             size.z / effective_grid.z));

  /* how many voxels object actually needs along each axis */
  need_v = mvx_v3_init(
      (float)mvx_ceilf(size.x / vxsize),
      (float)mvx_ceilf(size.y / vxsize),
      (float)mvx_ceilf(size.z / vxsize));

  need_v = mvx_v3_min(need_v, effective_grid);

  /* integer margins: the initial gap plus the centering margin within the effective grid */
  margin = mvx_v3i_init(
      grid_pad_x + mvx_floorf((effective_grid.x - need_v.x) / 2.0f),
      grid_pad_y + mvx_floorf((effective_grid.y - need_v.y) / 2.0f),
      grid_pad_z + mvx_floorf((effective_grid.z - need_v.z) / 2.0f));

  /* triangle sweep */
  for (t = 0; t < tricount; ++t)
  {
    ia = indices[3 * t + 0];
    ib = indices[3 * t + 1];
    ic = indices[3 * t + 2];

    if (ia < 0 || ib < 0 || ic < 0)
    {
      continue;
    }

    if ((unsigned long)ia >= vcount || (unsigned long)ib >= vcount || (unsigned long)ic >= vcount)
    {
      continue;
    }

    v0 = mvx_v3_init(vertices[3 * ia + 0], vertices[3 * ia + 1], vertices[3 * ia + 2]);
    v1 = mvx_v3_init(vertices[3 * ib + 0], vertices[3 * ib + 1], vertices[3 * ib + 2]);
    v2 = mvx_v3_init(vertices[3 * ic + 0], vertices[3 * ic + 1], vertices[3 * ic + 2]);

    /* tri AABB in world */
    tmin_b = mvx_v3_init(
        mvx_minf(v0.x, mvx_minf(v1.x, v2.x)),
        mvx_minf(v0.y, mvx_minf(v1.y, v2.y)),
        mvx_minf(v0.z, mvx_minf(v1.z, v2.z)));

    tmax_b = mvx_v3_init(
        mvx_maxf(v0.x, mvx_maxf(v1.x, v2.x)),
        mvx_maxf(v0.y, mvx_maxf(v1.y, v2.y)),
        mvx_maxf(v0.z, mvx_maxf(v1.z, v2.z)));

    /* map to voxel index range */
    i_min = mvx_v3i_init(
        mvx_floorf((tmin_b.x - min_b.x) / vxsize) + margin.x,
        mvx_floorf((tmin_b.y - min_b.y) / vxsize) + margin.y,
        mvx_floorf((tmin_b.z - min_b.z) / vxsize) + margin.z);

    i_max = mvx_v3i_init(
        mvx_ceilf((tmax_b.x - min_b.x) / vxsize) + margin.x,
        mvx_ceilf((tmax_b.y - min_b.y) / vxsize) + margin.y,
        mvx_ceilf((tmax_b.z - min_b.z) / vxsize) + margin.z);

    /* clamp to object's voxel range and overall grid */
    i_min.x = mvx_clampi(i_min.x, margin.x, margin.x + (int)need_v.x - 1);
    i_min.y = mvx_clampi(i_min.y, margin.y, margin.y + (int)need_v.y - 1);
    i_min.z = mvx_clampi(i_min.z, margin.z, margin.z + (int)need_v.z - 1);
    i_max.x = mvx_clampi(i_max.x, margin.x, margin.x + (int)need_v.x - 1);
    i_max.y = mvx_clampi(i_max.y, margin.y, margin.y + (int)need_v.y - 1);
    i_max.z = mvx_clampi(i_max.z, margin.z, margin.z + (int)need_v.z - 1);

    i_min.x = mvx_clampi(i_min.x, 0, grid_x - 1);
    i_min.y = mvx_clampi(i_min.y, 0, grid_y - 1);
    i_min.z = mvx_clampi(i_min.z, 0, grid_z - 1);
    i_max.x = mvx_clampi(i_max.x, 0, grid_x - 1);
    i_max.y = mvx_clampi(i_max.y, 0, grid_y - 1);
    i_max.z = mvx_clampi(i_max.z, 0, grid_z - 1);

    if (i_min.x > i_max.x || i_min.y > i_max.y || i_min.z > i_max.z)
    {
      continue;
    }

    /* scan candidate voxels */
    for (z = i_min.z; z <= i_max.z; ++z)
    {
      for (y = i_min.y; y <= i_max.y; ++y)
      {
        for (x = i_min.x; x <= i_max.x; ++x)
        {
          mvx_v3 boxc, boxh;
          float eps = 1e-6f;
          /* world center of voxel (account for margins) */
          boxc.x = min_b.x + ((float)(x - margin.x) + 0.5f) * vxsize;
          boxc.y = min_b.y + ((float)(y - margin.y) + 0.5f) * vxsize;
          boxc.z = min_b.z + ((float)(z - margin.z) + 0.5f) * vxsize;
          boxh.x = boxh.y = boxh.z = 0.5f * vxsize;

          if (mvx_point_in_box_eps_world(v0, boxc, boxh, eps) ||
              mvx_point_in_box_eps_world(v1, boxc, boxh, eps) ||
              mvx_point_in_box_eps_world(v2, boxc, boxh, eps) ||
              mvx_triangle_box_overlap(boxc, boxh, v0, v1, v2))
          {
            long id = (long)x + (long)y * grid_x + (long)z * grid_x * grid_y;

            output_voxels[id] = 1;
          }
        }
      }
    }
  }

  return 1;
}

/* Get voxel value with boundary checks. */
#define MVX_GET_VOXEL(voxels, x, y, z, gx, gy, gz)      \
  (((x) >= 0 && (x) < (gx) &&                           \
    (y) >= 0 && (y) < (gy) &&                           \
    (z) >= 0 && (z) < (gz))                             \
       ? (voxels)[(x) + (y) * (gx) + (z) * (gx) * (gy)] \
       : 0)

MVX_API MVX_INLINE int mvx_convert_voxels_to_mesh(
    unsigned char *voxels,
    int grid_x,
    int grid_y,
    int grid_z,
    float voxel_size,
    float *out_vertices,
    unsigned long out_vertices_cap,
    unsigned long *out_vert_count,
    int *out_indices,
    unsigned long out_indices_cap,
    unsigned long *out_index_count)
{
  long x, y, z;
  unsigned long current_vert_idx = 0;
  unsigned long current_index_idx = 0;
  float half_voxel_size = voxel_size * 0.5f;

  /* A set of 4 vertices for each of the 6 faces.
   * The order of vertices is chosen to provide consistent winding. */
  float mvx_face_vertices[6][4][3] = {
      /* +Z face */
      {{-1.0f, -1.0f, 1.0f}, {1.0f, -1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f}},
      /* -Z face */
      {{-1.0f, -1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}},
      /* +Y face */
      {{-1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, -1.0f}},
      /* -Y face */
      {{-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f}, {-1.0f, -1.0f, 1.0f}},
      /* +X face */
      {{1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, -1.0f, 1.0f}},
      /* -X face */
      {{-1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f}, {-1.0f, 1.0f, -1.0f}}};

  /* Indices for a quad (two triangles) from 4 vertices. */
  int mvx_face_indices[6] = {0, 1, 2, 0, 2, 3};

  /* Check for invalid input pointers. */
  if (!voxels || !out_vertices || !out_indices || !out_vert_count || !out_index_count)
  {
    return 0;
  }

  *out_vert_count = 0;
  *out_index_count = 0;

  for (z = 0; z < grid_z; ++z)
  {
    for (y = 0; y < grid_y; ++y)
    {
      for (x = 0; x < grid_x; ++x)
      {
        long id = x + y * grid_x + z * grid_x * grid_y;

        if (voxels[id])
        {
          /* Check for an empty neighbor in each direction. */

          /* +X neighbor */
          if (MVX_GET_VOXEL(voxels, x + 1, y, z, grid_x, grid_y, grid_z) == 0)
          {
            int i;

            if (current_vert_idx + 12 > out_vertices_cap || current_index_idx + 6 > out_indices_cap)
            {
              return 0;
            }

            for (i = 0; i < 4; ++i)
            {
              out_vertices[current_vert_idx++] = (float)x * voxel_size + half_voxel_size + mvx_face_vertices[4][i][0] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)y * voxel_size + half_voxel_size + mvx_face_vertices[4][i][1] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)z * voxel_size + half_voxel_size + mvx_face_vertices[4][i][2] * half_voxel_size;
            }

            for (i = 0; i < 6; ++i)
            {
              out_indices[current_index_idx++] = (int)((current_vert_idx / 3) - 4) + mvx_face_indices[i];
            }
          }

          /* -X neighbor */
          if (MVX_GET_VOXEL(voxels, x - 1, y, z, grid_x, grid_y, grid_z) == 0)
          {
            int i;

            if (current_vert_idx + 12 > out_vertices_cap || current_index_idx + 6 > out_indices_cap)
            {
              return 0;
            }

            for (i = 0; i < 4; ++i)
            {
              out_vertices[current_vert_idx++] = (float)x * voxel_size + half_voxel_size + mvx_face_vertices[5][i][0] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)y * voxel_size + half_voxel_size + mvx_face_vertices[5][i][1] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)z * voxel_size + half_voxel_size + mvx_face_vertices[5][i][2] * half_voxel_size;
            }

            for (i = 0; i < 6; ++i)
            {
              out_indices[current_index_idx++] = (int)((current_vert_idx / 3) - 4) + mvx_face_indices[i];
            }
          }

          /* +Y neighbor */
          if (MVX_GET_VOXEL(voxels, x, y + 1, z, grid_x, grid_y, grid_z) == 0)
          {
            int i;

            if (current_vert_idx + 12 > out_vertices_cap || current_index_idx + 6 > out_indices_cap)
            {
              return 0;
            }

            for (i = 0; i < 4; ++i)
            {
              out_vertices[current_vert_idx++] = (float)x * voxel_size + half_voxel_size + mvx_face_vertices[2][i][0] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)y * voxel_size + half_voxel_size + mvx_face_vertices[2][i][1] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)z * voxel_size + half_voxel_size + mvx_face_vertices[2][i][2] * half_voxel_size;
            }

            for (i = 0; i < 6; ++i)
            {
              out_indices[current_index_idx++] = (int)((current_vert_idx / 3) - 4) + mvx_face_indices[i];
            }
          }

          /* -Y neighbor */
          if (MVX_GET_VOXEL(voxels, x, y - 1, z, grid_x, grid_y, grid_z) == 0)
          {
            int i;

            if (current_vert_idx + 12 > out_vertices_cap || current_index_idx + 6 > out_indices_cap)
            {
              return 0;
            }

            for (i = 0; i < 4; ++i)
            {
              out_vertices[current_vert_idx++] = (float)x * voxel_size + half_voxel_size + mvx_face_vertices[3][i][0] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)y * voxel_size + half_voxel_size + mvx_face_vertices[3][i][1] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)z * voxel_size + half_voxel_size + mvx_face_vertices[3][i][2] * half_voxel_size;
            }

            for (i = 0; i < 6; ++i)
            {
              out_indices[current_index_idx++] = (int)((current_vert_idx / 3) - 4) + mvx_face_indices[i];
            }
          }

          /* +Z neighbor */
          if (MVX_GET_VOXEL(voxels, x, y, z + 1, grid_x, grid_y, grid_z) == 0)
          {
            int i;

            if (current_vert_idx + 12 > out_vertices_cap || current_index_idx + 6 > out_indices_cap)
            {
              return 0;
            }

            for (i = 0; i < 4; ++i)
            {
              out_vertices[current_vert_idx++] = (float)x * voxel_size + half_voxel_size + mvx_face_vertices[0][i][0] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)y * voxel_size + half_voxel_size + mvx_face_vertices[0][i][1] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)z * voxel_size + half_voxel_size + mvx_face_vertices[0][i][2] * half_voxel_size;
            }

            for (i = 0; i < 6; ++i)
            {
              out_indices[current_index_idx++] = (int)((current_vert_idx / 3) - 4) + mvx_face_indices[i];
            }
          }

          /* -Z neighbor */
          if (MVX_GET_VOXEL(voxels, x, y, z - 1, grid_x, grid_y, grid_z) == 0)
          {
            int i;

            if (current_vert_idx + 12 > out_vertices_cap || current_index_idx + 6 > out_indices_cap)
            {
              return 0;
            }

            for (i = 0; i < 4; ++i)
            {
              out_vertices[current_vert_idx++] = (float)x * voxel_size + half_voxel_size + mvx_face_vertices[1][i][0] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)y * voxel_size + half_voxel_size + mvx_face_vertices[1][i][1] * half_voxel_size;
              out_vertices[current_vert_idx++] = (float)z * voxel_size + half_voxel_size + mvx_face_vertices[1][i][2] * half_voxel_size;
            }

            for (i = 0; i < 6; ++i)
            {
              out_indices[current_index_idx++] = (int)((current_vert_idx / 3) - 4) + mvx_face_indices[i];
            }
          }
        }
      }
    }
  }

  *out_vert_count = current_vert_idx;
  *out_index_count = current_index_idx;

  return 1;
}

MVX_API MVX_INLINE int mvx_convert_voxels_to_mesh_greedy(
    unsigned char *voxels,
    int grid_x,
    int grid_y,
    int grid_z,
    float voxel_size,
    float *out_vertices,
    unsigned long out_vertices_cap,
    unsigned long *out_vert_count,
    int *out_indices,
    unsigned long out_indices_cap,
    unsigned long *out_index_count)
{
  /* declarations first (C89) */
  int axis, u, v;
  int dims[3];
  int i, j, k;
  int d; /* depth / sweep index */
  int w, h;
  int max_slice;
  int dim_u, dim_v;
  signed char *mask;
  static signed char static_mask[2048 * 2048]; /* adjust if you need larger */
  unsigned long current_vert_idx;
  unsigned long current_index_idx;
  float dpos_world;
  float u0f, u1f, v0f, v1f;
  float corner_uc, corner_vc;
  float coords[3];
  int base_idx;
  signed char m;

  /* validate inputs */
  if (!voxels || !out_vertices || !out_indices || !out_vert_count || !out_index_count)
  {
    return 0;
  }

  dims[0] = grid_x;
  dims[1] = grid_y;
  dims[2] = grid_z;

  /* compute maximum possible 2D slice size (u * v) */
  {
    int xy = grid_x * grid_y;
    int yz = grid_y * grid_z;
    int zx = grid_z * grid_x;
    max_slice = xy;
    if (yz > max_slice)
      max_slice = yz;
    if (zx > max_slice)
      max_slice = zx;
  }

  /* guard the static mask size */
  if (max_slice > (int)(sizeof(static_mask) / sizeof(static_mask[0])))
  {
    /* static mask too small for this grid */
    return 0;
  }
  mask = static_mask;

  current_vert_idx = 0UL;
  current_index_idx = 0UL;

  /* sweep each principal axis */
  for (axis = 0; axis < 3; ++axis)
  {
    u = (axis + 1) % 3;
    v = (axis + 2) % 3;

    dim_u = dims[u];
    dim_v = dims[v];

    /* sweep depth d from 0..dims[axis] (inclusive) comparing voxels at d-1 and d */
    for (d = 0; d <= dims[axis]; ++d)
    {
      /* build mask for this slice: mask[y*dim_u + x] where x in [0..dim_u), y in [0..dim_v) */
      for (j = 0; j < dim_v; ++j)
      {
        for (i = 0; i < dim_u; ++i)
        {
          int a_x, a_y, a_z;
          int b_x, b_y, b_z;
          unsigned char a_val;
          unsigned char b_val;

          /* map coordinates for voxel A (at depth d-1) and voxel B (at depth d) */
          a_x = a_y = a_z = 0;
          b_x = b_y = b_z = 0;

          /* set u / v coordinates */
          if (u == 0)
          {
            a_x = i;
            b_x = i;
          }
          else if (u == 1)
          {
            a_y = i;
            b_y = i;
          }
          else
          {
            a_z = i;
            b_z = i;
          }

          if (v == 0)
          {
            a_x = j;
            b_x = j;
          }
          else if (v == 1)
          {
            a_y = j;
            b_y = j;
          }
          else
          {
            a_z = j;
            b_z = j;
          }

          /* set depth coordinate */
          if (axis == 0)
          {
            a_x = d - 1;
            b_x = d;
          }
          else if (axis == 1)
          {
            a_y = d - 1;
            b_y = d;
          }
          else
          {
            a_z = d - 1;
            b_z = d;
          }

          /* get voxel values using macro (handles bounds) */
          a_val = (unsigned char)MVX_GET_VOXEL(voxels, a_x, a_y, a_z, grid_x, grid_y, grid_z);
          b_val = (unsigned char)MVX_GET_VOXEL(voxels, b_x, b_y, b_z, grid_x, grid_y, grid_z);

          /* mask value: 0 = none, 1 = face from A->B (A=1,B=0), 2 = face from B->A (A=0,B=1) */
          if (a_val && !b_val)
            mask[j * dim_u + i] = 1;
          else if (!a_val && b_val)
            mask[j * dim_u + i] = 2;
          else
            mask[j * dim_u + i] = 0;
        }
      }

      /* greedy merge the mask into rectangles */
      for (j = 0; j < dim_v; ++j)
      {
        i = 0;
        while (i < dim_u)
        {
          m = mask[j * dim_u + i];
          if (m)
          {
            /* find width */
            w = 1;
            while ((i + w) < dim_u && mask[j * dim_u + (i + w)] == m)
              ++w;

            /* find height */
            h = 1;
            while ((j + h) < dim_v)
            {
              int kk;
              int ok = 1;
              for (kk = 0; kk < w; ++kk)
              {
                if (mask[(j + h) * dim_u + (i + kk)] != m)
                {
                  ok = 0;
                  break;
                }
              }
              if (!ok)
                break;
              ++h;
            }

            /* plane position in world space (boundary at integer d) */
            dpos_world = (float)d * voxel_size;

            /* u/v world bounds for rectangle */
            u0f = (float)i * voxel_size;
            u1f = (float)(i + w) * voxel_size;
            v0f = (float)j * voxel_size;
            v1f = (float)(j + h) * voxel_size;

            /* produce 4 vertices on plane d (map u->axis u, v->axis v) */
            /* ordering: for m==2 (B==1) use (u0,v0),(u1,v0),(u1,v1),(u0,v1) */
            /* for m==1 (A==1) reverse to flip normal */
            if (current_vert_idx + 12UL > out_vertices_cap || current_index_idx + 6UL > out_indices_cap)
            {
              return 0;
            }

            if (m == 2)
            {
              /* positive-facing */
              /* corner 0: (u0, v0) */
              corner_uc = u0f;
              corner_vc = v0f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              /* corner 1: (u1, v0) */
              corner_uc = u1f;
              corner_vc = v0f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              /* corner 2: (u1, v1) */
              corner_uc = u1f;
              corner_vc = v1f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              /* corner 3: (u0, v1) */
              corner_uc = u0f;
              corner_vc = v1f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              base_idx = (int)((current_vert_idx / 3UL) - 4UL);
              out_indices[current_index_idx++] = base_idx + 0;
              out_indices[current_index_idx++] = base_idx + 1;
              out_indices[current_index_idx++] = base_idx + 2;
              out_indices[current_index_idx++] = base_idx + 0;
              out_indices[current_index_idx++] = base_idx + 2;
              out_indices[current_index_idx++] = base_idx + 3;
            }
            else
            {
              /* negative-facing (flip winding) */
              /* corner 0: (u0, v0) */
              corner_uc = u0f;
              corner_vc = v0f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              /* corner 1: (u0, v1) */
              corner_uc = u0f;
              corner_vc = v1f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              /* corner 2: (u1, v1) */
              corner_uc = u1f;
              corner_vc = v1f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              /* corner 3: (u1, v0) */
              corner_uc = u1f;
              corner_vc = v0f;
              coords[axis] = dpos_world;
              coords[u] = corner_uc;
              coords[v] = corner_vc;
              out_vertices[current_vert_idx++] = coords[0];
              out_vertices[current_vert_idx++] = coords[1];
              out_vertices[current_vert_idx++] = coords[2];

              base_idx = (int)((current_vert_idx / 3UL) - 4UL);
              out_indices[current_index_idx++] = base_idx + 0;
              out_indices[current_index_idx++] = base_idx + 2;
              out_indices[current_index_idx++] = base_idx + 1;
              out_indices[current_index_idx++] = base_idx + 0;
              out_indices[current_index_idx++] = base_idx + 3;
              out_indices[current_index_idx++] = base_idx + 2;
            }

            /* zero-out used mask cells */
            for (k = 0; k < h; ++k)
            {
              int kk;
              for (kk = 0; kk < w; ++kk)
              {
                mask[(j + k) * dim_u + (i + kk)] = 0;
              }
            }

            i += w;
          }
          else
          {
            ++i;
          }
        }
      }
    } /* end sweep d */
  } /* end axis */

  *out_vert_count = current_vert_idx;
  *out_index_count = current_index_idx;
  return 1;
}

#endif /* MVX_H */

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
