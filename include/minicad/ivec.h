/* ivec.h — integer vectors + 3x3 rotation matrices (fixed point).
 * All operations integer-only; matrices are 1.12 fixed point to match the
 * sine table and the GTE's expectations. */
#ifndef MINICAD_IVEC_H
#define MINICAD_IVEC_H

#include "minicad/fixed.h"

typedef struct { mym_t x, y, z; } Vec3i;   /* model-space point (myriometers) */
typedef struct { mym_t u, v;    } Vec2i;   /* sketch-plane coordinate         */
typedef struct { int32_t m[9];  } Mat3;    /* row-major 1.12 fixed-point       */

static inline Vec3i v3_add(Vec3i a, Vec3i b){ Vec3i r={a.x+b.x,a.y+b.y,a.z+b.z}; return r; }
static inline Vec3i v3_sub(Vec3i a, Vec3i b){ Vec3i r={a.x-b.x,a.y-b.y,a.z-b.z}; return r; }

/* Integer cross product (promotes to 64-bit internally). */
Vec3i  v3_cross(Vec3i a, Vec3i b);
mym2_t v3_dot  (Vec3i a, Vec3i b);

/* Build rotation matrices from sine-table angles (4096 = 360 deg). */
Mat3 mat3_identity(void);
Mat3 mat3_rot_x(int32_t angle);
Mat3 mat3_rot_y(int32_t angle);
Mat3 mat3_rot_z(int32_t angle);
Mat3 mat3_mul(Mat3 a, Mat3 b);

/* Apply a 1.12 matrix to a myriometer vector (result back in myriometers). */
Vec3i mat3_apply(Mat3 m, Vec3i v);

#endif /* MINICAD_IVEC_H */
