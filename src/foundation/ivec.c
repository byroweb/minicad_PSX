/* ivec.c — integer vector + fixed-point matrix math. */
#include "minicad/ivec.h"

Vec3i v3_cross(Vec3i a, Vec3i b) {
    Vec3i r;
    r.x = (mym_t)((mym2_t)a.y * b.z - (mym2_t)a.z * b.y);
    r.y = (mym_t)((mym2_t)a.z * b.x - (mym2_t)a.x * b.z);
    r.z = (mym_t)((mym2_t)a.x * b.y - (mym2_t)a.y * b.x);
    return r;
}
mym2_t v3_dot(Vec3i a, Vec3i b) {
    return (mym2_t)a.x * b.x + (mym2_t)a.y * b.y + (mym2_t)a.z * b.z;
}

Mat3 mat3_identity(void) {
    Mat3 m = {{ FX_ONE,0,0, 0,FX_ONE,0, 0,0,FX_ONE }};
    return m;
}
Mat3 mat3_rot_x(int32_t a) {
    int32_t s = fx_isin(a), c = fx_icos(a);
    Mat3 m = {{ FX_ONE,0,0,  0,c,-s,  0,s,c }};
    return m;
}
Mat3 mat3_rot_y(int32_t a) {
    int32_t s = fx_isin(a), c = fx_icos(a);
    Mat3 m = {{ c,0,s,  0,FX_ONE,0,  -s,0,c }};
    return m;
}
Mat3 mat3_rot_z(int32_t a) {
    int32_t s = fx_isin(a), c = fx_icos(a);
    Mat3 m = {{ c,-s,0,  s,c,0,  0,0,FX_ONE }};
    return m;
}

Mat3 mat3_mul(Mat3 a, Mat3 b) {
    Mat3 r;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            mym2_t acc = 0;
            for (int k = 0; k < 3; ++k)
                acc += (mym2_t)a.m[row*3+k] * b.m[k*3+col];
            r.m[row*3+col] = (int32_t)(acc >> FX_SHIFT);
        }
    }
    return r;
}

Vec3i mat3_apply(Mat3 m, Vec3i v) {
    Vec3i r;
    r.x = (mym_t)(((mym2_t)m.m[0]*v.x + (mym2_t)m.m[1]*v.y + (mym2_t)m.m[2]*v.z) >> FX_SHIFT);
    r.y = (mym_t)(((mym2_t)m.m[3]*v.x + (mym2_t)m.m[4]*v.y + (mym2_t)m.m[5]*v.z) >> FX_SHIFT);
    r.z = (mym_t)(((mym2_t)m.m[6]*v.x + (mym2_t)m.m[7]*v.y + (mym2_t)m.m[8]*v.z) >> FX_SHIFT);
    return r;
}
