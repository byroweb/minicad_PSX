/* fixed.h — integer myriometer math + fixed-point helpers.
 *
 * NO FLOATS. The PS1 has no FPU; every coordinate is an integer myriometer
 * (1 mym = 0.1 mm = 1/10,000 m). Trig comes from an integer sine table
 * (4096 steps = 360 deg, results in 1.12 fixed point, i.e. scaled by 4096).
 *
 * This header is host-portable: it compiles for both the MIPS/PSn00bSDK
 * target and a plain host build used for unit tests.
 */
#ifndef MINICAD_FIXED_H
#define MINICAD_FIXED_H

#include <stdint.h>

typedef int32_t mym_t;   /* model-space length: 1 = 0.1 mm                 */
typedef int16_t rnd_t;   /* render-space coordinate fed to the GTE         */
typedef int64_t mym2_t;  /* products/areas needing > 32-bit headroom       */

#define MYM_PER_MM      10
#define MYM_MAX         ((mym_t)0x7FFFFFFF)
#define MYM_MIN         ((mym_t)0x80000000)

/* Fixed-point scale used by the sine table and rotation matrices (1.12). */
#define FX_ONE          4096
#define FX_SHIFT        12

/* Sine table length: 4096 units == 360 degrees. */
#define SIN_LEN         4096
#define SIN_MASK        (SIN_LEN - 1)
#define DEG90           (SIN_LEN / 4)

/* isin/icos return values in [-FX_ONE, +FX_ONE] (1.12 fixed point). */
int32_t fx_isin(int32_t angle);
int32_t fx_icos(int32_t angle);

/* Multiply two 1.12 fixed-point values, result in 1.12. */
static inline int32_t fx_mul(int32_t a, int32_t b) {
    return (int32_t)(((mym2_t)a * (mym2_t)b) >> FX_SHIFT);
}

/* Rounding helpers (half away from zero — documented convention). */
static inline mym_t mym_div_round(mym2_t num, mym2_t den) {
    if (den == 0) return 0;
    mym2_t h = (den > 0 ? den : -den) / 2;
    return (mym_t)((num >= 0) ? (num + h) / den : (num - h) / den);
}

/* Format a myriometer value as a millimetre string, e.g. 12345 -> "1234.5".
 * Pure integer/digit work — NO float divide. `out` needs >= 14 bytes.
 * Returns `out`. */
char *mym_to_mm_str(mym_t v, char *out);

/* Integer abs without UB on INT_MIN (promotes through 64-bit). */
static inline mym_t mym_abs(mym_t v) {
    return (mym_t)((v < 0) ? -(mym2_t)v : v);
}

#endif /* MINICAD_FIXED_H */
