/**
 * @file
 * @brief  A duty-driven DotBot plant for the host tests
 *
 * @copyright Inria, 2026
 */
#include <math.h>
#include <string.h>

#include "geometry.h"
#include "plant.h"

float plant_uniform(plant_t *p) {
    p->rng = p->rng * 1664525u + 1013904223u;
    return ((float)(p->rng >> 8) + 0.5f) / 16777216.0f;
}

float plant_gauss(plant_t *p) {
    float u1 = plant_uniform(p);
    float u2 = plant_uniform(p);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

void plant_forward(float heading_deg, float *fx, float *fy) {
    float h = heading_deg * (float)M_PI / 180.0f;
    *fx     = -sinf(h);
    *fy     = cosf(h);
}

void plant_init(plant_t *p, float x, float y, float heading_deg, float noise_mm, uint32_t seed) {
    memset(p, 0, sizeof(*p));
    p->x           = x;
    p->y           = y;
    p->heading_deg = heading_deg;
    p->noise_mm    = noise_mm;
    p->rng         = seed ? seed : 1;
    float fx;
    float fy;
    plant_forward(heading_deg, &fx, &fy);
    for (unsigned i = 0; i <= PLANT_FIX_AGE; i++) {
        p->px[i] = x + DB_LH2_LEVER_ARM_EFFECTIVE * fx;
        p->py[i] = y + DB_LH2_LEVER_ARM_EFFECTIVE * fy;
    }
}

static float _target(float pwm, float v) {
    float a = fabsf(pwm);
    if (a <= PLANT_U_RUN || (fabsf(v) < 1.0f && a < PLANT_U_BREAK)) {
        return 0.0f;
    }
    return copysignf((a - PLANT_U_RUN) / PLANT_K_RUN, pwm);
}

void plant_step(plant_t *p, db_control_input_t *in) {
    float gain = 1.0f - expf(-PLANT_DT_S / PLANT_TAU_S);
    float tl   = p->brake_left ? 0.0f : _target(p->pwm_left, p->v_left);
    float tr   = p->brake_right ? 0.0f : _target(p->pwm_right, p->v_right);
    float vl   = p->v_left + gain * (tl - p->v_left);
    float vr   = p->v_right + gain * (tr - p->v_right);
    float dl   = 0.5f * (p->v_left + vl) * PLANT_DT_S;
    float dr   = 0.5f * (p->v_right + vr) * PLANT_DT_S;
    p->v_left  = vl;
    p->v_right = vr;

    float d      = p->airborne ? 0.0f : 0.5f * (dl + dr);
    float dtheta = p->airborne ? 0.0f : (dl - dr) / db_track_effective_mm(dl, dr) * 180.0f / (float)M_PI;
    float fx;
    float fy;
    plant_forward(p->heading_deg + dtheta / 2.0f, &fx, &fy);
    p->x += d * fx;
    p->y += d * fy;
    p->heading_deg += dtheta;
    if (p->heading_deg >= 180.0f) {
        p->heading_deg -= 360.0f;
    }
    if (p->heading_deg < -180.0f) {
        p->heading_deg += 360.0f;
    }

    p->frac_left += dl / DB_MM_PER_COUNT;
    p->frac_right += dr / DB_MM_PER_COUNT;
    in->counts_left  = (int32_t)truncf(p->frac_left);
    in->counts_right = (int32_t)truncf(p->frac_right);
    p->frac_left -= (float)in->counts_left;
    p->frac_right -= (float)in->counts_right;

    memmove(&p->px[1], &p->px[0], PLANT_FIX_AGE * sizeof(float));
    memmove(&p->py[1], &p->py[0], PLANT_FIX_AGE * sizeof(float));
    plant_forward(p->heading_deg, &fx, &fy);
    p->px[0] = p->x + DB_LH2_LEVER_ARM_EFFECTIVE * fx;
    p->py[0] = p->y + DB_LH2_LEVER_ARM_EFFECTIVE * fy;

    p->tick++;
    if (p->tick % PLANT_FIX_EVERY == 0) {
        float nx = p->noise_mm > 0 ? p->noise_mm * plant_gauss(p) : 0;
        float ny = p->noise_mm > 0 ? p->noise_mm * plant_gauss(p) : 0;
        p->fix_sequence++;
        p->fix_x = (uint32_t)lroundf(fmaxf(0, (p->airborne ? p->air_x : p->px[PLANT_FIX_AGE]) + nx));
        p->fix_y = (uint32_t)lroundf(fmaxf(0, (p->airborne ? p->air_y : p->py[PLANT_FIX_AGE]) + ny));
    }
    in->fix_sequence  = p->fix_sequence;
    in->fix_x         = p->fix_x;
    in->fix_y         = p->fix_y;
    in->elapsed_ticks = 1;
}

void plant_apply(plant_t *p, const db_control_output_t *out) {
    if (out->write) {
        p->pwm_left    = out->pwm_left;
        p->pwm_right   = out->pwm_right;
        p->brake_left  = out->brake_left;
        p->brake_right = out->brake_right;
    }
}
