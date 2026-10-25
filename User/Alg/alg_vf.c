/**
  ******************************************************************************
  * @file    alg_vf.c
  * @brief   Open-loop V/f ramp (see alg_vf.h).
  *
  * Layer: Alg, depends on Cfg for the V/f constants.
  ******************************************************************************
  */

#include "alg_vf.h"
#include "board_cfg.h"

#define VF_TWO_PI       6.283185307179586f

void alg_vf_init(alg_vf_t *vf)
{
    if (vf == 0) {
        return;
    }
    vf->theta    = 0.0f;
    vf->f_hz     = 0.0f;
    vf->f_target = 0.0f;
    vf->acc_hz_s = VF_ACC_HZ_PER_S;
    vf->m_scale  = 1.0f;
    vf->dir      = 0u;
    vf->on       = 0u;
}

void alg_vf_start(alg_vf_t *vf, float hz)
{
    if (vf == 0) {
        return;
    }
    if (hz < 0.0f)          { hz = 0.0f; }
    if (hz > VF_F_MAX)      { hz = VF_F_MAX; }
    vf->f_target = hz;
    vf->m_scale  = 1.0f;
    vf->on       = 1u;
}

void alg_vf_stop(alg_vf_t *vf)
{
    if (vf == 0) {
        return;
    }
    vf->on       = 0u;
    vf->f_target = 0.0f;
    vf->f_hz     = 0.0f;
    vf->m_scale  = 1.0f;
}

float alg_vf_step(alg_vf_t *vf, float ts)
{
    float df;
    float theta;

    if (vf == 0) {
        return 0.0f;
    }

    /* frequency ramp */
    df = vf->acc_hz_s * ts;
    if (df < 0.0f) {
        df = 0.0f;
    }
    if (vf->f_hz < vf->f_target) {
        vf->f_hz += df;
        if (vf->f_hz > vf->f_target) { vf->f_hz = vf->f_target; }
    } else if (vf->f_hz > vf->f_target) {
        vf->f_hz -= df;
        if (vf->f_hz < vf->f_target) { vf->f_hz = vf->f_target; }
    } else {
        /* nothing to do */
    }

    /* angle integration, kept wrapped so cosf() stays accurate */
    vf->theta += VF_TWO_PI * vf->f_hz * ts;
    while (vf->theta >= VF_TWO_PI) { vf->theta -= VF_TWO_PI; }
    while (vf->theta < 0.0f)       { vf->theta += VF_TWO_PI; }

    /* dir == 1 reverses the phase sequence (B/C swap) by negating the angle.
     * Negating both alpha and beta would be a 180 degree shift instead, which
     * does NOT reverse the rotation. */
    theta = (vf->dir == 0u) ? vf->theta : -vf->theta;
    return theta;
}

float alg_vf_mod_index(const alg_vf_t *vf)
{
    float f;
    float v_ll;
    float m;

    if (vf == 0) {
        return 0.0f;
    }

    f = vf->f_hz;
    if (f < 0.0f) { f = -f; }

    /* V/f curve with a low-speed IR boost, expressed as line-to-line RMS. */
    v_ll = VF_BOOST_V + VF_K_V_PER_HZ * f;

    /* V_LL_rms = 0.6124 * m * Vdc  ->  m = 1.633 * V_LL_rms / Vdc */
    m = VF_M_PER_V * v_ll;
    if (m > VF_M_MAX) { m = VF_M_MAX; }
    if (m < 0.0f)     { m = 0.0f; }

    m *= vf->m_scale;
    if (m < 0.0f) { m = 0.0f; }

    return m;
}
