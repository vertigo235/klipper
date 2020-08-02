// Kinematic input shapers to minimize motion vibrations in XY plane
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt, exp
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // struct move

/****************************************************************
 * Generic position calculation via shaper convolution
 ****************************************************************/

static inline double
get_axis_position(struct move *m, int axis, double move_time)
{
    double axis_r = m->axes_r.axis[axis - 'x'];
    double start_pos = m->start_pos.axis[axis - 'x'];
    double move_dist = move_get_distance(m, move_time);
    return start_pos + axis_r * move_dist;
}

static inline double
get_axis_position_across_moves(struct move *m, int axis, double time)
{
    while (likely(time < 0.)) {
        m = list_prev_entry(m, node);
        time += m->move_t;
    }
    while (likely(time > m->move_t)) {
        time -= m->move_t;
        m = list_next_entry(m, node);
    }
    return get_axis_position(m, axis, time);
}

struct shaper_pulse {
    double t, a;
};

// Calculate the position from the convolution of the shaper with input signal
static inline double
calc_position(struct move *m, int axis, double move_time
              , struct shaper_pulse *pulses, int n)
{
    double res = 0.;
    int i;
    for (i = 0; i < n; ++i)
        res += pulses[i].a * get_axis_position_across_moves(
                m, axis, move_time + pulses[i].t);
    return res;
}

/****************************************************************
 * Shaper-specific initialization
 ****************************************************************/

#define EI_SHAPER_VIB_TOL 0.05

enum INPUT_SHAPER_TYPE {
    INPUT_SHAPER_ZV = 0,
    INPUT_SHAPER_ZVD = 1,
    INPUT_SHAPER_MZV = 2,
    INPUT_SHAPER_EI = 3,
    INPUT_SHAPER_2HUMP_EI = 4,
    INPUT_SHAPER_3HUMP_EI = 5,
};

struct input_shaper {
    struct stepper_kinematics sk;
    struct stepper_kinematics *orig_sk;
    struct move m;
    struct shaper_pulse *x_pulses, *y_pulses;
    int x_n, y_n;
};

typedef void (*is_init_shaper_callback)(double shaper_freq
                                        , double damping_ratio
                                        , struct shaper_pulse **pulses, int *n);

static inline double
calc_ZV_K(double damping_ratio)
{
    if (likely(!damping_ratio))
        return 1.;
    return exp(-damping_ratio * M_PI / sqrt(1. - damping_ratio*damping_ratio));
}

static inline double
calc_half_period(double shaper_freq, double damping_ratio)
{
    return .5 / (shaper_freq * sqrt(1. - damping_ratio*damping_ratio));
}

static void
init_shaper_zv(double shaper_freq, double damping_ratio
               , struct shaper_pulse **pulses, int *n)
{
    *n = 2;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double half_period = calc_half_period(shaper_freq, damping_ratio);
    double K = calc_ZV_K(damping_ratio);
    double inv_D = 1. / (1. + K);

    (*pulses)[0].t = -half_period;
    (*pulses)[1].t = 0.;

    (*pulses)[0].a = K * inv_D;
    (*pulses)[1].a = inv_D;
}

static void
init_shaper_zvd(double shaper_freq, double damping_ratio
                , struct shaper_pulse **pulses, int *n)
{
    *n = 3;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double half_period = calc_half_period(shaper_freq, damping_ratio);
    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;
    double inv_D = 1. / (K2 + 2. * K + 1.);

    (*pulses)[0].t = -2. * half_period;
    (*pulses)[1].t = -half_period;
    (*pulses)[2].t = 0.;

    (*pulses)[0].a = K2 * inv_D;
    (*pulses)[1].a = 2. * K * inv_D;
    (*pulses)[2].a = inv_D;
}

static void
init_shaper_mzv(double shaper_freq, double damping_ratio
                , struct shaper_pulse **pulses, int *n)
{
    *n = 3;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double half_period = calc_half_period(shaper_freq, damping_ratio);
    double K = exp(-.75 * damping_ratio * M_PI
            / sqrt(1. - damping_ratio*damping_ratio));

    double a1 = 1. - 1. / sqrt(2.);
    double a2 = (sqrt(2.) - 1.) * K;
    double a3 = a1 * K * K;
    double inv_D = 1. / (a1 + a2 + a3);

    (*pulses)[0].t = -1.5 * half_period;
    (*pulses)[1].t = -.75 * half_period;
    (*pulses)[2].t = 0.;

    (*pulses)[0].a = a3 * inv_D;
    (*pulses)[1].a = a2 * inv_D;
    (*pulses)[2].a = a1 * inv_D;
}

static void
init_shaper_ei(double shaper_freq, double damping_ratio
               , struct shaper_pulse **pulses, int *n)
{
    *n = 3;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double half_period = calc_half_period(shaper_freq, damping_ratio);
    double K = calc_ZV_K(damping_ratio);
    double a1 = .25 * (1. + EI_SHAPER_VIB_TOL);
    double a2 = .5 * (1. - EI_SHAPER_VIB_TOL) * K;
    double a3 = a1 * K * K;
    double inv_D = 1. / (a1 + a2 + a3);

    (*pulses)[0].t = -2. * half_period;
    (*pulses)[1].t = -half_period;
    (*pulses)[2].t = 0.;

    (*pulses)[0].a = a3 * inv_D;
    (*pulses)[1].a = a2 * inv_D;
    (*pulses)[2].a = a1 * inv_D;
}

static void
init_shaper_2hump_ei(double shaper_freq, double damping_ratio
                     , struct shaper_pulse **pulses, int *n)
{
    *n = 4;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double half_period = calc_half_period(shaper_freq, damping_ratio);
    double K = calc_ZV_K(damping_ratio);

    double V2 = EI_SHAPER_VIB_TOL * EI_SHAPER_VIB_TOL;
    double X = pow(V2 * (sqrt(1. - V2) + 1.), 1./3.);
    double a1 = (3.*X*X + 2.*X + 3.*V2) / (16.*X);
    double a2 = (.5 - a1) * K;
    double a3 = a2 * K;
    double a4 = a1 * K * K * K;
    double inv_D = 1. / (a1 + a2 + a3 + a4);

    (*pulses)[0].t = -3. * half_period;
    (*pulses)[1].t = -2. * half_period;
    (*pulses)[2].t = -half_period;
    (*pulses)[3].t = 0.;

    (*pulses)[0].a = a4 * inv_D;
    (*pulses)[1].a = a3 * inv_D;
    (*pulses)[2].a = a2 * inv_D;
    (*pulses)[3].a = a1 * inv_D;
}

static void
init_shaper_3hump_ei(double shaper_freq, double damping_ratio
                     , struct shaper_pulse **pulses, int *n)
{
    *n = 5;
    *pulses = malloc(*n * sizeof(struct shaper_pulse));

    double half_period = calc_half_period(shaper_freq, damping_ratio);
    double K = calc_ZV_K(damping_ratio);
    double K2 = K * K;

    double a1 = 0.0625 * (1. + 3. * EI_SHAPER_VIB_TOL
            + 2. * sqrt(2. * (EI_SHAPER_VIB_TOL + 1.) * EI_SHAPER_VIB_TOL));
    double a2 = 0.25 * (1. - EI_SHAPER_VIB_TOL) * K;
    double a3 = (0.5 * (1. + EI_SHAPER_VIB_TOL) - 2. * a1) * K2;
    double a4 = a2 * K2;
    double a5 = a1 * K2 * K2;
    double inv_D = 1. / (a1 + a2 + a3 + a4 + a5);

    (*pulses)[0].t = -4. * half_period;
    (*pulses)[1].t = -3. * half_period;
    (*pulses)[2].t = -2. * half_period;
    (*pulses)[3].t = -half_period;
    (*pulses)[4].t = 0.;

    (*pulses)[0].a = a5 * inv_D;
    (*pulses)[1].a = a4 * inv_D;
    (*pulses)[2].a = a3 * inv_D;
    (*pulses)[3].a = a2 * inv_D;
    (*pulses)[4].a = a1 * inv_D;
}

// Shift pulses around 'mid-point' t=0 so that the input shaper is an identity
// transformation for constant-speed motion (i.e. input_shaper(v * T) = v * T)
static void
shift_pulses(int n, struct shaper_pulse *pulses)
{
    int i;
    double ts = 0.;
    for (i = 0; i < n; ++i)
        ts += pulses[i].a * pulses[i].t;
    for (i = 0; i < n; ++i)
        pulses[i].t -= ts;
}

/****************************************************************
 * Kinematics-related shaper code
 ****************************************************************/

#define DUMMY_T 500.0

// Optimized calc_position when only x axis is needed
static double
shaper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->x_n)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos.x = calc_position(m, 'x', move_time, is->x_pulses, is->x_n);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// Optimized calc_position when only y axis is needed
static double
shaper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->y_n)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos.y = calc_position(m, 'y', move_time, is->y_pulses, is->y_n);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

// General calc_position for both x and y axes
static double
shaper_xy_calc_position(struct stepper_kinematics *sk, struct move *m
                        , double move_time)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    if (!is->x_n && !is->y_n)
        return is->orig_sk->calc_position_cb(is->orig_sk, m, move_time);
    is->m.start_pos = move_get_coord(m, move_time);
    if (is->x_n)
        is->m.start_pos.x = calc_position(m, 'x', move_time
                                          , is->x_pulses, is->x_n);
    if (is->y_n)
        is->m.start_pos.y = calc_position(m, 'y', move_time
                                          , is->y_pulses, is->y_n);
    return is->orig_sk->calc_position_cb(is->orig_sk, &is->m, DUMMY_T);
}

static void
shaper_note_generation_time(struct input_shaper *is)
{
    double pre_active = 0., post_active = 0.;
    if ((is->sk.active_flags & AF_X) && is->x_n) {
        pre_active = is->x_pulses[is->x_n-1].t;
        post_active = -is->x_pulses[0].t;
    }
    if ((is->sk.active_flags & AF_Y) && is->y_n) {
        pre_active = is->y_pulses[is->y_n-1].t > pre_active
            ? is->y_pulses[is->y_n-1].t : pre_active;
        post_active = -is->y_pulses[0].t > post_active
            ? -is->y_pulses[0].t : post_active;
    }
    is->sk.gen_steps_pre_active = pre_active;
    is->sk.gen_steps_post_active = post_active;
}

int __visible
input_shaper_set_sk(struct stepper_kinematics *sk
                    , struct stepper_kinematics *orig_sk)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);
    int af = orig_sk->active_flags & (AF_X | AF_Y);
    if (af == (AF_X | AF_Y))
        is->sk.calc_position_cb = shaper_xy_calc_position;
    else if (af & AF_X)
        is->sk.calc_position_cb = shaper_x_calc_position;
    else if (af & AF_Y)
        is->sk.calc_position_cb = shaper_y_calc_position;
    else
        return -1;
    is->sk.active_flags = orig_sk->active_flags;
    is->orig_sk = orig_sk;
    return 0;
}

static is_init_shaper_callback init_shaper_callbacks[] = {
    [INPUT_SHAPER_ZV] = &init_shaper_zv,
    [INPUT_SHAPER_ZVD] = &init_shaper_zvd,
    [INPUT_SHAPER_MZV] = &init_shaper_mzv,
    [INPUT_SHAPER_EI] = &init_shaper_ei,
    [INPUT_SHAPER_2HUMP_EI] = &init_shaper_2hump_ei,
    [INPUT_SHAPER_3HUMP_EI] = &init_shaper_3hump_ei,
};

int __visible
input_shaper_set_shaper_params(struct stepper_kinematics *sk
                               , int shaper_type_x
                               , int shaper_type_y
                               , double shaper_freq_x
                               , double shaper_freq_y
                               , double damping_ratio_x
                               , double damping_ratio_y)
{
    struct input_shaper *is = container_of(sk, struct input_shaper, sk);

    if (shaper_type_x >= ARRAY_SIZE(init_shaper_callbacks) || shaper_type_x < 0)
        return -1;
    if (shaper_type_y >= ARRAY_SIZE(init_shaper_callbacks) || shaper_type_y < 0)
        return -1;

    int af = is->orig_sk->active_flags & (AF_X | AF_Y);
    free(is->x_pulses);
    if ((af & AF_X) && shaper_freq_x > 0.) {
        init_shaper_callbacks[shaper_type_x](
                shaper_freq_x, damping_ratio_x, &is->x_pulses, &is->x_n);
        shift_pulses(is->x_n, is->x_pulses);
    } else {
        is->x_pulses = NULL;
        is->x_n = 0;
    }
    free(is->y_pulses);
    if ((af & AF_Y) && shaper_freq_y > 0.) {
        init_shaper_callbacks[shaper_type_y](
                shaper_freq_y, damping_ratio_y, &is->y_pulses, &is->y_n);
        shift_pulses(is->y_n, is->y_pulses);
    } else {
        is->y_pulses = NULL;
        is->y_n = 0;
    }
    shaper_note_generation_time(is);
    return 0;
}

double __visible
input_shaper_get_step_generation_window(int shaper_type, double shaper_freq
                                        , double damping_ratio)
{
    if (shaper_freq <= 0.)
        return 0.;
    if (shaper_type >= ARRAY_SIZE(init_shaper_callbacks) || shaper_type < 0)
        return 0.;
    is_init_shaper_callback init_shaper_cb = init_shaper_callbacks[shaper_type];
    int n;
    struct shaper_pulse *pulses;
    init_shaper_cb(shaper_freq, damping_ratio, &pulses, &n);
    shift_pulses(n, pulses);
    double window = -pulses[0].t;
    if (pulses[n-1].t > window)
        window = pulses[n-1].t;
    free(pulses);
    return window;
}

struct stepper_kinematics * __visible
input_shaper_alloc(void)
{
    struct input_shaper *is = malloc(sizeof(*is));
    memset(is, 0, sizeof(*is));
    is->m.move_t = 2. * DUMMY_T;
    return &is->sk;
}
