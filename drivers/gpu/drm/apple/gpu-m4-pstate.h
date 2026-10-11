/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_GPU_M4_PSTATE_H
#define APPLE_GPU_M4_PSTATE_H

#include <linux/soc/apple/pmp-cooling.h>

/* J713/G16G performance-state ceiling. Firmware selects the actual state and
 * performs every voltage/clock transition through PMP; the host publishes
 * only a maximum, Globals +0x88/+0x8c/+0x98, in hundredths of a state.
 * Policy arithmetic plus the pre-INIT startup encode: no MMIO or clock access.
 *
 * The thermal policy is the working M4 G16 policy: the shared package backstop
 * in <linux/soc/apple/pmp-cooling.h> (thresholds, level table, confirmation,
 * release and trend prediction, used unchanged) and a C port of its Rust glue,
 * g16_thermal_policy.rs Policy and g16_thermal.rs Cooling::limit().
 */
#define GPU_M4_PSTATE_MAX		9	/* 1470 MHz; state 0 is off, not an OPP */
#define GPU_M4_PSTATE_COOLING_MAX	(GPU_M4_PSTATE_MAX - 1) /* level 8 selects S1 */

/* g16_thermal_policy.rs read_due(): one sensor read per 5 ms of monotonic ns. */
#define GPU_M4_THERMAL_SAMPLE_NS	5000000ULL

/* Optional ceiling inputs of g16_thermal.rs, compiled out in this tree; each
 * is then zero, exactly as the working driver without its provider:
 *
 * GOVERNOR: the "asahi-g16g" thermal cooling device (max state 8, registered
 * at state 8). Only the published apple-pmp-thermal producer binds it to its
 * passive trips; this tree's apple-pmp-thermal binds only the CPU clusters,
 * so an enabled but unbound device would hold S1 forever.
 *
 * POWER: min(apple_pmp_power_gpu_state(), 8), the measured-rail budget level
 * published by the apple-pmp-thermal power worker through the built-in
 * pmp-report mailbox (CONFIG_APPLE_PMP_REPORT). This tree's pmp-report has no
 * mailbox, so the term needs new built-in code first.
 */
#ifndef GPU_M4_THERMAL_GOVERNOR
#define GPU_M4_THERMAL_GOVERNOR		0
#endif
#ifndef GPU_M4_THERMAL_POWER
#define GPU_M4_THERMAL_POWER		0
#endif

/* g16_thermal.rs Cooling, less its mapping and cooling device (session glue
 * and module state). Everything is under the GPU mutex.
 */
struct gpu_m4_thermal {
	struct apple_pmp_cooling policy;	/* g16_thermal_policy.rs Policy */
	long mc;		/* last sample (temp, channel, ret); temp 0 on error */
	int channel, error;
	u32 governor, power;	/* last optional inputs, 0..8 */
	u32 thermal, ceiling;	/* Policy::limit(), and with the power term */
	u32 logged_limit;	/* last_transition, None while !logged_reason */
	const char *logged_reason;
};

/* REMOVE-DEBUG: where an actual-state sample was taken. HEALTH is every READY
 * health pass (publication, completion, periodic owner), POLL a pending
 * execution poll, FIXED the provider's idle/handoff fixed observation, which
 * never updates the ceiling.
 */
enum m4_pstate_site { M4_PSTATE_HEALTH, M4_PSTATE_POLL, M4_PSTATE_FIXED, M4_PSTATE_SITES };

struct m4_gpu_pstate {
	struct gpu_m4_thermal thermal;
	void __iomem *temperature;	/* read-only PMP records, or NULL */
	u32 configured;	/* session maximum S: initdata, health bound */
	u32 user;	/* sysfs ceiling, 1..9 */
	u32 effective;	/* value last published to Globals */
	/* REMOVE-DEBUG observations, under the GPU mutex. */
	u32 last;
	u64 updates, sensor_errors, decisions;
	u64 samples[M4_PSTATE_SITES][16];
};

/* Policy::new() and Cooling::new(): startup level 8 (S1), hard protection,
 * sample (0, -1, 0), no logged transition.
 */
static inline void gpu_m4_thermal_init(struct gpu_m4_thermal *t)
{
	memset(t, 0, sizeof(*t));
	apple_pmp_cooling_init(&t->policy);
	t->channel = -1;
	t->thermal = t->ceiling = GPU_M4_PSTATE_MAX - GPU_M4_PSTATE_COOLING_MAX;
}

/* Policy::read_due(). */
static inline bool gpu_m4_thermal_due(const struct gpu_m4_thermal *t, u64 now)
{
	return t->policy.last_ns == 0 || now - t->policy.last_ns >= GPU_M4_THERMAL_SAMPLE_NS;
}

/* Policy::sample() with Cooling::limit()'s record of the read. A failed read
 * (any error) is None: the shared policy selects level 8 and restarts its
 * confirmation; temp is passed as 0 with error 1.
 */
static inline void gpu_m4_thermal_sample(struct gpu_m4_thermal *t, u64 now, int error,
					 long mc, int channel)
{
	t->mc = mc;
	t->channel = channel;
	t->error = error;
	apple_pmp_cooling_sample(&t->policy, now, error ? 0 : mc, !!error);
}

/* Policy::limit(): retain a governor request in the release envelope, then
 * 9 - max(governor, level), each clamped to 8.
 */
static inline u32 gpu_m4_thermal_policy_limit(struct gpu_m4_thermal *t, u32 cooling_state)
{
	u32 governor = cooling_state < GPU_M4_PSTATE_COOLING_MAX ?
		cooling_state : GPU_M4_PSTATE_COOLING_MAX;
	u32 level;

	apple_pmp_cooling_observe(&t->policy, t->policy.last_ns, cooling_state);
	level = t->policy.level < GPU_M4_PSTATE_COOLING_MAX ?
		t->policy.level : GPU_M4_PSTATE_COOLING_MAX;
	return GPU_M4_PSTATE_MAX - (governor > level ? governor : level);
}

/* Policy::trace() reason. */
static inline const char *gpu_m4_thermal_reason(const struct gpu_m4_thermal *t)
{
	switch (t->policy.reason) {
	case 0: return "startup";
	case 1: return "sensor";
	case 2: return "measured-heat";
	case 3: return "prediction";
	case 5: return "recovery";
	default: return "governor";
	}
}

/* Cooling::limit() after any due sample: min(Policy::limit(governor),
 * 9 - power), power already clamped to 0..8. Returns true on a (limit, reason)
 * transition, the only time the decision is logged.
 */
static inline bool gpu_m4_thermal_ceiling(struct gpu_m4_thermal *t, u32 governor, u32 power,
					  u32 *limit, const char **reason)
{
	u32 thermal = gpu_m4_thermal_policy_limit(t, governor);

	*limit = thermal < GPU_M4_PSTATE_MAX - power ? thermal : GPU_M4_PSTATE_MAX - power;
	*reason = power > 0 && GPU_M4_PSTATE_MAX - power < thermal ? "power-budget" :
		gpu_m4_thermal_reason(t);
	t->governor = governor;
	t->power = power;
	t->thermal = thermal;
	t->ceiling = *limit;
	if (t->logged_reason && t->logged_limit == *limit && !strcmp(t->logged_reason, *reason))
		return false;
	t->logged_limit = *limit;
	t->logged_reason = *reason;
	return true;
}

/* Host startup policy, not a wire requirement: before the INIT root message
 * Globals carry S1 (100 each), while the Main table and PowerPerformance keep
 * the full configured S. Thermal service releases it after acceptance.
 */
static inline void gpu_m4_pstate_startup(struct m4_gpu_pstate *p, void *globals)
{
	put_unaligned_le32(100, globals + 0x88);
	put_unaligned_le32(100, globals + 0x8c);
	put_unaligned_le32(100, globals + 0x98);
	p->effective = 1;
}

/* update_thermal_limit(): the cooling ceiling bounded by configured S, and
 * here also the sysfs ceiling; never 0, which firmware would not treat as
 * unlimited.
 */
static inline u32 gpu_m4_pstate_limit(u32 configured, u32 ceiling, u32 user)
{
	u32 limit = ceiling < GPU_M4_PSTATE_MAX ? ceiling : GPU_M4_PSTATE_MAX;

	if (configured < limit)
		limit = configured;
	if (user < limit)
		limit = user;
	return limit ? limit : 1;
}

#if GPU_M4_THERMAL_GOVERNOR
#include <linux/atomic.h>
#include <linux/thermal.h>

/* g16_thermal.rs cooling device: the thermal governor's request, 0..8. The
 * callbacks only touch this atomic. The module is never unbound, so the one
 * registration lives as long as the module (the Rust REGISTERED guard).
 */
static atomic_t gpu_m4_thermal_cooling_state = ATOMIC_INIT(0);
static struct thermal_cooling_device *gpu_m4_thermal_cdev;

static int gpu_m4_thermal_get_max(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = GPU_M4_PSTATE_COOLING_MAX;
	return 0;
}

static int gpu_m4_thermal_get_cur(struct thermal_cooling_device *cdev, unsigned long *state)
{
	*state = atomic_read(&gpu_m4_thermal_cooling_state);
	return 0;
}

static int gpu_m4_thermal_set_cur(struct thermal_cooling_device *cdev, unsigned long state)
{
	if (state > GPU_M4_PSTATE_COOLING_MAX)
		return -EINVAL;
	atomic_set_release(&gpu_m4_thermal_cooling_state, state);
	return 0;
}

static const struct thermal_cooling_device_ops gpu_m4_thermal_cooling_ops = {
	.get_max_state = gpu_m4_thermal_get_max,
	.get_cur_state = gpu_m4_thermal_get_cur,
	.set_cur_state = gpu_m4_thermal_set_cur,
};

/* Cooling::new(): full cooling until the governor releases it. */
static inline int gpu_m4_thermal_governor_register(void)
{
	struct thermal_cooling_device *cdev;

	if (gpu_m4_thermal_cdev)
		return 0;
	atomic_set_release(&gpu_m4_thermal_cooling_state, GPU_M4_PSTATE_COOLING_MAX);
	cdev = thermal_cooling_device_register("asahi-g16g", NULL, &gpu_m4_thermal_cooling_ops);
	if (IS_ERR_OR_NULL(cdev))
		return cdev ? PTR_ERR(cdev) : -ENOMEM;
	gpu_m4_thermal_cdev = cdev;
	return 0;
}

static inline u32 gpu_m4_thermal_governor(void)
{
	return atomic_read_acquire(&gpu_m4_thermal_cooling_state);
}
#else
static inline int gpu_m4_thermal_governor_register(void)
{
	return 0;
}

static inline u32 gpu_m4_thermal_governor(void)
{
	return 0;
}
#endif

#if GPU_M4_THERMAL_POWER
#include <linux/soc/apple/pmp-power.h>

/* Read-only atomic mailbox; its provider returns 8 for absent/stale data. */
static inline u32 gpu_m4_thermal_power(void)
{
	u32 power = apple_pmp_power_gpu_state();

	return power < GPU_M4_PSTATE_COOLING_MAX ? power : GPU_M4_PSTATE_COOLING_MAX;
}
#else
static inline u32 gpu_m4_thermal_power(void)
{
	return 0;
}
#endif

#endif
