#include "warning.h"
#include <stddef.h>

static Warning_State classify(const Warning_Tracker *tracker)
{
    if (tracker->filtered_mm <= (int32_t)CRITICAL_DISTANCE_MM ||
        tracker->ttc_ms <= CRITICAL_TTC_MS) {
        return WARNING_CRITICAL;
    }
    if (tracker->filtered_mm <= (int32_t)WARNING_DISTANCE_MM ||
        tracker->ttc_ms <= WARNING_TTC_MS) {
        return WARNING_CAUTION;
    }
    return WARNING_SAFE;
}

void Warning_Reset(Warning_Tracker *tracker)
{
    if (tracker == NULL) return;
    tracker->has_sample = 0U;
    tracker->last_time_ms = 0U;
    tracker->raw_mm = 0U;
    tracker->filtered_mm = 0;
    tracker->closing_mmps = 0;
    tracker->ttc_ms = WARNING_TTC_UNKNOWN;
    tracker->state = WARNING_FAULT;
}

void Warning_Fault(Warning_Tracker *tracker)
{
    Warning_Reset(tracker);
}

/* No target within the demo's 2 m window is not an I2C failure.
 * Reset the previous speed/TTC so an old moving target is not remembered. */
void Warning_NoTarget(Warning_Tracker *tracker)
{
    if (tracker == NULL) return;
    Warning_Reset(tracker);
    tracker->state = WARNING_NO_TARGET;
}

void Warning_Process(Warning_Tracker *tracker, uint16_t distance_mm,
                     uint32_t sample_time_ms)
{
    uint32_t elapsed_ms;
    int32_t old_mm;
    int32_t instant_mmps;

    if (tracker == NULL) return;
    tracker->raw_mm = distance_mm;

    if (tracker->has_sample == 0U) {
        tracker->has_sample = 1U;
        tracker->filtered_mm = (int32_t)distance_mm;
        tracker->closing_mmps = 0;
        tracker->last_time_ms = sample_time_ms;
        tracker->ttc_ms = WARNING_TTC_UNKNOWN;
        tracker->state = classify(tracker);
        return;
    }

    elapsed_ms = (uint32_t)(sample_time_ms - tracker->last_time_ms);
    tracker->last_time_ms = sample_time_ms;
    if (elapsed_ms == 0U || elapsed_ms > 2000U) {
        tracker->filtered_mm = (int32_t)distance_mm;
        tracker->closing_mmps = 0;
        tracker->ttc_ms = WARNING_TTC_UNKNOWN;
        tracker->state = classify(tracker);
        return;
    }

    old_mm = tracker->filtered_mm;

    /* Exponential moving average: 75% previous + 25% new sample.
     * Integer math avoids float printf on STM32F103. */
    tracker->filtered_mm = (3 * old_mm + (int32_t)distance_mm + 2) / 4;

    /* d decreases -> positive approaching speed (mm/s).
     * speed = change in distance (mm) * 1000 / elapsed time (ms). */
    instant_mmps = ((old_mm - tracker->filtered_mm) * 1000) /
                   (int32_t)elapsed_ms;

    /* Smooth velocity as well; no floating-point printf on Cortex-M3. */
    tracker->closing_mmps = (3 * tracker->closing_mmps + instant_mmps) / 4;

    if (tracker->closing_mmps >= MIN_CLOSING_SPEED_MMPS) {
        /* TTC = current distance / closing speed (converted to ms). */
        tracker->ttc_ms = ((uint32_t)tracker->filtered_mm * 1000UL) /
                           (uint32_t)tracker->closing_mmps;
    } else {
        tracker->ttc_ms = WARNING_TTC_UNKNOWN;
    }
    tracker->state = classify(tracker);
}

const char *Warning_StateText(Warning_State state)
{
    switch (state) {
    case WARNING_NO_TARGET: return "NO TARGET";
    case WARNING_SAFE: return "SAFE";
    case WARNING_CAUTION: return "WARNING";
    case WARNING_CRITICAL: return "CRITICAL";
    default: return "SENSOR FAULT";
    }
}
