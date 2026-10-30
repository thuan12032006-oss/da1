#ifndef COLLISION_WARNING_H
#define COLLISION_WARNING_H

#include <stdint.h>

/* Experimental thresholds. Tune only after measuring real sensor noise. */
#define WARNING_DISTANCE_MM        600U
#define CRITICAL_DISTANCE_MM       200U
#define WARNING_TTC_MS            3500U
#define CRITICAL_TTC_MS           1500U
#define MIN_CLOSING_SPEED_MMPS      50
#define WARNING_TTC_UNKNOWN  0xFFFFFFFFUL

typedef enum {
    WARNING_FAULT = 0,
    WARNING_NO_TARGET,
    WARNING_SAFE,
    WARNING_CAUTION,
    WARNING_CRITICAL
} Warning_State;

typedef struct {
    uint8_t has_sample;
    uint32_t last_time_ms;
    uint16_t raw_mm;
    int32_t filtered_mm;
    int32_t closing_mmps;    /* Positive = distance decreasing. */
    uint32_t ttc_ms;         /* WARNING_TTC_UNKNOWN if not approaching. */
    Warning_State state;
} Warning_Tracker;

void Warning_Reset(Warning_Tracker *tracker);
void Warning_Fault(Warning_Tracker *tracker);
void Warning_NoTarget(Warning_Tracker *tracker);
void Warning_Process(Warning_Tracker *tracker, uint16_t distance_mm,
                     uint32_t sample_time_ms);
const char *Warning_StateText(Warning_State state);

#endif
