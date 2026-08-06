#include <assert.h>
#include <stdio.h>

#include "planning_service.h"
#include "motion_service.h"
#include "robot_config.h"

#define TEST_FRAME_CAPACITY 4096U

static MotionFrame_t s_frames[TEST_FRAME_CAPACITY];
static unsigned int s_frame_count;
static bool s_faulted;
static int32_t s_theory_m1;
static int32_t s_theory_m2;
static int32_t s_theory_m3;

bool MotionService_SubmitFrame(const MotionFrame_t *frame)
{
    if (s_faulted || frame == NULL || s_frame_count >= TEST_FRAME_CAPACITY) return false;
    s_frames[s_frame_count++] = *frame;
    return true;
}

bool MotionService_HasFault(void)
{
    return s_faulted;
}

uint16_t MotionService_GetQueueCount(void)
{
    return 0U;
}

void MotionService_AdjustTheorySteps(int32_t dm1, int32_t dm2, int32_t dm3)
{
    s_theory_m1 += dm1;
    s_theory_m2 += dm2;
    s_theory_m3 += dm3;
}

void MotionService_AbortForFault(MotionFaultReason_t reason)
{
    (void)reason;
    s_faulted = true;
}

static void ResetMotionStub(void)
{
    s_frame_count = 0U;
    s_faulted = false;
    s_theory_m1 = 0;
    s_theory_m2 = 0;
    s_theory_m3 = 0;
}

static void TestPlanningFacade(void)
{
    ErrorCode_t result;
    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    assert(PlanningService_StartLine(100.0f, 180.0f, 160.0f, 2000U) == ERR_PENDING);
    assert(PlanningService_IsBusy());
    assert(PlanningService_StartLine(110.0f, 180.0f, 160.0f, 2000U) == ERR_BUSY);

    do {
        PlanningService_Service();
    } while (!PlanningService_TakeStartResult(&result));
    assert(result == ERR_OK);

    while (PlanningService_IsBusy()) PlanningService_Service();
    assert(s_frame_count > 0U);
    assert(s_theory_m1 != 0 || s_theory_m2 != 0 || s_theory_m3 != 0);
}

static void TestUnreachableTarget(void)
{
    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    assert(PlanningService_StartLine(0.0f, 185.0f, 1000.0f, 1000U) == ERR_OUT_OF_RANGE);
    assert(!PlanningService_IsBusy());
}

static void TestSlowLongMoveKeepsItsRequestedDuration(void)
{
    ErrorCode_t result;
    const uint32_t duration_ms = 19444U;
    uint64_t submitted_ticks = 0U;
    unsigned int zero_step_frames = 0U;

    ResetMotionStub();
    assert(PlanningService_Init(HOMEPOSE_X_MM, HOMEPOSE_Y_MM, HOMEPOSE_Z_MM) == ERR_OK);
    assert(PlanningService_StartLine(28.0f, 233.0f, 160.0f, duration_ms) == ERR_PENDING);

    do {
        PlanningService_Service();
    } while (!PlanningService_TakeStartResult(&result));
    assert(result == ERR_OK);

    while (PlanningService_IsBusy()) PlanningService_Service();
    assert(s_frame_count == 973U);
    for (unsigned int i = 0U; i < s_frame_count; ++i) {
        submitted_ticks += s_frames[i].total_ticks;
        if (s_frames[i].delta_m1 == 0 && s_frames[i].delta_m2 == 0 &&
            s_frames[i].delta_m3 == 0)
            zero_step_frames++;
    }
    assert(submitted_ticks >= (uint64_t)duration_ms * TICKS_PER_MS);
    assert(zero_step_frames > 0U);
}

int main(void)
{
    TestPlanningFacade();
    TestUnreachableTarget();
    TestSlowLongMoveKeepsItsRequestedDuration();
    puts("planning_service_tests: passed");
    return 0;
}
