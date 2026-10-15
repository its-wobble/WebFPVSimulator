/*
 * bf_bank.c: the bank flight mode's seam to the simulator. BANK=1 builds only.
 *
 * The bank mode is developed in a newer Betaflight checkout and staged into
 * build/bank by scripts/bank-sync.sh. This file gives it what fc/core.c, the
 * scheduler and the accelerometer driver give it on hardware, and nothing
 * else. That is the rule for everything here: bank_mode.c may only see what a
 * flight controller sees (gyro, accelerometer, attitude, the RC link, arm
 * state). Never plant velocity, true drag or true thrust to weight, because a
 * pass that depends on those says nothing about real hardware.
 *
 * THE ACCELEROMETER. sim.c measures specific force (all non gravitational
 * force per unit mass, contact included) in the body frame from the craft's
 * actual velocity change over each step, which is what a MEMS accelerometer
 * measures. This file turns it into what an ICM-42688-P on a 16 g range
 * reports: 2048 LSB per g, int16 counts, saturating at the range, plus white
 * noise at the part's datasheet density over its bandwidth. Betaflight's own
 * sensors/acceleration.c then does what it does on hardware: alignment, trims
 * and the acc_lpf_hz PT2 (25 Hz, the 4.5.1 default). Calibration is taken as
 * done, as it is on a configured quad.
 *
 * Not modelled yet, and worth knowing when reading a result: frame vibration
 * on the accelerometer (the gyro has it, see sim_gyro_add_vibration), bias
 * left after calibration, and the attitude estimate. Attitude is the plant's
 * true attitude, as in angle mode, not imu.c's estimate from gyro and acc.
 * On hardware a sustained drag force pulls that estimate, so a bank mode
 * result here is optimistic about attitude until imu.c is compiled in.
 *
 * MODE SELECTION. On hardware BOXBANK is an aux switch and processRxModes()
 * in fc/core.c sets BANK_MODE. This build has no aux channels and does not
 * compile core.c, so bf_bank_step() is the stand in for that slice of it,
 * with core.c's preconditions and its rule that BANK overrides ANGLE.
 *
 * This file is part of WebFPVSimulator.
 *
 * WebFPVSimulator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * WebFPVSimulator is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WebFPVSimulator. If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef SIM_BANK_MODE

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"
#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "flight/bank_mode.h"
#include "flight/failsafe.h"
#include "pg/bank_mode.h"
#include "sensors/acceleration.h"
#include "sensors/acceleration_init.h"
#include "sensors/sensors.h"

#include "../sim_abi.h"
#include "../sim_internal.h"

/* Specific force in the body frame, m/s^2, written by sim.c every step. */
extern double SIM_ACC_BODY[3];

/* The part: ICM-42688-P at +-16 g, as accgyro_spi_icm426xx.c configures it. */
#define BANK_ACC_1G 2048
#define BANK_ACC_SAMPLE_HZ 1000
/* 70 ug/sqrt(Hz) datasheet density over the ~500 Hz output bandwidth at
 * 1 kHz ODR, rounded up: 0.0016 g RMS. */
#define BANK_ACC_NOISE_G_RMS 0.0016

/* From acceleration_init.c, which this build does not compile because it is
 * mostly hardware detection. accelerationRuntime is defined here instead. */
accelerationRuntime_t accelerationRuntime;
static flightDynamicsTrims_t g_acc_trims;

bool accIsCalibrationComplete(void) { return true; }
void performAcclerationCalibration(rollAndPitchTrims_t *t) { UNUSED(t); }
void performInflightAccelerationCalibration(rollAndPitchTrims_t *t) { UNUSED(t); }

static int g_bank_switch = 0;
static unsigned int g_acc_seed;
static uint32_t g_acc_due_us;

/* Approximately standard normal from a sum of uniforms, integer xorshift32
 * so it is bit identical everywhere. Twelve uniforms give variance 1. */
static double bank_gauss(void) {
  double sum = 0.0;
  for (int i = 0; i < 12; i += 1) {
    unsigned int x = g_acc_seed;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_acc_seed = x;
    sum += (double)x * (1.0 / 4294967296.0);
  }
  return sum - 6.0;
}

static bool sim_acc_read(accDev_t *dev) {
  /* In units of the LOCAL g, so a craft at rest reads 1 g whatever the
   * Weight setting: the shell flies the five inch at 1.62 g, and a quad on
   * Earth never reads more than 1 g at rest. The world at 1.62 g is felt as
   * a heavier craft, which is what the firmware sees on hardware. At rest
   * the specific force is the scene's gravity over len_scale (sim.c,
   * sim_measure_specific_force), so that is the 1 g here. */
  const double one_g = PLANT_W.gravity * SIM_GRAVITY / PLANT.len_scale;
  for (int a = 0; a < 3; a += 1) {
    const double g = SIM_ACC_BODY[a] / one_g + BANK_ACC_NOISE_G_RMS * bank_gauss();
    double counts = g * BANK_ACC_1G;
    counts = counts < -32768.0 ? -32768.0 : (counts > 32767.0 ? 32767.0 : counts);
    /* Round to nearest, as the part's ADC does. */
    dev->ADCRaw[a] = (int16_t)(counts < 0.0 ? counts - 0.5 : counts + 0.5);
  }
  return true;
}

void bf_bank_init(void) {
  /* Betaflight's default, from pgResetFn_accelerometerConfig. bf_stubs.c's
   * config is zero filled, and 0 would switch the filter off. */
  accelerometerConfigMutable()->acc_lpf_hz = 25;
  memset(&g_acc_trims, 0, sizeof(g_acc_trims));
  memset(&accelerationRuntime, 0, sizeof(accelerationRuntime));
  accelerationRuntime.accelerationTrims = &g_acc_trims;

  memset(&acc, 0, sizeof(acc));
  acc.dev.acc_1G = BANK_ACC_1G;
  acc.dev.acc_1G_rec = 1.0f / BANK_ACC_1G;
  acc.dev.accAlign = CW0_DEG;
  acc.dev.readFn = sim_acc_read;
  acc.sampleRateHz = BANK_ACC_SAMPLE_HZ;
  /* accInitFilters, verbatim in effect. */
  accelerationRuntime.accLpfCutHz = accelerometerConfig()->acc_lpf_hz;
  const float k = pt2FilterGain(accelerationRuntime.accLpfCutHz, 1.0f / acc.sampleRateHz);
  for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
    pt2FilterInit(&accelerationRuntime.accFilter[axis], k);
  }

  g_acc_seed = 0x6C8E9CF5u;
  g_acc_due_us = 0;
  /* bank_mode.c resets itself whenever BANK_MODE is down. */
  flightModeFlags &= (uint16_t)~BANK_MODE;
  bankModeUpdate(0);
}

void bf_bank_set_switch(int on) { g_bank_switch = on ? 1 : 0; }
int bf_bank_switch(void) { return g_bank_switch; }

/* Mode flags: before attitude is fed and before the PID, every step. */
bool bf_bank_mode_flags(void) {
  /* fc/core.c processRxModes, the BANK part. Fixed wing, 3D, GPS rescue,
   * altitude and position hold do not exist in this build. */
  const bool requested = g_bank_switch && sensors(SENSOR_ACC) && !failsafeIsActive();
  if (requested) {
    /* BANK is built on ANGLE, as core.c sets it. */
    flightModeFlags |= BANK_MODE | ANGLE_MODE;
    flightModeFlags &= (uint16_t)~HORIZON_MODE;
  } else {
    flightModeFlags &= (uint16_t)~BANK_MODE;
  }
  return requested;
}

/* The accelerometer task, at the part's rate, as TASK_ACCEL runs it. */
void bf_bank_acc_task(timeUs_t now_us) {
  if ((int32_t)(now_us - g_acc_due_us) >= 0) {
    accUpdate(now_us);
    g_acc_due_us = now_us + 1000000u / BANK_ACC_SAMPLE_HZ;
  }
}

/* processRxModes runs on a new RC frame, before updateRcCommands. */
void bf_bank_rx_frame(timeUs_t now_us) {
  /* The bank build always logs DEBUG_BANK: the mode's own estimates go into
   * debug[] as on a quad with debug_mode = BANK, and the telemetry below
   * hands them to the flight log. */
  debugMode = DEBUG_BANK;
  bankModeUpdate(now_us);
}

/*
 * Telemetry for the checks and the OSD. These are the firmware's own outputs
 * and inputs, everything a blackbox log of the mode would show:
 *  0 active, 1 roll target deg, 2 pitch target deg, 3 yaw rate dps,
 *  4 collective 0..1, 5 thrust to weight in use, 6..8 filtered acc g (x y z),
 *  9..16 debug[0..7] of DEBUG_BANK, scaled as bank_mode.c documents them.
 */
#define BANK_TELEMETRY_COUNT 17
int bf_bank_telemetry(double *out, int n) {
  if (n < BANK_TELEMETRY_COUNT) {
    return -1;
  }
  out[0] = bankModeIsActive() ? 1.0 : 0.0;
  out[1] = bankModeGetAngleTarget(FD_ROLL);
  out[2] = bankModeGetAngleTarget(FD_PITCH);
  out[3] = bankModeGetYawRate();
  out[4] = bankModeGetCollective();
  out[5] = bankModeGetThrustToWeight();
  out[6] = acc.accADC[X] * acc.dev.acc_1G_rec;
  out[7] = acc.accADC[Y] * acc.dev.acc_1G_rec;
  out[8] = acc.accADC[Z] * acc.dev.acc_1G_rec;
  for (int i = 0; i < 8; i += 1) {
    out[9 + i] = debug[i];
  }
  return BANK_TELEMETRY_COUNT;
}

#endif /* SIM_BANK_MODE */
