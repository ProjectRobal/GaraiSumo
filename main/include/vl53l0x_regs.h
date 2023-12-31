#pragma once

#define VL53L0X_ADRESS 0x29

#define REG_IDENTIFICATION_MODEL_ID (0xC0)
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV (0x89)
#define REG_MSRC_CONFIG_CONTROL (0x60)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)
#define REG_SYSTEM_SEQUENCE_CONFIG (0x01)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET (0x4F)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD (0x4E)
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT (0xB6)
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO (0x0A)
#define REG_GPIO_HV_MUX_ACTIVE_HIGH (0x84)
#define REG_SYSTEM_INTERRUPT_CLEAR (0x0B)
#define REG_RESULT_INTERRUPT_STATUS (0x13)
#define REG_SYSRANGE_START (0x00)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 (0xB0)
#define REG_RESULT_RANGE_STATUS (0x14)
#define REG_SLAVE_DEVICE_ADDRESS (0x8A)

#define RANGE_SEQUENCE_STEP_TCC (0x10) /* Target CentreCheck */
#define RANGE_SEQUENCE_STEP_MSRC (0x04) /* Minimum Signal Rate Check */
#define RANGE_SEQUENCE_STEP_DSS (0x28) /* Dynamic SPAD selection */
#define RANGE_SEQUENCE_STEP_PRE_RANGE (0x40)
#define RANGE_SEQUENCE_STEP_FINAL_RANGE (0x80)

#define VL53L0X_EXPECTED_DEVICE_ID (0xEE)
#define VL53L0X_DEFAULT_ADDRESS (0x29)

#define REG_MSRC_CONFIG_TIMEOUT_MACROP (0x46)

#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD (0x50)

#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI (0x51)

#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO  (0x52)

#define REG_SYSTEM_HISTOGRAM_BIN                        (0x81)
#define REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       (0x33)
#define REG_HISTOGRAM_CONFIG_READOUT_CTRL               (0x55)

#define REG_FINAL_RANGE_CONFIG_MIN_SNR                  (0x67)
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW          (0x47)
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         (0x48)
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT (0x44)

#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI            (0x61)
#define REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO            (0x62)

#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             (0x70)
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        (0x71)
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        (0x72)
#define REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       (0x20)

#define REG_MSRC_CONFIG_TIMEOUT_MACROP                  (0x46)

#define REG_SOFT_RESET_GO2_SOFT_RESET_N                 (0xBF)
#define REG_IDENTIFICATION_MODEL_ID                     (0xC0)
#define REG_IDENTIFICATION_REVISION_ID                  (0xC2)

#define REG_OSC_CALIBRATE_VAL                           (0xF8)

#define REG_PRE_RANGE_CONFIG_MIN_SNR                    (0x27)
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW            (0x56)
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH           (0x57)
#define REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          (0x64)

#define REG_GLOBAL_CONFIG_VCSEL_WIDTH                   (0x32)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            (0xB0)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1            (0xB1)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2            (0xB2)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3            (0xB3)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4            (0xB4)
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5            (0xB5)

#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT           (0xB6)
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         (0x4E)
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            (0x4F)
#define REG_POWER_MANAGEMENT_GO1_POWER_FORCE            (0x80)

#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           (0x89)

#define REG_ALGO_PHASECAL_LIM                           (0x30)
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT                (0x30)
/* There are two types of SPAD: aperture and non-aperture. My understanding
 * is that aperture ones let it less light (they have a smaller opening), similar
 * to how you can change the aperture on a digital camera. Only 1/4 th of the
 * SPADs are of type non-aperture. */
#define SPAD_TYPE_APERTURE (0x01)
/* The total SPAD array is 16x16, but we can only activate a quadrant spanning 44 SPADs at
 * a time. In the ST api code they have (for some reason) selected 0xB4 (180) as a starting
 * point (lies in the middle and spans non-aperture (3rd) quadrant and aperture (4th) quadrant). */
#define SPAD_START_SELECT (0xB4)
/* The total SPAD map is 16x16, but we should only activate an area of 44 SPADs at a time. */
#define SPAD_MAX_COUNT (44)
/* The 44 SPADs are represented as 6 bytes where each bit represents a single SPAD.
 * 6x8 = 48, so the last four bits are unused. */
#define SPAD_MAP_ROW_COUNT (6)
#define SPAD_ROW_SIZE (8)
/* Since we start at 0xB4 (180), there are four quadrants (three aperture, one aperture),
 * and each quadrant contains 256 / 4 = 64 SPADs, and the third quadrant is non-aperture, the
 * offset to the aperture quadrant is (256 - 64 - 180) = 12 */
#define SPAD_APERTURE_START_INDEX (12)