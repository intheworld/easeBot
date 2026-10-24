#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
// Private ABI v1. All calls for one session must be serialized by its owner.
// close releases transport only; it never enables/disables a motor.
typedef struct HeiMotorbridge HeiMotorbridge;
typedef struct {
  double position, velocity, torque;
  uint64_t age_ns, mode_age_ns, sequence;
  int32_t mode;
  uint8_t status, fault_status, mode_changed;
} HeiMotorbridgeState;
const char * hei_mb_error(void);
HeiMotorbridge * hei_mb_open(const char * path, uint32_t baud);
void hei_mb_close(HeiMotorbridge * session);
int hei_mb_add(HeiMotorbridge *, uint16_t id, uint16_t feedback_id,
  double pmax, double vmax, double tmax, int32_t expected_mode);
int hei_mb_poll(HeiMotorbridge *);
int hei_mb_refresh(HeiMotorbridge *);
int hei_mb_state(HeiMotorbridge *, uint32_t index, HeiMotorbridgeState *);
int hei_mb_enable(HeiMotorbridge *, uint32_t index, uint8_t enable);
int hei_mb_command(HeiMotorbridge *, uint32_t index, double value, double speed, uint8_t position);
// Explicit gripper7 operations; configure requires disabled feedback and only writes RAM CTRL_MODE=4.
int hei_mb_read_mode(HeiMotorbridge *, uint32_t index);
int hei_mb_configure_gripper(HeiMotorbridge *, uint32_t index);
// Optional arm setup: expected POS_VEL/VEL mode in RAM, fresh disabled feedback required.
int hei_mb_configure_arm_mode(HeiMotorbridge *, uint32_t index);
int hei_mb_force_position(HeiMotorbridge *, uint32_t index, double position, double speed, double current_ratio);
#ifdef __cplusplus
}
#endif
