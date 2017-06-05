#ifndef CALLBACK_H
#define CALLBACK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libsbp/navigation.h>

void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
#endif