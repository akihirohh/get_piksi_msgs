#include "callback.h"

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */

void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_pos_llh_t pos_llh = *(msg_pos_llh_t *)msg;
  printf("\n############################\n");
  printf( "Absolute Position:\n");
  printf( "\tLatitude\t: %4.10lf", pos_llh.lat);
  printf( "\n\tLongitude\t: %4.10lf", pos_llh.lon);
  printf( "\n\tHeight\t\t: %4.10lf", pos_llh.height);
  printf( "\n\tSatellites\t: %02d\n", pos_llh.n_sats);
  printf( "\n");
}
void sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_baseline_ned_t baseline_ned = *(msg_baseline_ned_t *)msg;
  printf( "Baseline (mm):\n");
  printf( "\tNorth\t\t: %6d\n", (int)baseline_ned.n);
  printf( "\tEast\t\t: %6d\n", (int)baseline_ned.e);
  printf( "\tDown\t\t: %6d\n", (int)baseline_ned.d);
  printf( "\n");
}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_vel_ned_t vel_ned = *(msg_vel_ned_t *)msg;
  printf( "Velocity (mm/s):\n");
  printf( "\tNorth\t\t: %6d\n", (int)vel_ned.n);
  printf( "\tEast\t\t: %6d\n", (int)vel_ned.e);
  printf( "\tDown\t\t: %6d\n", (int)vel_ned.d);
  printf( "\n");

}
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_dops_t dops = *(msg_dops_t *)msg;
  printf("Dilution of Precision:");
  printf( "\n\tGDOP\t\t: %4.2f", ((float)dops.gdop/100));
  printf( "\n\tHDOP\t\t: %4.2f", ((float)dops.hdop/100));
  printf( "\n\tPDOP\t\t: %4.2f", ((float)dops.pdop/100));
  printf( "\n\tTDOP\t\t: %4.2f", ((float)dops.tdop/100));
  printf( "\n\tVDOP\t\t: %4.2f\n", ((float)dops.vdop/100));
}
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  msg_gps_time_t gps_time = *(msg_gps_time_t *)msg;
}