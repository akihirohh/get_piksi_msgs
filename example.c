#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include "callback.h"


/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_t      pos_llh;
msg_baseline_ned_t baseline_ned;
msg_vel_ned_t      vel_ned;
msg_dops_t         dops;
msg_gps_time_t     gps_time;

/*
* SBP callback nodes must be statically allocated. Each message ID / callback
* pair must have a unique sbp_msg_callbacks_node_t associated with it.
*/
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t baseline_ned_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;

void sbp_setup(sbp_state_t *sbp_state)
{
  /* SBP parser state must be initialized before sbp_process is called. */
  sbp_state_init(sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback, NULL, &gps_time_node);
  sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback, NULL, &pos_llh_node);
  sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &sbp_baseline_ned_callback, NULL, &baseline_ned_node);
  sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback, NULL, &vel_ned_node);
  sbp_register_callback(sbp_state, SBP_MSG_DOPS, &sbp_dops_callback,NULL, &dops_node);
}

char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;
static sbp_msg_callbacks_node_t heartbeat_callback_node;

void usage(char *prog_name) {
  fprintf(stderr, "usage: %s [-p serial port]\n", prog_name);
}

void setup_port()
{
  int result;

  result = sp_set_baudrate(piksi_port, 115200);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set port baud rate!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set flow control!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_bits(piksi_port, 8);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set data bits!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_parity(piksi_port, SP_PARITY_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set parity!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_stopbits(piksi_port, 1);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set stop bits!\n");
    exit(EXIT_FAILURE);
  }
}

u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}

int main(int argc, char **argv)
{
  int opt;
  int result = 0;

  sbp_state_t s;

  if (argc <= 1) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }

  while ((opt = getopt(argc, argv, "p:")) != -1) {
    switch (opt) {
      case 'p':
        serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
        if (!serial_port_name) {
          fprintf(stderr, "Cannot allocate memory!\n");
          exit(EXIT_FAILURE);
        }
        strcpy(serial_port_name, optarg);
        break;
      case 'h':
        usage(argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  if (!serial_port_name) {
    fprintf(stderr, "Please supply the serial port path where the Piksi is connected!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_get_port_by_name(serial_port_name, &piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot find provided serial port!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_open(piksi_port, SP_MODE_READ);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
    exit(EXIT_FAILURE);
  }

  setup_port();
  sbp_setup(&s);

  while(1) {
    sbp_process(&s, &piksi_port_read);
  }

  result = sp_close(piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
  }

  sp_free_port(piksi_port);

  free(serial_port_name);

  return 0;
}