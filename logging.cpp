
#include <ctime>
#include <sys/time.h>
#include <fstream>
#include <string>
#include <iomanip>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include "callback.h"
#define DESIRED_LOOP_TIME 50

//For data logging purpose. It returns a string
//with current time to uniquely name log files
const std::string currentDateTime(){
	time_t		now = time(0);
	struct tm	tstruct;
	char		buf[80];
	tstruct = *localtime(&now);
	strftime(buf,sizeof(buf), "%Y%m%d_%Hh%Mm%Ss",&tstruct);
	return buf;
}

//Given a reference tStart, it returns time 
//ellapsed in miliseconds
int millis(timeval tStart)
{
	struct timeval t;
	gettimeofday(&t,NULL);
	return (t.tv_sec - tStart.tv_sec)*1000 + (t.tv_usec - tStart.tv_usec)/1000;	
}

/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_t      pos_llh;
msg_baseline_ned_t baseline_ned;
msg_vel_ned_t      vel_ned;
msg_dops_t         dops;
msg_gps_time_t     gps_time;
msg_utc_time_t     utc_time;

/*
* SBP callback nodes must be statically allocated. Each message ID / callback
* pair must have a unique sbp_msg_callbacks_node_t associated with it.
*/
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t baseline_ned_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;
sbp_msg_callbacks_node_t utc_time_node;


/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */

void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}

void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
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
  baseline_ned = *(msg_baseline_ned_t *)msg;
  printf( "Baseline (mm):\n");
  printf( "\tNorth\t\t: %6d\n", (int)baseline_ned.n);
  printf( "\tEast\t\t: %6d\n", (int)baseline_ned.e);
  printf( "\tDown\t\t: %6d\n", (int)baseline_ned.d);
  printf( "\n");
}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
  printf( "Velocity (mm/s):\n");
  printf( "\tNorth\t\t: %6d\n", (int)vel_ned.n);
  printf( "\tEast\t\t: %6d\n", (int)vel_ned.e);
  printf( "\tDown\t\t: %6d\n", (int)vel_ned.d);
  printf( "\n");

}
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  dops = *(msg_dops_t *)msg;
  printf("Dilution of Precision:");
  printf( "\n\tGDOP\t\t: %4.2f", ((float)dops.gdop/100));
  printf( "\n\tHDOP\t\t: %4.2f", ((float)dops.hdop/100));
  printf( "\n\tPDOP\t\t: %4.2f", ((float)dops.pdop/100));
  printf( "\n\tTDOP\t\t: %4.2f", ((float)dops.tdop/100));
  printf( "\n\tVDOP\t\t: %4.2f\n", ((float)dops.vdop/100));
}
void sbp_utc_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  utc_time = *(msg_utc_time_t *)msg;
}

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
  sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &sbp_utc_callback, NULL, &utc_time_node);
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

int main(int argc, char* argv[])
{	
	std::string filename;
	filename.append(currentDateTime());
	filename.append("_gps.txt");
	std::fstream gps_file(filename.c_str(), std::ios_base::out);
  gps_file << "utc_time_tow | utc_time_flags | utc_time_year |  utc_time_month | utc_time_day | utc_time_hours | utc_time_minutes | utc_time_seconds | utc_time_ns | pos_llh_tow | pos_llh_flags | pos_llh_h_accuracy | pos_llh_v_accuracy |  pos_llh_lat | pos_llh_lon | pos_llh_n_sats | baseline_ned_tow | baseline_ned_flags |  baseline_ned_n | baseline_ned_e | baseline_ned_d | dops_tow | dops_flags | dops_gdop | dops_pdop | dops_tdop | dops_hdop | dops_vdop " <<  std::endl;
	int looptime;

	struct timeval t;

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

	gettimeofday(&t, NULL);
	while(1)
	{
    sbp_process(&s, &piksi_port_read);
		looptime = millis(t);
		if (looptime > DESIRED_LOOP_TIME)
		{
			gettimeofday(&t,NULL);
			gps_file << utc_time.tow << "|" << (int)utc_time.flags << "|" << utc_time.year << "|" << (int) utc_time.month << "|" << (int)utc_time.day << "|" << (int)utc_time.hours << "|" << (int)utc_time.minutes << "|" << (int)utc_time.seconds << "|" << utc_time.ns << "|" << pos_llh.tow << "|" << (int)pos_llh.flags << "|" << pos_llh.h_accuracy << "|" << pos_llh.v_accuracy << "|" <<  std::fixed << std::setprecision(10) << pos_llh.lat << "|" << std::fixed << std::setprecision(10) << pos_llh.lon << "|" << (int)pos_llh.n_sats << "|" << baseline_ned.tow << "|" << (int)baseline_ned.flags << "|" <<  baseline_ned.n << "|" << baseline_ned.e << "|" << baseline_ned.d << "|" << dops.tow << "|" << (int)dops.flags << "|" << dops.gdop << "|" << dops.pdop << "|" << dops.tdop << "|" << dops.hdop << "|" << dops.vdop <<  std::endl;
		}
	}

  result = sp_close(piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
  }

	gps_file.close();
  sp_free_port(piksi_port);

  free(serial_port_name);

  return 0;


}