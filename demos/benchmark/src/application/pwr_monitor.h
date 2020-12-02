/*
    pwr_monitor.h

    Utility to measure power consumption of the ZCU102 board from Linux userspace.

    Modified version of code by Xilinx forum user "pmpakos": https://forums.xilinx.com/t5/Xilinx-Evaluation-Boards/Power-monitoring-through-Linux-application-ZCU102/td-p/810128
    Based on https://github.com/witjaksana/zcu_power_monitor which itself is based on https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/18841995/Zynq-7000+AP+SoC+Low+Power+Techniques+part+4+-+Measuring+ZC702+Power+with+a+Linux+Application+Tech+Tip
*/

#include <string>

#include <sched.h>
#include <pthread.h>
#include <sys/time.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include "i2c-dev.h"

//I2C channel
#define I2C_CHANNEL "/dev/i2c-3"
#define I2C_CHANNEL_PL "/dev/i2c-4"


//logging flag for rails
#define LOG_RAIL_FALSE 0x00
#define LOG_RAIL_TRUE 0x01

//PMBUS Commands
#define CMD_PAGE      0x00

#define DEFAULT_ITER_NB 1000

#define REG_CONFIG  0x00
#define REG_SHUNT_V 0x01
#define REG_BUS_V   0x02
#define REG_POWER   0x03
#define REG_CURRENT 0x04
#define REG_CAL     0x05
#define REG_EN      0x06
#define REG_ALERT   0x07
#define REG_ID      0xFE
#define REG_DIE     0xFF

struct voltage_rail {
    std::string name;
    unsigned char device;
    double average_current;
    double average_power;
    unsigned char log;
    int calibration_value;
};

void writeData(int fdi2c, unsigned char address, unsigned char reg, int value);
int readData(int fdi2c, unsigned char address, unsigned char reg);

double readBusVoltage(int fdi2c, unsigned char address);
double readCurrent(int fdi2c, unsigned char address);
double readPower(int fdi2c, unsigned char address);

void initCalibRail(int fdi2c, struct voltage_rail rail[], int j);

/////////////////////////////////////////////////////////////////////////////////////
// void *zcu102_read_samples(void *argv);
void *zcu102_read_samples_pl(void *argv);
void *zcu102_read_samples_ps(void *argv);
void *zcu102_read_samples_ps_and_pl(void *argv);
void zcu102_read_sample_pthread(const std::string& logfilename, int ps_or_pl);
void zcu102_read_sample_start();

void zcu102_close_thread();
void zcu102_close_files(int ps_or_pl);

void power_monitoring_start(const std::string& logfilename, int ps_or_pl);
void power_monitoring_stop(int ps_or_pl);
