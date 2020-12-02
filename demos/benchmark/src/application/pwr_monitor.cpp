/*
    pwr_monitor.cpp

    Utility to measure power consumption of the ZCU102 board from Linux userspace.

    Modified version of code by Xilinx forum user "pmpakos": https://forums.xilinx.com/t5/Xilinx-Evaluation-Boards/Power-monitoring-through-Linux-application-ZCU102/td-p/810128
    Based on https://github.com/witjaksana/zcu_power_monitor which itself is based on https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/18841995/Zynq-7000+AP+SoC+Low+Power+Techniques+part+4+-+Measuring+ZC702+Power+with+a+Linux+Application+Tech+Tip
*/

#include "pwr_monitor.h"
#include <iostream>
#include <fstream>

void writeData(int fdi2c, unsigned char address, unsigned char reg, int value){
    int status;

    if (ioctl(fdi2c, I2C_SLAVE_FORCE, address) < 0){
        printf("ERROR: Unable to set I2C slave address 0x%02X\n", address);
        exit(1);
    }

    status = i2c_smbus_write_byte_data(fdi2c, CMD_PAGE, address);
    if (status < 0) {
        printf("ERROR: Unable to write page address to I2C slave at 0x%02X: %d\n", address, status);
        exit(1);
    }

    value = (value >> 8) | ((value & 0xff) << 8); /** turn the byte around */
    status = i2c_smbus_write_word_data(fdi2c, reg, value);
    if (status < 0) {
        printf("ERROR: Unable to write value to I2C reg at 0x%02X: %d\n", reg, status);
        exit(1);
    }
}

int readData(int fdi2c, unsigned char address, unsigned char reg){
    int status;
    int value;

    if (ioctl(fdi2c, I2C_SLAVE_FORCE, address) < 0){
        printf("ERROR: Unable to set I2C slave address 0x%02X\n", address);
        exit(1);
    }

    status = i2c_smbus_write_byte_data(fdi2c, CMD_PAGE, address);
    if (status < 0) {
        printf("ERROR: Unable to write page address to I2C slave at 0x%02X: %d\n", address, status);
        exit(1);
    }

    value = i2c_smbus_read_word_data(fdi2c, reg);
    value = (value >> 8) | ((value & 0xff) << 8); /** turn the byte around */
    return value;
}

double readBusVoltage(int fdi2c, unsigned char address){
    int raw_value;
    double voltage;

    raw_value = readData(fdi2c, address, REG_BUS_V);

    voltage = (float)raw_value * 0.00125; //LSB = 1.25 [mV]
    return voltage;
}

double readCurrent(int fdi2c, unsigned char address){
    int raw_value;
    double current;

    raw_value = readData(fdi2c, address, REG_CURRENT);
    // in case it's negative
    if ((raw_value & 0x8000) != 0){
        raw_value |= 0xffff0000;
    }

    current = (float)raw_value; //LSB = Current_LSB set to 1mA via calibration_values
    return current;
}

double readPower(int fdi2c, unsigned char address){
    int raw_value;
    double power;

    raw_value = readData(fdi2c, address, REG_POWER);

    power = (float)raw_value * 0.025; //LSB = 25 * Current_LSB [A]
    return power;
}

void initCalibRail(int fdi2c, struct voltage_rail rail[], int j){
    //calibration_value = 0.00512/(Current_LSB * Shunt_Resistance)
    //set in power_main.cpp
    //for convenience we use Current_LSB = 1mA for all rails

    for (int i = 0; i < j; i++){
        writeData(fdi2c,rail[i].device,REG_CAL,rail[i].calibration_value); 
    }
}


int start;
pthread_mutex_t mutex;

int stop;
pthread_t  thread_ID;
void      *exit_status;

// Full Power Domain
struct voltage_rail zcu102_fp_rails[]={
    {
          "vccpsintfp     ",
          0x40,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "mgtravcc       ",
          0x44,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "mgtravtt       ",
          0x45,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vcco_psddr_504 ",
          0x46,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vccpsddrpll    ",
          0x4b,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
};
// Low Power Domain
struct voltage_rail zcu102_lp_rails[]={
    {
          "vccpsintlp     ",
          0x41,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vccpsaux       ",
          0x42,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vccpspll       ",
          0x43,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vccops         ",
          0x47,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vccops3        ",
          0x4a,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
};
// Prog Logic Domain
struct voltage_rail zcu102_pl_rails[]={
    {
          "vccint         ",
          0x40,
          0,
          0, 
          LOG_RAIL_TRUE,
          2560 //careful: different shunt resistance (0.002 Ohm instead of 0.005)
    },
    {
          "vccbram        ",
          0x41,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vccaux         ",
          0x42,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vcc1v2         ",
          0x43,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vcc3v3         ",
          0x44,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "vadj_fmc       ",
          0x45,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "mgtavcc        ",
          0x46,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
    {
          "mgtavtt        ",
          0x47,
          0,
          0, 
          LOG_RAIL_TRUE,
          1024
    },
};

int fdi2c;
int fdi2c_pl;
std::ofstream logfile;

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void *zcu102_read_samples_ps(void *argv) {
  int start_flag = 0;
  int stop_flag = 0;

  int i, h;
  int loop_fp, loop_lp;
  double totalPower;
  double power;
  double fp_power, lp_power;
  double voltage;
  double current;

  double maxpower = 0.0f;

  //opening log file
  std::string logfilename = *reinterpret_cast<std::string*>(argv);
  logfile.open(logfilename);

  //opening device for read and write
  fdi2c = open(I2C_CHANNEL, O_RDWR);
  if(fdi2c < 0)
      std::cout << "Cannot open the device\n";

  loop_fp = sizeof(zcu102_fp_rails)/sizeof(struct voltage_rail);
  loop_lp = sizeof(zcu102_lp_rails)/sizeof(struct voltage_rail);

  // Initialization
  initCalibRail(fdi2c, zcu102_fp_rails, loop_fp); // init full power rails
  initCalibRail(fdi2c, zcu102_lp_rails, loop_lp); // init low power rails
  
  h = 0;
  logfile << "+---------------------------------------------------------------------------------+\n";
  logfile << "|                                       Power Monitor                             |\n";
  logfile << "|RAIL                           |   Voltage(V)    |   Current(mA) |    Power(mW)  |\n";
  logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
          
  while(1) {
      start_flag = 1;
      pthread_mutex_lock(&mutex);
      if(start == 0)
          start_flag = 0;
      pthread_mutex_unlock(&mutex);
      if(start_flag == 0) 
          continue;
      ///////////////////////////////
      totalPower = 0.0f;
      fp_power = 0.0f;
      lp_power = 0.0f;
      power = 0.0f;

      logfile << "+---------------------------------------------------------------------------------+\n";
      logfile << "|                               Full Power Domain                                 |\n";
      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";

      for(i = 0; i < loop_fp; i++) {
          voltage = readBusVoltage(fdi2c, zcu102_fp_rails[i].device);
          current = readCurrent(fdi2c, zcu102_fp_rails[i].device);
          power = voltage * current; 
          fp_power += power;
          totalPower += power;
          
          logfile << zcu102_fp_rails[i].name <<  " \t\t| " << voltage << " V\t\t" << current << " mA\t\t" << power << " mW\n";
      }

      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
      logfile << "TOTAL FULL POWER           " << fp_power << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";
      logfile << "|                                 Low Power Domain                                |\n";
      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";

      for(i = 0; i < loop_lp; i++) {
          voltage = readBusVoltage(fdi2c, zcu102_lp_rails[i].device);
          current = readCurrent(fdi2c, zcu102_lp_rails[i].device);
          power = voltage * current;
          lp_power += power;
          totalPower += power;

          logfile << zcu102_lp_rails[i].name <<  " \t\t| " << voltage << " V\t\t" << current << " mA\t\t" << power << " mW\n";
      }

      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
      logfile << "TOTAL LOW POWER            " << lp_power << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";

      if (maxpower < totalPower)
          maxpower = totalPower;

      logfile << "TOTAL POWER " << totalPower << " mW\t\t\t MAX POWER  " << maxpower << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";
      /////////////////////////////

      h++;
      stop_flag = 0;
      pthread_mutex_lock(&mutex);
      if (stop == 1)
          stop_flag = 1;
      pthread_mutex_unlock(&mutex);
      if(stop_flag==1)
          break;

      //use a fixed logging interval for now
      sleep(1); //seconds
  }

  return 0;
}

void *zcu102_read_samples_pl(void *argv) {
  int start_flag = 0;
  int stop_flag = 0;

  int i, h;
  int loop_pl;
  double totalPower;
  double power;
  double pl_power;
  double voltage;
  double current;

  double maxpower = 0.0f;

  //opening log file
  std::string logfilename = *reinterpret_cast<std::string*>(argv);
  logfile.open(logfilename);

  //opening device for read and write
  fdi2c_pl = open(I2C_CHANNEL_PL, O_RDWR);
  if(fdi2c_pl < 0)
      std::cout << "Cannot open the PL device\n";

  loop_pl = sizeof(zcu102_pl_rails)/sizeof(struct voltage_rail);

  // Initialization
  initCalibRail(fdi2c_pl, zcu102_pl_rails, loop_pl); // init logic power rails
  
  h = 0;
  logfile << "+---------------------------------------------------------------------------------+\n";
  logfile << "|                                       Power Monitor                             |\n";
  logfile << "|RAIL                           |   Voltage(V)    |   Current(mA) |    Power(mW)  |\n";
  logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
          
  while(1) {
      start_flag = 1;
      pthread_mutex_lock(&mutex);
      if(start == 0)
          start_flag = 0;
      pthread_mutex_unlock(&mutex);
      if(start_flag == 0) 
          continue;
      ///////////////////////////////
      totalPower = 0.0f;
      pl_power = 0.0f;
      power = 0.0f;

      logfile << "+-------------------------------------------------+-------------------------------+\n";
      logfile << "|                             Prog Logic Power Domain                             |\n";
      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";

      for(i = 0; i < loop_pl; i++) {
          voltage = readBusVoltage(fdi2c_pl, zcu102_pl_rails[i].device);
          current = readCurrent(fdi2c_pl, zcu102_pl_rails[i].device);
          power = voltage * current; 
          pl_power += power;
          totalPower += power;
          logfile << zcu102_pl_rails[i].name <<  " \t\t| " << voltage << " V\t\t" << current << " mA\t\t" << power << " mW\n";
      }

      if (maxpower < totalPower)
          maxpower = totalPower;

      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
      logfile << "TOTAL PROG LOGIC POWER " << totalPower << " mW\t\t\t MAX POWER  " << maxpower << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";

      h++;
      ///////////////////////////////
      stop_flag = 0;
      pthread_mutex_lock(&mutex);
      if (stop == 1)
          stop_flag = 1;
      pthread_mutex_unlock(&mutex);
      if(stop_flag==1)
          break;

      //use a fixed logging interval for now
      sleep(1); //seconds
  }

  return 0;
}

void *zcu102_read_samples_ps_and_pl(void *argv) {
  int start_flag = 0;
  int stop_flag = 0;

  int i, h;
  int loop_fp, loop_lp, loop_pl;
  double totalPower;
  double power;
  double fp_power, lp_power, pl_power;
  double voltage;
  double current;

  double maxpower = 0.0f;

  //opening log file
  std::string logfilename = *reinterpret_cast<std::string*>(argv);
  logfile.open(logfilename);

  //opening device for read and write
  fdi2c = open(I2C_CHANNEL, O_RDWR);
  if(fdi2c < 0)
      std::cout << "Cannot open the device\n";
  fdi2c_pl = open(I2C_CHANNEL_PL, O_RDWR);
  if(fdi2c_pl < 0)
      std::cout << "Cannot open the PL device\n";

  loop_fp = sizeof(zcu102_fp_rails)/sizeof(struct voltage_rail);
  loop_lp = sizeof(zcu102_lp_rails)/sizeof(struct voltage_rail);
  loop_pl = sizeof(zcu102_pl_rails)/sizeof(struct voltage_rail);

  // Initialization
  initCalibRail(fdi2c, zcu102_fp_rails, loop_fp); // init full power rails
  initCalibRail(fdi2c, zcu102_lp_rails, loop_lp); // init low power rails
  initCalibRail(fdi2c_pl, zcu102_pl_rails, loop_pl); // init logic power rails
  
  h = 0;
  logfile << "+---------------------------------------------------------------------------------+\n";
  logfile << "|                                       Power Monitor                             |\n";
  logfile << "|RAIL                           |   Voltage(V)    |   Current(mA) |    Power(mW)  |\n";
  logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
          
  while(1) {
      start_flag = 1;
      pthread_mutex_lock(&mutex);
      if(start == 0)
          start_flag = 0;
      pthread_mutex_unlock(&mutex);
      if(start_flag == 0) 
          continue;
      ///////////////////////////////
      totalPower = 0.0f;
      fp_power = 0.0f;
      lp_power = 0.0f;
      pl_power = 0.0f;
      power = 0.0f;

      logfile << "+---------------------------------------------------------------------------------+\n";
      logfile << "|                               Full Power Domain                                 |\n";
      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";

      for(i = 0; i < loop_fp; i++) {
          voltage = readBusVoltage(fdi2c, zcu102_fp_rails[i].device);
          current = readCurrent(fdi2c, zcu102_fp_rails[i].device);
          power = voltage * current; 
          fp_power += power;
          totalPower += power;
          
          logfile << zcu102_fp_rails[i].name <<  " \t\t| " << voltage << " V\t\t" << current << " mA\t\t" << power << " mW\n";
      }

      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
      logfile << "TOTAL FULL POWER           " << fp_power << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";
      logfile << "|                                 Low Power Domain                                |\n";
      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";

      for(i = 0; i < loop_lp; i++) {
          voltage = readBusVoltage(fdi2c, zcu102_lp_rails[i].device);
          current = readCurrent(fdi2c, zcu102_lp_rails[i].device);
          power = voltage * current;
          lp_power += power;
          totalPower += power;

          logfile << zcu102_lp_rails[i].name <<  " \t\t| " << voltage << " V\t\t" << current << " mA\t\t" << power << " mW\n";
      }

      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
      logfile << "TOTAL LOW POWER            " << lp_power << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";
      logfile << "|                             Prog Logic Power Domain                             |\n";
      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";

      for(i = 0; i < loop_pl; i++) {
          voltage = readBusVoltage(fdi2c_pl, zcu102_pl_rails[i].device);
          current = readCurrent(fdi2c_pl, zcu102_pl_rails[i].device);
          power = voltage * current; 
          pl_power += power;
          totalPower += power;
          logfile << zcu102_pl_rails[i].name <<  " \t\t| " << voltage << " V\t\t" << current << " mA\t\t" << power << " mW\n";
      }

      logfile << "+-------------------------------+-----------------+---------------+---------------+\n";
      logfile << "TOTAL PROG LOGIC POWER " << pl_power << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";


      if (maxpower < totalPower)
          maxpower = totalPower;

      logfile << "TOTAL POWER " << totalPower << " mW\t\t\t MAX POWER  " << maxpower << " mW\n";
      logfile << "+-------------------------------------------------+-------------------------------+\n";
      /////////////////////////////

      h++;
      stop_flag = 0;
      pthread_mutex_lock(&mutex);
      if (stop == 1)
          stop_flag = 1;
      pthread_mutex_unlock(&mutex);
      if(stop_flag==1)
          break;

      //use a fixed logging interval for now
      sleep(1); //seconds
  }

  return 0;
}
/////////////////////////////////////////////////////////////////////////////////////
void zcu102_read_sample_pthread(const std::string& logfilename, int ps_or_pl) {
  pthread_mutex_init(&mutex, NULL);  
	pthread_mutex_lock(&mutex);
	stop = 0;
	start = 0;
	pthread_mutex_unlock(&mutex);

	cpu_set_t cpu_set2;
	CPU_SET(1, &cpu_set2);
  if(ps_or_pl == 0)
    pthread_create(&thread_ID, NULL, zcu102_read_samples_ps,(void*)&logfilename);
  else if(ps_or_pl == 1)
    pthread_create(&thread_ID, NULL, zcu102_read_samples_pl,(void*)&logfilename);
  else
    pthread_create(&thread_ID, NULL, zcu102_read_samples_ps_and_pl,(void*)&logfilename);
	sched_setaffinity(thread_ID, sizeof(cpu_set_t), &cpu_set2);
}

void zcu102_read_sample_start() {
	pthread_mutex_lock(&mutex);
	start = 1;
	stop  = 0;
	pthread_mutex_unlock(&mutex);
}
/////////////////////////////////////////////////////////////////////////////////////
void power_monitoring_start(const std::string& logfilename, int ps_or_pl){
	zcu102_read_sample_pthread(logfilename, ps_or_pl);
	zcu102_read_sample_start();  
}
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void zcu102_close_thread(){
	pthread_mutex_lock(&mutex);
	stop = 1;
  //start = 0; this can cause a deadlock, especially with a sleep() in the thread loop!
  pthread_mutex_unlock(&mutex);
	pthread_join(thread_ID, &exit_status);
  pthread_detach(thread_ID);
}

void zcu102_close_files(int ps_or_pl){
    logfile.close();
    if(ps_or_pl == 0)
      close(fdi2c);
    else if(ps_or_pl == 1)
      close(fdi2c_pl);
    else{
      close(fdi2c);
      close(fdi2c_pl);
    }

}
/////////////////////////////////////////////////////////////////////////////////////
void power_monitoring_stop(int ps_or_pl){
    zcu102_close_thread();
    zcu102_close_files(ps_or_pl);
}
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
