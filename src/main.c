#include "mgos.h"
#include "mgos_i2c.h"
#include "mgos_imu.h"

int count = 0, check = 0;
float imu_x[3], imu_y[3], imu_z[3];
float sum_x, sum_y, sum_z;
float avg_x, avg_y, avg_z;

void get_imu_reading_cb(void *user_data)
{

  LOG(LL_INFO, ("TCU: Fetching IMU reading"));
  struct mgos_imu *imu = (struct mgos_imu *)user_data;
  float ax, ay, az;

  if (!imu)
    return;

  if (mgos_imu_accelerometer_get(imu, &ax, &ay, &az))
    LOG(LL_INFO, ("TCU: type=%-10s Accel X=%.2f Y=%.2f Z=%.2f", mgos_imu_accelerometer_get_name(imu), ax, ay, az));
}

void get_average_value(int ax, int ay, int az)
{

  if (count == 3)
  {
    count = 0;
    imu_x[count] = ax;
    imu_y[count] = ay;
    imu_z[count] = az;
    count += 1;
  }
  else
  {
    imu_x[count] = ax;
    imu_y[count] = ay;
    imu_z[count] = az;
    count += 1;
  }
  check += 1;
  if (check > 2)
  {
    for (int num = 0; num < 3; num++)
    {
      sum_x = sum_x + imu_x[num];
      sum_y = sum_y + imu_y[num];
      sum_z = sum_z + imu_z[num];
    }
  }
  avg_x = sum_x / 3;
  avg_y = sum_y / 3;
  avg_z = sum_z / 3;
  LOG(LL_INFO, ("TCU: Acceleration average value X=%.2f Y=%.2f Z=%.2f", avg_x, avg_y, avg_z));
}

enum mgos_app_init_result mgos_app_init(void)
{
  struct mgos_i2c *i2c = mgos_i2c_get_global();
  struct mgos_imu *imu = mgos_imu_create();
  struct mgos_imu_acc_opts acc_opts;
  struct mgos_imu_gyro_opts gyro_opts;
  struct mgos_imu_mag_opts mag_opts;

  if (!i2c)
  {
    LOG(LL_ERROR, ("I2C bus missing, set i2c.enable=true in mos.yml"));
    return false;
  }

  if (!imu)
  {
    LOG(LL_ERROR, ("Cannot create IMU"));
    return false;
  }

  acc_opts.type = ACC_MPU6050;
  acc_opts.scale = 16.0; // G
  acc_opts.odr = 100;    // Hz
  if (!mgos_imu_accelerometer_create_i2c(imu, i2c, 0x68, &acc_opts))
    LOG(LL_ERROR, ("Cannot create accelerometer on IMU"));

  mgos_set_timer(1000, true, get_imu_reading_cb, imu);
  return true;
}