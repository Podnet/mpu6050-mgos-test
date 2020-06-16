#include "mgos.h"
#include "mgos_i2c.h"
#include "mgos_imu.h"
#include "common/mbuf.h"

float *imuptr;
int count = 0;

/*! \mainpage IMU testing
 *
 * \section Documentation
 *
 * This is a test app for MPU6050. The app gives average accelerometer and gyro values.
 *
 */

float average_me(float reading, int n, float *ptr)
{

  float average = 1, sum = 0, max = 0, min = reading;
  ptr = malloc(n * sizeof(int));

  // if memory cannot be allocated
  if (ptr == NULL)
  {
    LOG(LL_ERROR, ("Error! memory not allocated."));
    exit(0);
  }
  *(ptr + count) = reading;
  count++;
  if (count == n)
  {
    count = 0;
    for (int i = 0; i < n; ++i)
    {
      sum += *(ptr + i);
      if (max < *(ptr + i))
            max = *(ptr + i);
      if (min < *(ptr + i))
            min = *(ptr + i);
      
    }
    average = sum / n;
  }

  // deallocating the memory
  free(ptr);

  return average;
}

void get_imu_reading_cb(void *user_data)
{

  LOG(LL_INFO, ("TCU: Fetching IMU reading"));
  struct mgos_imu *imu = (struct mgos_imu *)user_data;
  float ax, ay, az;

  if (!imu)
    return;

  if (mgos_imu_accelerometer_get(imu, &ax, &ay, &az))
    LOG(LL_INFO, ("TCU: type=%-10s Accel X=%.2f Y=%.2f Z=%.2f", mgos_imu_accelerometer_get_name(imu), ax, ay, az));
  ax = average_me(ax, 10, imuptr);
  ay = average_me(ay, 10, imuptr);
  az = average_me(az, 10, imuptr);
}
/**< Get acceleration values */

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

/**< Mongoose app intialization function */