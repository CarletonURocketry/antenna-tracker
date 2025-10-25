#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/gnss.h>
#include <nuttx/signal.h>
#include <debug.h>

#include "radio_sensor.h"

struct radio_sensor_s
{
  union
    {
      struct sensor_lowerhalf_s lower;
      struct gnss_lowerhalf_s gnss;
    };

  int type;
  struct file data;
  uint32_t interval;
  uint32_t batch;
  int raw_start;
  FAR const char *file_path;
  sem_t wakeup;
  volatile bool running;
};

static struct sensor_ops_s g_fakesensor_ops =
{
  .activate = fakesensor_activate,
  .set_interval = fakesensor_set_interval,
  .batch = fakesensor_batch,
};

static struct gnss_ops_s g_fakegnss_ops =
{
  .activate = fakegnss_activate,
  .set_interval = fakegnss_set_interval,
};

int radio_sensor_init(int type, FAR const char *file_name,
                    int devno, uint32_t batch_number)
{
  FAR struct radio_sensor_s *sensor;
  FAR char *argv[2];
  char arg1[32];
  uint32_t nbuffer[] = {
    [SENSOR_GNSS_IDX_GNSS] = batch_number,
    [SENSOR_GNSS_IDX_GNSS_SATELLITE] = batch_number,
    [SENSOR_GNSS_IDX_GNSS_MEASUREMENT] = batch_number,
    [SENSOR_GNSS_IDX_GNSS_CLOCK] = batch_number,
    [SENSOR_GNSS_IDX_GNSS_GEOFENCE] = batch_number,
  };

  int ret;

  /* Alloc memory for sensor */

  sensor = kmm_zalloc(sizeof(struct radio_sensor_s));
  if (!sensor)
    {
      snerr("Memory cannot be allocated for fakesensor\n");
      return -ENOMEM;
    }

  sensor->file_path = file_name;
  sensor->type = type;

  nxsem_init(&sensor->wakeup, 0, 0);

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", sensor);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("fakesensor_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       fakesensor_thread, argv);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ERROR;
    }

  /*  Register sensor */

  if (type == SENSOR_TYPE_GNSS || type == SENSOR_TYPE_GNSS_SATELLITE)
    {
      sensor->gnss.ops = &g_fakegnss_ops;
      gnss_register(&sensor->gnss, devno, nbuffer, nitems(nbuffer));
    }
  else
    {
      sensor->lower.type = type;
      sensor->lower.ops = &g_fakesensor_ops;
      sensor->lower.nbuffer = batch_number;
      sensor_register(&sensor->lower, devno);
    }

  return OK;
}