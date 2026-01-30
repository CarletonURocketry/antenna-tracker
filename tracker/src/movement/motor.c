#include "../syslogging.h"
#include <errno.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#include "motor.h"

typedef struct pwm_state_s {
    bool initialized;
#ifdef CONFIG_PWM_MULTICHAN
    FAR char *devpath;
    uint8_t channels[CONFIG_ANTENNA_TRACKER_PWM_NCHANNELS];
#else
    FAR char *devpath[CONFIG_ANTENNA_TRACKER_PWM_NTIMERS];
#endif
    uint8_t duties[CONFIG_ANTENNA_TRACKER_PWM_NDUTIES];
    int32_t freq;
} pwm_state_t;

static pwm_state_t pwm_state;
struct pwm_info_s pwm_info;

int pwm_state_setup(void);

int move_angle(int angle, int timer);

/*
 * Sets up pwm_state variable
 * Possibly not needed? We can use the info_s from pwm.h to store everything directly? idk want your thoughts on this
 */
int pwm_state_setup(void) {
    /* Ugly but it works and I can't think of a better way to config it */

    pwm_state.duties[0] = CONFIG_ANTENNA_TRACKER_PWM_DUTY1;
#if CONFIG_ANTENNA_TRACKER_PWM_NDUTIES > 1
    pwm_state.duties[1] = CONFIG_ANTENNA_TRACKER_PWM_DUTY2;
#endif
#if CONFIG_ANTENNA_TRACKER_PWM_NDUTIES > 2
    pwm_state.duties[2] = CONFIG_ANTENNA_TRACKER_PWM_DUTY3;
#endif

#ifdef CONFIG_PWM_MULTICHAN
    pwm_state.devpath = strdup(CONFIG_ANTENNA_TRACKER_PWM_PATH1);
    pwm_state.channels[0] = CONFIG_ANTENNA_TRACKER_PWM_CHANNEL1;
#if CONFIG_ANTENNA_TRACKER_PWM_NCHANNELS > 1
    pwm_state.channels[1] = CONFIG_ANTENNA_TRACKER_PWM_CHANNEL2;
#endif
#if CONFIG_ANTENNA_TRACKER_PWM_NCHANNELS > 2
    pwm_state.channels[2] = CONFIG_ANTENNA_TRACKER_PWM_CHANNEL3;
#endif

#else
    pwm_state.devpath[0] = strdup(CONFIG_ANTENNA_TRACKER_PWM_PATH1);
#if CONFIG_ANTENNA_TRACKER_NTIMERS > 1
    pwm_state.devpath[1] = strdup(CONFIG_ANTENNA_TRACKER_PWM_PATH2);
#endif
#if CONFIG_ANTENNA_TRACKER_NTIMERS > 2
    pwm_state.devpath[2] = strdup(CONFIG_ANTENNA_TRACKER_PWM_PATH3);
#endif

#endif

    pwm_state.freq = CONFIG_ANTENNA_TRACKER_PWM_FREQUENCY;

    pwm_state.initialized = true;
    return 0;
}

/*
 * Maps a value from one range to another. Copied from arduino implementation.
 */
static long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * Moves motor to specific angle using PWM
 */
int move_angle(int angle, int timer) {

    int ret = 0;
    if (!pwm_state.initialized) {
        inerr("PWM state not initialized\n");
        return -1;
    }

#ifdef CONFIG_PWM_MULTICHAN
    int fd = open(pwm_state.devpath, O_RDONLY);
    if (fd < 0) {
        inerr("Failed to open PWM device %s: %d\n", pwm_state.devpath, errno);
        return -1;
    }
#else
    int fd = open(pwm_state.devpath[timer], O_RDONLY);
    if (fd < 0) {
        inerr("Failed to open PWM device %s: %d\n", pwm_state.devpath[timer], errno);
        return -1;
    }
#endif

    ret = ioctl(fd, PWMIOC_GETCHARACTERISTICS, &pwm_info);
    if (ret < 0) {
        return errno;
    }
    ininfo("Current PWM frequency: %d\n", pwm_info.frequency);
    ininfo("Test\n");

    /* Need to calculate the duty cycle needed to move to specific angle */
    int duty_cycle =
        (int)map((long)angle, (long)CONFIG_ANTENNA_TRACKER_MIN_ANGLE, (long)CONFIG_ANTENNA_TRACKER_MAX_ANGLE,
                 (long)CONFIG_ANTENNA_TRACKER_MIN_DUTY_CYCLE, (long)CONFIG_ANTENNA_TRACKER_MAX_DUTY_CYCLE);
    /* Update the duty cycle in pwm_state */
    pwm_state.duties[timer] = duty_cycle;

    ininfo("Moving motor to angle %d with duty cycle %d\n", angle, duty_cycle);

    pwm_info.frequency = pwm_state.freq;

#ifdef CONFIG_PWM_MULTICHAN
    for (int i = 0; i < CONFIG_ANTENNA_TRACKER_PWM_NCHANNELS; i++) {
        pwm_info.channels[i].channel = pwm_state.channels[i];
        /* pwm_info.channels[i].duty is a value between 0 and 65000~ */
        pwm_info.channels[i].duty = pwm_state.duties[i] ? b16divi(uitoub16(pwm_state.duties[i]) - 1, 100) : 0;
        ininfo("Setting up channel %d with duty %d\n", pwm_info.channels[i].channel, pwm_info.channels[i].duty);
        ininfo("channel: %d duty: %08" PRIx32, pwm_info.channels[i].channel, pwm_info.channels[i].duty);
        ininfo("\n");
    }
#else
    /* pwm_info.duty is a value between 0 and 65000~ */
    pwm_info.duty = pwm_state.duties[timer] ? b16divi(uitoub16(pwm_state.duties[timer]) - 1, 100) : 0;
#endif

    ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm_info));
    if (ret < 0) {
        inerr("ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
        close(fd);
        return -1;
    }

    ret = ioctl(fd, PWMIOC_START, 0);
    if (ret < 0) {
        inerr("ioctl(PWMIOC_START) failed: %d\n", errno);
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}