#ifndef _ANTENNA_TRACKER_SYSLOGGING_H_
#define _ANTENNA_TRACKER_SYSLOGGING_H_

#include "tracker.h"

#ifndef DESKTOP_BUILD
#include <nuttx/config.h>
#endif

#define __HLOGSTR(fstring) "%s::" fstring

/* Debug output */

#ifdef CONFIG_ANTENNA_TRACKER_SYSLOG_DEBUG
#define indebug(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define indebug(fstring, ...)
#endif

/* Info output */

#ifdef CONFIG_ANTENNA_TRACKER_SYSLOG_INFO
#define ininfo(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define ininfo(fstring, ...)
#endif

/* Warning output */

#ifdef CONFIG_ANTENNA_TRACKER_SYSLOG_WARN
#define inwarn(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define inwarn(fstring, ...)
#endif

/* Error output */

#ifdef CONFIG_ANTENNA_TRACKER_SYSLOG_ERR
#define inerr(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define inerr(fstring, ...)
#endif

int setup_syslogging(void);
void syslog_tee(const char *fmt, ...);

#endif // _ANTENNA_TRACKER_SYSLOGGING_H_