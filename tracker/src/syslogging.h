#ifndef _INSPACE_SYSLOGGING_H_
#define _INSPACE_SYSLOGGING_H_

#ifndef DESKTOP_BUILD
#include <nuttx/config.h>
#endif

#define __HLOGSTR(fstring) "%s::" fstring

/* Debug output */

#ifdef CONFIG_INSPACE_SYSLOG_DEBUG
#define indebug(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define indebug(fstring, ...)
#endif

/* Info output */

#ifdef CONFIG_INSPACE_SYSLOG_INFO
#define ininfo(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define ininfo(fstring, ...)
#endif

/* Warning output */

#ifdef CONFIG_INSPACE_SYSLOG_WARN
#define inwarn(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define inwarn(fstring, ...)
#endif

/* Error output */

#ifdef CONFIG_INSPACE_SYSLOG_ERR
#define inerr(fstring, ...) syslog_tee(__HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define inerr(fstring, ...)
#endif

#endif // _INSPACE_SYSLOGGING_H_