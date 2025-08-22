#pragma once

#if defined(__GNUC__)
#define PINRERUN_DEPRECATED(msg) __attribute__((deprecated(msg)))
#else
#define PINRERUN_DEPRECATED(msg)
#endif
