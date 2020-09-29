
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;
typedef int64_t     s64;

typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;
typedef uint64_t    u64;

static inline char * _bn(char *s)
{
    char *p = strrchr(s, '/');
    return p ? &p[1] : s;
}

#define RESET_CLR         "0"
#define GREY              "30"
#define RED               "31"
#define BROWN             "32"
#define YELLOW            "33"
#define BLUE              "34"
#define PURPLE            "35"
#define CYAN              "36"
#define WHITE             "37"
#define GREEN             "38"
#define BOLD              "1"
#define RESET_BOLD        "21"
#define BOLD_TXT()         textattr(BOLD)
#define RESET_BOLD_TXT()   textattr(RESET_BOLD)
#define RESET_TXT_CLR()    textattr(RESET_CLR)
#define GREEN_TXT()        textattr(GREEN)
#define GREY_TXT()         textattr(GREY)
#define RED_TXT()          textattr(RED)
#define YELLOW_TXT()       textattr(YELLOW)
#define WHITE_TXT()        textattr(WHITE)
#define BLUE_TXT()         textattr(BLUE)
#define CYAN_TXT()         textattr(CYAN)
static inline void textattr(char *s)
{
    printf("\033[%sm", s);
}

#define DBGFMT              "%s:%d %s() "
#define DBGARGS             _bn(__FILE__), __LINE__, __func__
#define DBG(fmt, a...)      fprintf(stderr, DBGFMT fmt "\n", DBGARGS, ## a)

#define INFO                printf

#define DIM(x)              (sizeof(x)/sizeof((x)[0]))

#define BIT(x)              (1 << (x))

#ifndef MIN
#define MIN(x, y)           ((x) < (y) ? (x) : (y))
#endif

#define offset_of(s, e)         ((intptr_t)&((s *)NULL)->e)
#define container_of(x, s, e)   ((void*)((intptr_t)(x) - offset_of(s, e)))

static inline uint64_t tickcount_us(void)
{
#if 1
    struct timespec t;

    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t))
        return 0;
    return t.tv_sec * 1000000 + (t.tv_nsec/1000);
#else
    struct timeval tv;

    if (!gettimeofday(&tv, NULL))
        return 0;
    return tv.tv_sec * 1000000 + tv.tv_usec;
#endif
}
