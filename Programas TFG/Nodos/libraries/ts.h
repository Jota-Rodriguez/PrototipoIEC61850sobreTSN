/**
    @file ts.h
    @author Alberto Ballesteros
    @brief Timespec library.

    @note In all the functions of this library that accept a number of
    nanoseconds as a parameter, the values cannot surpass 4,294,967,295
    nanoseconds, that is, 4,294 seconds.
*/

#ifndef TS_H
#define TS_H

#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#define TIMESPEC_0 ((struct timespec){0L, 0L})

#define NSEC_PER_SEC 1000000000
#define NSEC_PER_MSEC 1000000
#define NSEC_PER_USEC 1000

#define USEC_PER_MSEC 1000

////////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////////////////////

static struct timespec timespec_from_ns(uint64_t ns);
static uint64_t timespec_get_ns(struct timespec ts);

static bool timespec_eq(struct timespec ts1, struct timespec ts2);
static bool timespec_gt(struct timespec ts1, struct timespec ts2);

static struct timespec timespec_add(struct timespec ts1, struct timespec ts2);
static void timespec_add_ns(struct timespec *ts, uint64_t ns);

static uint64_t timespec_diff(struct timespec start, struct timespec end);
static int64_t timespec_sub(struct timespec ts1, struct timespec ts2);
static void timespec_sub_ns(struct timespec *ts, uint32_t ns);

static void timespec_print(struct timespec ts);
static inline void timespec_string_print(struct timespec ts, char * buffer, size_t size);

static clockid_t get_clockid(int fd);
static struct timespec get_next_time(struct timespec period);
static struct timespec new_get_next_time(struct timespec period, clockid_t clkid);
static void smart_sleep(struct timespec ts);
void new_smart_sleep(struct timespec ts, clockid_t clkid);

/******************************************************************************/
/** @name Conversion **********************************************************/
/******************************************************************************/
//@{

/**
    @brief Converts nanoseconds to timespec.
    @param [in] ns Number of nanoseconds (up to 18,446,744,073,709,551,615).
    @return The timespec
*/
static inline struct timespec timespec_from_ns(uint64_t ns)
{
    struct timespec x = {0, 0};
    timespec_add_ns(&x, ns);
    return x;
}

/**
    @brief Converts a timespec to nanoseconds.
    @param [in] ts The timespec.
    @return The value in nanoseconds (up to 18,446,744,073,709,551,615).
*/
static inline uint64_t timespec_get_ns(struct timespec ts)
{
    return (ts.tv_nsec + NSEC_PER_SEC * (uint64_t)ts.tv_sec);
}

//@}

/******************************************************************************/
/** @name Comparison **********************************************************/
/******************************************************************************/
//@{

/**
    @brief Checks if two timespecs are equal.
    @param [in] ts1 The first timespec.
    @param [in] ts2 The second timespec.
    @return true if equal, false otherwise.
*/
static inline bool timespec_eq(struct timespec ts1, struct timespec ts2)
{
    return ((ts1.tv_sec == ts2.tv_sec) && (ts1.tv_nsec == ts2.tv_nsec));
}

/**
    @brief Checks if one timespec is bigger than another.
    @param [in] ts1 The first timespec.
    @param [in] ts2 The second timespec.
    @return true if <tt>ts1</tt> is bigger than <tt>ts2</tt>, false otherwise.
*/
static inline bool timespec_gt(struct timespec ts1, struct timespec ts2)
{
    if (ts1.tv_sec < ts2.tv_sec)
        return false;
    if (ts1.tv_sec == ts2.tv_sec)
        return (ts1.tv_nsec > ts2.tv_nsec);
    if (ts1.tv_sec > ts2.tv_sec)
        return true;
}

//@}

/******************************************************************************/
/** @name Addition ************************************************************/
/******************************************************************************/
//@{

/**
    @brief Adds two timespecs.
    @param [in] ts1 The first timespec.
    @param [in] ts2 The second timespec.
    @return The resultant timespec.
*/
static inline struct timespec timespec_add(struct timespec ts1, struct timespec ts2)
{
    struct timespec temp;

    if ((ts1.tv_nsec + ts2.tv_nsec) > NSEC_PER_SEC)
    {
        temp.tv_sec = ts1.tv_sec + ts2.tv_sec + 1;
        temp.tv_nsec = ts1.tv_nsec + ts2.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
        temp.tv_sec = ts1.tv_sec + ts2.tv_sec;
        temp.tv_nsec = ts1.tv_nsec + ts2.tv_nsec;
    }

    return temp;
}

/**
    @brief Adds nanoseconds to a timespec.
    @param [in, out] ts The timespec. The result is stored here.
    @param [in] ns Number of nanoseconds (up to 18,446,744,073,709,551,615).
*/
static inline void timespec_add_ns(struct timespec *ts, uint64_t ns)
{
    uint64_t temp;

    temp = ns;

    temp += ts->tv_nsec;

    while (temp >= NSEC_PER_SEC)
    {
        temp -= NSEC_PER_SEC;
        ts->tv_sec++;
    }

    ts->tv_nsec = temp;
}

//@}

/******************************************************************************/
/** @name Substraction ********************************************************/
/******************************************************************************/
//@{

/**
    @brief Calculates the difference between two timespecs.
    @param [in] start The first timespec.
    @param [in] end The second timespec.
    @return The number of nanoseconds between <tt>start</tt> and <tt>end</tt>.
            The result is always positive even if <tt>end</tt> is smaller than
            <tt>start</tt>.
*/
static inline uint64_t timespec_diff(struct timespec start, struct timespec end)
{
    struct timespec ts1, ts2;
    struct timespec temp;

    if (timespec_gt(end, start))
    {
        ts1 = start;
        ts2 = end;
    }
    else
    {
        ts1 = end;
        ts2 = start;
    }

    temp.tv_sec = ts2.tv_sec - ts1.tv_sec;
    temp.tv_nsec = ts2.tv_nsec - ts1.tv_nsec;

    if (ts1.tv_nsec > ts2.tv_nsec)
    {
        temp.tv_sec -= 1;
        temp.tv_nsec += NSEC_PER_SEC;
    }

    return timespec_get_ns(temp);
}

/**
    @brief Substracts one timespec from another.
    @param [in] ts1 The first timespec.
    @param [in] ts2 The second timespec.
    @return The number of nanoseconds between <tt>ts1</tt> and <tt>ts2</tt>. The
            result can be negative.
*/
static inline int64_t timespec_sub(struct timespec ts1, struct timespec ts2)
{
    uint64_t temp;

    temp = timespec_diff(ts1, ts2);

    if (timespec_gt(ts2, ts1))
        temp *= -1;

    return temp;
}

/**
    @brief Substracts nanoseconds from a timespec.
    @param [in,out] ts The timespec. The result is stored here.
    @param [in] ns The Number of nanoseconds (up to 4,294,967,295).
*/
static inline void timespec_sub_ns(struct timespec *ts, uint32_t ns)
{
    while (ns >= NSEC_PER_SEC)
    {
        ns -= NSEC_PER_SEC;
        ts->tv_sec--;
    }

    if (ns > ts->tv_nsec)
    {
        ts->tv_sec--;
        ns -= ts->tv_nsec;
        ts->tv_nsec = NSEC_PER_SEC - ns;
    }
    else
    {
        ts->tv_nsec -= ns;
    }
}

//@}

/******************************************************************************/
/** @name Printing ************************************************************/
/******************************************************************************/
//@{

/**
    @brief Prints a timespec in a readeable format (secs:msecs.usecs.nsecs).
    @param [in] ts The timespec.
*/
static inline void timespec_print(struct timespec ts)
{
    printf(
        "%lu:%03lu.%03lu.%03lu\n",
        ts.tv_sec,
        (ts.tv_nsec / 1000000),
        (ts.tv_nsec / 1000) % 1000,
        (ts.tv_nsec % 1000));
}

/**
    @brief Prints a timespec in a readeable format (secs:msecs.usecs.nsecs).
    @param [in] ts The timespec.
    @param return a string with 
*/
static inline void timespec_string_print(struct timespec ts, char * buffer, size_t size)
{
    snprintf(buffer, size, "%lu,%09lu",
        ts.tv_sec,
        ts.tv_nsec);
}

//@}

/******************************************************************************/
/** @name Waiting *************************************************************/
/******************************************************************************/
//@{

static clockid_t get_clockid(int fd)
{
#define CLOCKFD 3
    return (((unsigned int)~fd) << 3) | CLOCKFD;
}

/**
    @brief Starting from this time instant, determines the next time instant
           from the given period.
    @param [in] period The period of time for the next time.
    @param return The next time after the period has passed.
*/
static struct timespec get_next_time(struct timespec period)
{
    struct timespec t_act, t_next;
    int n;

    if (clock_gettime(CLOCK_REALTIME, &t_act))
    {
        perror("clock_gettime");
    } /*else {
        printf("clock time: ");
        timespec_print(t_act);
    }*/
    if (timespec_get_ns(period) == 0)
    {
        t_next.tv_sec = 0;
        t_next.tv_nsec = 0;
    }
    else
    {
        n = (timespec_get_ns(t_act) / timespec_get_ns(period));
        t_next = timespec_from_ns((n + 1) * timespec_get_ns(period));
    }

    /*printf("t_act: ");
    timespec_print(t_act);
    printf("cycle_time: ");
    timespec_print(period);
    printf("n: %i\n", n);
    printf("t_next: ");
    timespec_print(t_next);*/

    return t_next;
}

/**
    @brief Starting from this time instant, determines the next time instant
           from the given period.
    @param [in] period The period of time for the next time.
    @param [in] clkid The id of the clock wnating to use.
    @param return The next time after the period has passed.
*/
static struct timespec new_get_next_time(struct timespec period, clockid_t clkid)
{
    struct timespec t_act, t_next;

    if (clock_gettime(clkid, &t_act))
    {
        perror("clock_gettime");
    } /*else {
        printf("clock time: ");
        timespec_print(t_act);
    }*/

    int n = (timespec_get_ns(t_act) / timespec_get_ns(period));
    t_next = timespec_from_ns((n + 1) * timespec_get_ns(period));

    /*printf("t_act: ");
    timespec_print(t_act);
    printf("cycle_time: ");
    timespec_print(period);
    printf("n: %i\n", n);
    printf("t_next: ");
    timespec_print(t_next);*/

    return t_next;
}

/* Clocknanosleep function */
#define BUSY_WAIT_THRESH_US 900 // us

/**
    @brief Performs a smart sleep.
    @details Most of the time the thread is slept thanks to a clock_nanosleep
             function. However, the thread is woken up before the corresponding
             time to perform a busy wait. This makes the thread most responsive
             without having to perform a full busy wait.
    @param [in] ts The timespec insidicating the time to wake up.
*/
static inline void smart_sleep(struct timespec ts)
{
    struct timespec now;
    struct timespec sleep;
    signed long long temp;

    // printf("ts: ");
    // timespec_print(ts);
    clock_gettime(CLOCK_REALTIME, &now);
    // printf("now 1: ");
    // timespec_print(now);

    temp = timespec_sub(ts, now);
    // printf("temp: ");
    // timespec_print(timespec_from_ns(temp));

    if (temp > (BUSY_WAIT_THRESH_US * 1000))
    {
        sleep = timespec_from_ns(temp - (BUSY_WAIT_THRESH_US * 1000));
        clock_nanosleep(CLOCK_REALTIME, 0, &sleep, NULL);
        // printf("sleep: ");
        // timespec_print(sleep);

        clock_gettime(CLOCK_REALTIME, &now);
        // printf("now 2: ");
        // timespec_print(now);
    }

    if (timespec_gt(ts, now))
    {
        while (timespec_gt(ts, now))
        {
            clock_gettime(CLOCK_REALTIME, &now);
            // printf("now 3: ");
            // timespec_print(now);
        }
    }
    else
    {
        // printf("No busy wait, this means the thread slept too much. Consider increasing 'BUSY_WAIT_THRESH_US'\n");
    }
}

/**
    @brief Performs a smart sleep, but you can indicate the id of the clock you
           want to use.
    @param [in] ts The timespec insidicating the time to wake up.
    @param [in] clkid The id of the clock
*/
void new_smart_sleep(struct timespec ts, clockid_t clkid)
{
    struct timespec now;
    struct timespec sleep;
    signed long long temp;

    clock_gettime(clkid, &now);

    temp = timespec_sub(ts, now);

    if (temp > (BUSY_WAIT_THRESH_US * 1000))
    {
        sleep = timespec_from_ns(temp - (BUSY_WAIT_THRESH_US * 1000));
        clock_nanosleep(clkid, 0, &sleep, NULL);

        clock_gettime(clkid, &now);
    }

    if (timespec_gt(ts, now))
    {
        while (timespec_gt(ts, now))
        {
            clock_gettime(clkid, &now);
        }
        // printf("now: ");
        // timespec_print(now);
    }
    else
    {
        printf("No busy wait, this means the thread slept too much. Consider increasing 'BUSY_WAIT_THRESH_US'\n");
    }
}

//@}

#endif
