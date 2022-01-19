#include <sys/types.h>
#include <syscall.h>

#include <unistd.h>
#include <sched.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <errno.h>

#include <linux/can.h>

#include "ioprio.h"

struct sched_attr {
    __u32 size;              /* Size of this structure */
    __u32 sched_policy;      /* Policy (SCHED_*) */
    __u64 sched_flags;       /* Flags */
    __s32 sched_nice;        /* Nice value (SCHED_OTHER,
                                SCHED_BATCH) */
    __u32 sched_priority;    /* Static priority (SCHED_FIFO,
                                SCHED_RR) */
    /* Remaining fields are for SCHED_DEADLINE */
    __u64 sched_runtime;
    __u64 sched_deadline;
    __u64 sched_period;
};

extern int set_deadline_scheduling(u_int64_t sched_runtime, u_int64_t sched_deadline, u_int64_t sched_period)
{
    struct sched_attr attributes = { .size = sizeof(struct sched_attr), .sched_policy = SCHED_DEADLINE,
                                     .sched_flags = 0, .sched_nice = 0, .sched_priority = 0, .sched_runtime = sched_runtime,
                                     .sched_deadline = sched_deadline, .sched_period = sched_period };

    return (syscall(SYS_sched_setattr, 0, &attributes, 0) == -1 ? errno : 0);
}

extern int set_ioprio_highest()
{
    return (ioprio_set(IOPRIO_WHO_PROCESS, 0 /* set for calling thread */,
        IOPRIO_PRIO_VALUE(IOPRIO_CLASS_RT, 0 /* highest priority level within RT */)) == -1 ? errno : 0);
}