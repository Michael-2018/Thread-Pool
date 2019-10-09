/*
 * Copyright (c) 2019-2020 Michael.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __TPOOL_SERVICE__
#define __TPOOL_SERVICE__

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @brief  Task cost, for load balancing.
 */
enum
{
    TPOOL_TASK_COST_LOWEST  = 1,   //!< Lowest cost, a very simple task.
    TPOOL_TASK_COST_LOWER   = 2,   //!< Lower cost, a task can be completed quickly.
    TPOOL_TASK_COST_NORMAL  = 5,   //!< Normal cost, relates to the I/O (such as printf/cout) operations.
    TPOOL_TASK_COST_HIGHER  = 50,  //!< Higher cost, a very heavy task.
    TPOOL_TASK_COST_HIGHEST = 200, //!< Highest cost, it takes a long time to perform this task.(such as to sleep, play audio).

    TPOOL_TASK_COST_BLOCK   = -1,  //!< A task running on the work-thread invoked a function that blocked waitting for something.
};

/**
 *  Initial the thread poll.
 *
 *  @param n  how many threads.
 *
 *  @return NULL if initial failed.
 *  @return Pointer of the thread-poll.
 */
void * thread_pool_init(unsigned int n);

/**
 *  Destroy the thread poll.
 *
 *  @param p  Pointer of the thread-poll.
 */
void thread_pool_destroy(void * p);

/**
 *  Add a task into thread-pool.
 *
 *  @param p     Pointer of the thread-poll.
 *  @param func  Task handler.
 *  @param arg   Task params.
 *  @param cost  This task cost.
 *
 *  @return  0 if successfully.
 *  @return -1 if failed.
 */
int thread_pool_add_task(void * p, void(* func)(void *), void * arg, int cost);

#ifdef __cplusplus
}
#endif

#endif /* __TPOOL_SERVICE__ */
