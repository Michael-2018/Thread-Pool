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
#include "tpool_service.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>

/*
 *  Common functions
 */
#define POWERS_OF_2(n)      (1 << n)                                    // 2^N           
#define DO_CALLOC(len)      calloc(len, sizeof(char))                   // Alloc and clean Memory 
#define DO_SAFELY_FREE(p)   do{ if(p!=NULL){free(p);p =NULL;} }while(0) // Free safely 

/*
 *  Atomic operations
 */
#define SYNC_VALUE(v)			__sync_val_compare_and_swap(&(v), 0, 0)         // Get
#define SYNC_VALUE_ADD(v, a)	__sync_fetch_and_add(&(v), a)                   // Increase
#define SYNC_VALUE_SUB(v, s)	__sync_fetch_and_sub(&(v), s)                   // Decrease 
#define SYNC_VALUE_SET_TRUE(v)  __sync_bool_compare_and_swap(&(v), false, true) // Set as true
#define SYNC_VALUE_SET_FALSE(v) __sync_bool_compare_and_swap(&(v), true, false) // Set as false

/*
 *  2^N, range of N: 0~7
 */
#define WTASK_LIST_SIZE         POWERS_OF_2(6)

/*
 *  New a thread-pool
 */
#define NEW_THREAD_POOL(n)      (thread_pool_t *)DO_CALLOC(sizeof(thread_pool_t) + sizeof(queue_t) * n)

/*
 *  Queue operations
 */
#define QUEUE_IS_BROKEN(p)      ((p)->pipe_fd[0] == -1 || (p)->pipe_fd[1] == -1)
#define QUEUE_SET_AS_BROKEN(p)  do{ (p)->pipe_fd[0] = (p)->pipe_fd[1] = -1; }while(0)
#define QUEUE_CLOSE_PIPE(p)     do{ close((p)->pipe_fd[0]); close((p)->pipe_fd[1]); }while(0)
#define QUEUE_NOTIFY_THREAD(p)  do{ char _r='r';if(write((p)->pipe_fd[1],&_r,1)<=0)break; }while(0)
#define QUEUE_CLOSE_THREAD(p)   do{ char _x='x';if(write((p)->pipe_fd[1],&_x,1)>0)pthread_join((p)->tid,0); }while(0)

/*
 *  List operations
 */
#define WTASK_LIST_LEN(list)       ((list)->in - SYNC_VALUE((list)->out))
#define WTASK_LIST_IS_EMPTY(list)  (WTASK_LIST_LEN(list) == 0)
#define WTASK_LIST_IS_FULL(list)   (WTASK_LIST_LEN(list) == WTASK_LIST_SIZE)
#define WTASK_LIST_OFFSET(in_out)  (in_out & (WTASK_LIST_SIZE-1))

typedef struct {
    void (* func)(void *);
    void  * arg;
} task_t;

typedef struct {
	task_t task;
	int    cost;
} weighted_task_t;

/*
 *  Weighted task list
 */
typedef struct {
    /*  
     *  Circular queue index.
     */
    unsigned char   in,   // Start index of the pushing task.
                    out;  // Start index of the poping task.

    /*
     *  Every task has cost, total_cost is the sum of all tasks.
     *  We can to use total cost for load balancing.
     */
    unsigned int    total_cost;

    weighted_task_t wtask[WTASK_LIST_SIZE];
} wtask_list_t;

/*
 *  Queue, every thread has a task queue.
 */
typedef struct {
    pthread_t    tid;        // Thread id
    bool         working;    // Thread is busy or not

    /*
     *  Main-thread to communicate with the work-thread by a pipe.
     *  Such as wakethe work-thread up to work.
     */
    int          pipe_fd[2];
    
    wtask_list_t wtask_list; 
} queue_t;

/*
 *  Thread-pool, including a group of task queue.
 */
typedef struct {
    unsigned int  n;
    queue_t       queue_list[];
} thread_pool_t;


static void * tpool_work_thrad(void * arg);

static void tpool_build_queue_list(queue_t * queue_list, unsigned int n);
static void tpool_queue_init(queue_t * queue);

static queue_t * tpool_schedule(thread_pool_t * tp);
static void tpool_add_task(queue_t * queue, void(* func)(void *), void * arg, int cost);

static void tpoll_shutdown(queue_t * queue);
static void tpool_handler(queue_t * queue);

/*
 *  Initial thread-pool
 */
void *thread_pool_init(unsigned int n)
{
	thread_pool_t * tp;

    if ( n == 0 || (tp = NEW_THREAD_POOL(n)) == NULL )
        return NULL;

    tp->n = n;
    tpool_build_queue_list(tp->queue_list, n);

    return tp;
}

/*
 *  Destroy thread-pool
 */
void thread_pool_destroy(void * p)
{
	unsigned int    i;
	thread_pool_t * tp;

	if ( (tp = (thread_pool_t *)p) == NULL )
        return;

    for ( i = 0; i < tp->n; i++ )
        tpoll_shutdown(&tp->queue_list[i]);

    DO_SAFELY_FREE(tp);
}

/*
 *  Add a task into thread-pool
 */
int thread_pool_add_task(void * p, void(* func)(void *), void * arg, int cost)
{
	thread_pool_t * tp;
	queue_t       * queue;

	if ( (tp = (thread_pool_t *)p) == NULL )
        return -1;

    if ( (queue = tpool_schedule(tp)) == NULL )
    	return -1;

    tpool_add_task(queue, func, arg, cost);

	return 0;
}

/*
 *  Work thread
 */
static void * tpool_work_thrad(void * arg)
{
	queue_t * queue = (queue_t *)arg;

    while(1)
    {
        char r;
        if ( read(queue->pipe_fd[0], &r, 1) < 0 )
            continue;

        switch (r)
        {
        case 'r':
            /* State: Busy */
            SYNC_VALUE_SET_TRUE(queue->working);

            /* Working */
            tpool_handler(queue);

            /* State: Idel */
            SYNC_VALUE_SET_FALSE(queue->working);
        break;

        case 'x':
            pthread_exit(NULL);
        break;
        
        default:
        break;
        }
    }
}

/*
 *  Build a queue list.
 */
static void tpool_build_queue_list(queue_t * queue_list, unsigned int n)
{
    unsigned int i;

    for ( i = 0; i < n; i++ ) 
    	tpool_queue_init(&queue_list[i]);
}

/*
 *  Initial queue.
 */
static void tpool_queue_init(queue_t *queue)
{
	if ( pipe(queue->pipe_fd) < 0 ) {
		QUEUE_SET_AS_BROKEN(queue);
		return ;
	}

	if ( pthread_create(&queue->tid, NULL, tpool_work_thrad, (void *)queue) != 0 ) {
		QUEUE_CLOSE_PIPE(queue);
		QUEUE_SET_AS_BROKEN(queue);
	}

	memset(&queue->wtask_list, 0, sizeof(wtask_list_t));
}

/*
 *  Schedule, get a task queue from thread-pool.
 */
static queue_t * tpool_schedule(thread_pool_t * tp)
{
    int cost_min       = 0;
    unsigned int index = tp->n;
    
    unsigned int i;
    for ( i = 0; i < tp->n; i++ ) 
    {
        if ( QUEUE_IS_BROKEN(&tp->queue_list[i]) )
            continue;

        if ( WTASK_LIST_IS_FULL(&tp->queue_list[i].wtask_list) )
            continue;
        
        int cost_tmp = SYNC_VALUE(tp->queue_list[i].wtask_list.total_cost);

        /* The minimun cost is zero. */
        if ( cost_tmp == 0 ) {
            index = i;
            break;
        }

		/* Compare and to keep the minimun cost. */
        if ( (cost_min == 0 || cost_min > cost_tmp) &&
              cost_tmp < WTASK_LIST_SIZE * TPOOL_TASK_COST_HIGHEST ) // Maybe this work-thrad was blocking.
        {
            cost_min = cost_tmp;
            index = i;
        }
    }

    return ( index >= tp->n )?NULL : &tp->queue_list[index];
}

/*
 *  Add a task into the weighted task list.
 */
static void tpool_add_task(queue_t * queue, void(* func)(void *), void * arg, int cost)
{
    /*
     *  We will need to compress a larger range of indexs into a smaller range of index numbers.
     *  Range(in): sizeof(unsigned char)
     *  Range(wtask_list): 0 ~ WTASK_LIST_SIZE
     */
	weighted_task_t * wtask = &queue->wtask_list.wtask[WTASK_LIST_OFFSET(queue->wtask_list.in)];
	
	int add_cost = (cost < 0)?(WTASK_LIST_SIZE * TPOOL_TASK_COST_HIGHEST):cost;

	wtask->task.func = func;
	wtask->task.arg  = arg;
	wtask->cost      = add_cost;

	SYNC_VALUE_ADD(queue->wtask_list.total_cost, (unsigned int)add_cost);
	queue->wtask_list.in++;

	if ( !SYNC_VALUE(queue->working) )
		QUEUE_NOTIFY_THREAD(queue);
}

/*
 *  Shutdown a queue.
 */
static void tpoll_shutdown(queue_t * queue)
{
	if ( QUEUE_IS_BROKEN(queue) )
		return;

	QUEUE_CLOSE_THREAD(queue);
	QUEUE_CLOSE_PIPE(queue);
}

/*
 *  Work-thread handler
 */
static void tpool_handler(queue_t * queue)
{
	while ( !WTASK_LIST_IS_EMPTY(&queue->wtask_list) ) {
        /*
         *  Range(out): sizeof(unsigned char)
         *  Range(wtask_list): 0 ~ WTASK_LIST_SIZE
         */
		weighted_task_t * wtask = &queue->wtask_list.wtask[WTASK_LIST_OFFSET(queue->wtask_list.out)];

		wtask->task.func(wtask->task.arg);

		SYNC_VALUE_SUB(queue->wtask_list.total_cost, wtask->cost);
		SYNC_VALUE_ADD(queue->wtask_list.out, 1);
	}
}

