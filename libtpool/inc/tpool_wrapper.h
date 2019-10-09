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
#ifndef __TPOOL_WRAPPER__
#define __TPOOL_WRAPPER__

#include "tpool_service.h"
#include <stdio.h>

/*
 *  The number of the threads in the thread-pool.
 */
#define TPOOL_THREAD_NUM    5

/*
 *  Usage example: 
 *    TPOOL_INST()->add_task(test_func, arg, tpool_wrapper::HEAVY);
 *    ...
 *
 *  Finally, we need to release:
 *    TPOOL_TERMINATE();
 */
#define TPOOL_INST()        tpool_wrapper::inst(TPOOL_THREAD_NUM)
#define TPOOL_TERMINATE()   do{ delete TPOOL_INST(); }while(0)

/*
 *  Class: tpool_wrapper
 */
class tpool_wrapper
{
public:
    enum 
    { 
        EASY   = TPOOL_TASK_COST_LOWER, 
        NORMAL = TPOOL_TASK_COST_NORMAL, 
        HEAVY  = TPOOL_TASK_COST_HIGHEST,
        BLOCK  = TPOOL_TASK_COST_BLOCK,	
    };

public:
    static tpool_wrapper * inst(unsigned int n) 
    { 
        static tpool_wrapper *inst = NULL; 
        return ( inst == NULL )?(inst = new tpool_wrapper(n)):inst;
    }

public:
    tpool_wrapper(unsigned int n)
    { 
        _tpool = thread_pool_init(n); 
    }

    ~tpool_wrapper()
    { 
        thread_pool_destroy(_tpool); 
    }

    /**
     *  Add a task into thread-pool.
     *
     *  @param func  Task handler.
     *  @param arg   Task params.
     *  @param load  Load level.
     *
     *  @return  0 if successfully.
     *  @return -1 if failed.
     */
    int add_task(void(*func)(void *), void *arg, int load = NORMAL)
    {
        return thread_pool_add_task(_tpool, func, arg, load);
    }

private:
    void * _tpool;
};

#endif /* __TPOOL_WRAPPER__ */

