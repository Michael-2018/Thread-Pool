#include "tpool_service.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

void test_block(void *t)
{
    while(1)
	{
		usleep(1000 * 1000 * 5);
		printf("test-block\n");
	}
}

void test_1(void *t)
{
    printf("Task => 1\n");
}

void test_2(void *t)
{
	usleep(1000 * 100);
    printf("Task => 2\n");
}

void test_3(void *t)
{
    usleep(1000 * 500);
	printf("Task => 3\n");
}

void test_4(void *t)
{
    usleep(1000 * 1000);
    printf("Task => 4\n");
}

void test_5(void *t)
{
    usleep(1000 * 1000 * 10);
    printf("Task => 5\n");
}

void main()
{
	void *tp  = thread_pool_init(5);
	char *str = "HW_12";

	thread_pool_add_task(tp, test_block, NULL, TPOOL_TASK_COST_BLOCK);

	while(1)
	{
		thread_pool_add_task(tp, test_1, NULL, TPOOL_TASK_COST_LOWEST);
		thread_pool_add_task(tp, test_2, NULL, TPOOL_TASK_COST_LOWER);
		thread_pool_add_task(tp, test_3, (void *)str, TPOOL_TASK_COST_NORMAL);
		thread_pool_add_task(tp, test_4, NULL, TPOOL_TASK_COST_HIGHER);
		thread_pool_add_task(tp, test_5, NULL, TPOOL_TASK_COST_HIGHEST);

		usleep(1000 * 500);
	}

	thread_pool_destroy(tp);
}
