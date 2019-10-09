#include "tpool_wrapper.h"
#include <iostream>
#include <unistd.h>

using namespace std;

void task_block(void *arg)
{
    while(1)
	{
		usleep(1000*1000);
		cout << "task -> block" << endl;
	}
}

void task1(void *arg)
{
    usleep(1000*100);
	cout << "task -> 1" << endl;
}

void task2(void *arg)
{
    usleep(1000*500);
	cout << "task -> 2" << endl;
}

void task3(void *arg)
{
    cout << "task -> 3 [" << (char *)arg << "]" << endl;
}

int main()
{
    char *msg = (char *)"Hello World!";

	if(TPOOL_INST()->add_task(task_block, NULL, tpool_wrapper::BLOCK) != 0)
		cout << "Add block-task failed!" << endl;

	while(1){
		if(TPOOL_INST()->add_task(task1, (void *)msg) != 0)
			cout << "Add task-1 failed!" << endl;
		
		if(TPOOL_INST()->add_task(task2, (void *)msg, tpool_wrapper::HEAVY) != 0)
			cout << "Add task-2 failed!" << endl;
		
		if(TPOOL_INST()->add_task(task3, (void *)msg, tpool_wrapper::EASY) != 0)
			cout << "Add task-3 failed!" << endl;
		
		usleep(1000*600);
	}
	
    TPOOL_TERMINATE();

    return 0;
}
