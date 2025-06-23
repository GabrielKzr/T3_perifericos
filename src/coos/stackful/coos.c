#include <stm32f4xx_conf.h>
#include <hal.h>
#include <coos.h>
#include <libc.h>


static struct task_s *ptasks;
static struct task_s *pcurrtask;
static int cur = -1;


/* CoOS API */

/* tasks init, add, schedule and sched init (only called on main or internally) */

void task_pinit(struct task_s *tasks)
{
	ptasks = tasks;
	pcurrtask = 0;
}

void task_add(struct task_s *tasks, void *(task_ptr)(void *), unsigned char priority, unsigned int guard)
{
	struct task_s *ptask;
	struct mq_s *pmqueue;
	int i;
	
	for (i = 0; i < MAX_TASKS; i++) {
		ptask = &tasks[i];
		if (!ptask->task) break;
	}
	ptask->task = task_ptr;
	ptask->guard = guard;
	ptask->priority = priority;
	ptask->pcounter = priority;
	
	pmqueue = &ptask->mqueue;
	pmqueue->size = MAX_MESSAGES;
	pmqueue->mask = MAX_MESSAGES - 1;
	pmqueue->head = pmqueue->tail = 0;
	pmqueue->items = 0;
	
	cur++;
}

struct task_s *task_schedule(struct task_s *tasks)
{
	struct task_s *ptask;
	int i;
	
	for (;;) {
		for (i = 0; i < MAX_TASKS; i++) {
			ptask = &tasks[i];
			if (!ptask->task) break;
			
			if (!--ptask->pcounter) {
				cur = i;
				pcurrtask = ptask;
				ptask->pcounter = ptask->priority;
				
				return ptask;
			}
		}
	}
}

void sched_init(struct task_s *tasks)
{
	pcurrtask = &tasks[cur];
	pcurrtask->task((void *)0);
}


/* task operations (called inside tasks) */

void task_init()
{
	pcurrtask = &ptasks[cur];
	
	char guard[pcurrtask->guard];		/* reserve some stack space */
	
	memset((char *)guard, 0x69, pcurrtask->guard);
	
	if (!setjmp(pcurrtask->context)) {
		if (cur > 0) {
			pcurrtask = &ptasks[--cur];
			pcurrtask->task((void *)0);
		}
	}	
}

void task_yield()
{
	struct task_s *ptask;
	
	if (!setjmp(pcurrtask->context)) {
		ptask = task_schedule(ptasks);
		longjmp(ptask->context, 1);
	}
}

int task_id()
{
	return cur;
}


/* task queue operations (called inside tasks) */

int task_mq_enqueue(void *(task_ptr)(void *), struct message_s *message)
{
	struct task_s *ptask;
	struct mq_s *pmqueue;
	int i;
	int tail;
	
	for (i = 0; i < MAX_TASKS; i++) {
		ptask = &ptasks[i];
		if (!ptask->task) return -1;
		if (ptask->task == task_ptr) break;
	}
	
	pmqueue = &ptask->mqueue;

	tail = (pmqueue->tail + 1) & pmqueue->mask;
	if (tail == pmqueue->head)
		return -1;

	pmqueue->queue[pmqueue->tail] = message;
	pmqueue->tail = tail;
	pmqueue->items++;
	
	return 0;
}

struct message_s *task_mq_dequeue(void)
{
	struct mq_s *pmqueue;
	int head;
	struct message_s *message;

	pmqueue = &pcurrtask->mqueue;

	if (pmqueue->head == pmqueue->tail)
		return 0;

	head = pmqueue->head;
	message = pmqueue->queue[head];
	pmqueue->head = (pmqueue->head + 1) & pmqueue->mask;
	pmqueue->items--;

	return message;
}

int task_mq_items(void)
{
	struct mq_s *pmqueue;

	pmqueue = &pcurrtask->mqueue;
	
	return pmqueue->items;
}

