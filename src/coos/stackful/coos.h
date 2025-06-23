#define MAX_TASKS	20		/* maximum number of tasks */
#define MAX_MESSAGES	16		/* per task message queue size (must be power of 2) */

/* message and message queue data structures */
struct message_s {
	int type;
	void *data;
};

struct mq_s {
	struct message_s *queue[MAX_MESSAGES];
	int size;
	int mask;
	int head, tail;
	int items;
};

/* task data structure and pointers */
struct task_s {
	void *(*task)(void *);
	jmp_buf context;
	unsigned int guard;
	unsigned char priority;
	unsigned char pcounter;
	struct mq_s mqueue;
};


/* CoOS API */

/* tasks init, add and schedule (only called on main) */
void task_pinit(struct task_s *tasks);
void task_add(struct task_s *tasks, void *(task_ptr)(void *),
	unsigned char priority, unsigned int guard);
struct task_s *task_schedule(struct task_s *tasks);
void sched_init();

/* task management (called inside tasks) */
void task_init();
void task_yield();
int task_id();

/* task queue operations (called inside tasks) */
int task_mq_enqueue(void *(task_ptr)(void *), struct message_s *message);
struct message_s *task_mq_dequeue(void);
int task_mq_items(void);
