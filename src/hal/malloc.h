#define align4(x) ((((x) + 3) >> 2) << 2)

struct mem_block_s {
	struct mem_block_s *next;		/* pointer to the next block */
	size_t size;				/* aligned block size. the LSB is used to define if the block is used */
};

void heap_init(size_t *zone, uint32_t len);
void free(void *ptr);
void *malloc(uint32_t size);
