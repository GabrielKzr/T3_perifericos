#define NULL			((void *)0)
#define USED			1
#define TRUE			1
#define FALSE			0
#define isprint(c)		(' '<=(c)&&(c)<='~')
#define isspace(c)		((c)==' '||(c)=='\t'||(c)=='\n'||(c)=='\r')
#define isdigit(c)		('0'<=(c)&&(c)<='9')
#define islower(c)		('a'<=(c)&&(c)<='z')
#define isupper(c)		('A'<=(c)&&(c)<='Z')
#define isalpha(c)		(islower(c)||isupper(c))
#define isalnum(c)		(isalpha(c)||isdigit(c))
#define min(a,b)		((a)<(b)?(a):(b))

void putchar(char value);
int32_t kbhit(void);
int32_t getchar(void);
char *strcpy(char *s1, const char *s2);
char *strncpy(char *s1, const char *s2, int32_t n);
char *strcat(char *s1, const char *s2);
char *strncat(char *s1, const char *s2, int32_t n);
int32_t strcmp(const char *s1, const char *s2);
int32_t strncmp(const char *s1, const char *s2, int32_t n);
char *strstr(const char *s1, const char *s2);
int32_t strlen(const char *s1);
char *strchr(const char *s1, int32_t c);
char *strpbrk(const char *s1, const char *s2);
char *strsep(char **pp, const char *delim);
char *strtok(char *s, const char *delim);
char *strtok_r(char *s, const char *delim, char **holder);
int32_t strtol(const char *s, char **end, int32_t base);
int32_t atoi(const char *s);
void itoa(int32_t i, char *s, int32_t base);
void *memcpy(void *dst, const void *src, uint32_t n);
void *memmove(void *dst, const void *src, uint32_t n);
int32_t memcmp(const void *cs, const void *ct, uint32_t n);
void *memset(void *s, int32_t c, uint32_t n);
int32_t abs(int32_t n);
int32_t random(void);
void srand(uint32_t seed);
int32_t puts(const char *str);
char *gets(char *s);
char *fgets(char *s, int n, void *f);
char *getline(char *s);
int32_t printf(const char *fmt, ...);
int32_t sprintf(char *out, const char *fmt, ...);

