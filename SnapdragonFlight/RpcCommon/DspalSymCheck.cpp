#include <dlfcn.h>
#include <sys/cdefs.h>
#include <dspal_platform.h>
#include <semaphore.h>
#include <unistd.h>
#include <dspal_time.h>
#include <pthread.h>

extern "C" {
/*__EXPORT*/ int _start_main(void);
};

#ifdef __cplusplus
extern "C" {
#endif

int ioctl(int fd, int request, void *argp) {  return 0;  }

void HAP_debug(const char *msg, int level, const char *filename, int line) { }
void HAP_power_request(int a, int b, int c) { }

int sem_init(sem_t *, int, unsigned int) {  return 0;  }
int sem_trywait(sem_t *) {  return 0;  }
int sem_wait(sem_t *) {  return 0;  }
int sem_getvalue(sem_t *, int *) {  return 0;  }
int sem_post(sem_t *) {  return 0;  }
int sem_destroy(sem_t *) {  return 0;  }

int tcgetattr(int, struct termios *) {  return 0;  }
int tcsetattr(int, int,
	      const struct termios *) {  return 0;  }

int usleep(useconds_t usec) {  return 0;  }

int clock_gettime(clockid_t clk_id, struct timespec *tp) {  return 0;  }

// Qualcomm (not POSIX compliant) additions to pthread get/set attribute functions:
int pthread_attr_setthreadname(pthread_attr_t *attr, const char *name) {  return 0;  }
int pthread_attr_getthreadname(const pthread_attr_t *attr, char *name, int size) {  return 0;  }
int pthread_attr_settimetestid(pthread_attr_t *attr, unsigned int tid) {  return 0;  }
int pthread_attr_gettimetestid(const pthread_attr_t *attr, unsigned int *tid) {  return 0;  }
int pthread_getattr_np(pthread_t thread, pthread_attr_t *restrict attr) {  return 0;  }

// POSIX compliant function prototype definitions:
int pthread_create(pthread_t *, const pthread_attr_t *, void *(*)(void *), void *) {  return 0;  }
void pthread_exit(void *) {  }
int pthread_join(pthread_t, void **) {  return 0;  }
pthread_t pthread_self(void) {  pthread_t temp = 0; return temp;  }
int pthread_cancel(pthread_t) {  return 0;  }
int pthread_kill(pthread_t thread, int sig) {  return 0;  }
int pthread_getschedparam(pthread_t pthread, int *restrict, struct sched_param *restrict) {  return 0;  }
int pthread_setschedparam(pthread_t, int, const struct sched_param *) {  return 0;  }
int pthread_attr_init(pthread_attr_t *) {  return 0;  }
int pthread_attr_destroy(pthread_attr_t *) {  return 0;  }
int pthread_attr_setschedparam(pthread_attr_t *restrict, const struct sched_param *restrict) {  return 0;  }
int pthread_attr_getschedparam(const pthread_attr_t *, struct sched_param *) {  return 0;  }
int pthread_attr_setstacksize(pthread_attr_t *, size_t) {  return 0;  }
int pthread_attr_getstacksize(const pthread_attr_t *, size_t *) {  return 0;  }
int pthread_attr_setstackaddr(pthread_attr_t *, void *) {  return 0;  }
int pthread_attr_getstackaddr(const pthread_attr_t *, void **) {  return 0;  }
int pthread_mutex_init(pthread_mutex_t *__mutex, const pthread_mutexattr_t *) {  return 0;  }
int pthread_mutex_lock(pthread_mutex_t *__mutex) {  return 0;  }
int pthread_mutex_unlock(pthread_mutex_t *__mutex) {  return 0;  }
int pthread_mutex_trylock(pthread_mutex_t *__mutex) {  return 0;  }
int pthread_mutex_destroy(pthread_mutex_t *__mutex) {  return 0;  }
int pthread_mutexattr_init(pthread_mutexattr_t *attr) {  return 0;  }
int pthread_mutexattr_destroy(pthread_mutexattr_t *) {  return 0;  }
int pthread_mutexattr_gettype(const pthread_mutexattr_t *restrict, int *restrict) {  return 0;  }
int pthread_mutexattr_settype(pthread_mutexattr_t *, int) {  return 0;  }
int pthread_mutexattr_getprotocol(const pthread_mutexattr_t *restrict, int *restrict) {  return 0;  }
int pthread_mutexattr_setprotocol(pthread_mutexattr_t *, int) {  return 0;  }
int pthread_mutexattr_getpshared(const pthread_mutexattr_t *restrict, int *restrict) {  return 0;  }
int pthread_mutexattr_setpshared(pthread_mutexattr_t *, int) {  return 0;  }
// Not Supported in DSPAL
//int pthread_spin_init(pthread_spinlock_t *__spin, int) {  return 0;  }
//int pthread_spin_destroy(pthread_spinlock_t *__spin) {  return 0;  }
//int pthread_spin_lock(pthread_spinlock_t *__spin) {  return 0;  }
//int pthread_spin_trylock(pthread_spinlock_t *__spin) {  return 0;  }
//int pthread_spin_unlock(pthread_spinlock_t *__spin) {  return 0;  }
int pthread_condattr_init(pthread_condattr_t *) {  return 0;  }
int pthread_condattr_destroy(pthread_condattr_t *) {  return 0;  }
int pthread_condattr_setpshared(pthread_condattr_t *, int) {  return 0;  }
int pthread_condattr_getpshared(const pthread_condattr_t *restrict, int *restrict) {  return 0;  }
int pthread_cond_init(pthread_cond_t *, const pthread_condattr_t *) {  return 0;  }
int pthread_cond_destroy(pthread_cond_t *) {  return 0;  }
int pthread_cond_signal(pthread_cond_t *) {  return 0;  }
int pthread_cond_broadcast(pthread_cond_t *) {  return 0;  }
int pthread_cond_wait(pthread_cond_t *, pthread_mutex_t *__mutex) {  return 0;  }
int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *time) {  return 0;  }
// Not supported in DSPAL
//int pthread_barrier_init(pthread_barrier_t *restrict barrier, const pthread_barrierattr_t *restrict attr, unsigned count) {  return 0;  }
//int pthread_barrier_destroy(pthread_barrier_t *barrier) {  return 0;  }
//int pthread_barrier_wait(pthread_barrier_t *barrier) {  return 0;  }
//int pthread_barrierattr_init(pthread_barrierattr_t *attr) {  return 0;  }
//int pthread_barrierattr_destroy(pthread_barrierattr_t *attr) {  return 0;  }
//int pthread_barrierattr_getpshared(const pthread_barrierattr_t *restrict attr, int *restrict pshared) {  return 0;  }
int pthread_key_create(pthread_key_t *, void (*)(void *)) {  return 0;  }
int pthread_key_delete(pthread_key_t) {  return 0;  }
int pthread_setspecific(pthread_key_t, const void *) {  return 0;  }
void *pthread_getspecific(pthread_key_t) {  return NULL;  }
int pthread_attr_setaffinity_np(pthread_attr_t *attr, size_t cpusetsize, const cpu_set_t *cpuset) {  return 0;  }

#ifdef __cplusplus
}; // end extern "C"
#endif

int dlinit(int a, char **b)
{
  return 1;
}

int main(int argc, char *argv[])
{
  int ret = 0;

  return ret;
}

int _start_main()
{
  return -1;
}
