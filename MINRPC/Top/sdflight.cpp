#include <stdio.h>
#include <pthread.h>
#include <assert.h>
#include <HEXREF/Rpc/hexref.h>

void *hi(void * dummy)
{
   printf("Hello World!\n");
}

int main()
{
  pthread_t hi_thread;
  assert(!pthread_create(&hi_thread, NULL, hi, NULL));
  pthread_join(hi_thread, NULL);

  hexref_init();
}
