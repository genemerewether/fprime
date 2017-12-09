#include <dlfcn.h>
#include <sys/cdefs.h>
#include <dspal_platform.h>

extern "C" {
/*__EXPORT*/ int _start_main(void);
};

void HAP_debug(const char *msg, int level, const char *filename, int line)
{
}

void HAP_power_request(int a, int b, int c)
{
}

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
