#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
#endif

extern "C" {
  int main(int argc, char* argv[]);
};

int main(int argc, char* argv[]) {
  hexref_init();
  return 0;
}
