#include <stdio.h>

extern "C" {
  void sw_main(void);
  extern const unsigned int SW_HASH;
  extern const char* SW_TAG;
}

extern void constructApp(void);
void cycleForever(void);

void sw_main(void) {

  printf("MAIN!\nHASH: 0x%08X TAG: %s\n",SW_HASH,SW_TAG);
  constructApp();
  cycleForever();

}
