#include <stdio.h>

extern "C" {
  void sw_main(void);
  extern const unsigned int FSW_HASH;
  extern const char* FSW_TAG;
}

extern void constructApp(void);
void cycleForever(void);

void sw_main(void) {

  printf("MAIN!\nHASH: 0x%08X TAG: %s\n",FSW_HASH,FSW_TAG);
  constructApp();
  cycleForever();

}
