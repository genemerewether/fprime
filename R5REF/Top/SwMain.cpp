#include <stdio.h>

extern "C" {
  void fsw_main(void);
  extern const unsigned int FSW_HASH;
  extern const char* FSW_TAG;
}

extern void constructApp(void);
void cycleForever(void);

void fsw_main(void) {

  printf("MAIN!\nHASH: 0x%08X TAG: %s\n",FSW_HASH,FSW_TAG);
  constructApp();
  cycleForever();

}
