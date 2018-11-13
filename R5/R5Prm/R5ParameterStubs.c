#include <R5/R5Prm/R5Parameter.h>

void init_parameters(void) {}
void set_parameter(uint32_t param_id, uint32_t value) {}
bool have_parameter(uint32_t param_id) { return false; }
uint32_t get_parameter(uint32_t param_id) { return 0; }
void start_commit(void) {}
int check_commit(void) {return 0;}
int get_commit_status(void) {return 0;}
