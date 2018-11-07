#include <HAP_farf.h>
#include <semaphore.h>

#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
/*
void block_indefinite(void);
void _Read_uleb(void);
void _Parse_fde_instr(void);
void _Parse_csd(void);
void _Valbytes(void);
void _Get_eh_data(void);
void _Parse_lsda(void);
void __cxa_guard_release(void);
void _Read_enc_ptr(void);
void _Read_sleb(void);
void __cxa_guard_acquire(void);
void __cxa_pure_virtual(void);
*/

extern "C" {
  
void block_indefinite(void)
{
  sem_t forever;
  sem_init(&forever, 0, 0);
  sem_wait(&forever);
}

void _Read_uleb(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Parse_fde_instr(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Parse_csd(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Valbytes(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Get_eh_data(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Parse_lsda(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void __cxa_guard_release(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Read_enc_ptr(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void _Read_sleb(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

void __cxa_guard_acquire(void)
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  //block_indefinite();
}

void __cxa_pure_virtual()
{
  DEBUG_PRINT("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
  block_indefinite();
}

}; //extern "C"
