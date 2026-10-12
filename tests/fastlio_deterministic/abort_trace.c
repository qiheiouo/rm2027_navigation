// Reused from the fixed audit's diagnostic observer. Observe then delegate to
// libc abort; never suppress the original bounds failure or change method bodies.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
void abort(void) {
  static volatile sig_atomic_t entered=0;
  if(!entered) {
    entered=1;
    const char start[]="AUDIT_ABORT_TRACE_BEGIN\n";
    write(STDERR_FILENO,start,sizeof(start)-1);
    void *frames[32]; int n=backtrace(frames,32);
    backtrace_symbols_fd(frames,n,STDERR_FILENO);
    const char end[]="AUDIT_ABORT_TRACE_END\n";
    write(STDERR_FILENO,end,sizeof(end)-1);
  }
  void(*real_abort)(void)=dlsym(RTLD_NEXT,"abort");
  if(real_abort) real_abort();
  _exit(134);
}
