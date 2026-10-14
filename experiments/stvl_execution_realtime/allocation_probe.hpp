// Allocation counts only in the standalone benchmark; native plugins unaffected.
#pragma once
#include <cstdlib>
#include <new>
namespace allocation {inline thread_local bool active=false;inline thread_local size_t count=0,bytes=0;inline void reset(){count=bytes=0;active=true;}inline void record(size_t n){if(active){count++;bytes+=n;}}}
void* operator new(std::size_t n){if(auto p=std::malloc(n?n:1)){allocation::record(n);return p;}throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void*p)noexcept{std::free(p);}void operator delete[](void*p)noexcept{std::free(p);}
void operator delete(void*p,std::size_t)noexcept{std::free(p);}void operator delete[](void*p,std::size_t)noexcept{std::free(p);}
void* operator new(std::size_t n,std::align_val_t a){void*p=nullptr;if(posix_memalign(&p,static_cast<size_t>(a),n?n:1))throw std::bad_alloc();allocation::record(n);return p;}
void operator delete(void*p,std::align_val_t)noexcept{std::free(p);}void operator delete(void*p,std::size_t,std::align_val_t)noexcept{std::free(p);}
