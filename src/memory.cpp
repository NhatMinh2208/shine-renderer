#include <nori/memory.h>
#include <cstdlib>
NORI_NAMESPACE_BEGIN

void *AllocAligned(size_t size) {
   return _aligned_malloc(size, L1_CACHE_LINE_SIZE);
   // return malloc(size);
}

void FreeAligned(void *ptr) {
   if (!ptr) return;
   _aligned_free(ptr);
   // free(ptr);
}



NORI_NAMESPACE_END