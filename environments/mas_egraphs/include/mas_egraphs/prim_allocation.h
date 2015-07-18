#ifndef PRIM_ALLOCATION_H
#define PRIM_ALLOCATION_H
#endif

#include <sbpl/headers.h>

class PrimAllocation{
 public:
  PrimAllocation();
  
 private:
  struct Node{
    int id;
    std::vector<int> successor_ids;
  }
}
