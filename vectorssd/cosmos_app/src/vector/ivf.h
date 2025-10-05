#ifndef IVF_H_
#define IVF_H_

#define IVF_CENTROID_N 10240


#include "../memory_map.h"

// issue : do we need to manage these HNSWNode and HNSWNodeLinks (graph) in device ? we need to consider (memory usage vs limit) 
// typedef struct {
//   unsigned count : 2;               // Number of neighbors
//   unsigned int neighbors[M_PARAM];  // unsigned int Neighbor links, as vector id is unsigned it
// } HNSWNodeLinks;

// typedef struct {
//   unsigned int id;
//   unsigned int max_level;           // Max. level of the node
//   HNSWNodeLinks links[MAX_LEVELS];  // Connections at each level
// } HNSWNode;

typedef struct _centroid {

  unsigned int cluster_id_list_lba;
} centroid;

typedef struct _IVFIndex {
  //   VectorStorage vectors;                           // All vector datas
    // HNSWNode nodes[MAX_ELEMENTS];                    // All nodes
    // int cur_element_count;                           // Current number of vectors
    // int entry_point_idx;                             // Entry point to the graph
    // int max_level;                                   // Max. level in the graph
  unsigned todo;
  // unsigned int centroids[IVF_CENTROID_N];

} IVFIndex;

// uint8_t visited_bitmap[(MAX_ELEMENTS + 7) / 8];  // Visit bitmap can be exist uniquly
// void hnsw_build(int building_phase);
unsigned int ivf_build(int building_phase);
unsigned int ivf_search(int building_phase);

#endif