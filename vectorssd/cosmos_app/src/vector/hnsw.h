#ifndef VECTORSSD_COSMOS_APP_SRC_VECTOR_HNSW_H_
#define VECTORSSD_COSMOS_APP_SRC_VECTOR_HNSW_H_

#include "binary_heap.h"
#include "hnsw_config.h"
#include "index.h"

#include "../ftl_config.h"

/**
 * @brief Maximum number of vector, which acts as a role of the maximum size of
 * segment.
 */
// #define PREDEFINED_MAX_NUMBER_OF_VECTOR_PER_SEGMENT (MAX_SKIPLIST_NODE * (MAX_SEGMENT_TABLE_LEVEL0 - 1))  // 16384
#define PREDEFINED_MAX_NUMBER_OF_VECTOR_PER_SEGMENT (MAX_SKIPLIST_NODE * (MAX_SEGMENT_TABLE_LEVEL0 - 1) * 2) // 32768
#define NUM_OF_HNSWNODE_PER_DATA_BUFFER_BLOCK SAFE_NUM_OF_HNSWNODE_PER_DATA_BUFFER_BLOCK
#define SAFE_NUM_OF_HNSWNODE_PER_DATA_BUFFER_BLOCK ((BYTES_PER_DATA_REGION_OF_SLICE - 64) / sizeof(HNSWNode))
#define NUM_OF_PAGE_PER_INDEX ((PREDEFINED_MAX_NUMBER_OF_VECTOR_PER_SEGMENT / SAFE_NUM_OF_HNSWNODE_PER_DATA_BUFFER_BLOCK) + 1)

#define CLEAR_VISITED(g) memset((g)->visitedBitmap, 0, (PREDEFINED_MAX_NUMBER_OF_VECTOR_PER_SEGMENT + 7) / 8)
#define SET_VISITED(g, idx) ((g)->visitedBitmap[(idx) / 8] |= (1 << ((idx) % 8)))
#define IS_VISITED(g, idx) ((g)->visitedBitmap[(idx) / 8] & (1 << ((idx) % 8)))

#define NOT_FOUND 0xffffffff

#ifdef VECTOR_BUILD_PREFETCH_DEGREE
typedef struct {
  unsigned int dramAddr[(PREDEFINED_VECTOR_SIZE / BYTES_PER_DATA_REGION_OF_SLICE) + 2];
  unsigned int n;
  unsigned int aux;
} prefetchedVectorDataAddr;
#endif

typedef struct {
  unsigned int neighbors[M_PARAM]; // unsigned int Neighbor links, as vector id
  // is unsigned it
  unsigned int count; // Number of neighbors
} HNSWNodeLinks;

typedef struct {
  unsigned int key;
  int maxLevel : 16;               // Max. level of the node
  HNSWNodeLinks links[MAX_LEVELS]; // Connections at each level
} HNSWNode;                        // 116 byte when MAX_LEVELS is three

// NOTE(Dhmin): Change to 16KB Page-aligned size
typedef struct _HNSWOndemandEntry {
  HNSWNode hnswnodes[NUM_OF_HNSWNODE_PER_DATA_BUFFER_BLOCK];
  // char padding[BYTES_PER_DATA_REGION_OF_SLICE - sizeof(HNSWNode) * SAFE_NUM_OF_HNSWNODE_PER_DATA_BUFFER_BLOCK];
  char pad[144];     // M_PARAM=8
} HNSWOndemandEntry; // Total 16KB, occupying 163 vectors' nodes

typedef struct _HNSWIndex {
  // NOTE(Dhmin): The total number of entry inside the HNSW index is known in
  // Segment info structure.
  unsigned int entryPointKeyIndex; // Entry point to the graph
  unsigned int maxLevel : 16;
  unsigned int segmentId : 16;

  unsigned int ondemandEntryLpn[NUM_OF_PAGE_PER_INDEX];

  char visitedBitmap[(PREDEFINED_MAX_NUMBER_OF_VECTOR_PER_SEGMENT + 7) / 8]; // Visit bitmap
} HNSWIndex;

typedef struct _VectorIndex {
  HNSWIndex hnsw;
  // TODO(Dhmin): IVF version would be supported in the future.
} VectorIndex;

/**
 * HNSW index primitive operations supported by on-demand paging-based SSD
 *
 * The primitive operations include index build and vector search.
 * The vector insertion procedure is supported by Segment maker.
 */
int buildOndemandHnsw(SUPER_SEGMENT_INFO *targetSegForIndexing, unsigned int *reverse_write_pointer_lpn, int currentIndex, int endIndex, const int preempted);
int searchOndemandHnsw(SUPER_SEGMENT_INFO *targetSegForIndexing, const unsigned int topK);
static HNSWOndemandEntry *loadHnswNode(const unsigned int keyIndex, unsigned int *need_sync_ret);
int loadVectorWithCache(unsigned int vectorIdx, unsigned int dramAddr[], int *maxSlices, unsigned int *need_sync);
static int loadRawVectorData(unsigned int retDramAddr[], unsigned int startLba, unsigned int endLba, unsigned int *need_sync_ret);
static int loadNeighborVectorData(unsigned int neighborIdx, unsigned int neighborVectorDramAddr[], unsigned int *need_sync_ret);
static unsigned int getKeyIndexFromSegWith(const unsigned int keyData);
static void initializeIndexWith(SUPER_SEGMENT_INFO *targetSegForIndexing, unsigned int *reverse_write_pointer_lpn);
static void initializeIndexUsingMemooryWith(SUPER_SEGMENT_INFO *targetSegForIndexing, unsigned int *reverse_write_pointer_lpn);
static unsigned int greedySearchLevel(const unsigned int queryVectorDramAddr[], const unsigned int entryPointKeyIndex, const int level, const int queryVectorIdx);
static void searchNeighborsEf(const unsigned int queryVectorDramAddr[], const unsigned int entryPointKeyIndex, const int level, const unsigned int ef, unsigned int candidates[], float distances[], int *candCount,
                              const int queryVectorIdx);
static void connectNeighbors(unsigned int vectorKeyIndex, unsigned int candidates[], float distances[], const int candCount, const int level);
// TODO(Dhmin): Support of preemptive index building mechanism
void buildOndemandPreemptiveHnsw(SUPER_SEGMENT_INFO *targetSegForIndexing);
int bruteForceSearch(SUPER_SEGMENT_INFO *targetSegForSearching, const unsigned int topK, const int caching);

/**
 * Helper functions regarding managing the HNSW index or vector data.
 */
float getDistanceCached(unsigned int a, unsigned int b, unsigned int *vecAData, unsigned int *vecBData);
static int getRandomLevel();
void printFloatArray(float *arr, int count);

// ============================================================================
// uint8_t visited_bitmap[(MAX_ELEMENTS + 7) / 8];  // Visit bitmap can be exist
// uniquly

/*
unsigned int hnsw_ondemand_search(unsigned int top_k, unsigned int element_n);
unsigned int hnsw_search(unsigned int top_k, unsigned int element_n);
*/

#endif // VECTORSSD_COSMOS_APP_SRC_VECTOR_HNSW_H_
