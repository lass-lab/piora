# Vector Codes inside PIORA

This directory contains the core implementation of PIORA's vector database components for in-storage processing.

## Directory Structure

```
src/
└── vector/           # Vector database implementation
    ├── README.md          # This file
    ├── binary_heap.c/.h    # Binary heap data structure
    ├── hnsw.c/.h          # Hierarchical Navigable Small World implementation
    ├── hnsw_config.h      # HNSW algorithm configuration
    └── index.c/.h         # Vector index interface
```

## Components

### Vector Index Implementations

#### HNSW (Hierarchical Navigable Small World)
- **Files**: `hnsw.c`, `hnsw.h`, `hnsw_config.h`
- Proximity graph-based approximate nearest neighbor search algorithm
- Provides robust similarity search in high-dimensional vector spaces
- Configuration parameters in `hnsw_config.h`

#### Binary Heap
- **Files**: `binary_heap.c`, `binary_heap.h`
- Binary heap using array implementation
- Used for k-nearest neighbor result management

#### Index Interface
- **Files**: `index.c`, `index.h`
- Unified interface for vector indexing operations
- Abstracts different index implementations

## Usage

This source code is designed to run on Cosmos+ OpenSSD hardware with in-storage processing capabilities. For detailed usage and API documentation, see the __benchmark__ directory or __src__ directory, which will be relased in the near future.

## License
This code is part of the PIORA project and is licensed under GPLv3.