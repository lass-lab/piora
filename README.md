
# PIORA: Preemptible In-Storage Multi-Task Execution of Vector Databases with SLO Guarantees
## [NOTICE] UNDER-REVIEW.
Currently, the proposed system is under review. The whole source code will be released soon.

## Overview
PIORA is a in-storage processing system that enables preemptible in-storage multi-task execution for vector databases while maintaining Service Level Objective (SLO) guarantees.
This project addresses the challenge of efficiently managing multiple concurrent tasks in vector database systems by leveraging in-storage computation and intelligent preemption mechanisms.

## Key Features

- **Preemptible Multi-Task Execution**: Enables dynamic task scheduling and preemption for optimal resource utilization.
- **In-Storage Computing**: Reduces data movement overhead by performing computations in storage.
- **SLO Guarantees**: Maintains SLO guarantees through intelligent resource management.

## Repository layout
```
/benchmark       # benchmarking harness, workloads, experiment scripts
/cosmos_hw       # hardware-related code (Cosmos SSD prototype, if present)
/util            # utility scripts (data preprocessing, plotting helpers)
/vector          # Not included now
LICENSE
README.md
```

## Requirements
- Linux (Ubuntu 20.04+ recommended)
- gcc/clang toolchain for C code
- Python 3.8+ with pip
- Cosmos+ OpenSSD board (hardware)
- Xilinx SDK

## Benchmarks & experiments

The _benchmark_ directory contains API for communicating with NVMe SSD and its how to use.
In particular, PIORA provides user-friendly python-based API, which has been pybinded.

## License

This project is licensed under the MIT License — see the LICENSE file for details