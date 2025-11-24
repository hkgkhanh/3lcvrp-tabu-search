# 3lcvrp-tabu-search

This project implements a practical prototype of the algorithms described in:
Gendreau, M., Iori, M., Laporte, G., & Martello, S. (2006).
"A Tabu Search Algorithm for a Routing and Container Loading Problem."
Transportation Science 40(3), 342–350.

### What is included
- A simplified but faithful implementation of:
  * BL3L-SV: bottom-left 3D packing heuristic (normal positions)
  * TA3L-SV: touching-area heuristic (3D extension)
  * TS3L-SV: inner tabu search for single-vehicle loading (sequence reordering)
  * A simple outer tabu search for the 3L-CVRP (client moves between routes) with scoring
- Example instance and a `run_example.py` to show a small demo run.

### Limitations & approximations
- LIFO constraint is enforced via contiguous depth bands per client (each client's items are packed within a client-specific contiguous z-interval in visiting order). This guarantees unloadability but is a conservative approximation.
- The normal-position generation is simplified (uses container faces and placed-item faces).
- Performance/tuning: parameters are modest and not tuned to the original paper's experiments.
- This is a research prototype for learning and experimentation, not a high-performance industrial solver.

### How to run
- Open a terminal (or run in a Python environment) and execute:
    python3 run_example.py

### Files
- run_example.py      : example driver that creates an instance and runs the solver
- data.py             : data structures and instance loader/generator
- packing.py          : BL3L-SV and TA3L-SV heuristics + feasibility checks
- ts3l_sv.py          : inner tabu search for single-vehicle loading
- tabu_3l_cvrp.py     : outer tabu search for routing+loading
- utils.py            : helper functions

### Output
- The example prints packing/loading results and writes `solution_example.json` in project folder.