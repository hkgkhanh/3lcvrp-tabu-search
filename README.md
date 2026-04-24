# 3lcvrp-tabu-search

Algorithm implementation of [A Tabu Search Algorithm for a Routing and Container Loading Problem](https://doi.org/10.1287/trsc.1050.0145)

## How to run

    ```
    python complete_gendreau_3lcvrp.py
        --instances dataset/Gendreau_et_al_2006
        --out [output folder]
        --time [time limit for each iteration]
        --iter [number of max iteration per instance]
        --repeats [number of repeats for each instance]
        --start-instance [continue experimentation from which instance]
        --start-repeat [continue the paused instance from which repeat]
    ```

Example:

    ```
    python complete_gendreau_3lcvrp.py --instances dataset/Gendreau_et_al_2006 --out output/Gendreau_et_al_2006 --time 3600 --iter 100 --repeats 1 --start-instance 3l_cvrp01.txt --start-repeat 0
    ```

## References
- Gendreau, M., Iori, M., Laporte, G., & Martello, S. (2006). "A Tabu Search Algorithm for a Routing and Container Loading Problem." Transportation Science 40(3), 342–350. https://doi.org/10.1287/trsc.1050.0145


failed: 3l_cvrp07, 3l_cvrp19, 3l_cvrp20, 3l_cvrp21, 3l_cvrp22, 3l_cvrp25, 3l_cvrp26, 3l_cvrp27

partial: 3l_cvrp11, 3l_cvrp13, 3l_cvrp18, 3l_cvrp23, 3l_cvrp24