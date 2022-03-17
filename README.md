# EECBS
 A bounded-suboptimal solver for Multi-Agent Path Finding


Explicit Estimation Conflict-Based Search (EECBS) is an effifent bounded-suboptimal algorithm for solving Multi-Agent Path Finding (MAPF). 
EECBS is 2-level search algorithm based on the popular optimal MAPF algorithm CBS. 
It speeds up CBS by using Explicit Estimation Search (EES) on its high level and focal search on its low level. 
It also incorporates with many CBS improvements, including 
bypassing conflicts, prioritizing conflicts, high-level heuristics, and symmetry reasoning.
More details can be found in our paper at AAAI 2021 [1].

In addition to the techniques described in [1], we also add rapid random restart technique [2] to the code. 
The default restart times is 0.  

The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake: 
```
cmake .
make
```

You also need to download the MAPF instances from the MAPF benchmark (https://movingai.com/benchmarks/mapf/index.html). In particular, the format of the scen files is explained here: https://movingai.com/benchmarks/formats.html. For a given number of agents k, the first k rows of the scen file are used to generate the k pairs of start and target locations.

Then, you are able to run the code:
```
./eecbs -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv --outputPaths=paths.txt -k 50 -t 60 --suboptimality=1.2 
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- o: the output file that contains the search statistics
- outputPaths: the output file that contains the paths 
- k: the number of agents
- t: the runtime limit
- suboptimality: the suboptimality factor w

You can find more details and explanations for all parameters with:
```
./eecbs --help
```

## License
EECBS is released under USC – Research License. See license.md for further details.
 
## References
[1] Jiaoyang Li, Wheeler Ruml and Sven Koenig.
EECBS: Bounded-Suboptimal Search for Multi-Agent Path Finding.
In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), (in print), 2021.

