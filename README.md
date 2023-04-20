# EECBS
![test_ubuntu](https://github.com/Jiaoyang-Li/EECBS/actions/workflows/test_ubuntu.yml/badge.svg)
![test_macos](https://github.com/Jiaoyang-Li/EECBS/actions/workflows/test_macos.yml/badge.svg)

 A bounded-suboptimal solver for Multi-Agent Path Finding


Explicit Estimation Conflict-Based Search (EECBS) is an efficient bounded-suboptimal algorithm for solving Multi-Agent Path Finding (MAPF). 
EECBS is 2-level search algorithm based on the popular optimal MAPF algorithm CBS. 
It speeds up CBS by using Explicit Estimation Search (EES) on its high level and focal search on its low level. 
It also incorporates with many CBS improvements, including 
bypassing conflicts, prioritizing conflicts, high-level heuristics, and symmetry reasoning.
More details can be found in our paper at AAAI 2021 [1].

In addition to the techniques described in [1], we also add rapid random restart technique [2] to the code. 
The default restart times is 0.  

Moreover, we also added a SIPP option that uses SIPPS [3] (instead of state-time A*) in the low level of EECBS to plan paths for agents.

The code requires the external library BOOST (https://www.boost.org/). After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake: 

## Usage
The code requires the external library [boost](https://www.boost.org/).
If you are using Ubuntu, you can install it simply by
```shell script
sudo apt install libboost-all-dev
``` 
Another easy way of installing the boost library is to install anaconda/miniconda and then
```shell script
conda install -c anaconda libboost
```
which works for a variety of [systems](https://anaconda.org/anaconda/libboost)
(including linux, osx, and win).

A third way you can try is to install Homebrew and then
```shell script
brew install boost
```

If none of the above method works, you can also follow the instructions
on the [boost](https://www.boost.org/) website and install it manually.


After you installed boost and downloaded the source code, go into the directory of the source code and compile it with CMake:
```shell script
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```

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

To test the code on more instances,
you can download the MAPF instances from the [MAPF benchmark](https://movingai.com/benchmarks/mapf/index.html).
In particular, the format of the scen files is explained [here](https://movingai.com/benchmarks/formats.html).
For a given number of agents k, the first k rows of the scen file are used to generate the k pairs of start and target locations.

## License
EECBS is released under USC – Research License. See license.md for further details.
 
## References
[1] Jiaoyang Li, Wheeler Ruml and Sven Koenig.
EECBS: Bounded-Suboptimal Search for Multi-Agent Path Finding.
In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), pages 12353-12362, 2021.

[2] Liron Cohen, Glenn Wagner, David M. Chan, Howie Choset, Nathan R. Sturtevant, Sven Koenig and T. K. Satish Kumar.
Rapid Randomized Restarts for Multi-Agent Path Finding Solvers.
In Proceedings of the Symposium on Combinatorial Search (SoCS), pages 148-152, 2018.

[3] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey and Sven Koenig. 
MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search.
In Proceedings of the AAAI Conference on Artificial Intelligence, pages 10256-10265, 2022.

