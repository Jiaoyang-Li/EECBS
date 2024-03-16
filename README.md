# W-EECBS
![test_ubuntu](https://github.com/Jiaoyang-Li/EECBS/actions/workflows/test_ubuntu.yml/badge.svg)
![test_macos](https://github.com/Jiaoyang-Li/EECBS/actions/workflows/test_macos.yml/badge.svg)


This branch contains the (relatively small) modifications required for W-EECBS with the low-level focal queue sorted by $g + w_h * (h + r * c)$ rather than just $c$.
We additionally added a helpful `batch_runner.py` python file to run experiments to compare EECBS and W-EECBS on different maps with different overall suboptimality $w_{so}$ and different W-EECBS parameters $w_h, r$.

For more details on W-EECBS please checkout the arXiv paper which is more up-to-date than the AAAI publication.

## Usage
### Step 1: Building
First create build folders to keep the main folder unpopulated. We recommend creating two, one for debugging and one for running experiments.

```shell script
## Debug version
mkdir build_debug && cd build_release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make
```

Make sure to `cd ..` to go back to your main workspace folder.

```shell script
## Release version
mkdir build_release && cd build_release
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make
```

### Step 2: Run Sanity Check

You should be able to run the code:
```
./build_release/eecbs -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv --outputPaths=paths.txt -k 50 -t 60 --suboptimality=1.2 
```

You can run W-EECBS by adding in the required `r_weight` and `h_weight` parameters and setting `useWeightedFocalSearch=True`.
```
./build_release/eecbs -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv --outputPaths=paths.txt -k 50 -t 60 --suboptimality=1.2 --r_weight=4 --h_weight=8 --useWeightedFocalSearch=True
```
You can find more details and explanations for all parameters with:
```
./build_release/eecbs --help
```
It will also be helpful to read `src/driver.cpp`.


### Step 3: Set-up data folder with benchmarks
We use the [standard 2D MAPF benchmark from MovingAI](https://movingai.com/benchmarks/mapf/index.html).

Make sure to `cd ..` to go back to your main workspace folder.
```shell script
mkdir data
cd data

# Download maps
wget https://movingai.com/benchmarks/mapf/mapf-map.zip
unzip mapf-map.zip -f mapf-map

# Download random scens; note other scens are available to try too
wget https://movingai.com/benchmarks/mapf/mapf-scen-random.zip
unzip mapf-scen-random.zip && mv scen-random mapf-scen-random  # Rename unzipped folder for consistency

mkdir logs  # Optional, recommended for consistency with batch_runner.py
``` 

### Step 4: Use batch_python.py to compare W-EECBS with EECBS


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

