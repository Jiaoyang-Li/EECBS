# Effective Integration of Weighted Cost-to-Go and Conflict Heuristic within Suboptimal CBS (W-EECBS)
![test_ubuntu](https://github.com/Jiaoyang-Li/EECBS/actions/workflows/test_ubuntu.yml/badge.svg)
![test_macos](https://github.com/Jiaoyang-Li/EECBS/actions/workflows/test_macos.yml/badge.svg)


This branch contains the (relatively small) modifications required for W-EECBS with the low-level focal queue sorted by $g + w_h * (h + r * c)$ rather than just $c$.
We additionally added a helpful `batch_runner.py` python file to run experiments to compare EECBS and W-EECBS on different maps with different overall suboptimality $w_{so}$ and different W-EECBS parameters $w_h, r$.

For more details on W-EECBS please checkout the [arXiv paper](https://arxiv.org/abs/2205.11624) which is more up-to-date than the [AAAI publication](https://ojs.aaai.org/index.php/AAAI/article/view/26381).

## Usage
### Step 1: Building
First create build folders to keep the main folder unpopulated. We recommend creating two, one for debugging and one for running experiments.

```shell script
## Debug version
mkdir build_debug && cd build_debug
cmake -DCMAKE_BUILD_TYPE=DEBUG ..
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

You should be able to run the code from your main workspace folder (`cd ..` accordingly):
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

Make sure to `cd` accordingly to go back to your main workspace folder.
```shell script
mkdir data
cd data

# Download maps
wget https://movingai.com/benchmarks/mapf/mapf-map.zip
unzip mapf-map.zip -d mapf-map

# Download random scens; note other scens are available to try too
wget https://movingai.com/benchmarks/mapf/mapf-scen-random.zip
unzip mapf-scen-random.zip && mv scen-random mapf-scen-random  # Rename unzipped folder for consistency

mkdir logs  # Optional, recommended for consistency with batch_runner.py
``` 

### Step 4: Use batch_python.py to compare W-EECBS with EECBS
We provided an initial python script to compare W-EECBS against EECBS.
```shell script
python batch_runner.py den312d --logPath data/logs/comparison --cutoffTime 10 --suboptimality 2 --r_weight=4 --h_weight=8
```
Feel free to try it out with different parameters or map instances. Feel free to modify this script as needed as well.

## Performance dependent on conflict heuristic computation
When working on another project we realized that W-EECBS had a more accurate conflict estimate that accidentally hurt EECBS's performance. The arXiv discusses this more in-depth and shows updated results with the less accurate but better baseline performance conflict computation.

One can get the original results (with the more accurate but hurts baseline version) by replacing `src/SpaceTimeAStar.cpp` lines 113-114 with lines 109-112.

For reproduceability we have included a `create_results.py` script for running the main experiments on each map. Running a single map takes about 1 day. We additionally have provided the [updated raw results in these csvs](https://drive.google.com/drive/folders/14CYl_WAOop0yhHOEl1gqPaEIHjLRP5vO?usp=drive_link) for your convenience.

## Reference

@article{Veerapaneni_effective_integration_weecbs_2023, \
&nbsp;    title={Effective Integration of Weighted Cost-to-Go and Conflict Heuristic within Suboptimal CBS}, \
&nbsp;    volume={37}, \
&nbsp;    url={https://ojs.aaai.org/index.php/AAAI/article/view/26381}, \
&nbsp;    DOI={10.1609/aaai.v37i10.26381}, \
&nbsp;    number={10}, \
&nbsp;    journal={Proceedings of the AAAI Conference on Artificial Intelligence}, \
&nbsp;    author={Veerapaneni, Rishi and Kusnur, Tushar and Likhachev, Maxim}, \
&nbsp; year={2023}, \
&nbsp;    month={Jun.}, \
&nbsp;    pages={11691-11698} \
}
