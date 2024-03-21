import os
import argparse
import subprocess  # For executing eecbs script
import pandas as pd  # For smart batch running
import pdb # For debugging
import matplotlib.pyplot as plt  # For plotting
import numpy as np  # For utils
from batch_runner import helperCreateScens, runOnSingleMap, detectExistingStatus, runOnSingleInstance, mapsToMaxNumAgents

####### Set the font size of the plots
SMALL_SIZE = 12
MEDIUM_SIZE = 14
BIGGER_SIZE = 20

plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=MEDIUM_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=SMALL_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title


def run_arxiv_results():
    """
    Run arxiv results
    python batch_runner.py [mapname]
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("mapName", help="map name without .map", type=str) # Note: Positional is required
    parser.add_argument("--dataPath", help="path to benchmark dataset, should contain mapf-map/ and mapf-scen-random/ folders",
                                      type=str, default="data")
    parser.add_argument("--logPath", help="path to log folder", type=str, default="data/logs/arxiv") 
    parser.add_argument("--outputCSV", help="outputCSV", type=str, default="") # Will be saved to logPath+outputCSV
    args = parser.parse_args()

    ### Create the folder for the output file if it does not exist
    if args.outputCSV == "":
        args.outputCSV = args.mapName + ".csv"
    totalOutputPath = "{}/{}".format(args.logPath, args.outputCSV)
    if not os.path.exists(os.path.dirname(totalOutputPath)):
        os.makedirs(os.path.dirname(totalOutputPath))

    eecbsArgs = {
        "mapFile": "{}/mapf-map/{}.map".format(args.dataPath, args.mapName),
        "output": totalOutputPath,
        "cutoffTime": 60, 
    }
    seeds = list(range(1,2))
    scens = helperCreateScens(25, args.mapName, args.dataPath)
    
    increment = 100
    agentNumbers = list(range(increment, mapsToMaxNumAgents[args.mapName]+1, increment))

    ####################################################
    #### Figure 4: Different h_weights and r_weights
    eecbsArgs["suboptimality"] = 2
    eecbsArgs["useWeightedFocalSearch"] = False
    runOnSingleMap(eecbsArgs, args.mapName, agentNumbers, seeds, scens)

    eecbsArgs["useWeightedFocalSearch"] = True
    for r_w in [1, 2, 4, 8, 16, 100]:
        for h_w in [2, 4, 8, 16]:
            eecbsArgs["r_weight"] = r_w
            eecbsArgs["h_weight"] = h_w
            runOnSingleMap(eecbsArgs, args.mapName, agentNumbers, seeds, scens)

    ####################################################
    #### Table 4: Different suboptimality
    eecbsArgs["r_weight"] = 5
    eecbsArgs["h_weight"] = 8
    eecbsArgs["useWeightedFocalSearch"] = False
    for s in [1.01, 1.1, 1.2, 1.5, 2, 4, 8]:
        eecbsArgs["suboptimality"] = s
        runOnSingleMap(eecbsArgs, args.mapName, agentNumbers, seeds, scens)

    eecbsArgs["useWeightedFocalSearch"] = True
    for s in [1.01, 1.1, 1.2, 1.5, 2, 4, 8]:
        eecbsArgs["suboptimality"] = s
        runOnSingleMap(eecbsArgs, args.mapName, agentNumbers, seeds, scens)

def weight_r_h_plots():
    ORDEREDMAPNAMES = ["Paris_1_256", "den520d", "ht_chantry", "den312d", "empty-48-48", "empty-32-32", "random-32-32-10", "random-32-32-20"]
    ncols = 4
    nrows= 2
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(ncols*4+4,nrows*4))

    cm = plt.get_cmap('gist_rainbow')
    colors = [cm(1*i/6) for i in range(6)]
    lines = ["--", "-.", ":", "-"]
    markers = ["v", "^", "<", ">"]
    
    combinedDf = pd.DataFrame()  # Use for computing table 5
    for i, mapName in enumerate(ORDEREDMAPNAMES):
        df = pd.read_csv("data/logs/arxiv/{}.csv".format(mapName))
        df = df[(df["cutoffTime"] == 60) & (df["suboptimality"] == 2)]
        dfRegEECBS = df[df["useWeightedFocalSearch"] == False]
        dfWEECBS = df[df["useWeightedFocalSearch"] == True]

        df = pd.merge(dfWEECBS, dfRegEECBS, on=["agentNum", "seed", "agentsFile", "suboptimality"], how="inner", suffixes=("_w", "_reg"))
        df = df[(df["solution cost_reg"] != -1) & (df["solution cost_w"] != -1)]
        df["speedup"] = df["runtime_reg"] / df["runtime_w"]
        combinedDf = pd.concat([combinedDf, df])  # Use for computing table 5

        curAx = axes[i//ncols, i%ncols]
        curAx.set_title("{}".format(mapName))
        for r_ind, r_w in enumerate([1, 2, 4, 8, 16, 100]):
            for h_ind, h_w in enumerate([2, 4, 8, 16]):
                tmpdf = df[(df["r_weight_w"] == r_w) & (df["h_weight_w"] == h_w)]
                tmpdf = tmpdf.groupby("agentNum").median()
                curAx.plot(tmpdf.index, tmpdf["speedup"], label="r{},h{}".format(r_w,h_w),
                         linestyle=lines[h_ind], marker=markers[h_ind], color=colors[r_ind])
        curAx.axhline(y=1, color='k', linestyle='--', alpha=0.5)

    plt.setp(axes[-1, :], xlabel="# agents")
    plt.setp(axes[:, 0], ylabel="Speed up")

    handles, labels = curAx.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.05), ncol=6)
    fig.suptitle("Speedup of W-EECBS based on focal weights")
    plt.savefig("weight_study.pdf", bbox_inches='tight', dpi=600)

    ### Table 5
    pd.set_option('chained_assignment', None) # Get rid of warning for the next part
    tab5df = combinedDf[(combinedDf["suboptimality"] == 2) & (combinedDf["mapFile_w"].str.contains("den312d")) &
                       (combinedDf["agentNum"] == 200)]
    print("---------Table 5")
    tab5df = tab5df[(tab5df["useWeightedFocalSearch_w"] == True)]
    tab5df["total_high"] = tab5df["#high-level generated_w"] + tab5df["#adopt bypasses_w"]
    tab5df["low_per_high"] = tab5df["#low-level expanded_w"] / (tab5df["total_high"] + tab5df["agentNum"] - 1)
    for r_w in [16, 8, 4, 2]:
        print("-------W-EECBS r_weight: {}".format(r_w))
        tmpdf = tab5df[(tab5df["r_weight_w"] == r_w) & (tab5df["h_weight_w"] == 4)]
        assert(tmpdf.shape[0] == 25)  # Succeeded on all 25 scens
        for aKey in ["speedup", "total_high", "low_per_high", "#low-level expanded_w"]:
            print("{}: {:.2f}".format(aKey, tmpdf[aKey].mean()))
    ### Print out statistics for baseline EECBS
    # the EECBS baseline is already there in the _reg columns so just look at that side for an arbitrary r & h
    tab5df = tab5df[(tab5df["r_weight_w"] == 8) & (tab5df["h_weight_w"] == 4)] # Any r & h that works fine will do
    assert(tab5df.shape[0] == 25)  # Succeeded on all 25 scens
    tab5df["total_high"] = tab5df["#high-level generated_reg"] + tab5df["#adopt bypasses_reg"] # Use the reg side
    tab5df["low_per_high"] = tab5df["#low-level expanded_reg"] / (tab5df["total_high"] + tab5df["agentNum"] - 1) # Use the reg side
    print("-------EECBS")
    for aKey in ["total_high", "low_per_high", "#low-level expanded_reg"]:
        print("{}: {:.2f}".format(aKey, tab5df[aKey].mean()))

def suboptimality_plots():
    ORDEREDMAPNAMES = ["Paris_1_256", "den520d", "ht_chantry", "den312d", "empty-48-48", "empty-32-32", "random-32-32-10", "random-32-32-20"]
    ncols = 4
    nrows= 2
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(ncols*4+4,nrows*4))

    cm = plt.get_cmap('gist_rainbow')
    colors = [cm(1*i/7) for i in range(7)]
    
    combinedDf = pd.DataFrame()  # Use for computing table 4
    regCombinedDf = pd.DataFrame()
    wCombinedDf = pd.DataFrame()
    for i, mapName in enumerate(ORDEREDMAPNAMES):
        df = pd.read_csv("data/logs/arxiv/{}.csv".format(mapName))
        df = df[(df["cutoffTime"] == 60)]
        dfRegEECBS = df[df["useWeightedFocalSearch"] == False]
        dfWEECBS = df[(df["useWeightedFocalSearch"] == True) & (df["r_weight"] == 5) & (df["h_weight"] == 8)]
        regCombinedDf = pd.concat([regCombinedDf, dfRegEECBS])  # Use for computing table 4
        wCombinedDf = pd.concat([wCombinedDf, dfWEECBS])  # Use for computing table 4

        df = pd.merge(dfWEECBS, dfRegEECBS, on=["agentNum", "seed", "agentsFile", "suboptimality"], how="inner", suffixes=("_w", "_reg"))
        df = df[(df["solution cost_reg"] != -1) & (df["solution cost_w"] != -1)]
        df["speedup"] = df["runtime_reg"] / df["runtime_w"]
        combinedDf = pd.concat([combinedDf, df])  # Use for computing table 4

        curAx = axes[i//ncols, i%ncols]
        curAx.set_title("{}".format(mapName))
        for ind, subopt in enumerate([1.01, 1.1, 1.2, 1.5, 2, 4, 8]):
            tmpdf = df[(df["suboptimality"] == subopt)]
            tmpdf = tmpdf.groupby("agentNum").median()
            curAx.plot(tmpdf.index, tmpdf["speedup"], label="w_so={}".format(subopt), color=colors[ind])
        curAx.axhline(y=1, color='k', linestyle='--', alpha=0.5)

    plt.setp(axes[-1, :], xlabel="# agents")
    plt.setp(axes[:, 0], ylabel="Speed up")

    handles, labels = curAx.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.05), ncol=4)
    fig.suptitle("Speedup of W-EECBS (r=5,h=8) across different suboptimalities")
    plt.savefig("suboptimality_study.pdf", bbox_inches='tight', dpi=600)

    ### Table 4
    pd.set_option('chained_assignment', None) # Get rid of warning for the next part
    for subopt in [1.01, 1.1, 1.2, 1.5, 2, 4, 8]:
        print("---------Suboptimality: {}".format(subopt))
        tmpdf = combinedDf[(combinedDf["suboptimality"] == subopt)]
        speedup_med = tmpdf["speedup"].median()
        # speedup_max = tmpdf["speedup"].max()
        speedup_max = tmpdf.groupby(["mapFile_w", "agentNum"]).median()["speedup"].max() # Aggregate across map & agents
        faster_than_baseline = (tmpdf["speedup"] > 1).mean()
        print("Max speedup: {:.2f}".format(speedup_max))
        print("Median speedup: {:.2f}".format(speedup_med))
        print("Faster than baseline: {:.0f}%".format(faster_than_baseline*100))

        ## Compute the number of solved instances
        tmpdf = regCombinedDf[(regCombinedDf["suboptimality"] == subopt)]
        tmpdf["success"]= tmpdf["solution cost"] != -1
        regSolved = tmpdf.groupby(["mapFile", "agentNum"]).mean()["success"] >= 0.5
        tmpdf = wCombinedDf[(wCombinedDf["suboptimality"] == subopt)]
        tmpdf["success"] = tmpdf["solution cost"] != -1
        wSolved = tmpdf.groupby(["mapFile", "agentNum"]).mean()["success"] >= 0.5
        print("Solved W-EECBS vs EECBS: {}/{}".format(wSolved.sum(), regSolved.sum()))


if __name__ == "__main__":
    # run_arxiv_results()
    # weight_r_h_plots()
    # suboptimality_plots()
    pass