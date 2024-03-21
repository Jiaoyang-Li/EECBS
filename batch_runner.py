import os
import argparse
import subprocess  # For executing eecbs script
import pandas as pd  # For smart batch running
import pdb # For debugging
import matplotlib.pyplot as plt  # For plotting
import numpy as np  # For utils

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


mapsToMaxNumAgents = {
    "Paris_1_256": 1000, # Verified
    "random-32-32-20": 409, # Verified
    "random-32-32-10": 461, # Verified
    "den520d": 1000, # Verified
    "den312d": 1000, # Verified
    "empty-32-32": 511, # Verified
    "empty-48-48": 1000, # Verified
    "ht_chantry": 1000, # Verified
}


def runOnSingleInstance(eecbsArgs, numAgents, seed, scenfile):
    # ### Instance
    # command = "./build_release/eecbs -m {} -a {}".format(mapfile, scenfile)
    # command += " --seed={} -k {}".format(seed, numAgents)
    # command += " -t {}".format(eecbsArgs["timeout"])
    # command += " --output={}".format(eecbsArgs["output"])
    # command += " --outputPaths={}".format(eecbsArgs["outputPaths"])

    # ### EECBS parameters
    # command += " --suboptimality={}".format(eecbsArgs["suboptimality"])

    # ### W-EECBS parameters
    # command += " --useWeightedFocalSearch={}".format(eecbsArgs["useWeightedFocalSearch"])
    # if (eecbsArgs["useWeightedFocalSearch"]):
    #     command += " --r_weight={}".format(eecbsArgs["r_weight"])
    #     command += " --h_weight={}".format(eecbsArgs["h_weight"])

    command = "./build_release/eecbs"
    for aKey in eecbsArgs:
        command += " --{}={}".format(aKey, eecbsArgs[aKey])
    command += " --agentNum={} --seed={} --agentsFile={}".format(numAgents, seed, scenfile)
    print(command)
    subprocess.run(command.split(" "), check=True) # True if want failure error
    
    
def detectExistingStatus(eecbsArgs, aNum, seed, scen):
    """
    Output:
        If has been run before
        Success if run before
    """
    if not os.path.exists(eecbsArgs["output"]):
        return False, 0
    df = pd.read_csv(eecbsArgs["output"])

    ### Checks if the corresponding runs in the df have been completed already
    for aKey, aValue in eecbsArgs.items():
        ### If this is false, then we don't care about the r_weight and h_weight
        if not eecbsArgs["useWeightedFocalSearch"]:
            if aKey == "r_weight":
                continue
            if aKey == "h_weight":
                continue
        if aKey == "output":
            continue
        df = df[df[aKey] == aValue]  # Filter the dataframe to only include the runs with the same parameters
    df = df[(df["agentsFile"] == scen) & (df["agentNum"] == aNum) & (df["seed"] == seed)]
    if len(df) > 0:
        assert(len(df) == 1)
        success = (df["solution cost"] != -1).values[0]
        return True, success
    else:
        return False, 0

def runOnSingleMap(eecbsArgs, mapName, agentNumbers, seeds, scens):
    for aNum in agentNumbers:
        print("Starting to run {} agents on map {}".format(aNum, mapName))
        numSuccess = 0
        numToRunTotal = len(scens) * len(seeds)
        for scen in scens:
            for seed in seeds:
                runBefore, status = detectExistingStatus(eecbsArgs, aNum, seed, scen)
                if not runBefore:
                    runOnSingleInstance(eecbsArgs, aNum, seed, scen)
                    runBefore, status = detectExistingStatus(eecbsArgs, aNum, seed, scen)
                    assert(runBefore)
                numSuccess += status

        if numSuccess < numToRunTotal/2:
            print("Early terminating as only succeeded {}/{} for {} agents on map {}".format(
                                            numSuccess, numToRunTotal, aNum, mapName))
            break

def helperCreateScens(numScens, mapName, dataPath):
    scens = []
    for i in range(1, numScens+1):
        scenPath = "{}/mapf-scen-random/{}-random-{}.scen".format(dataPath, mapName, i)
        scens.append(scenPath)
    return scens

def eecbs_vs_weecsb(args):
    """
    Compare the runtime of EECBS and W-EECBS
    """

    ### Create the folder for the output file if it does not exist
    if args.outputCSV == "":
        args.outputCSV = args.mapName + ".csv"
    totalOutputPath = "{}/{}".format(args.logPath, args.outputCSV)
    if not os.path.exists(os.path.dirname(totalOutputPath)):
        os.makedirs(os.path.dirname(totalOutputPath))

    eecbsArgs = {
        "mapFile": "{}/mapf-map/{}.map".format(args.dataPath, args.mapName),
        "output": totalOutputPath,
        "suboptimality": args.suboptimality,
        "cutoffTime": args.cutoffTime,
        "useWeightedFocalSearch": False,
    }
    seeds = list(range(1,2))
    scens = helperCreateScens(1, args.mapName, args.dataPath)

    increment = 100
    agentNumbers = list(range(increment, mapsToMaxNumAgents[args.mapName]+1, increment))

    ### Run baseline EECBS
    runOnSingleMap(eecbsArgs, args.mapName, agentNumbers, seeds, scens)

    ### Run W-EECBS
    eecbsArgs["r_weight"] = args.r_weight
    eecbsArgs["h_weight"] = args.h_weight
    eecbsArgs["useWeightedFocalSearch"] = True
    runOnSingleMap(eecbsArgs, args.mapName, agentNumbers, seeds, scens)

    ### Load in the data
    df = pd.read_csv(totalOutputPath)
    # Select only those with the correct cutoff time and suboptimality
    df = df[(df["cutoffTime"] == args.cutoffTime) & (df["suboptimality"] == args.suboptimality)]

    dfRegEECBS = df[df["useWeightedFocalSearch"] == False]
    dfWEECBS = df[df["useWeightedFocalSearch"] == True]
    # Select only those with the correct weights
    dfRegEECBS = dfRegEECBS[(dfRegEECBS["r_weight"] == args.r_weight) & (dfRegEECBS["h_weight"] == args.h_weight)]

    ### Compare the relative speed up when the num agents and seeds are the same
    df = pd.merge(dfRegEECBS, dfWEECBS, on=["agentNum", "seed", "agentsFile"], how="inner", suffixes=("_reg", "_w"))
    df = df[(df["solution cost_reg"] != -1) & (df["solution cost_w"] != -1)] # Only include the runs that were successful
    df["speedup"] = df["runtime_reg"] / df["runtime_w"]

    ### Plot speed up of W-EECBS over EECBS for each agent number
    df.boxplot(column="speedup", by="agentNum", grid=False)
    plt.title("W-EECBS w_so={} r={} w_h={} on {}".format(args.suboptimality, args.r_weight, args.h_weight, args.mapName))
    plt.xlabel("Number of agents")
    plt.ylabel("EECBS/W-EECBS Runtime Ratio")
    plt.savefig("{}/{}_weecbsSo{}R{}H{}_speedup.pdf".format(args.logPath, args.mapName, args.suboptimality, args.r_weight, args.h_weight))


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
                # dfs.append(tmpdf)
                # medians.append(tmpdf["speedup"])
                # pdb.set_trace()
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
    tab5df["low_per_high"] = tab5df["#low-level generated_w"] / tab5df["total_high"]
    for r_w in [16, 8, 4, 2]:
        print("-------W-EECBS r_weight: {}".format(r_w))
        tmpdf = tab5df[(tab5df["r_weight_w"] == r_w) & (tab5df["h_weight_w"] == 4)]
        assert(tmpdf.shape[0] == 25)  # Succeeded on all 25 scens
        for aKey in ["speedup", "total_high", "low_per_high", "#low-level generated_w"]:
            print("{}: {:.2f}".format(aKey, tmpdf[aKey].median()))
    ### Print out statistics for baseline EECBS
    # the EECBS baseline is already there in the _reg columns so just look at that side for an arbitrary r & h
    tab5df = tab5df[(tab5df["r_weight_w"] == 8) & (tab5df["h_weight_w"] == 4)] # Any r & h that works fine will do
    assert(tab5df.shape[0] == 25)  # Succeeded on all 25 scens
    tab5df["total_high"] = tab5df["#high-level generated_reg"] + tab5df["#adopt bypasses_reg"] # Use the reg side
    tab5df["low_per_high"] = tab5df["#low-level generated_reg"] / tab5df["total_high"] # Use the reg side
    print("-------EECBS")
    for aKey in ["total_high", "low_per_high", "#low-level generated_reg"]:
        print("{}: {:.2f}".format(aKey, tab5df[aKey].median()))

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
        print("Median speedup: {:.2f}".format(speedup_med))
        print("Max speedup: {:.2f}".format(speedup_max))
        print("Faster than baseline: {:.0f}%".format(faster_than_baseline*100))

        ## Compute the number of solved instances
        tmpdf = regCombinedDf[(regCombinedDf["suboptimality"] == subopt)]
        tmpdf["success"]= tmpdf["solution cost"] != -1
        regSolved = tmpdf.groupby(["mapFile", "agentNum"]).mean()["success"] >= 0.5
        tmpdf = wCombinedDf[(wCombinedDf["suboptimality"] == subopt)]
        tmpdf["success"] = tmpdf["solution cost"] != -1
        wSolved = tmpdf.groupby(["mapFile", "agentNum"]).mean()["success"] >= 0.5
        print("Solved W-EECBS vs EECBS: {}/{}".format(wSolved.sum()*2, regSolved.sum()*2))



def multi_plot():

    parser = argparse.ArgumentParser()
    parser.add_argument("mapName", help="map name without .map", type=str) # Note: Positional is required
    parser.add_argument("--dataPath", help="path to benchmark dataset, should contain mapf-map/ and mapf-scen-random/ folders",
                                      type=str, default="data")
    parser.add_argument("--logPath", help="path to log folder", type=str, default="data/logs/arxiv") 
    parser.add_argument("--outputCSV", help="outputCSV", type=str, default="") # Will be saved to logPath+outputCSV
    args = parser.parse_args()


    if args.outputCSV == "":
        args.outputCSV = args.mapName + ".csv"
    totalOutputPath = "{}/{}".format(args.logPath, args.outputCSV)

    df = pd.read_csv(totalOutputPath)
    # Select only those with the correct cutoff time and suboptimality
    # df = df[(df["cutoffTime"] == args.cutoffTime) & (df["suboptimality"] == args.suboptimality)]
    
    dfRegEECBS = df[df["useWeightedFocalSearch"] == False]
    dfWEECBS = df[df["useWeightedFocalSearch"] == True]
    # Select only those with the correct weights
    # dfWEECBS = dfWEECBS[(dfWEECBS["r_weight"] == args.r_weight) & (dfWEECBS["h_weight"] == args.h_weight)]


    ### Compare the relative speed up when the num agents and seeds are the same
    df = pd.merge(dfWEECBS, dfRegEECBS, on=["agentNum", "seed", "agentsFile", "suboptimality"], how="inner", suffixes=("_w", "_reg"))
    df = df[(df["solution cost_reg"] != -1) & (df["solution cost_w"] != -1)] # Only include the runs that were successful
    df["speedup"] = df["runtime_reg"] / df["runtime_w"]

    # dfs = []
    # medians = []
    ### plot x-axis number of agents, y-axis speedup, each line is a different r_weight
    whichPlot = ["r_weight", "suboptimality"][0]
    if whichPlot == "r_weight":
        cm = plt.get_cmap('gist_rainbow')
        colors = [cm(1*i/6) for i in range(6)]
        lines = ["--", "-.", ":", "-"]
        markers = ["v", "^", "<", ">"]

        plt.figure()
        for r_ind, r_w in enumerate([1, 2, 4, 8, 16, 100]):
            for h_ind, h_w in enumerate([2, 4, 8, 16]):
                tmpdf = df[(df["r_weight_w"] == r_w) & (df["h_weight_w"] == h_w)]
                tmpdf = tmpdf.groupby("agentNum").median()
                # dfs.append(tmpdf)
                # medians.append(tmpdf["speedup"])
                plt.plot(tmpdf.index, tmpdf["speedup"], label="r{},h{}".format(r_w,h_w),
                         linestyle=lines[h_ind], marker=markers[h_ind], color=colors[r_ind])
        
        plt.title("Speedup of W-EECBS over EECBS on {}".format(args.mapName))
        plt.xlabel("Number of agents")
        plt.ylabel("Speedup")
        plt.axhline(y=1, color='k', linestyle='--', alpha=0.5)
        # plt.ylim(0, None)
        # plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.02), ncol=6)
        ## place legend on bottom center
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=6)

        # plt.show()
        plt.savefig("{}/r_speedup_{}.pdf".format(args.logPath, args.mapName), bbox_inches='tight')
    elif whichPlot == "suboptimality":
        plt.figure()
        for s in [1.01, 1.1, 1.2, 1.5, 2, 4, 8]:
            # pdb.set_trace()
            tmpdf = df[(df["suboptimality"] == s) & (df["useWeightedFocalSearch_w"] == True) 
                       & (df["r_weight_w"] == 5) & (df["h_weight_w"] == 8)]
            tmpdf = tmpdf.groupby("agentNum").median()
            # dfs.append(tmpdf)
            # medians.append(tmpdf["speedup"])
            plt.plot(tmpdf.index, tmpdf["speedup"], label="suboptimality={}".format(s))
        
        plt.title("Speedup of W-EECBS over EECBS on {}".format(args.mapName))
        plt.xlabel("Number of agents")
        plt.ylabel("Speedup")
        plt.axhline(y=1, color='k', linestyle='--', alpha=0.5, label="baseline")
        # plt.ylim(0, None)
        plt.legend()
        # plt.show()
        plt.savefig("{}/suboptimality_speedup_{}.pdf".format(args.logPath, args.mapName), bbox_inches='tight')
    
    ### Plot speed up of W-EECBS over EECBS for each agent number
    # df.boxplot(column="speedup", by="agentNum", grid=False)
    # plt.title("Speedup of W-EECBS over EECBS on {}".format(args.mapName))
    # plt.xlabel("Number of agents")
    # plt.ylabel("Speedup")
    # plt.show()

# python batch_runner.py warehouse-10-20-10-2-1 --logPath data/logs/fix --cutoffTime 60 --suboptimality 2

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("mapName", help="map name without .map", type=str) # Note: Positional is required
    # parser.add_argument("--dataPath", help="path to benchmark dataset, should contain mapf-map/ and mapf-scen-random/ folders",
    #                                   type=str, default="data")
    # parser.add_argument("--logPath", help="path to log folder", type=str, default="data/logs/") 
    # parser.add_argument("--outputCSV", help="outputCSV", type=str, default="") # Will be saved to logPath+outputCSV
    # parser.add_argument("--cutoffTime", help="cutoffTime", type=int, default=60)
    # parser.add_argument("--suboptimality", help="suboptimality", type=float, default=2)
    # parser.add_argument("--r_weight", help="r_weight", type=float, default=4)
    # parser.add_argument("--h_weight", help="h_weight", type=float, default=8)
    # args = parser.parse_args()

    # eecbs_vs_weecsb(args)
    # run_arxiv_results()
    # multi_plot()
    weight_r_h_plots()
    # suboptimality_plots()