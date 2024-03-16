import os
import argparse
import subprocess  # For executing eecbs script
import pandas as pd  # For smart batch running
import pdb # For debugging
import matplotlib.pyplot as plt  # For plotting

mapsToMaxNumAgents = {
    "Paris_1_256": 1000, # Verified
    "random-32-32-20": 409, # Verified
    "random-32-32-10": 461, # Verified
    "den520d": 1000, # Verified
    "den312d": 1000, # Verified
    "empty-32-32": 511, # Verified
    "empty-48-48": 1000, # Verified
    "ht_chantry": 1000, # Verified
    "warehouse-10-20-10-2-1": 1000, # Verified
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
    parser = argparse.ArgumentParser()
    parser.add_argument("mapName", help="map name without .map", type=str) # Note: Positional is required
    parser.add_argument("--dataPath", help="path to benchmark dataset, should contain mapf-map/ and mapf-scen-random/ folders",
                                      type=str, default="data")
    parser.add_argument("--logPath", help="path to log folder", type=str, default="data/logs/arxiv") 
    parser.add_argument("--outputCSV", help="outputCSV", type=str, default="") # Will be saved to logPath+outputCSV
    args = parser.parse_args()

    """
    Run arxiv results
    python batch_runner.py [mapname]
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

def multi_plot(args):
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
    whichPlot = ["r_weight", "suboptimality"][1]
    if whichPlot == "r_weight":
        plt.figure()
        for r in [4,8,16,32]:
            tmpdf = df[(df["r_weight_w"] == r) & (df["h_weight_w"] == 4)]
            tmpdf = tmpdf.groupby("agentNum").median()
            # dfs.append(tmpdf)
            # medians.append(tmpdf["speedup"])
            plt.plot(tmpdf.index, tmpdf["speedup"], label="r_weight={}".format(r))
        
        plt.title("Speedup of W-EECBS over EECBS on {}".format(args.mapName))
        plt.xlabel("Number of agents")
        plt.ylabel("Speedup")
        plt.axhline(y=1, color='k', linestyle='--', alpha=0.5, label="baseline")
        # plt.ylim(0, None)
        plt.legend()
        # plt.show()
        plt.savefig("{}/r_speedup_{}.pdf".format(args.logPath, args.mapName))
    elif whichPlot == "suboptimality":
        plt.figure()
        for s in [1.01, 1.1, 1.2, 1.5, 2, 4, 8]:
            # pdb.set_trace()
            tmpdf = df[(df["suboptimality"] == s) & (df["useWeightedFocalSearch_w"] == True)]
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
        plt.savefig("{}/suboptimality_speedup_{}.pdf".format(args.logPath, args.mapName))
    
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
    run_arxiv_results()
    # multi_plot(args)