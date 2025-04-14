# TrajImpMC

This repository contains code for the **TrajImpMC** trajectory imputation framework. This framework aims to fill large gaps in AIS data by working with ship location estimates from multiple coastal cameras. The paper has been accepted by the [IEEE MDM 2025](https://mdm2025.github.io/) conference.

Because some files are big, please go to [Zenodo](https://zenodo.org/records/15212397) to download them.

## Project Structure

- The file **trajimpmc.py** in the **algos** folder contains source code for the framework.
- The **data** folder contains data for two settings **InsideSkagen** and **OutsideSkagen** respectivly.
- Under each setting, the file **params.csv** contains configuration of the multi-camera setting. The **InsideSkagen** setting has two cameras, and the **OutsideSkagen** has three cameras.
- Under each setting, the file **gapSamples21600-200.csv** contains the 1,200 imputation tasks, which includes 200 tasks for each gap duration of 1,2,3,4,5,6 hours.
- Under each setting, the file **trajs.csv** contains the real ship trajectories (after pre-processing) that are visible in at least two cameras. The file **trajs25832.csv** is a copy of **trajs.csv** and simply transforms the lon/lat coordinates to coordinates in **EPSG:25832**.
- Under each setting, the three files with prefix **"trajsGuessFull"** are the polygon-based ship location estimates from multiple coastal cameras. These estimates are obtained by applying the algorithm [MCbSLE](https://github.com/songwu0001/MCbSLE). Note that a slightly different version of **MCbSLE** is applied here, i.e. the predicate of whether
a pixel set is contained in another pixel set is not checked.
    1. The file **trajsGuessFullDiffPolys.csv** contains the mapping from a PolygonID to its boundary coordinates in **EPSG:25832**.
    2. The file **trajsGuessFullsimplify.csv** contains the ship location estimates at each timestamp, i.e. a mapping from a timestamp to a set of polygons that may contain one or more ships.
    3. The file **trajsGuessFullIndex.csv** contains a mapping from a polygon to its polygon neighbors, where "neighbor" means the distance between two polygons are within 25.72222 meters. This distance would be 51.44444 meters for **trajsGuessFullIndex2.csv** and 128.61111 meters for **trajsGuessFullIndex5.csv**.
- Under each setting, the file **PATH-AISClean.csv** contains the imputed trajectory results by the algorithm [AISClean](https://www.sciencedirect.com/science/article/pii/S0029801824013258), and the file **PATH-DAISTIN.CSV** contains the imputed trajectory results by the algorithm [DAISTIN](https://dl.acm.org/doi/10.1145/3609956.3609961).
- The file **experiments.py** contains code to run **TrajImpMC** every 1 (or 2 or 5) seconds. The imputed trajectory results will be saved in a file called **PATH-TrajImpMC.csv** under each setting, and the runtime is saved in a file called **TIME-TrajImpMC.csv**.

## Citation

You are welcome to use our code for research purposes, and do not forget to cite our paper :).

