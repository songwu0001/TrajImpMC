from algos.trajimpmc import TrajImpMC
from datetime import datetime
import pickle
import os


def run(DIR, whichAlgo):
    with open(DIR + 'gapSamples21600-200.csv', 'r') as fIn:
        gaps = [line.strip().split(',') for line in fIn.readlines()[1:]]

    pathFile = DIR + 'PATH-' + whichAlgo + '.csv'
    timeFile = DIR + 'TIME-' + whichAlgo + '.csv'
    pathMode = 'w' if not os.path.exists(pathFile) else 'a'
    timeMode = 'w' if not os.path.exists(timeFile) else 'a'
    with open(pathFile, pathMode) as fOut, open(timeFile, timeMode) as fOut2:
        if pathMode == 'w':
            fOut.write('gapID,order,lon,lat,label\n')
        if timeMode == 'w':
            fOut2.write('gapID,duration,runtime,label\n')
        if whichAlgo.startswith('TrajImpMC'):
            if whichAlgo == "TrajImpMC":
                algo = TrajImpMC(DIR)
            else:
                timestep = int(whichAlgo[9:])
                algo = TrajImpMC(DIR, timestep)
        else:
            raise Exception('No such algo exists')

        for gap in gaps:
            print("processing gap#" + gap[0])
            if whichAlgo.startswith('TrajImpMC'):
                tStart = datetime.now()
                path = algo.solveGap(gap[2], int(gap[1]), [float(gap[5]), float(gap[4])], [float(gap[8]), float(gap[7])], float(gap[11]), float(gap[12]))
                tEnd = datetime.now()

            outLines = []
            for idx,point in enumerate(path):
                outLines.append('{},{},{},{},{}\n'.format(gap[0], idx, point[0], point[1], whichAlgo))
            fOut.write(''.join(outLines))

            runtime = (tEnd - tStart).total_seconds()
            fOut2.write(','.join([gap[0], gap[1], str(runtime), whichAlgo]))
            fOut2.write('\n')
            fOut2.flush()


if __name__ == '__main__':
    DIRs = ["data/InsideSkagen/", 'data/OutsideSkagen/']
    run(DIRs[0], 'TrajImpMC')
    # run(DIRs[1], 'TrajImpMC2')
    # run(DIRs[1], 'TrajImpMC5')
