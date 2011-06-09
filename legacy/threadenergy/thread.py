#!/usr/bin/python
import os
import sys
import time
from numpy import *

def prepareParams(params):
    toret = ["rot 1", "pos 3", "c "+str(params[0]), "t "+str(params[1]), "df "+str(params[2]), "tf "+str(params[3]), "te 500", "g "+str(params[4]/1000000.0)]
    return "\n".join(r for r in toret)

def systemcall(string):
    print "system call: ",string
    os.system(string)                   #

if __name__=="__main__":
    cmd = sys.argv[1]

    if(cmd == "data"):
        # run estimate_params
        systemcall("ls ../vision/suture_sets/")

    if(cmd == "prepare"):
        outdir = "results/latest"
        if(len(sys.argv)<4):
            print "Usage: ./thread.py prepare <trainingset1,trainingset2,...> <testset1,testset2,...> <(optional) outdirname>"
        if(len(sys.argv)>4):
            outdir = sys.argv[4]

        # make a backup first
        systemcall("cp -r %s results/%s" % (outdir,int(time.time())))
        systemcall("rm -rf %s"%outdir)
        systemcall("mkdir %s"%outdir)
        systemcall("mkdir %s/test"%outdir)
        systemcall("mkdir %s/training"%outdir)

        matchstrings = [x for x in sys.argv[2].split(',')]
        testmatchstrings = [x for x in sys.argv[3].split(",")]
        all_sets = [x for x in os.listdir('../vision/suture_sets/')]
        sets = [x for x in all_sets if x in matchstrings]
        testsets = [x for x in all_sets if x in testmatchstrings]
        print "The following sets were found: ", all_sets
        print "The following sets will be used for training: ", sets
        print "The following sets will be used for testing: ",testsets

        systemcall("rm %s/training/*.points"%outdir)
        systemcall("rm %s/test/*.points"%outdir)

        tocopy = ["cp ../vision/suture_sets/%s/*traj.txt %s/training/%d_traj.txt"%(x,outdir,i) for i,x in enumerate(sets)]
        tocopy += ["cp ../vision/suture_sets/%s/*imgs.txt %s/training/%d_traj_imgs.txt"%(x,outdir, i) for i,x in enumerate(sets)]
        for string in tocopy:
            systemcall(string)
        systemcall("echo %d > %s/numtraining"% (len(tocopy)/2, outdir))
        systemcall("echo \"%s\" > %s/trainingdata"% ("\n".join(sets),outdir))

        tocopy = ["cp ../vision/suture_sets/%s/*traj.txt %s/test/%d_traj.txt"%(x,outdir,i) for i,x in enumerate(testsets)]
        tocopy += ["cp ../vision/suture_sets/%s/*imgs.txt %s/test/%d_traj_imgs.txt"%(x,outdir,i) for i,x in enumerate(testsets)]
        for string in tocopy:
            systemcall(string)
        systemcall("echo %d > %s/numtest" % (len(tocopy)/2, outdir))
        systemcall("echo \"%s\" > %s/testdata"% ("\n".join(testsets), outdir))
        systemcall("cp ../config/test.params %s/test.params"%outdir)

        # modify the C++ eval function to read in the number of trajectories, create trajectory reader, send them all through
        # modify the trajectory reader to read from a file specified

    if(cmd=="backup"):
        filename = int(time.time())
        outdir = "results/latest"
        if(len(sys.argv) > 2):
            outdir = sys.argv[2]
        if(len(sys.argv) > 3):
            filename = sys.argv[3]

        systemcall("cp *.optimize.txt %s/"%outdir)
        systemcall("cp -r %s results/%s" % (outdir,filename))


        # generate all xvalid directories at once
        # look for the presence of a results file
    if(cmd == "splice"):
        #
        if len(sys.argv) < 3:
            print "usage: thread.py splice <setname>"
            exit(1)
        oldfile = sys.argv[2]

        folder_name = [x for x in os.listdir('../vision/suture_sets/') if oldfile in x]

        # read in annotation file
        annotations = "../vision/suture_sets/%s/%s.txt"%(oldfile, oldfile)
        goods = []
        greats = []
        okay = []
        totalCount = 0
        with open(annotations,'r') as f:
            for line in f:
                line = line[:-1].split(",")
                if line[1].upper()=="PPP":
                    greats.append(int(line[0])-1)
                    goods.append(int(line[0])-1)
                elif not "B" in line[1].upper():
                    goods.append(int(line[0])-1)
                    okay.append(int(line[0])-1)
                totalCount += 1

        print "Great ones: ", greats
        print "Good ones: ", goods
        print "Okay ones: ", okay
        print "Summary: Great: %d Good: %d Okay: %d Total: %d"%(len(greats), len(goods), len(okay), totalCount)


        # write out 2 splicer directories, one for great and one for okay
        # copy everything to both

        oldset = "../vision/suture_sets/%s"%oldfile
        greatset = oldfile+"_perfect"
        goodset = oldfile+"_good"

        tocopy = [[greatset, greats], [goodset, goods]]
        for set, lst in tocopy:
            systemcall("rm -rf ../vision/suture_sets/%s" % set)
            systemcall("mkdir ../vision/suture_sets/%s" % set)
            systemcall("cp %s/%s_traj.txt ../vision/suture_sets/%s/%s_traj.txt.input" % (oldset, oldfile, set, set))
            systemcall("cp %s/%s_traj_imgs.txt ../vision/suture_sets/%s/%s_traj_imgs.txt.input" % (oldset, oldfile, set, set))
            systemcall("cp %s/%s.txt ../vision/suture_sets/%s/%s.txt" % (oldset, oldfile, set, set))

            systemcall("echo %d > ../vision/suture_sets/%s/splice" % (len(lst), set))
            systemcall("echo \"%s\" >> ../vision/suture_sets/%s/splice" % (" ".join(str(x) for x in lst), set))

            systemcall("./spliceTrajectory %s"%set)
            systemcall("rm -f ../vision/suture_sets/%s/%s_traj_imgs.points"%(set, set))

    if(cmd == "lookup"):
        if(len(sys.argv) < 5):
            print "Usage: ./thread.py lookup <dirname> <trajectory num> <thread num>"
            sys.exit(1)
        dirname = sys.argv[2]
        trajnum = int(sys.argv[3])
        index = int(sys.argv[4])

        # look up
        if "train" in dirname:
            datafile = dirname+"/../trainingdata"
        elif "test" in dirname:
            datafile = dirname+"../testdata"

        # read the trajnum line, go to the right set, pull the splice file, return the appropriate index
        with open(datafile) as f:
            for i in range(trajnum):
                f.readline()
            line = f.readline()[:-1]
            print "source was: " + line[:-1]

        # line has directory: read splice file
        with open("../vision/suture_sets/"+line+"/splice") as f:
            f.readline()
            lst = f.readline()[:-1].split()

        print "source image (1-indexed) was image: "+str(int(lst[index])+1)


    if(cmd == "time"):
        if(len(sys.argv) < 3):
            print "Usage: ./thread.py time <results_directory> <testnum (0)>"
            sys.exit(1)
        test = "0"
        if(len(sys.argv) > 3):
            test = sys.argv[3]
        dirname = sys.argv[2]

        with open(dirname+"/intermediate_data", "r") as f:
            systemcall("cp %s/test/perSamplesScore %s/test/perSamplesScoreBackup"%(dirname, dirname))

            for line in f:
                data = line[:-1].split()
                # if(len(data) < 5): break
                # params = [float(data[i]) for i in range(-5,0)]
                if "took" in line: break
                params = [float(data[i]) for i in range(-2,0)]
                params += [0.0,0.0,0.0]

                print "Running on iter %d params: %s"%(int(data[1])," ".join(str(x) for x in params))
                systemcall("echo \"%s\" > %s/new%d.params"%(prepareParams(params), dirname, int(data[1])))
                systemcall("./estimateThreadParams eval %s/new%d.params %s/test/%s"%(dirname,int(data[1]), dirname, test))
                systemcall("cp %s/test/%s_perSamplesScore_eval %s/test/%s_time%d"%(dirname, test, dirname, test, int(data[1])))

            systemcall("cp %s/test/perSamplesScoreBackup %s/test/perSamplesScore"%(dirname,dirname))
            print "Took %f seconds"% float(data[1])

    if(cmd == "plottime"):
        if(len(sys.argv) < 3):
            print "Usage: ./thread.py plottime <results_directory> <testnum (0)>"
            sys.exit(1)
        test = "0"
        if(len(sys.argv) > 3):
            test = sys.argv[3]
        dirname = sys.argv[2]

        # for all "test_time%d" files in the given directory, read them in and find mean / median
        filesToOpen = [x for x in os.listdir(dirname) if (test+"_time") in x]
        data = []
        for fn in filesToOpen:
            ts = fn[fn.find("time")+4:]
            with open(dirname+"/"+fn) as f:
                lst = []
                for line in f:
                    lst.append(float(line[:-1].split()[2]))
                data.append([int(ts), mean(lst), median(lst)])

        data.sort()
        with open(dirname+"/timeseries","w") as f:
            for tup in data:
                f.write(" ".join(str(x) for x in tup)+"\n")
