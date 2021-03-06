import numpy as np
import matplotlib.pyplot as plot

f = open('parallel_read_2.txt', 'r')
tmp = f.read().strip().split()
parallel_read = np.array(tmp, dtype=np.double)
f = open('parallel_2.txt', 'r')
tmp = f.read().strip().split()
parallel = np.array(tmp, dtype=np.double)
f = open('no_parallel_2.txt', 'r')
tmp = f.read().strip().split()
no_parallel = np.array(tmp, dtype=np.double)

print "mean parallel_read " + str(np.mean(parallel_read))
print "mean parallel " + str(np.mean(parallel))
print "mean no_parallel " + str(np.mean(no_parallel))
print "median parallel_read " + str(np.median(parallel_read))
print "median parallel " + str(np.median(parallel))
print "median no_parallel " + str(np.median(no_parallel))

myplot = plot.figure(figsize=(20, 20))
plot.clf()
plot.ylim([0.1, 0.2])
plot.plot(parallel_read[0:50], '-b', label='Parallelize Video Stitching Pipeline Using C++ Threads')
plot.grid(True, which='both')
plot.minorticks_on()
plot.plot(parallel[0:50], '-r', label='Parallel Video Streaming Pipeline Using TBB')
plot.plot(no_parallel[0:50], 'g', label='Serial Pipeline')
plot.legend(loc='upper left')
plot.show()
plot.draw()
myplot.savefig("result2.png")
