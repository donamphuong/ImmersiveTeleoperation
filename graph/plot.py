import numpy as np
import matplotlib.pyplot as plot

f = open('parallel_read.txt', 'r')
tmp = f.read().strip().split()
parallel_read = np.array(tmp, dtype=np.double)
f = open('parallel.txt', 'r')
tmp = f.read().strip().split()
parallel = np.array(tmp, dtype=np.double)
f = open('no_parallel.txt', 'r')
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
plot.plot(parallel_read[1:], '-b', label='parallel_read')
plot.plot(parallel[1:], '-r', label='parallel')
plot.plot(no_parallel[1:], 'g', label='no_parallel')
plot.legend(loc='upper left')
plot.show()
plot.draw()
myplot.savefig("test.png")
