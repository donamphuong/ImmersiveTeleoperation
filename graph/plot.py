import numpy as np
import matplotlib.pyplot as plot

def plot_data(data):
  plot.clf()
  plot.plot(data)
  plot.show()
  plot.savefig("test.png")
  print np.mean(data)

if __name__ == "__main__":
  counter = 0
  while True:
    try:
	f = open('parallel_read.txt', 'r')
        tmp = f.read().strip().split()
        data = np.array(tmp, dtype=np.double)
        print data
    except EOFError:
        print "Input has terminated! Exiting"
        exit()
    except ValueError:
        print "Invalid input, skipping. Input was: %s"%tmp
        continue

    print "Plotting plot number %d"%counter
    plot_data(data)
    counter += 1
