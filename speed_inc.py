import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import pandas as pd
import numpy as np

def main():
	num_circs = np.arange(9,19)
	speedup = np.array([0.0025, 0.236, 0.596, 0.467, 1.57, 0.549, 1.15, 4.5, 4.5, 6.7])
	plt.plot(num_circs, speedup)
	plt.xlabel("number of circles [2^x]")
	plt.ylabel("speed up factor")
	plt.title("GPU vs CPU speed up vs number of circles")

	plt.plot(np.arange(9,19), np.zeros(10) + 1, '--')
	plt.legend(['speed up', '1.0x speed up'])
	# plt.xticks()

	plt.show()
if __name__ == '__main__':
	main()