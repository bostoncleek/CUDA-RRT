import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import pandas as pd
import numpy as np


def plot(obstacles, graph, path):
	fig, ax = plt.subplots()
	#
	# loc = plticker.MultipleLocator(base=1)
	# ax.xaxis.set_major_locator(loc)
	# ax.yaxis.set_major_locator(loc)
	# ax.grid(which='major', axis='both')
	# plt.grid(True, color='blue', linestyle='--')

	plt.plot()
	plt.xlim((0,100))
	plt.ylim((0,100))

	for i in range(obstacles.shape[0]):
		circ = obstacles.iloc[i, :].to_numpy()
		circle = plt.Circle((circ[0],circ[1]), circ[2], color='k')
		ax.add_artist(circle)

	for i in range(graph.shape[0]):
		edge = graph.iloc[i, :].to_numpy()
		if (edge[-1] == -1): #root case
			plt.scatter(edge[0], edge[1], color='yellow', marker='X')
		elif (edge[-1] == 1): #goal case
			plt.scatter(edge[0], edge[1], color='red', marker='P')
		else:
			plt.plot([edge[0], edge[2]],[edge[1], edge[3]], '-ob')

	for i in range(path.shape[0]):
		edge = path.iloc[i, :].to_numpy()
		plt.plot([edge[0], edge[2]],[edge[1], edge[3]], '-or')


	plt.title("CUDA RRT 8192 Obstacles")


def main():
	obstacles = pd.read_csv("rrtout/obstacles.csv", header = None, index_col = False)
	# print(obstacles)
	graph = pd.read_csv("rrtout/graph.csv", header = None, index_col = False)
	# print(graph)
	path = pd.read_csv("rrtout/path.csv", header = None, index_col = False)
	plot(obstacles, graph,path)
	# print(obstacles.shape[0])
	# graph = pd.read_csv("rrtout/graph.csv")


if __name__ == '__main__':
	main()
	plt.show()
