import sys
import os
fname = sys.argv[1]
with open(fname, 'r') as f:
	gg = f.readlines()
head = gg[0]
tail = gg[-1]
h = head.split(",")
t = tail.split(",")
for i in range(len(h)):
	print("%-50s %-30s"%(h[i],t[i]))