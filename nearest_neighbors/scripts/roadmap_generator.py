import random,sys

num_nodes = 200
dimension = 8

min_bound = [-2.6, -3.0, -1.8, -2.8, -2.2, -3.0, -1.8, -3.0, 0  ]
max_bound = [ 2.6,  3.0,  1.8,  2.8,  2.2,  3.0,  1.8,  3.0, 1.4]

random.seed(12345)

f = open("n"+str(num_nodes)+".roadmap",'w')

f.write(str(num_nodes))

for x in xrange(num_nodes):

	f.write("\n"+str(x)+"\n")
	for i in xrange(len(min_bound)-1):
		f.write(str(random.uniform(min_bound[i],max_bound[i]))+",")
	f.write(str(random.uniform(min_bound[len(min_bound)-1],max_bound[len(min_bound)-1])))
f.close()