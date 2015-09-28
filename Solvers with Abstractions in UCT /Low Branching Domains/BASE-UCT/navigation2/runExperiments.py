import sys, os

#Miscellaneous
dimension=[25, 50, 75, 100]

#AAMAS parameters
#batches=[2, 4, 8, 16]

#UCT 
width=[50, 100, 150, 200]
par=0
trials=500
horizonScale=[0.25, 0.5, 1, 2]

for dim in dimension:
	for wid in width:
		for scale in horizonScale:
			horizon=dim*scale
			#print(sys.argv[1]+"/sailing -f -a 0 -h 1 -s 0 -t "+ str(trials) +" "+str(dim)+" random uct "+str(wid)+" "+ str(horizon)+" "+str(par))
			os.system(sys.argv[1]+"/sailing -f -a 0 -h 1 -s 0 -t "+ str(trials) +" "+str(dim)+" random uct "+str(wid)+" "+ str(horizon)+" "+str(par))
