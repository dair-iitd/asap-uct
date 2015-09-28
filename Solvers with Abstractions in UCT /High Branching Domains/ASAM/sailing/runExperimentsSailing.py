import sys, os

#Miscellaneous


#AAMAS parameters
#batches=[2, 4, 8, 16]

#UCT 
#width=[[97,191,270,352,525,720,875],[95,186,263,340,505,700,850],[93,182,254,325,490,675,805],[90,175,250,325,475,635,775],[88,165,238,308,458,610,735]]
width=[100,200,300,400,600,800]
time=[0.001,0.002,0.003,0.004,0.006,0.009]
l=[3,2,6,3,4,6]



for j in range(6):
	print(sys.argv[1]+"/sailing  -f -a 0 -h 0 -s 0 -t 1000 100 random uct "+str(l[j])+" 50 "+str(time[j]))
	os.system(sys.argv[1]+"/sailing  -f -a 0 -h 0 -s 0 -t 1000 100 random uct "+str(l[j])+" 50 "+str(time[j])+" >> " +sys.argv[2])
