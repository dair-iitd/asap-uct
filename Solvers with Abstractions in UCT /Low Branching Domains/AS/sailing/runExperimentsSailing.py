import sys, os

#Miscellaneous


#AAMAS parameters
#batches=[2, 4, 8, 16]

#UCT 
#width=[[97,191,270,352,525,720,875],[95,186,263,340,505,700,850],[93,182,254,325,490,675,805],[90,175,250,325,475,635,775],[88,165,238,308,458,610,735]]
width=[100,200,300,400,600,800]

l=[2,2,3,3,2,6]



for j in range(6):
	print(sys.argv[1]+"/sailing  -f -a 0 -h 0 -s 0 -t 1000 100 random uct "+str(width[j])+" 50 "+str(l[j]))
	os.system(sys.argv[1]+"/sailing  -f -a 0 -h 0 -s 0 -t 1000 100 random uct "+str(width[j])+" 50 "+str(l[j])+" >> " +sys.argv[2])
