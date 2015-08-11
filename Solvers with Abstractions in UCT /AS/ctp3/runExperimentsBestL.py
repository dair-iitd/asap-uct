import sys, os

#Miscellaneous


#AAMAS parameters
#batches=[2, 4, 8, 16]

#UCT 
#width=[[97,191,270,352,525,720,875],[95,186,263,340,505,700,850],[93,182,254,325,490,675,805],[90,175,250,325,475,635,775],[88,165,238,308,458,610,735]]
width=[50,100,200,300,400,600,800]

l=[5,3,6,2,2,2,5]



for j in range(7):
	print(sys.argv[1]+"/ctp3 -d 0 -f -a 0 -h 0 -t 1000 AAAI-graphs/test06_50_T.graph random uct "+str(width[j])+" 25 "+str(l[j]))
	os.system(sys.argv[1]+"/ctp3 -d 0 -f -a 0 -h 0 -t 1000 AAAI-graphs/test06_50_T.graph random uct "+str(width[j])+" 25 "+str(l[j])+" >> " +sys.argv[2])
