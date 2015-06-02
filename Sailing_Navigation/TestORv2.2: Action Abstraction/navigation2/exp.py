import sys, os

#Miscellaneous


#AAMAS parameters
#batches=[2, 4, 8, 16]

#UCT 
#width=[[97,191,270,352,525,720,875],[95,186,263,340,505,700,850],[93,182,254,325,490,675,805],[90,175,250,325,475,635,775],[88,165,238,308,458,610,735]]
#width=[[88,178,240,311,445,575,705],[91,166,225,290,420,530,655],[88,156,206,265,375,482,565],[83,143,195,263,350,445,560],[83,140,190,245,342,435,505]]
time=0












for j in range(7):
	time=time+50
	print(sys.argv[1]+"/navigation  -f -a 0 -h 0 -s 0 -t 1000 100 random uct 2 50 "+str(time))
	os.system(sys.argv[1]+"/navigation  -f -a 0 -h 0 -s 0 -t 1000 100 random uct 2 50 "+str(time)+" >> " +sys.argv[2])
