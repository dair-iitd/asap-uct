# asap-uct
This repository contains all source files corresponding to a novel MDP Planner - which combines abstractions/symmteries
and UCT framework. 


"How to Compile and Run":

Compile and Run : 

1. In respective domain directory: make

2. ./domain-name -f -a 0 -h 0 -s 0 -t domain-szie random uct l w time
 
    where 

        l : number of times abstractions should be applied
        
        w : plannign horizon

        time: total time of trial
        
    The other parameters are kept static and as per MDP engine, can be looked in Detail     
   
   e.g run:
   
      ./sailing  -f -a 0 -h 0 -s 0 -t 1000 100 random uct 2 30 50 



Papers Based on this work :

1. Ankit Anand, Aditya Grover, Mausam, Parag Singla:
ASAP-UCT: Abstraction of State-Action Pairs in UCT. IJCAI 2015: 1509-1515
2. Ankit Anand, Aditya Grover, Mausam, Parag Singla:
A Novel Abstraction Framework for Online Planning: Extended Abstract. AAMAS 2015: 1901-1902



.........................................................................................................


The repository is organized in the following way: The base directory consists of follwign directories: 
AS (Abstraction of States Framework)
ASAM (Abstraction of States with Action Mapping Framework)
ASAP (Abstraction of States-Action pairs)
where each directory contains :
engine: this includes basic MDP engine and is further organized in various directories.
various domains: Sailing, Navigation and Sysadmin
Each of these domains can be written separately and independently and "domain-name.h" contains the dynamics of that domain.

..........................................................................................................


Most of abstraction code is written in a single files with in engine:
"uct.h" (Location : engine/uct.h)
though some modifications are also made to policy.h to adapt to changes in abstractions.


........................................................................................................

The base code is adapted from MDP Engine of Blai Bonet (http://code.google.com/p/mdp-engine/) . 
