make clean
make
cd ../ctp3
make clean
make
./ctp3 -d 0 -f -a 0 -h 1 AAAI-graphs/test00_20_T.graph random uct 10 20 0
cd ../engine