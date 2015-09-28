make clean
make
cd ../sailing
make clean
make
#./sailing -a 16 -f -h 0 -s 0 -t 1 15 random uct 80 5 0
./sailing  -f -a 0 -h 1 -s 0 -t $1 100 random uct $2 50 $3
cd ../engine
