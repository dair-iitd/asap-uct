make clean
make
cd ../sailing
make clean
make
./sailing  -f -a 0 -h 0 -s 0 -t $1 100 random uct $2 50 0
cd ../engine
