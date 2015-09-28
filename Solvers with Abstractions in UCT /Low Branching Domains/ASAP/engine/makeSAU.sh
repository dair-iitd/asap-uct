make clean
make
cd ../sailing
make clean
make
timeout 10000000 ./sailing -f -a 0 -h 1 -s 0 -t $1 100 random uct $2 50 $3
cd ../engine
