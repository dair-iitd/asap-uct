make clean
make
cd ../sailing
make clean
make
timeout 90s ./sailing  -f -a 0 -h 1 -s 0 -t 1000 100 random uct 300 50 0
cd ../engine
