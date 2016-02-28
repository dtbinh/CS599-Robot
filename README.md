# CS599-Robot-P3

### How to compile?
```
make all
```

### How to start the Player/Stage?
```
player player/multi.cfg
```

### How to run the robots?
```
./run_disperse.sh (for dispersion)
./run_aggregate.sh (for aggregation)
```
Note: robots run as background processes

### How to stop the robots?
```
./stop.sh
```
Note: robots are listening to broadcasting messages, once hear a STOP message they will exit

