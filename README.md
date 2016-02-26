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
sh run_disperse.sh (for dispersion)
sh run_aggregate.sh (for aggregation)
```
Note: robots run as background processes

### How to stop the robots?
```
sh stop.sh
```
Note: robots are listening to broadcasting messages, once hear a STOP message they will exit