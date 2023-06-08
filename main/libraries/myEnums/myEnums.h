enum commandType { //what the pi wants it to do
    LT = 1,
    EVAC = 2
};

enum currType { //what the pico needs to do
    //^ Linetrack
    LINETRACK = 1,
    LEFTGREEN = 2,
    RIGHTGREEN = 3,
    DOUBLEGREEN = 4,
    RED = 5,
    //^ Forced stuff
    STOP = 101,
    MOVEDIST = 102,
    TURNANGLE = 103
};