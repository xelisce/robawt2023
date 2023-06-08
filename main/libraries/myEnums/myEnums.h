enum commandType { //what the pi wants it to do
    LT,
    EVAC
};

enum currType { //what the pico needs to do
    //^ Linetrack
    LINETRACK,
    LEFTGREEN,
    RIGHTGREEN,
    DOUBLEGREEN,
    RED,
    //^ Forced stuff
    STOP,
    MOVEDIST,
    TURNANGLE
};