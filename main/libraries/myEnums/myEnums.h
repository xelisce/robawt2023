enum currType { //what the pico needs to do
    //^ Linetrack
    TCS_LINETRACK = -1,
    EMPTY_LINETRACK = 0,
    LEFT_GREEN = 1,
    RIGHT_GREEN = 2,
    DOUBLE_GREEN = 3,
    RED = 4,
    ALIGN_SWEEP = 10,
    AFTER_ALIGN_SWEEP = 11,
    //^ Forced stuff
    STOP = 100,
    MOVE_DIST = 101,
    TURN_ANGLE = 102
};