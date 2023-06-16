enum currType { //what the pico needs to do
    //^ Linetrack
    TCS_LINETRACK,
    EMPTY_LINETRACK,
    LEFT_GREEN,
    RIGHT_GREEN,
    DOUBLE_GREEN,
    RED,
    ALIGN_SWEEP,
    AFTER_ALIGN_SWEEP,
    //^ Forced stuff
    STOP,
    MOVE_DIST,
    TURN_ANGLE
};