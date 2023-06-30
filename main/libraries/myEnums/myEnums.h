enum currType { //what the pico needs to do
    //^ Linetrack
    TCS_LINETRACK = -1,
    EMPTY_LINETRACK = 0,
    LEFT_GREEN = 1,
    RIGHT_GREEN = 2,
    DOUBLE_GREEN = 3,
    RED = 4,
    BEFORE_BLUE_TURN = 7,
    BLUE = 8,
    LINEGAP = 12,
    BLUE_PICKUP = 26,
    BEFORE_OBSTACLE_REVERSE = 30,
    BEFORE_OBSTACLE_TURN = 31,
    OBSTACLE = 32,
    //^ Forced stuff
    STOP = 100,
    MOVE_DIST = 101,
    TURN_ANGLE = 102,
    TURN_TIME = 103,
    //^ POST SHIT
    POST_DOUBLE_GREEN = 200,
    AFTER_BLUE_INIT = 23,
    AFTER_BLUE_REVERSE = 24,
    AFTER_BLUE_TURN = 25,
    AFTER_OBSTACLE_TURN = 33,
};