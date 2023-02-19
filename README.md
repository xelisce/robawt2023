# robawt2023

For robocupz 2023

To Note:
- PID for motors has not been tuned perfectly, robot stays jerky at constant speed. Solution: Use PI controller tuning to properly tune
- PID for linetrack not tuned properly. Solution: None
- Lidar library is bugging out, i suspect this is due to the addresses and pointers being super messy and confusing and i dont understand it for now. Solution for time being: dont use library lolz

Checklist of things yet to be done on software side:
- Ball tracking
- Green squares
- Stop at red line (should be fairly simple)
- 135 turn
- Entire evac portion