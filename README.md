# RWA4

## Assignment Description
This assignment consists of completing a combined task and a kitting task.
- The Order with the combined task is announced at the beginning of the competition. For the kitting part of the combined task, use the tray with id 0.
- The Order with the kitting task is announced 50s after the competition has started. This second Order is a high-priority Order which should be handled as soon as it is announced.

An in detail Assignment description can be found in
[RWA4](RWA4/RWA4_ENPM663_SPRING2023.pdf)

## Challenges
- High-priority Order: A kitting Order, which is of high-priority, is announced 50 s after the competition has started. For this assignment, you will use the ﬂoor robot to perform kitting (including kitting during a combined task) and the ceiling robot to perform assembly. The high-priority Order must be started as soon as it is announced. If the robot is holding a part when the new Order is announced, the competitor control system (CCS) should place the part before starting working on the new Order. Once the high-priority Order is completed, the CCS should resume the other Order. There is a need to keep track of the steps within an Order so it can be resumed.
- Faulty Part Challenge: The ﬁrst part placed in quadrant on AGV 4 is faulty. Once a part is detected as faulty, it must be removed and replaced with another one. Make sure you discard the faulty part in one of the blue bins.

## Execution

1. Launch the ARIAC environment with the rwa2 trail file.

    ```
       $ ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4 competitor_pkg:=ARIAC-2023 sensor_config:=sensors
    ```

    Note: Initially, [rwa4.yaml](RWA4/rwa4.yaml) needs to be placed inside the config/trials folder present in the ariac_gazebo package (as done in previous assignments)

2. Launch the move_group node

    ```
       $ ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
    ```

3. Launch CCS

    ```
       $ ros2 launch ARIAC-2023 ccs.launch.py
    ```

## Example Output

RWA 4 run (16x)

![Fig.1 RWA 4 complete run gif](RWA4/imgs/rwa4_16x.gif)


