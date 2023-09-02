# RWA2

## Tasks 
- Starting the Competition (RWA1).
- Retrieving Orders (RWA1).
- Locate parts required in the orders.
- Identify challenges.
- Call functions to do Kitting/Assembly.
- Submitting Orders (RWA1).
- Ending the Competition (RWA1).

[RWA2](RWA2/RWA2_ENPM663_SPRING2023.pdf)

## Execution

1. Launch the ARIAC environment with rwa2 trail file.

    ```
       $ ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa2
    ```

    Note: Initially, [rwa2.yaml](RWA2/rwa2.yaml) needs to be placed inside the config/trials folder present in the ariac_gazebo package (as shown in the package structure below)

    ![Fig. 1 - Trial File Location](RWA2/imgs/pkg_struct.png)


3. Launch CCS

    ```
       $ ros2 launch ARIAC-2023 ccs.launch.py
    ```

## Example Output
ARIAC server terminal
![Fig. 2 - ariac server terminal](RWA1/imgs/sample_output1.png)

CCS terminal
![Fig. 3 - CCS terminal](RWA1/imgs/sample_output2.png)

