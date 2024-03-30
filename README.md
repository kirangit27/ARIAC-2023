
# Overview

<p align="justify">
The ARIAC competition, hosted by NIST, annually challenges robots to perform diverse tasks in a dynamic warehouse environment. Agility here encompasses adaptability, efficiency, and autonomy, crucial for real-world manufacturing. Insights from the competition inform the development of standard metrics and test methods for robotic agility in manufacturing.
</p>

The competition consists of two main actors, the CCS and the AM. The competition is set to different states while running, and the CCS needs to subscribe to the topic ```/ariac/competition_state``` to implement the programming logic properly.

<div style="text-align: justify;">
  <ul style="list-style-type: none;">
    <li>Competitor Control System (CCS): Competitors' software responsible for communication with the competition environment and task execution.</li>
    <li>ARIAC Manager (AM): The interface for competitors, providing simulation environment and ROS interfaces.</li>
  </ul>
</div>

Further detailed information about the competition overview can be found [here](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/competition/overview.html)


# Installation
For installing ARIAC please visit [ARIAC Installation](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/getting_started/installation.html) page.

Note: ARIAC 2023 is built for ROS2 Galactic running on Ubuntu 20.04 (Focal Fossa).

# Environment
![Environment](RWA4/imgs/environment_overview.png)
 - Parts - There are four unique part types and each part can be one of five colors.
![Parts](RWA4/imgs/parts.png)
An in detail information about the ARIAC environment can be found [here](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/competition/environment.html)

# Tasks 
## Kitting Task
In a kitting task, the Competitor Control System (CCS) is required to:
- Position a kit tray onto one of the four AGVs.
- Place parts onto the kit tray within a designated quadrant.
- Guide the AGV to the warehouse.
- Evaluate the submitted kit for scoring.
  
  ![Fig.1 kitting task](RWA4/imgs/kitting.gif)

## Assembly Task
Assembly is a manufacturing process where interchangeable parts are sequentially added to a product to create the final product.
- Lock the AGV trays.
- Move the AGVs to the correct assembly station.
- Call the pre-assembly poses service.
- Assemble the parts into an insert.
- Submit the assembly for scoring.
  
   ![Fig.2 assembly task](RWA4/imgs/assembly_8x.gif)
  
## Combined Task
In a combined task, the Competitor Control System (CCS) is required to execute a kitting task followed by an assembly task.

# Agility Challenge
A scenario or set of scenarios in ARIAC is designed to test robotic adaptability and efficiency in dynamic manufacturing environments. There are 8 possible agility challenges in ARIAC 2023. Besides the Human challenge, all other challenges can occur multiple times in a trial.

## High-priority Orders
The high-priority orders challenge simulates an order that takes precedence over regular-priority orders. The CCS must complete and submit the high-priority order before processing regular-priority orders. This challenge assesses the CCS's capability to prioritize high-priority orders, requiring it to detect the announcement of a high-priority order and switch tasks accordingly.

## Insufficient Parts
The insufficient parts challenge replicates a scenario where the workcell lacks the necessary parts to fulfill one or more orders.
This challenge aims to evaluate the CCS's ability to recognize insufficient parts for fulfilling orders. During this challenge, the CCS must submit orders even if they are incomplete due to the shortage of parts.

## Conveyor Belt
The challenge evaluates the CCS's capability to identify required parts from the conveyor belt through the topic ```/ariac/conveyor_parts```. It assesses the CCS's ability to utilize suitable robots to retrieve parts from the belt and store them appropriately.

![Fig.3 Conveyor task](RWA4/imgs/conveyor_pick.gif)


## Faulty Parts
Faulty parts are components that are deemed unsuitable for competition use due to their poor condition. The challenge aims to assess the CCS's capability to: 
- Effectively utilize the quality check sensor for identifying faulty parts.
- Substitute faulty parts with new ones.

![Fig.4 faulty task](RWA4/imgs/faulty.gif)

## Flipped Parts
The challenge aims to assess the CCS's method for flipping parts when the environment starts with parts in an upside-down orientation. This evaluation focuses on the CCS's strategy for correcting the orientation of flipped parts to ensure they are properly aligned for use in orders.

![Fig.5 flipped task](RWA4/imgs/flipped.gif)

Further detailed information about these and the remaining agility challenges can be found [here](https://pages.nist.gov/ARIAC_docs/en/2023.5.0/competition/challenges.html#agility-challenges)

# Assignments

The coursework comprised four RWAs, each building upon the previous one by incorporating additional features. This sequential progression aimed to culminate in a CCS package capable of handling all tasks and agility challenges by the end of the fourth RWA.

## RWA1
RWA1 mainly deals with retrieving Orders, representing them using OOPs and storing them in a data structure. Assignment description and implementation can be found in the [RWA1](https://github.com/kirangit27/ARIAC-2023/tree/RWA1?tab=readme-ov-file) branch. 

## RWA2
RWA2 as mentioned, is built over RWA1 and primarily deals with Task-level planning consisting of high-level actions to complete tasks. These actions will
eventually be translated to low-level commands (motion planning). Assignment description and implementation can be found in the [RWA2](https://github.com/kirangit27/ARIAC-2023/tree/RWA2?tab=readme-ov-file) branch. 

## RWA3
In RWA3, the environment becomes dynamic as we configure sensors and cameras, and utilize MoveIt for pick-and-place operations with the floor robot. This assignment involves creating a kitting task order and includes the conveyor part pick agility challenge. Detailed instructions and implementation can be found in [RWA3](https://github.com/kirangit27/ARIAC-2023/tree/RWA3?tab=readme-ov-file) branch. 

![Fig.1 RWA 3 complete run gif](RWA4/imgs/rwa3_8x.gif)

## RWA4
In RWA4, the objective is to present a fully functional package capable of handling all tasks and agility challenges. This assignment's configuration file includes a combined task and a kitting task (high-priority). Additionally, it incorporates the High-priority Order and Faulty Part Challenge for the Agility challenges. Detailed instructions and implementation can be found in [RWA4](https://github.com/kirangit27/ARIAC-2023/tree/RWA4?tab=readme-ov-file) branch. 

![Fig.1 RWA 4 complete run gif](RWA4/imgs/rwa4_16x.gif)


### Contributors
- [Aniruddh Balram](https://github.com/aniruddhbalram97)
- [Badrinarayanan](https://github.com/Badri-R-S)
- [Kiran Patil](https://github.com/kirangit27)
- [Vyshnav Achuthan](https://github.com/Achuthankrishna)


