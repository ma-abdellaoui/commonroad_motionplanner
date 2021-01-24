
# commonroad_motionplanner
A Motion Planner to solver driving scenarios for the [Commonroad](https://commonroad.in.tum.de/)  framework  

Instructions: 
- this code provides a motion planner as in  [commonroad search repository](https://gitlab.lrz.de/tum-cps/commonroad-search/) 
- to duplicate the results of the motion planner locally, pull the [commonroad_search repo](https://gitlab.lrz.de/tum-cps/commonroad-search/) and replace the  following files:
SMP/motion_planner/search_algorithms/best_first_search.py  
SMP/motion_planner/search_algorithms/base_class.py  
SMP/motion_planner/search_algorithms/student.py  
/path/to/your/anaconda3/envs/commonroad-py37/lib/python3.7/site-packages/commonroad/planning/goal.py  
.  
Furthermore, the following files needs to be added to the /SMP/motion_planner/search_algorithms/ folder:  
SMP/motion_planner/search_algorithms/DisableObstAvoidance.txt  
SMP/motion_planner/search_algorithms/DisableRefPath.txt  
SMP/motion_planner/search_algorithms/EnableSpeedLimit.txt  
These files contain a list of scenarios where the motion planner needs to enable/disable certain features such as the speed limiter in corners or the front obstacle avoidance to be able to find a solution  
- Also follow the instructions of the search repo on how to set up the environment. 


## Search Results:
- evaluation on the [2020a Version of Commonroad](https://gitlab.lrz.de/tum-cps/commonroad-scenarios) (2076 scenarios total)
- evaluation with BMW320i, KS2, SM1 and a [default motion primitves set] (https://gitlab.lrz.de/tum-cps/commonroad-search/-/tree/master/SMP/maneuver_automaton/primitives) ```V_0.0_20.0_Vstep_2.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i```
- Batch_processing: time limit set to 100 seconds, 3 Threads (Intel i5, 8gb RAM)

|Property  |    number of scenarios|
| ------------- |:-------------:|
|Total number of scenarios:  	  |      2076|
|Solution found:               	|      1920(~92.5%)|
|Solution found but invalid:   	|        12|
|Solution not found:           	|        60|
|Exception occurred:            |         2|
|Time out:                     	|        82|


## Summary of the Motion Planner 

This is a general summary on how the motion planner works. For further details, check out the comments in the source code. 
There are 4 types of scenarios available. The planner decides which scenarios type is currently loaded and uses a different set of functions depending on the scenario. 
The motion planner can decide on the type of the scenario depending on the attributes of the desired goal. These can be one of the following:
['time_step']
['position', 'time_step']
['position', 'velocity', 'time_step']
['position', 'orientation', 'time_step']
['position', 'velocity', 'orientation', 'time_step']

After the type of the scenario is decided, the motion planner calls the route_planner class to create a reference path to the goal position. The reference path will work as a guide to the search process. Along the reference path, all motion primitives that end at a point further than a certain distance from the reference path will not be considered. the accelerates and guides the search directly to the goal position. The problem with this strategy is static objects. if there is a non moving objects along the reference path then the motion planner will not be able to avoid it and drive pass it because if it constrained by the maximum allowed distance to the reference path. I will be addressing this problem in the next weeks, the idea is to relax the constraint of the maximum distance, if a static object is detected in-front. 
As for the heuristic function of the search algorithm, please refer the source code, as everything is nicely separated in functions and commented.


## A selection of some generated solutions:

-   **Green circle**: initial state projected onto the position domain
-   **Red rectangle**: static obstacle
-   **Blue rectangle**: dynamic obstacle
-   **green rectangle**: ego vehicle --> the vehicle controlled by the motion planner
-   **Yellow rectangle**: goal region projected onto the position domain


USA_US101-2_1_T-1:
![](/solution_gifs/DEU_Flensburg-74_1_T-1-2020a.gif  " DEU_Flensburg-74_1_T-1-2020a.gif")

USA_Lanker-2_10_T-1:

![](/solution_gifs/DEU_Flensburg-86_1_T-1-2020a.gif  " DEU_Flensburg-86_1_T-1-2020a.gif")

KS2-SM1-ZAM_Zip-1_16_T-1-2020a

![](/solution_gifs/EU_Flensburg-94_1_T-1-2020a.gif  " EU_Flensburg-94_1_T-1-2020a.gif")



##
- For collaborations, ideas or questions please feel free to contact me or create an issue. (contact me on my [github](https://github.com/ma-abdellaoui) )
