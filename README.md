
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
- Also follow the instructions of the search repo on how to set up the docker. 


## Seach Results:
- evaluation on the [2020a Version of Commonroad](https://gitlab.lrz.de/tum-cps/commonroad-scenarios) (2076 scenarios total)
- evaluation with BMW320i, KS2, SM1 and a [default motion primitves set] (https://gitlab.lrz.de/tum-cps/commonroad-search/-/tree/master/SMP/maneuver_automaton/primitives) ```V_0.0_20.0_Vstep_2.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i```
- Batch_processing: timelimit set to 100 seconds, 3 Threads (Intel i5, 8gb RAM)

|Property  |    number of scenarios|
| ------------- |:-------------:|
|Total number of scenarios:  	  |      2076|
|Solution found:               	|      1920(~92.5%)|
|Solution found but invalid:   	|        12|
|Solution not found:           	|        60|
|Exception occurred:            |         2|
|Time out:                     	|        82|


## Summary of the Motion Planner 

For the [USA_US101-21_1_T-1](https://commonroad.in.tum.de/submissions/ranking/KS2:SM1:USA_US101-21_1_T-1:2020a) scanario, we extract the *relevant* part (from start of the ego vehicle to goal) of the reference path. Since the interval of time_steps of arrival is 70-80, it is a good assumption the relevant reference path can  be completed in ~75 steps. Given the closest position towards the reference path and time_step at any giiven time of the ego vehicle, an estimate of the average speed needed and the progess of the reference path that should be achieved at this time_step, can be returned. Also distance and orientation towards the reference path help to guide the low level search. 

Extracted part of reference_route from start to finish of the USA_US101-21_1_T-1 Scenario:

![Image of the USA_US101 Planned Route.](/png/USA_US101-21_1_T-1_route.png "USA_US101-21_1_T-1route")
Now we "only" need to guide the search along the reference path in time and in space. This how the solution looks in Action. 

![USA_US101 GIF](/png/USA_US101-21_1_T-1demo.gif "USA_US101-21_1_T-1demo.gif")


(until 18.Jan 2020, 3 out of ~300 users found a solution for this [scenario](https://commonroad.in.tum.de/submissions/ranking/KS2:SM1:USA_US101-21_1_T-1:2020a) )


## A selection of some generated solutions:

USA_US101-2_1_T-1:

![](/solution_gifs/DEU_Flensburg-74_1_T-1-2020a.gif  " DEU_Flensburg-74_1_T-1-2020a.gif")

USA_Lanker-2_10_T-1:

![](/solution_gifs/DEU_Flensburg-86_1_T-1-2020a.gif  " DEU_Flensburg-86_1_T-1-2020a.gif")

KS2-SM1-ZAM_Zip-1_16_T-1-2020a

![](/solution_gifs/EU_Flensburg-94_1_T-1-2020a.gif  " EU_Flensburg-94_1_T-1-2020a.gif")



##
- For collaborations, ideas or questions please feel free to contact me or create an issue. (contact me on my [github](https://github.com/ma-abdellaoui) )
