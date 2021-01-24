from SMP.motion_planner.node import PriorityNode
import numpy as np
from heapq import nsmallest
import sys
from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.search_algorithms.best_first_search import GreedyBestFirstSearch
# imports for route planner:

class StudentMotionPlanner(GreedyBestFirstSearch):
    """
    Motion planner implementation by students.
    Note that you may inherit from any given motion planner as you wish, or come up with your own planner.
    Here as an example, the planner is inherited from the GreedyBestFirstSearch planner.
    """

    def __init__(self, scenario, planningProblem, automata, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automata,
                         plot_config=plot_config)
        #self.routeplannerresult = self.create_RoutePlanner_path()




    def evaluation_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own evaluation function here.                   #
        ########################################################################
        # Copied from greedy best first search:
        """
        Evaluation function of GBFS is f(n) = h(n)
        """

        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority


    def heuristic_function(self, node_current: PriorityNode) -> float:
        ########################################################################
        # todo: Implement your own heuristic cost calculation here.            #
        # Hint:                                                                #
        #   Use the State of the current node and the information from the     #
        #   planning problem, as well as from the scenario.                    #
        #   Some helper functions for your convenience can be found in         #
        #   ./search_algorithms/base_class.py                             #
        ########################################################################
        """
        Function that evaluates the heuristic cost h(n) in student class.
        Created by Mohamed A. Abdellaoui 10.01.2021
        @return:
        """
        output_logs = False
        if output_logs:
            print("##################")
            print("current time step: ", node_current.list_paths[-1][-1].time_step)
            print("current problem mode", self.planningProblemType)
            print("depth tree: ", node_current.depth_tree)
        currentorient = node_current.list_paths[-1][-1].orientation
        currentpos = node_current.list_paths[-1][-1].position
        currenttimestep = node_current.list_paths[-1][-1].time_step
        currentVel = node_current.list_paths[-1][-1].velocity

        ########################## Test if car infront has velocity 0:
        if self.reached_goal(node_current.list_paths[-1]):
            return 0.0

        if self.routeplannerresult is None:
            return np.inf

        ############ Detect cars in front:
        # calc cost based on distance to gool folliwng the refrence path:
        # loop through all obstacles at time step x and find if any is close of current pos:
        if not self.disableObstAvoidance:
            for obst in self.list_obstacles:
                obstPos = obst.state_at_time(currenttimestep)
                if currentorient is not None and obstPos is not None:
                    disttoobst = self.euclidean_distance(currentpos, obstPos.position)
                    lookaheadVar = 1.375 * currentVel + 2.5
                    if disttoobst <= lookaheadVar:
                        # calc orientation diff between car and obstacle:
                        vectorToObst = np.array([currentpos, obstPos.position])
                        vectorToObstOrient = self.calc_angle_of_position(vectorToObst, currentpos)
                        orientdiff = self.calc_orientation_diff(currentorient, vectorToObstOrient)
                        if abs(orientdiff) <= 0.261799:
                            if not 'velocity' in obstPos.attributes:
                                continue
                            if node_current.list_paths[-1][-1].velocity > obstPos.velocity and obstPos.velocity != 0:
                                return np.inf
            index_smallest_dist = self.get_index_nearest_obst_infront(node_current)
            if index_smallest_dist != -1:
                # found the index of vehicle with smallest distance to ego car:
                obst = self.list_obstacles[index_smallest_dist]
                obstPos = obst.state_at_time(currenttimestep)
                if obstPos is not None and 'velocity' in obstPos.attributes:
                    if obstPos.velocity == 0:
                        cost = node_current.list_paths[-1][-1].velocity
                        return cost
                    if node_current.list_paths[-1][-1].velocity > obstPos.velocity:
                        return np.inf
                    cost = abs(node_current.list_paths[-1][-1].velocity - obstPos.velocity)
                    return cost
        #########################################################

        # Decide based on planning problem type how to calculate cost
        if self.planningProblemType == 'ModeA':
            # Call function for planning problem with desired time, position, speed and orientation
            cost = self.cost_for_modeA_problem(node_current, output_logs)
            if output_logs:
                print("Cost from modeA cost func: ", cost)
            if cost < 0:
                return 0
            return cost
        elif self.planningProblemType == 'ModeB':
            # Call function for planning problem with desired time and position:
            cost = self.cost_for_modeB_problem(node_current, output_logs)
            if output_logs:
                print("Cost from modeB cost func: ", cost)
            if cost < 0:
                return 0
            return cost
        elif self.planningProblemType == 'ModeC':
            # Call function for planning problem with desired time and position:
            cost = self.cost_for_modeC_problem(node_current, output_logs)
            if output_logs:
                print("Cost from modeB cost func: ", cost)
            if cost < 0:
                return 0
            return cost
        elif self.planningProblemType == 'ModeD':
            # Call function for planning problem with desired time and position:
            cost = self.cost_for_modeD_problem(node_current, output_logs)
            if output_logs:
                print("Cost from modeB cost func: ", cost)
            if cost < 0:
                return 0
            return cost
        elif self.planningProblemType == 'Survival':
            # Call function for planning problem with desired time and position:
            cost = self.cost_for_Survival_problem(node_current, output_logs)
            if output_logs:
                print("Cost from modeB cost func: ", cost)
            if cost < 0:
                return 0
            return cost

    def cost_for_modeA_problem(self, node_current, output_logs):
        # Function for planning problem with desired time, position, speed and orientation
        if self.position_desired is None:
            if output_logs:
                print("exit Cost function because position desired is None!")
            return self.time_desired.start - node_current.list_paths[-1][-1].time_step
        else:
            velocity = node_current.list_paths[-1][-1].velocity
            path_last = node_current.list_paths[-1]
            if np.isclose(velocity, 0):
                return np.inf
            else:
                # Calc Variables:
                distance = self.calc_euclidean_distance(current_node=node_current)
                angleToGoal = self.calc_angle_to_goal(path_last[-1])
                orientationToGoalDiff = self.calc_orientation_diff(angleToGoal, path_last[-1].orientation)
                orientationToGoalDiffdegree = (abs(orientationToGoalDiff) * 180) / 3.14
                desired_orient = (self.orientation_desired.end + self.orientation_desired.start) / 2
                desired_velocity = (self.velocity_desired.start + self.velocity_desired.end) / 2
                diff_desiredOrient = abs(self.calc_orientation_diff(desired_orient, path_last[-1].orientation))
                diff_deiredVelocity = abs(velocity - desired_velocity)
                angle_intervall = abs(abs(self.orientation_desired.start) - abs(self.orientation_desired.end))

                # Output data for debugging:
                if output_logs:
                    print("Distance to goal of current node is: ", distance)
                    print("Velocity of current node is: ", velocity)
                    print("Orientation of current position: ", node_current.list_paths[-1][-1].orientation)
                    print("Angle to goal of current node is: ", angleToGoal)
                    print("orientation diff to goal of current node is(deg): ", orientationToGoalDiffdegree)
                    print("diff desired orient of current node is(deg): ", diff_desiredOrient)
                    print("diff desired velocity  of current node is(deg): ", diff_deiredVelocity)
                # test 16.01:
                current_orient = path_last[-1].orientation
                if distance <= 10:
                    if current_orient < self.orientation_desired.start or current_orient > self.orientation_desired.end:
                        return np.inf
                    if velocity < self.velocity_desired.start or velocity > self.velocity_desired.end:
                       return np.inf

                weight = 10
                # if very colse to goal, minimize the diff velocity and diff orient
                cost = (distance / velocity)  + weight* diff_deiredVelocity + weight* diff_desiredOrient
                #cost = distance +  diff_desiredOrient + diff_deiredVelocity
                return cost

    def cost_for_modeB_problem(self, node_current, output_logs):
        # Function for planning problem with desired time, position, speed
        if self.position_desired is None:
            if output_logs:
                print("exit Cost function because position desired is None!")
            return self.time_desired.start - node_current.list_paths[-1][-1].time_step
        else:

            velocity = node_current.list_paths[-1][-1].velocity
            path_last = node_current.list_paths[-1]
            if np.isclose(velocity, 0):
                return np.inf
            else:
                # Calc Variables:
                distance = self.calc_distance_to_goal_from_point(node_current.list_paths[-1][-1])
                angleToGoal = self.calc_angle_to_goal(path_last[-1])
                orientationToGoalDiff = self.calc_orientation_diff(angleToGoal, path_last[-1].orientation)
                orientationToGoalDiffdegree = (abs(orientationToGoalDiff)*180)/3.14
                desired_velocity = (self.velocity_desired.start + self.velocity_desired.end)/2
                diff_deiredVelocity = abs(velocity - desired_velocity)
                self.test_if_in_goal_lanelet(node_current)
                # Output data for debugging:
                if output_logs:
                    print("Distance to goal of current node is: ", distance)
                    print("Velocity of current node is: ", velocity)
                    print("Orientation of current position: ",node_current.list_paths[-1][-1].orientation)
                    print("Angle to goal of current node is: ", angleToGoal)
                    print("orientation diff to goal of current node is(deg): ", orientationToGoalDiffdegree)
                    print("diff desired velocity  of current node is(deg): ", diff_deiredVelocity)

                # If very close to target but time is still not reached:
                #if distance <= 0.1 and node_current.list_paths[-1][-1].time_step < self.time_desired.start:
                #    return self.time_desired.start - node_current.list_paths[-1][-1].time_step * 0.01
                if self.planningProblem.goal.is_reached_only_pos(node_current.list_paths[-1][-1]):
                    cost = (self.time_desired.start - node_current.list_paths[-1][-1].time_step) * 0.01
                    cost = cost + diff_deiredVelocity + velocity *0.01
                    return cost

                cost = ( distance / velocity ) + 2 * diff_deiredVelocity + velocity*0.01
                return  cost

    def cost_for_modeC_problem(self, node_current, output_logs):
        # Function for planning problem with desired time, position, speed and orientation
        if self.position_desired is None:
            if output_logs:
                print("exit Cost function because position desired is None!")
            return self.time_desired.start - node_current.list_paths[-1][-1].time_step
        else:
            velocity = node_current.list_paths[-1][-1].velocity
            path_last = node_current.list_paths[-1]
            if np.isclose(velocity, 0):
                return np.inf
            else:
                # Calc Variables:
                distance = self.calc_euclidean_distance(current_node=node_current)
                angleToGoal = self.calc_angle_to_goal(path_last[-1])
                orientationToGoalDiff = self.calc_orientation_diff(angleToGoal, path_last[-1].orientation)
                orientationToGoalDiffdegree = (abs(orientationToGoalDiff)*180)/3.14
                desired_orient = (self.orientation_desired.end + self.orientation_desired.start) / 2
                diff_desiredOrient = self.calc_orientation_diff(desired_orient, path_last[-1].orientation)
                angle_intervall = abs(abs(self.orientation_desired.start) - abs(self.orientation_desired.end))

                # Calcualte distance between currrent position and reference path:
                arry = node_current.list_paths[-1][-1].position
                a = np.array([arry[0], arry[1]])
                if self.routeplannerresult is not None:
                    distance_to_refrence = self.calc_distance_to_nearest_point(self.routeplannerresult.reference_path,
                                                                             a)
                else:
                    distance_to_refrence = 0

                # Output data for debugging:
                if output_logs:
                    print("distance to reference path: ", distance_to_refrence)
                    print("Distance to goal of current node is: ", distance)
                    print("Velocity of current node is: ", velocity)
                    print("Orientation of current position: ",node_current.list_paths[-1][-1].orientation)
                    print("Angle to goal of current node is: ", angleToGoal)
                    print("orientation diff to goal of current node is(deg): ", orientationToGoalDiffdegree)
                    print("diff desired orient of current node is(deg): ", diff_desiredOrient)

                # If very close to target but time is still not reached:
                if distance <= 0.1 and node_current.list_paths[-1][-1].time_step < self.time_desired.start:
                    return self.time_desired.start - node_current.list_paths[-1][-1].time_step

                if self.planningProblem.goal.is_reached_only_pos(node_current.list_paths[-1][-1]):
                    cost = (self.time_desired.start - node_current.list_paths[-1][-1].time_step) * 0.01
                    cost = cost + diff_desiredOrient + velocity *0.01
                    return cost

                cost = ( distance / velocity ) + 2 * diff_desiredOrient + velocity*0.01
                return cost

    def cost_for_modeD_problem(self, node_current, output_logs):

        totaltogoal = self.calc_distance_to_goal_from_point(node_current.list_paths[-1][-1])
        if self.position_desired is None:
            if output_logs:
                print("exit Cost function because position desired is None!")
            return self.time_desired.start - node_current.list_paths[-1][-1].time_step
        else:
            if self.planningProblem.goal.is_reached_only_pos(node_current.list_paths[-1][-1]):
                return (self.time_desired.start - node_current.list_paths[-1][-1].time_step) *0.01
            velocity = node_current.list_paths[-1][-1].velocity
            if np.isclose(velocity, 0):
                return np.inf
            cost = totaltogoal / node_current.list_paths[-1][-1].velocity
            return cost

    def cost_for_Survival_problem(self, node_current, output_logs):
        currentorient = node_current.list_paths[-1][-1].orientation
        currentpos = node_current.list_paths[-1][-1].position
        currenttimestep = node_current.list_paths[-1][-1].time_step
        currentVel = node_current.list_paths[-1][-1].velocity
        for obst in self.list_obstacles:
            obstPos = obst.state_at_time(currenttimestep)
            if currentorient is not None and obstPos is not None:
                disttoobst = self.euclidean_distance(currentpos, obstPos.position)
                lookaheadVar = 1.375 * currentVel + 2.5
                if disttoobst <= lookaheadVar:
                    # calc orientation diff between car and obstacle:
                    vectorToObst = np.array([currentpos, obstPos.position])
                    vectorToObstOrient = self.calc_angle_of_position(vectorToObst, currentpos)
                    orientdiff = self.calc_orientation_diff(currentorient, vectorToObstOrient)
                    if abs(orientdiff) <= 0.261799:
                        if not 'velocity' in obstPos.attributes:
                            continue
                        if node_current.list_paths[-1][-1].velocity > obstPos.velocity and obstPos.velocity != 0:
                            return np.inf
        return self.time_desired.start - node_current.list_paths[-1][-1].time_step

    def calc_distance_to_ref_from_point(self, state):
        #calc distance of points to each point of refrence path:
        currentpos = state.position
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(currentpos, p))
        smallest_points = nsmallest(2, distances)
        index1 = distances.index(smallest_points[0])
        index2 = distances.index(smallest_points[1])
        p1 = self.refPathParsedPnts[index1]
        p2 = self.refPathParsedPnts[index2]
        distance_to_refrence = np.abs(np.cross(p2 - p1, currentpos - p1) / np.linalg.norm(p2 - p1))
        return distance_to_refrence

    def calc_distance_to_goal_from_point(self, state):
        #calc distance of points to each point of refrence path:
        currentpos = state.position
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(currentpos, p))
        index_smallest_dist = distances.index(min(distances))
        totaltogoal = 0
        for p in range(index_smallest_dist, len(self.refPathParsedPnts) - 1):
            totaltogoal = totaltogoal + self.euclidean_distance(self.refPathParsedPnts[p],self.refPathParsedPnts[p+1])

        return totaltogoal

    def get_index_nearest_obst_infront(self,node_current):
        # loop through all obstacles at time step x and find if any is close of current pos:
        currentorient = node_current.list_paths[-1][-1].orientation
        currentpos = node_current.list_paths[-1][-1].position
        currenttimestep = node_current.list_paths[-1][-1].time_step
        currentVel = node_current.list_paths[-1][-1].velocity
        disttoobst = [np.inf] * len(self.list_obstacles)
        for i in range(len(self.list_obstacles)):
            obst = self.list_obstacles[i]
            obstPos = obst.state_at_time(currenttimestep)
            if currentorient is not None and obstPos is not None:
                dist = self.euclidean_distance(currentpos, obstPos.position)
                lookaheadVar = 1.375 * currentVel + 2.5
                if dist <= lookaheadVar:
                    # calc orientation diff between car and obstacle:
                    vectorToObst = np.array([currentpos, obstPos.position])
                    vectorToObstOrient = self.calc_angle_of_position(vectorToObst, currentpos)
                    orientdiff = self.calc_orientation_diff(currentorient, vectorToObstOrient)
                    if abs(orientdiff) <= 0.261799:
                        disttoobst[i]= dist
                    else:
                        disttoobst[i]= np.inf
                else:
                    disttoobst[i]= np.inf
        index_smallest_dist = disttoobst.index(min(disttoobst))
        if disttoobst[index_smallest_dist] == np.inf:
            index_smallest_dist = -1
        return  index_smallest_dist


    def test_if_in_goal_lanelet(self, node_current):
        pos = [node_current.list_paths[-1][-1].position]
        currentlanelet = self.scenario.lanelet_network.find_lanelet_by_position(pos)
        currentlanelet = currentlanelet[0][0]
        #result = self.is_goal_in_lane(currentlanelet)
        result = False
        if self.planningProblem.goal.lanelets_of_goal_position is not None:
            if currentlanelet in self.planningProblem.goal.lanelets_of_goal_position.get(0):
                result = True
        return result

    def cost_for_modeA_problem_old(self, node_current, output_logs):
        # Function for planning problem with desired time, position, speed and orientation
        if self.position_desired is None:
            if output_logs:
                print("exit Cost function because position desired is None!")
            return self.time_desired.start - node_current.list_paths[-1][-1].time_step
        else:
            velocity = node_current.list_paths[-1][-1].velocity
            path_last = node_current.list_paths[-1]
            if np.isclose(velocity, 0):
                return np.inf
            else:
                # Calc Variables:
                distance = self.calc_euclidean_distance(current_node=node_current)
                angleToGoal = self.calc_angle_to_goal(path_last[-1])
                orientationToGoalDiff = self.calc_orientation_diff(angleToGoal, path_last[-1].orientation)
                orientationToGoalDiffdegree = (abs(orientationToGoalDiff)*180)/3.14
                desired_orient = (self.orientation_desired.end + self.orientation_desired.start) / 2
                desired_velocity = (self.velocity_desired.start + self.velocity_desired.end)/2
                diff_desiredOrient = self.calc_orientation_diff(desired_orient, path_last[-1].orientation)
                diff_deiredVelocity = abs(velocity - desired_velocity)
                angle_intervall = abs(abs(self.orientation_desired.start) - abs(self.orientation_desired.end))

                # Output data for debugging:
                if output_logs:
                    print("Distance to goal of current node is: ", distance)
                    print("Velocity of current node is: ", velocity)
                    print("Orientation of current position: ",node_current.list_paths[-1][-1].orientation)
                    print("Angle to goal of current node is: ", angleToGoal)
                    print("orientation diff to goal of current node is(deg): ", orientationToGoalDiffdegree)
                    print("diff desired orient of current node is(deg): ", diff_desiredOrient)
                    print("diff desired velocity  of current node is(deg): ", diff_deiredVelocity)

                # if very colse to goal, minimize the diff velocity and diff orient
                if distance <= 1:
                    desired_vel_weight = 1
                    desired_orient_weight = 1
                    cost =  desired_vel_weight * diff_deiredVelocity
                    if angle_intervall < 1 and angle_intervall != 0:
                        cost = cost + desired_orient_weight * diff_desiredOrient
                    return cost


                # If very close to target but time is still not reached:
                if distance <= 0.1 and node_current.list_paths[-1][-1].time_step < self.time_desired.start:
                    return (self.time_desired.start - node_current.list_paths[-1][-1].time_step) *0.001

                # check if goal in in field of view:
                if orientationToGoalDiffdegree > 45:
                    # goal is not in field of view:
                    # give more weight to speed and follow reference path blindly:
                    # block to differentiate between large distance to goal and small distance:
                    if distance >= 10:  # too far away from target, just follow the least distance and target lanelet.
                        velocity_weight = 1
                        cost = distance / velocity

                        return cost

                    if distance < 10 and distance >= 5:  # almost close, reduce speed.
                        return np.inf

                    if distance < 5:  # very close andjust orientation angle..
                        return np.inf
                else:
                    # goal is in field of view:
                    # give more weight to distance and speed and orientation goals:
                    # goal is not in field of view:
                    # give more weight to speed and follow reference path blindly:
                    # block to differentiate between large distance to goal and small distance:
                    if distance >= 10:  # too far away from target, just follow the least distance and target lanelet.
                        velocity_weight = 1
                        cost = distance / velocity * velocity_weight
                        return cost

                    if distance < 10 and distance >= 5:  # almost close, reduce speed.
                        velocity_weight = 0.5
                        desired_vel_weight = 1
                        desired_orient_weight = 1
                        cost = distance / velocity
                        cost = cost + desired_vel_weight * diff_deiredVelocity
                        if angle_intervall < 1 and angle_intervall != 0:
                            cost = cost + desired_orient_weight * diff_desiredOrient
                        return cost

                    if distance < 5:  # very close andjust orientation angle..
                        cost = distance / velocity
                        desired_vel_weight = 3
                        desired_orient_weight = 3
                        cost = cost + desired_vel_weight * diff_deiredVelocity
                        if angle_intervall < 1 and angle_intervall != 0:
                            cost = cost + desired_orient_weight * diff_desiredOrient
                        return cost
