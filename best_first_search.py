import copy
import numpy as np
import time
import pathlib
import os

from abc import abstractmethod, ABC
from typing import Tuple, Dict, Any, List, Union

from commonroad.scenario.trajectory import State

from SMP.maneuver_automaton.motion_primitive import MotionPrimitive
from SMP.motion_planner.node import PriorityNode
from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.utility import MotionPrimitiveStatus, initial_visualization, update_visualization
from SMP.motion_planner.queue import PriorityQueue
from SMP.motion_planner.search_algorithms.base_class import SearchBaseClass
from SMP.route_planner.route_planner.route_planner import RoutePlanner
from heapq import nsmallest
import matplotlib.pyplot as plt

from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes


class BestFirstSearch(SearchBaseClass, ABC):
    """
    Abstract class for Best First Search algorithm.
    """

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)
        self.frontier = PriorityQueue()

    @abstractmethod
    def evaluation_function(self, node_current: PriorityNode):
        """
        Function that evaluates f(n) in the inherited classes.
        @param node_current:
        @return: cost
        """
        pass

    def heuristic_function(self, node_current: PriorityNode) -> float:
        """
        Function that evaluates the heuristic cost h(n) in inherited classes.
        The example provided here estimates the time required to reach the goal state from the current node.
        @param node_current: time to reach the goal
        @return:
        """
        if self.reached_goal(node_current.list_paths[-1]):
            return 0.0

        if self.position_desired is None:
            return self.time_desired.start - node_current.list_paths[-1][-1].time_step
        else:
            velocity = node_current.list_paths[-1][-1].velocity

            if np.isclose(velocity, 0):
                return np.inf
            else:
                return self.calc_euclidean_distance(current_node=node_current) / velocity

    def execute_search(self) -> Tuple[Union[None, List[List[State]]], Union[None, List[MotionPrimitive]], Any]:
        depthmax = 200
        timemax = 200
        """
        Implementation of Best First Search (tree search) using a Priority queue.
        The evaluation function of each child class is implemented individually.
        """
        # for visualization in jupyter notebook
        list_status_nodes = []
        dict_status_nodes: Dict[int, Tuple] = {}



        # first node
        node_initial = PriorityNode(list_paths=[[self.state_initial]],
                                    list_primitives=[self.motion_primitive_initial], depth_tree=0, priority=0)
        initial_visualization(self.scenario, self.state_initial, self.shape_ego, self.planningProblem, self.config_plot,
                              self.path_fig)

        # add current node (i.e., current path and primitives) to the frontier
        f_initial = self.evaluation_function(node_initial)
        self.frontier.insert(item=node_initial, priority=f_initial)

        dict_status_nodes = update_visualization(primitive=node_initial.list_paths[-1],
                                                 status=MotionPrimitiveStatus.IN_FRONTIER,
                                                 dict_node_status=dict_status_nodes, path_fig=self.path_fig,
                                                 config=self.config_plot,
                                                 count=len(list_status_nodes))
        list_status_nodes.append(copy.copy(dict_status_nodes))

        # initialte time varible to stop search if time is too long:
        start_time = time.time()

        # get mode of planning problem:
        self.get_planningProblem_type()

        # call route planner to create oath to goal
        self.routeplannerresult = self.create_RoutePlanner_path()

        # Parse points on refrence path:
        self.parse_ref_path_to_points()

        # set theta of points:
        self.calc_curve_of_points()

        # set speed limit array based on theta of points:
        self.set_speedlimit_array()

        # read file of special scenarios and set value of self.removeSpeedLimit.
        self.check_if_scenario_in_dictionary()

        #######################################################################
        ################## change desired positon if scenario mode is survival:
        ########################################################

        # check is postitiondesired is alreeady set, if not then create one:
        if self.position_desired is None and self.planningProblemType == 'ModeD':  # desired pos empty and mode is survival:
            # set desired pos from GroupShape of goal
            if hasattr(self.planningProblem.goal.state_list[0], 'position'):
                if hasattr(self.planningProblem.goal.state_list[0].position, 'shapes'):
                    self.position_desired = self.calc_goal_interval(
                        self.planningProblem.goal.state_list[0].position.shapes[0].vertices)
        ##########################################

        if self.position_desired is None and self.planningProblemType == 'ModeB':  # desired pos empty and mode is survival:
            # set desired pos from GroupShape of goal
            if hasattr(self.planningProblem.goal.state_list[0], 'position'):
                if hasattr(self.planningProblem.goal.state_list[0].position, 'shapes'):
                    self.position_desired = self.calc_goal_interval(
                        self.planningProblem.goal.state_list[0].position.shapes[0].vertices)
        ##########################################
        #self.get_closest_ref_point_to_goal_center()

        while not self.frontier.empty():
            # pop the last node
            node_current = self.frontier.pop()

            # Delete all nodes in frontier that are 4 levels shallower to the most deep node:
            currentDepth = node_current.depth_tree
            if currentDepth >5:  # was 5
                 toDelete = []
                 for element in self.frontier.list_elements:
                     mindepth = currentDepth -4 # was 4
                     if element[-1].depth_tree < mindepth:
                         toDelete.append(self.frontier.list_elements.index(element))
                 if toDelete:
                     for index in reversed(toDelete):
                         self.frontier.list_elements.pop(index)


            dict_status_nodes = update_visualization(primitive=node_current.list_paths[-1],
                                                     status=MotionPrimitiveStatus.CURRENTLY_EXPLORED,
                                                     dict_node_status=dict_status_nodes,
                                                     path_fig=self.path_fig, config=self.config_plot,
                                                     count=len(list_status_nodes))
            list_status_nodes.append(copy.copy(dict_status_nodes))

            # goal test
            if self.reached_goal(node_current.list_paths[-1]):
                path_solution = self.remove_states_behind_goal(node_current.list_paths)
                list_status_nodes = self.plot_solution(path_solution=path_solution, node_status=dict_status_nodes,
                                                       list_states_nodes=list_status_nodes)
                # return solution
                return path_solution, node_current.list_primitives, list_status_nodes


            #if self.calc_euclidean_distance(node_current) <= 10 and self.newFarthersetPointSet == False:
            if self.newFarthersetPointSet == False:
                self.set_new_farthest_point()

            # Print current Node Depth:
            #print("current depth tree: ", node_current.depth_tree)
            #print("current N Speed: ", node_current.list_paths[-1][-1].velocity)

            #Kill search and return result if max depth is reached:
            if node_current.depth_tree == depthmax:
                path_solution = self.remove_states_behind_goal(node_current.list_paths)
                list_status_nodes = self.plot_solution(path_solution=path_solution, node_status=dict_status_nodes,
                                                       list_states_nodes=list_status_nodes)
                print("Search terminated because of depth limit !!!!")
                # return solution
                return path_solution, node_current.list_primitives, list_status_nodes

            #Kill searchv and return result if max time is reached:
            if (time.time() - start_time) > timemax:
                path_solution = self.remove_states_behind_goal(node_current.list_paths)
                list_status_nodes = self.plot_solution(path_solution=path_solution, node_status=dict_status_nodes,
                                                       list_states_nodes=list_status_nodes)
                print("Search terminated because of time limit !!!!")
                # return solution
                return path_solution, node_current.list_primitives, list_status_nodes

            #########################################

            # check all possible successor primitives(i.e., actions) for current node
            for primitive_successor in node_current.get_successors():

                # translate/rotate motion primitive to current position
                list_primitives_current = copy.copy(node_current.list_primitives)
                path_translated = self.translate_primitive_to_current_state(primitive_successor,
                                                                            node_current.list_paths[-1])
                # check for collision, if is not collision free it is skipped
                if not self.is_collision_free(path_translated):
                    list_status_nodes, dict_status_nodes = self.plot_colliding_primitives(current_node=node_current,
                                                                                          path_translated=path_translated,
                                                                                          node_status=dict_status_nodes,
                                                                                          list_states_nodes=list_status_nodes)
                    continue

                 #check check if primitive is in lanelet of road to path:
                if not self.calc_distance_to_ref_path(path_translated):
                     list_status_nodes, dict_status_nodes = self.plot_colliding_primitives(current_node=node_current,
                                                                                           path_translated=path_translated,
                                                                                           node_status=dict_status_nodes,
                                                                                           list_states_nodes=list_status_nodes)
                     continue

                 #check if speed of primite is higher tha limit fro point:
                if not self.check_speed_is_underLimit(path_translated):
                     list_status_nodes, dict_status_nodes = self.plot_colliding_primitives(current_node=node_current,
                                                                                           path_translated=path_translated,
                                                                                           node_status=dict_status_nodes,
                                                                                           list_states_nodes=list_status_nodes)
                     continue

                list_primitives_current.append(primitive_successor)

                path_new = node_current.list_paths + [[node_current.list_paths[-1][-1]] + path_translated]
                node_child = PriorityNode(list_paths=path_new,
                                          list_primitives=list_primitives_current,
                                          depth_tree=node_current.depth_tree + 1,
                                          priority=node_current.priority)
                f_child = self.evaluation_function(node_current=node_child)

                # insert the child to the frontier:
                dict_status_nodes = update_visualization(primitive=node_child.list_paths[-1],
                                                         status=MotionPrimitiveStatus.IN_FRONTIER,
                                                         dict_node_status=dict_status_nodes, path_fig=self.path_fig,
                                                         config=self.config_plot,
                                                         count=len(list_status_nodes))
                list_status_nodes.append(copy.copy(dict_status_nodes))
                self.frontier.insert(item=node_child, priority=f_child)

            dict_status_nodes = update_visualization(primitive=node_current.list_paths[-1],
                                                     status=MotionPrimitiveStatus.EXPLORED,
                                                     dict_node_status=dict_status_nodes, path_fig=self.path_fig,
                                                     config=self.config_plot,
                                                     count=len(list_status_nodes))
            list_status_nodes.append(copy.copy(dict_status_nodes))

        return None, None, list_status_nodes

    #########################################
    ######### Added by Abdelloaui, Mohamed A. 26.12.20 ##################
    #########################################
    ######### Added by Abdelloaui, Mohamed A. 26.12.20 ##################

    def check_if_scenario_in_dictionary(self):
        # comapare the current scenario id with all ids in the text files and if a math execute command:
        # speed limit dictionary:
        filepath = os.path.join(pathlib.Path(__file__).parent.absolute(),"EnableSpeedLimit.txt")
        id = self.scenario.scenario_id.__str__()
        with open(filepath, "r") as f:
            text = f.read()
            if id in text:
                self.disableSpeedLimit = False
        f.close()

        #self.disableSpeedLimit = True

        # Reference path dictionary:
        filepath = os.path.join(pathlib.Path(__file__).parent.absolute(),"DisableRefPath.txt")
        id = self.scenario.scenario_id.__str__()
        with open(filepath, "r") as f:
            text = f.read()
            if id in text:
                self.disableRefPath = True
        f.close()

        #self.disableRefPath = False

        # front obstacle avoidance:
        filepath = os.path.join(pathlib.Path(__file__).parent.absolute(),"DisableObstAvoidance.txt")
        id = self.scenario.scenario_id.__str__()
        with open(filepath, "r") as f:
            text = f.read()
            if id in text:
                self.disableObstAvoidance = True
        f.close()

        #self.disableObstAvoidance = True

    def get_closest_ref_point_to_goal_center(self):
        #get the closest point of ref path to the center of goal.
        goalcenter= self.planningProblem.goal.state_list[0].position.center
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(goalcenter, p))
        index_smallest_dist = distances.index(min(distances))
        totaltogoal = 0
        #remove all points behind closeset point
        self.refPathParsedPnts = self.refPathParsedPnts[:index_smallest_dist]



    def create_RoutePlanner_path(self):
        # Create Route Planner path and return routeplanner object
        route_planner = RoutePlanner(self.scenario, self.planningProblem, backend=RoutePlanner.Backend.NETWORKX)
        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()
        route = candidate_holder.retrieve_first_route()

        return route

    def check_for_cars_infront(self, path: List[State]) -> bool:
        disttoobst = []
        currentorient = path[-1].orientation
        currentpos = path[-1].position
        currenttimestep = path[-1].time_step
        currentVel = path[-1].velocity
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
                        if currentVel > obstPos.velocity and obstPos.velocity != 0:
                            return False
        return True


    def find_all_successor_lanelets(self):

        pos = [self.planningProblem.initial_state.position]
        print("intial Pos: ",pos)
        self.planningProblem.initial_state.position
        startlanelet = self.scenario.lanelet_network.find_lanelet_by_position(pos)
        print("intial lanelet: ",startlanelet[0][0])

        id = startlanelet[0][0]
        lanelets = [id]
        lanelet = self.scenario.lanelet_network.find_lanelet_by_id(id)
        print("ä#####")
        while lanelet.successor:
            lanelets.append(lanelet.successor[0])
            print("Next lanelet found: ", lanelets )
            lanelet =self.scenario.lanelet_network.find_lanelet_by_id(lanelets[-1])
        if lanelets:
            self.routeplannerresult = self.create_RoutePlanner_path()



    def is_in_path_routeplanner_lanelets(self, path: List[State]) -> bool:
        rrr = None
        cost_route_planner = 0
        for state in path:
            arry = state.position
            a = np.array([arry[0], arry[1]])
            listt2 = [a]

            if arry[0] != 0:
                rrr = self.scenario.lanelet_network.find_lanelet_by_position(listt2)
            route = self.routeplannerresult
            if rrr is not None and route.list_ids_lanelets is not None:
                # Check if current node is in lanelets of road to goal:
                for id in rrr[-1]:
                    if id in route.list_ids_lanelets:
                        return True
            return False

    def calc_distance_to_ref_path(self, path: List[State]) -> bool:
        # if survival mode -> no goal -> no ref path.
        if self.planningProblemType == 'Survival':
            return True

        if self.disableRefPath:
            return True

        ref_tolerance = 1.2    # was 1.2
        p3 = path[-1].position
        #calc distance of points to each point of refrence path:
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(p3, p))

        smallest_points = nsmallest(2, distances)
        index1 = distances.index(smallest_points[0])
        index2 = distances.index(smallest_points[1])
        p1 = self.refPathParsedPnts[index1]
        p2 = self.refPathParsedPnts[index2]
        distance_to_refrence = np.abs(np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1))
        if distance_to_refrence > ref_tolerance:
            return False
        return True

    def check_speed_is_underLimit(self, path: List[State]) -> bool:
        #By default there is no speed limit on primitves. in some scenarios, a speed limit in corners must be applied.
        if self.disableSpeedLimit:
            return True

        endVel = path[-1].velocity
        p3 = path[-1].position
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(p3, p))
        index_smallest_dist = distances.index(min(distances))
        speedLimit = self.speedlimitsarry[index_smallest_dist]
        if endVel > speedLimit:
            return False
        return True

    def get_point_with_farthest_distance(self):
        p1startstart = np.array([self.position_desired[0].start, self.position_desired[1].start])
        p2startend = np.array([self.position_desired[0].start, self.position_desired[1].end])
        p3endstart = np.array([self.position_desired[0].end, self.position_desired[1].start])
        p4endend = np.array([self.position_desired[0].end, self.position_desired[1].end])
        dist1 = self.euclidean_distance(self.state_initial.position, p1startstart)
        dist2 = self.euclidean_distance(self.state_initial.position, p2startend)
        dist3 = self.euclidean_distance(self.state_initial.position, p3endstart)
        dist4 = self.euclidean_distance(self.state_initial.position, p4endend)
        listofpoints = {'a': dist1, 'b': dist2, 'c': dist3, 'd': dist4}
        farthest = max(listofpoints, key=listofpoints.get)
        return farthest

    def set_new_farthest_point(self):
        # get point with farthest distance to initial state:
        if not self.planningProblemType == 'ModeD':  # run function oly when pos and time are desired
            return
        fartherestpoint = self.get_point_with_farthest_distance()

        if fartherestpoint == 'a':
            p1startstart = np.array([self.position_desired[0].start, self.position_desired[1].start])
            self.position_desired[0].end = self.position_desired[0].start + 1
            self.position_desired[1].end = self.position_desired[1].start + 1
        elif fartherestpoint == 'b':
            p2startend = np.array([self.position_desired[0].start, self.position_desired[1].end])
            self.position_desired[0].end = self.position_desired[0].start + 1
            self.position_desired[1].start = self.position_desired[1].end - 1
        elif fartherestpoint == 'c':
            p3endstart = np.array([self.position_desired[0].end, self.position_desired[1].start])
            self.position_desired[0].start = self.position_desired[0].end - 1
            self.position_desired[1].end = self.position_desired[1].start + 1
        elif fartherestpoint == 'd':
            p4endend = np.array([self.position_desired[0].end, self.position_desired[1].end])
            self.position_desired[0].start = self.position_desired[0].end - 1
            self.position_desired[1].start = self.position_desired[1].end - 1
        self.newFarthersetPointSet = True
        #print("farthest point set: ", fartherestpoint)


    def get_planningProblem_type(self):
        attributes = self.planningProblem.goal.state_list[0].attributes
        if 'position' in attributes and 'time_step' in attributes and 'velocity' in attributes and 'orientation' in attributes:
            self.planningProblemType = 'ModeA'

        elif 'position' in attributes and 'time_step' in attributes and 'velocity' in attributes:
            self.planningProblemType = 'ModeB'

        elif 'position' in attributes and 'time_step' in attributes and 'orientation' in attributes:
            self.planningProblemType = 'ModeC'

        elif 'position' in attributes and 'time_step':
            self.planningProblemType = 'ModeD'

        elif 'time_step' in attributes:
            self.planningProblemType = 'Survival'



    def parse_ref_path_to_points(self):
        self.refPathParsedPnts = [self.routeplannerresult.reference_path[0]]
        dist = 0
        for x in range(len(self.routeplannerresult.reference_path) - 1):
            dist = dist + self.euclidean_distance(self.routeplannerresult.reference_path[x], self.routeplannerresult.reference_path[x + 1])
            if dist >= 1 and dist <= 2:
                dist = 0
                self.refPathParsedPnts.append(self.routeplannerresult.reference_path[x + 1])

    def calc_curve_of_points(self):
        Theta = []
        length = len(self.refPathParsedPnts)
        lookahead_var = list(range(2, 8))

        for p in range(length - lookahead_var[5]):
            vec = [self.refPathParsedPnts[p + lookahead_var[0]] - self.refPathParsedPnts[p],
                   self.refPathParsedPnts[p + lookahead_var[1]] - self.refPathParsedPnts[p],
                   self.refPathParsedPnts[p + lookahead_var[2]] - self.refPathParsedPnts[p],
                   self.refPathParsedPnts[p + lookahead_var[3]] - self.refPathParsedPnts[p],
                   self.refPathParsedPnts[p + lookahead_var[4]] - self.refPathParsedPnts[p],
                   self.refPathParsedPnts[p + lookahead_var[5]] - self.refPathParsedPnts[p]]
            mag = []
            for i in range(6):
                mag.append(np.sqrt(vec[i].dot(vec[i])))
            theta = []
            for i in range(5):
                theta.append(np.arccos(np.dot(vec[0], vec[i + 1]) / (mag[0] * mag[i + 1])))
            summ = 0
            for i in range(5):
                summ = summ + theta[i]

            if np.isnan(summ) or abs(summ) <= 0.0872665:
                summ = 0
            sumdegree = (summ * 180) / 3.14
            Theta.append(sumdegree)
        for i in range(len(lookahead_var)+1):
            Theta.append(0)
        self.thetaofpoints = Theta

    def set_velocity_limit(self, current_node):
        currentpos = current_node.list_paths[-1][-1].position
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(currentpos, p))
        # print(distances.index(min(distances)))
        index_smallest_dist = distances.index(min(distances))
        currentthetaangle = self.thetaofpoints[index_smallest_dist]
        if self.thetaofpoints[index_smallest_dist] > 0:
            #new_limit = -0.05 * (self.thetaofpoints[index_smallest_dist]+10) + 9.5

            if currentthetaangle <= 30:
                self.velocityLimit = 8.3
            elif currentthetaangle > 30 and currentthetaangle <= 60:
                self.velocityLimit = 7.5
            elif currentthetaangle > 60:
                self.velocityLimit = 6.5
        if currentthetaangle == 0:
            self.velocityLimit = 20

    def set_speedlimit_array(self):
        maxspeed = 20

        self.speedlimitsarry = [maxspeed] * len(self.thetaofpoints)
        for i in range(len(self.thetaofpoints)):
            if 40 <= self.thetaofpoints[i]:
                # theta angle is over 30:
                #start index
                if i - 25 < 0:
                    startindex = 0
                else:
                    startindex = i - 25
                # end index
                if i - 15 > len(self.thetaofpoints):
                    endindex = len(self.thetaofpoints)
                else:
                    endindex = i - 15
                # set values
                for k in range(startindex, endindex):
                    self.speedlimitsarry[k] = 6.5
                ##############
                # second range
                if i - 15 < 0:
                    startindex = 0
                else:
                    startindex = i - 15
                # end index
                if i + 5 > len(self.thetaofpoints):
                    endindex = len(self.thetaofpoints)
                else:
                    endindex = i +0
                # set values
                for k in range(startindex, endindex):
                    self.speedlimitsarry[k] = 6

            if 50 <= self.thetaofpoints[i]:
                # theta angle is over 30:
                #start index
                if i - 25 < 0:
                    startindex = 0
                else:
                    startindex = i - 25
                # end index
                if i - 15 > len(self.thetaofpoints):
                    endindex = len(self.thetaofpoints)
                else:
                    endindex = i - 15
                # set values
                for k in range(startindex, endindex):
                    self.speedlimitsarry[k] = 6  # was 5
                ############## 
                # second range
                if i - 15 < 0:
                    startindex = 0
                else:
                    startindex = i - 15
                # end index
                if i + 5 > len(self.thetaofpoints):
                    endindex = len(self.thetaofpoints)
                else:
                    endindex = i +0
                # set values
                for k in range(startindex, endindex):
                    self.speedlimitsarry[k] = 6  # was 5

            if 65 <= self.thetaofpoints[i]:
                # theta angle is over 30:
                #start index
                if i - 25 < 0:
                    startindex = 0
                else:
                    startindex = i - 25
                # end index
                if i - 15 > len(self.thetaofpoints):
                    endindex = len(self.thetaofpoints)
                else:
                    endindex = i - 15
                # set values
                for k in range(startindex, endindex):
                    self.speedlimitsarry[k] = 6  # was 5
                ##############
                # second range
                if i - 15 < 0:
                    startindex = 0
                else:
                    startindex = i - 15
                # end index
                if i + 5 > len(self.thetaofpoints):
                    endindex = len(self.thetaofpoints)
                else:
                    endindex = i +0
                # set values
                for k in range(startindex, endindex):
                    self.speedlimitsarry[k] = 6  # was 5

        x = range(0, len(self.speedlimitsarry))

####################################################################
    def smooth(self, y, box_pts):
        box = np.ones(box_pts) / box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth
################################################################

    def set_velocity_limit2(self, current_node):
        currentpos = current_node.list_paths[-1][-1].position
        distances = []
        for p in self.refPathParsedPnts:
            distances.append(self.euclidean_distance(currentpos, p))
        # print(distances.index(min(distances)))
        index_smallest_dist = distances.index(min(distances))
        currentspeedlimit = self.speedlimitsarry[index_smallest_dist]
        if currentspeedlimit == 0:
            self.velocityLimit = 11
        else:
            self.velocityLimit = currentspeedlimit

    def check_collsion_speed_in_front(self, path: List[State]):
        for state in path:
            currentpos = state.position
            list = [currentpos]
            #a = np.array([arry[0], arry[1]])
            rrr = self.scenario.lanelet_network.find_lanelet_by_position(list)
            currentorient = state.orientation
            currenttimestep = state.time_step
            if rrr[-1][0] is not None:
                for obst in self.list_obstacles:
                    obstPos = obst.state_at_time(currenttimestep)
                    if currentorient is not None and obstPos is not None:
                        disttoobst = self.euclidean_distance(currentpos, obstPos.position)
                        if disttoobst <= 15:
                            orientdiff = self.calc_orientation_diff(currentorient, obstPos.orientation)
                            if abs(orientdiff) <= 0.349:
                                if state.velocity > obstPos.velocity:
                                    return False
            else:
                return True
        return True


class UniformCostSearch(BestFirstSearch):
    """
    Class for Uniform Cost Search (Dijkstra) algorithm.
    """

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)

        if plot_config.SAVE_FIG:
            self.path_fig = '../figures/ucs/'
        else:
            self.path_fig = None

    def evaluation_function(self, node_current: PriorityNode) -> float:
        """
        Evaluation function of UCS is f(n) = g(n)
        """

        # calculate g(n)
        if self.reached_goal(node_current.list_paths[-1]):
            node_current.list_paths = self.remove_states_behind_goal(node_current.list_paths)
        node_current.priority += (len(node_current.list_paths[-1]) - 1) * self.scenario.dt

        return node_current.priority


class GreedyBestFirstSearch(BestFirstSearch):
    """
    Class for Greedy Best First Search algorithm.
    """

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)

        if plot_config.SAVE_FIG:
            self.path_fig = '../figures/gbfs/'
        else:
            self.path_fig = None

    def evaluation_function(self, node_current: PriorityNode) -> float:
        """
        Evaluation function of GBFS is f(n) = h(n)
        """

        node_current.priority = self.heuristic_function(node_current=node_current)
        return node_current.priority


class AStarSearch(BestFirstSearch):
    """
    Class for A* Search algorithm.
    """

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)

        if plot_config.SAVE_FIG:
            self.path_fig = '../figures/astar/'
        else:
            self.path_fig = None

    def evaluation_function(self, node_current: PriorityNode) -> float:
        """
        Evaluation function of A* is f(n) = g(n) + h(n)
        """
        if self.reached_goal(node_current.list_paths[-1]):
            node_current.list_paths = self.remove_states_behind_goal(node_current.list_paths)
        # calculate g(n)
        node_current.priority += (len(node_current.list_paths[-1]) - 1) * self.scenario.dt

        # f(n) = g(n) + h(n)
        return node_current.priority + self.heuristic_function(node_current=node_current)
