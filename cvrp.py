'''
import os

cwd = os.getcwd()  # Get the current working directory (cwd)
files = os.listdir(cwd)  # Get all the files in that directory
print("Files in %r: %s" % (cwd, files))
'''


from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class Graph():

    def __init__(self):
        self.graph = []
        self.length = 0
        self.depot = 0
        self.demand = 0
        self.capacity = 0
        self.num_vehicles = 0
        self.distMatrix = 0

    def buildGraph(self, filename):

        with open(filename) as f:
            lines = f.readlines()

            for l in range(len(lines)):
                if lines[l] == "NODE_COORD_SECTION\n":
                    l1 = l
                elif lines[l] == "DEMAND_SECTION\n":
                    l2 = l
                elif lines[l] == "DEPOT_SECTION\n":
                    l3 = l

            DEPOT = lines[l3+1]
            self.depot = int(DEPOT[:-1])

            NODE_COORD = lines[l1+1:l2]
            DEMAND_SECTION = lines[l2+1:l3]

            TRUCK_POP = int(lines[4][11:])
            CAPACITY = int(lines[6][11:])

            self.num_vehicles = TRUCK_POP
            self.capacity = CAPACITY


            for i in range(len(NODE_COORD)):
                if NODE_COORD[i][-1] == '\n':
                    NODE_COORD[i] = NODE_COORD[i][:-1]
                NODE_COORD[i] = NODE_COORD[i].split(' ')

            for i in range(len(DEMAND_SECTION)):
                if DEMAND_SECTION[i][-1] == '\n':
                    DEMAND_SECTION[i] = DEMAND_SECTION[i][:-1]
                DEMAND_SECTION[i] = DEMAND_SECTION[i].split(' ')

            graph = [0] * (int(NODE_COORD[-1][0]) + 1)
            count = 0
            while count < len(NODE_COORD):
                graph[int(NODE_COORD[count][0])] = (int(NODE_COORD[count][1]), int(NODE_COORD[count][2]))
                count += 1

            demand = [0]
            count = 0
            while count < len(DEMAND_SECTION):
                demand.append(int(DEMAND_SECTION[count][1]))
                count += 1

        self.demand = demand
        self.graph = graph
        self.length = len(self.graph)


        distMatrix = [[0 for k in range(len(self.graph))]for l in range(len(self.graph))]

        for i in range(1,len(self.graph)):
            for j in range(1,len(self.graph)):
                dist = abs(self.graph[i][0]-self.graph[j][0]) + abs(self.graph[i][1]-self.graph[j][1])
                distMatrix[i][j] = dist
                distMatrix[j][i] = dist

        self.distMatrix = distMatrix
        


def create_data_model():
    """Stores the data for the problem."""
    graph = Graph()
    graph.buildGraph("A-n32-k5.txt")
    data = {}
    data['distance_matrix'] = graph.distMatrix
    data['demands'] = graph.demand
    data['num_vehicles'] = graph.num_vehicles
    data['vehicle_capacities'] = graph.num_vehicles * [graph.capacity]
    data['depot'] = graph.depot
    return data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))


def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()
