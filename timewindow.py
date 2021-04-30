"""Vehicles Routing Problem (VRP) with Time Windows."""

"""RULES: 
1. Truck can only arrive inside time window.
2. A Truck can stay after the time window, only if it respected the first rule.
3. Static and constant unload timer for every Truck at the docks = 30 min (0.5 h).
4. Each company MUST construct a feasible solution, by adding more Trucks if needed.
5. Maximum of 5 Vehicles
6. Number of Companies X
"""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from company import Company


MAX_VEHICLES = 5
DOCK_NODE = 4
DOCK_BEGIN = 6
DOCK_END = 12
DEPOT = 0
NUM_COMPANIES = 4

MAX_DOCK_BEGIN = 6
MAX_DOCK_END = 12


def create_data_model(num_vehicles):
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = [
        [0, 6, 9, 8, 7, 3, 6, 2],
        [6, 0, 8, 3, 2, 6, 8, 4],
        [9, 8, 0, 11, 10, 6, 3, 9],
        [8, 3, 11, 0, 1, 7, 10, 6],
        [7, 2, 10, 1, 0, 6, 9, 4],
        [3, 6, 6, 7, 6, 0, 2, 3],
        [6, 8, 3, 10, 9, 2, 0, 6],
        [2, 4, 9, 6, 4, 3, 6, 0],
    ]
    data['time_windows'] = [
        (6, 20),  # depot
        (6, 20),  # 1
        (6, 20),  # 2
        (6, 20),  # 3
        (DOCK_BEGIN, DOCK_END),  # 4 docks
        (6, 20),  # 5
        (6, 20),  # 6
        (6, 20),  # 7
    ]
    data['num_vehicles'] = num_vehicles
    data['depot'] = DEPOT
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""

    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) -> '.format(
                manager.IndexToNode(index),
                solution.Min(time_var),
                solution.Max(time_var))
            if manager.IndexToNode(index) == DOCK_NODE:
                dock_arrival = solution.Min(time_var)

            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                    solution.Min(time_var),
                                                    solution.Max(time_var))
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        print(plan_output)
        total_time += solution.Min(time_var)
    #print('Total time of all routes: {}min'.format(total_time))

    company_dict = {
        'depot': DEPOT,
        'num_vehicles': data['num_vehicles'],
        'plan_output': plan_output,
        'total_time': total_time,
        'dock_arrival': dock_arrival
    }

    return company_dict


def main(num_vehicles=1):
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model(num_vehicles)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == data['depot']:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    depot_idx = data['depot']
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data['time_windows'][depot_idx][0],
            data['time_windows'][depot_idx][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        num_vehicles += 1
        if num_vehicles <= MAX_VEHICLES:
            main(num_vehicles)
        else:
            print("Maximum Vehicle Reached!")
            return None


def engine():
    pass

if __name__ == '__main__':
    main()