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
DOCK_END = 20
DEPOT = 0
NUM_COMPANIES = 4

MAX_DOCK_BEGIN = 6
MAX_DOCK_END = 20


def create_data_model(num_vehicles, depot, dock_begin, dock_end):
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
        (dock_begin, dock_end),  # 4 docks
        (6, 20),  # 5
        (6, 20),  # 6
        (6, 20),  # 7
    ]
    data['num_vehicles'] = num_vehicles
    data['depot'] = depot
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""

    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    total_plan_output = []
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
        total_time += solution.Min(time_var)
        total_plan_output += [plan_output]

    company_dict = {
        'num_vehicles': data['num_vehicles'],
        'plan_output': total_plan_output,
        'total_time': total_time,
        'dock_arrival': dock_arrival
    }

    return company_dict


def main(company, num_vehicles=1):
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model(num_vehicles, company.DEPOT, company.DOCK_BEGIN, company.DOCK_END)

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
        company_dict = print_solution(data, manager, routing, solution)
        return company_dict
    else:
        num_vehicles += 1
        if num_vehicles <= MAX_VEHICLES:
            return main(company, num_vehicles)
        else:
            print("Maximum Vehicle Reached!")
            return None

def time_window_factory(time_window, time_split):
    new_dock_begin = None
    new_dock_end = None
    print('wtf', time_split, time_window)
    if time_split + 1 < MAX_DOCK_END :
        new_dock_begin = time_split + 1
    if (time_split - 1) > MAX_DOCK_BEGIN and (time_split - 1) > time_window[0]:
        new_dock_end = time_split - 1
    print('1 and 2: ', new_dock_end, new_dock_begin)
    if new_dock_begin and new_dock_end:
        splited_time_window = [(time_window[0], new_dock_end), (new_dock_begin, time_window[1])]
    elif new_dock_begin:
        splited_time_window = [(new_dock_begin, time_window[1])]
    elif new_dock_end:
        splited_time_window = [(time_window[0], new_dock_end)]
    else:
        return
    print('splited_time_window: ', splited_time_window)
    return splited_time_window 

def engine():
    time_windows = []
    time_windows += [(DOCK_BEGIN, DOCK_END)]
    companies = create_companies()
    for company in companies:
        try:
            tw = time_windows.pop()
        except:
            print('No Time Window Available!')
            return
        print('TimeWindow: ', tw)
        company.set_time_window(tw)
        company_dict = main(company)
        company.set_company_dict(company_dict)
        for plan_output in company.PLAN_OUTPUT:
            print(plan_output)
        if company.DOCK_ARRIVAL:
            time_windows += time_window_factory(tw, company.DOCK_ARRIVAL)
        

def create_companies():
    companies = []
    companies += [Company(5)]
    companies += [Company(0)]
    companies += [Company(1)]
    companies += [Company(3)]
    return companies

if __name__ == '__main__':
    engine()