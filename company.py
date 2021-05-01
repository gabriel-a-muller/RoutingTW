class Company(object):
    DEPOT = None
    NUM_VEHICLES = 0
    PLAN_OUTPUT = None
    TOTAL_TIME = 0
    DOCK_ARRIVAL = None
    DOCK_BEGIN = None
    DOCK_END = None

    def __init__(self, depot):
        self.DEPOT = depot

    def set_time_window(self, time_window):
        self.DOCK_BEGIN = time_window[0]
        self.DOCK_END = time_window[1]

    def set_company_dict(self, company_dict):
        if company_dict:
            self.NUM_VEHICLES = company_dict['num_vehicles']
            self.TOTAL_TIME = company_dict['total_time']
            self.DOCK_ARRIVAL = company_dict['dock_arrival']
            self.PLAN_OUTPUT = company_dict['plan_output']
        else:
            print('Company Dict is NONE!')

        