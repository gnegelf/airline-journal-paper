import re
import math
import time
import sys
from operator import itemgetter
if not 'cplex' in globals():
    import cplex
    from cplex.callbacks import IncumbentCallback
    from cplex.callbacks import LazyConstraintCallback
    from cplex.callbacks import MIPInfoCallback
    from cplex.callbacks import UserCutCallback
    from cplex.exceptions import CplexSolverError

from sets import Set

EPSILON = 1e-6


# -------
# Functions for convience
# -------

def rotateList(l, n):
    return l[n:] + l[:n]


def varToKey(varStr, pos):
    return re.split('[_#]', varStr)[pos]


def arcAmount(path, arc):
    res = 0
    for i in range(len(path) - 1):
        if path[i] == arc[0] and path[i + 1] == arc[1]:
            res += 1

    return res


# turns a list of strings with variable names into a list of paths
def solToAirports(solutionStringList, p,PLANE):
    airportList = []
    for string in solutionStringList:
        airportList.append(re.split('[_#]', string)[1])
    airportList2 = []
    for string in solutionStringList:
        airportList2.append(re.split('[_#]', string)[2])
    airportDict = {}
    airportDict1 = {}
    airportDict2 = {}
    for s in airportList:
        airportDict1[s] = 0
        airportDict2[s] = 0
    for s in airportList2:
        airportDict1[s] = 0
        airportDict2[s] = 0
    for s in airportList:
        airportDict1[s] += 1

    for s in airportList2:
        airportDict2[s] += 1

    for s in airportDict1:
        airportDict[s] = max([airportDict1[s], airportDict2[s]])
        if s == PLANE[p].origin:
            airportDict[s] += 1

    return airportDict


# turns a list of strings with variable names into a list of paths
def solToPaths(solutionStringList):
    pathList = []
    for string in solutionStringList:
        pathList.append([re.split('[_#]', string)[1], re.split('[_#]',
                        string)[2]])
    prevLength = 0
    curLength = len(pathList)
    while prevLength != curLength:
        prevLength = len(pathList)
        for (i, path) in enumerate(pathList):
            for j in range(len(pathList)):
                if i != j:
                    if path[-1] == pathList[j][0]:
                        pathList[j].pop(0)
                        path += pathList.pop(j)
                        break
                    else:
                        if pathList[j][0] == pathList[j][-1]:
                            breaker = 0
                            for (num, vert) in enumerate(path):
                                for (num2, vert2) in enumerate(pathList[j]):
                                    if vert2 == vert:
                                        tempPath = pathList.pop(j)
                                        tempPath.pop(-1)
                                        tempPath = rotateList(tempPath,num2)
                                        path[num:num] = tempPath

                                        breaker = 1
                                        break
                                if breaker:
                                    break
                            if breaker:
                                break

            curLength = len(pathList)
            if curLength != prevLength:
                break
    return pathList


# ---------
# callbacks
# ---------

class CountNodesCallback(MIPInfoCallback):

    def __call__(self):

        self.number_of_nodes = self.get_num_nodes()
        self.best_obj_val = self.get_best_objective_value()
        self.mip_gap = self.get_MIP_relative_gap()

        return


class breakIncumbentCallback(IncumbentCallback):
    
    def __call__(self):
        all_values = self.get_values()
        self.totallySolved = 1

        paths = {}
        flyTime = {}
        assignedRequests = {}
        PLANE = self.MIPMODEL.DATA.PLANE
        y = self.MIPMODEL.y
        x = self.MIPMODEL.x
        name2idx = self.MIPMODEL.name2idx
        turnover_travel_timesteps = self.MIPMODEL.DATA.turnover_travel_timesteps
        
        TRIP0 = self.MIPMODEL.DATA.TRIP0
        TRIP = self.MIPMODEL.DATA.TRIP
        REQUEST = self.MIPMODEL.DATA.REQUEST
        
        AirportNum = self.MIPMODEL.DATA.current_airport_num
        
        for p in PLANE:
            r2 = self.fullModel[p].r2
            fullModel = self.fullModel[p].model
            y2 = self.fullModel[p].y
            assignedRequests[p] = {}
            yString = []
            flyTime[p] = 0
            for (key, val) in y.iteritems():
                if key[2] == p:
                    valStore = all_values[name2idx[val]]
                    for k in range(10):  
                        if valStore > 0.1 + k:
                            yString += [val]
                            flyTime[p] += \
                                turnover_travel_timesteps[key[0],
                                    key[1], key[2]]
                        else:
                            break
            paths[p] = solToPaths(yString)

            if paths[p] == []:
                continue

            for (i, j) in TRIP0:
                fullModel.variables.set_upper_bounds([(y2[i, j, p],
                        arcAmount(paths[p][0], [i, j]))])
                fullModel.variables.set_lower_bounds([(y2[i, j, p],
                        arcAmount(paths[p][0], [i, j]))])
            requestArcs = {}
            for (i, j) in TRIP:
                requestArcs[i, j] = 0

            for r in REQUEST:
                assignedRequests[p][r] = 0

            # find assigned requests and set arcs
            for (key, val) in x.iteritems():
                if key[1] == p:
                    valStore = all_values[name2idx[val]]
                    if valStore > 0.1:
                        assignedRequests[p][key[0]] = 1

            for (r, assigned) in assignedRequests[p].iteritems():
                fullModel.variables.set_upper_bounds([(r2[r],
                        assigned)])
                fullModel.variables.set_lower_bounds([(r2[r],
                        assigned)])
                                    
            # converts the analyzed y variables to a set of longest paths
            
            airports = solToAirports(yString, p,PLANE)
            fullModel.solve()
            if not fullModel.solution.is_primal_feasible():
                self.number_of_infeasibles += 1
                self.totallySolved = 0
                for s in airports:
                    if AirportNum[p, s][-1] < airports[s]:
                        AirportNum[p, s].append(AirportNum[p, s][-1]
                                + 1)
                return

        if self.totallySolved:
            if self.get_objective_value() < self.bestSolution:
                self.bestSolution = self.get_objective_value()
                self.best_plane_solution = {}
                self.best_request_solution = {}
                for (key, val) in y.iteritems():
                    valStore = all_values[name2idx[val]]
                    if valStore > 0.1:
                        if (key[2], key[0], key[1]) \
                            in self.best_plane_solution:
                            self.best_plane_solution[key[2], key[0],
                                    key[1]] += 1
                        else:
                            self.best_plane_solution[key[2], key[0],
                                    key[1]] = 1
        
        return


# -------
# classes for data
# -------

class __PLANE__(object):

    def __init__(
        self,
        cost=None,
        seats=None,
        plane_departure=None,
        departure_min_fuel=None,
        departure_max_fuel=None,
        plane_arrival=None,
        arrival_min_fuel=None,
        arrival_max_fuel=None,
        required_fueltype=None,
        fuel=None,
        speed=None,
        max_fuel=None,
        empty_weight=None,
        add_turnover_time=None,
        reserve_fuel=None,
        contigence_ratio=None,
        pilot_weight=None,
        ):
        self.cost = float(cost)
        self.seats = int(seats)
        self.plane_departure = plane_departure
        self.departure_min_fuel = float(departure_min_fuel)
        self.departure_max_fuel = float(departure_max_fuel)
        self.plane_arrival = plane_arrival
        self.arrival_min_fuel = float(arrival_min_fuel)
        self.arrival_max_fuel = float(arrival_max_fuel)
        self.required_fueltype = int(required_fueltype)
        self.fuel = float(fuel)
        self.speed = float(speed)
        self.max_fuel = float(max_fuel)
        self.empty_weight = float(empty_weight)
        self.add_turnover_time = int(add_turnover_time)
        self.reserve_fuel = float(reserve_fuel)
        self.contigence_ratio = float(contigence_ratio)
        self.pilot_weight = float(pilot_weight)
        self.origin = plane_departure
        self.destination = plane_arrival


class __AIRPORT__(object):

    def __init__(self, turnover_time=None):
        self.turnover_time = int(turnover_time)
        self.fuel = {}


class __REQUEST__(object):

    def __init__(
        self,
        request_departure=None,
        request_arrival=None,
        earliest_departure_time=None,
        earliest_departure_day=None,
        latest_arrival_time=None,
        latest_arrival_day=None,
        passengers=None,
        weight=None,
        max_stops=None,
        max_detour=None,
        ):
        self.request_departure = request_departure
        self.request_arrival = request_arrival
        self.origin = request_departure
        self.destination = request_arrival
        self.earliest_departure_time = int(earliest_departure_time)
        self.earliest_departure_day = int(earliest_departure_day)
        self.latest_arrival_time = int(latest_arrival_time)
        self.latest_arrival_day = int(latest_arrival_day)
        self.passengers = int(passengers)
        self.weight = float(weight)
        self.max_stops = int(max_stops)
        self.max_detour = float(max_detour)

        self.earliest_departure = 1440 * (self.earliest_departure_day
                - 1) + self.earliest_departure_time
        self.latest_arrival = 1440 * (self.latest_arrival_day - 1) \
            + self.latest_arrival_time


class __WEIGHTLIMIT__(object):

    def __init__(self, max_takeoff_weight=None,
                 max_landing_weight=None):
        self.max_takeoff_weight = float(max_takeoff_weight)
        self.max_landing_weight = float(max_landing_weight)


class __TRIP__(object):

    def __init__(self, distance=None):
        self.distance = float(distance)


# -------
# classes for derived data
# -------

class __REQUESTPATH__(object):

    def __init__(self,v,earliest_start_time,min_duration,distance):
        self.v = v
        self.est = earliest_start_time
        self.min_duration = min_duration
        self.distance = distance

    def hasArc(self, a):
        for i in range(len(self.v) - 1):
            if self.v[i] == a[0] and self.v[i + 1] == a[1]:
                return 1
        return 0
        


class __SOLUTION__(object):
    
    def __init__(self,gap,dualBound,bestValue,loop_iterations):
        self.gap = gap
        self. dualBound = dualBound
        self.bestValue = bestValue
        self.loop_iterations = loop_iterations
        
class __DATA__(object):
    
    def __init__(self,AIRPORT,PLANE,REQUEST,TRIP,WEIGHTLIMIT,PLANE_SOLUTION,REQUEST_SOLUTION,timedelta):
        self.AIRPORT = AIRPORT
        self.PLANE = PLANE
        self.REQUEST = REQUEST
        self.TRIP = TRIP
        self.WEIGHTLIMIT = WEIGHTLIMIT
        self.PLANE_SOLUTION = PLANE_SOLUTION
        self.REQUEST_SOLUTION = REQUEST_SOLUTION
        self.timedelta = timedelta
        self.deriveData()

    def deriveData(self):
        PLANE = self.PLANE
        REQUEST = self.REQUEST
        AIRPORT = self.AIRPORT
        timedelta = self.timedelta
        TRIP = self.TRIP
        WEIGHTLIMIT = self.WEIGHTLIMIT
        PLANE_SOLUTION = self.PLANE_SOLUTION
        REQUEST_SOLUTION = self.REQUEST_SOLUTION
        
        
        turnover_timesteps = {}
        
        for i in AIRPORT:
          for p in PLANE:
            turnover_timesteps[i,p] = int(max(1,math.ceil((AIRPORT[i].turnover_time + PLANE[p].add_turnover_time) / timedelta)))
        
        
        travelcost = {}
        
        for p in PLANE:
          for i, j in TRIP:
            travelcost[i,j,p] = TRIP[i,j].distance * PLANE[p].cost
        
        self.travelcost = travelcost
        travel_time = {}
        
        for p in PLANE:
          for i, j in TRIP:
            travel_time[i,j,p] = int(math.floor(TRIP[i,j].distance / ((PLANE[p].speed / 60) * 5)) * 5)
            #travel_time[i,j,p] = TRIP[i,j].distance / (PLANE[p].speed / 60)
        
        
        travel_timesteps = {}
        
        for p in PLANE:
          for i, j in TRIP:
            travel_timesteps[i,j,p] = int(max(1,math.ceil(travel_time[i,j,p] / timedelta)))
        
        
        turnover_travel_timesteps = {}
        
        for p in PLANE:
          for i, j in TRIP:
            if i == j:
              turnover_travel_timesteps[i,j,p] = 1
            else:
              turnover_travel_timesteps[i,j,p] = int(travel_timesteps[i,j,p] + turnover_timesteps[i,p])
        
        self.turnover_travel_timesteps = turnover_travel_timesteps
                        
        max_takeoff_payload = {}
        
        for p in PLANE:
          for i in AIRPORT:
            max_takeoff_payload[i,p] = WEIGHTLIMIT[i,p].max_takeoff_weight - PLANE[p].empty_weight - PLANE[p].reserve_fuel - PLANE[p].pilot_weight
        
        self.max_takeoff_payload = max_takeoff_payload
        
        
        max_landing_payload = {}
        
        for p in PLANE:
          for i in AIRPORT:
            max_landing_payload[i,p] = WEIGHTLIMIT[i,p].max_landing_weight - PLANE[p].empty_weight - PLANE[p].reserve_fuel - PLANE[p].pilot_weight
        
        self.max_landing_payload = max_landing_payload
        
        fuelconsumption = {}
        
        for p in PLANE:
          for i, j in TRIP:
            fuelconsumption[i,j,p] = math.ceil(travel_time[i,j,p] * PLANE[p].fuel * PLANE[p].speed * PLANE[p].contigence_ratio / 60.0);
        
        self.fuelconsumption = fuelconsumption
        
        max_trip_payload = {}
        
        for p in PLANE:
          for i, j in TRIP:
            max_trip_payload[i,j,p] = min(max_takeoff_payload[i,p] + fuelconsumption[i,j,p], max_landing_payload[j,p])
        
        self.max_trip_payload = max_trip_payload 
        
        max_trip_fuel = {}
        
        for p in PLANE:
          for i, j in TRIP:
            max_trip_fuel[i,j,p] = PLANE[p].max_fuel - fuelconsumption[i,j,p] - PLANE[p].reserve_fuel
        
        self.max_trip_fuel = max_trip_fuel
        
        
        earliest_departure_timesteps = {}
        
        for r in REQUEST:
          earliest_departure_timesteps[r] = int(math.ceil((REQUEST[r].earliest_departure_time + 1440 * (REQUEST[r].earliest_departure_day - 1)) / timedelta))
          
        self.earliest_departure_timesteps = earliest_departure_timesteps
        latest_arrival_timesteps = {}
        
        for r in REQUEST:
          latest_arrival_timesteps[r] = int(math.floor((REQUEST[r].latest_arrival_time + 1440 * (REQUEST[r].latest_arrival_day - 1)) / timedelta))
        
        self.latest_arrival_timesteps = latest_arrival_timesteps
        
        direct_flight_timesteps = {}
        
        for p in PLANE:
          for r in REQUEST:
            if PLANE[p].plane_departure == REQUEST[r].request_departure:
              direct_flight_timesteps[p,r] = 0
            else:
              direct_flight_timesteps[p,r] = turnover_travel_timesteps[PLANE[p].plane_departure,REQUEST[r].request_departure,p]
        
            #print "direct_flight_timesteps plane ",p,", request ",r,": ",direct_flight_timesteps[p,r]
        
        max_refuel_flight_timesteps = {}
        
        for p in PLANE:
          for r in REQUEST:
            max_refuel_flight_timesteps[p,r] = 0
            
            for i in AIRPORT:
              if (AIRPORT[i].fuel[PLANE[p].required_fueltype] == '1' and i != PLANE[p].plane_departure and i != REQUEST[r].request_departure):
                aux = turnover_travel_timesteps[PLANE[p].plane_departure,i,p] + turnover_travel_timesteps[i,REQUEST[r].request_departure,p]
                max_refuel_flight_timesteps[p,r] = max(max_refuel_flight_timesteps[p,r],aux)
        
            #print "max_refuel_flight_timesteps plane ",p,", request ",r,": ",max_refuel_flight_timesteps[p,r]
        
        plane_min_timestep = {}
        
        for p in PLANE:
          plane_min_timestep[p] = 99999
          
          for r in REQUEST:
            if REQUEST[r].passengers <= PLANE[p].seats:
              aux = earliest_departure_timesteps[r] - turnover_timesteps[REQUEST[r].request_departure,p] - max(direct_flight_timesteps[p,r], max_refuel_flight_timesteps[p,r])
              plane_min_timestep[p] = int(min(plane_min_timestep[p],aux))
        
          #print "plane ",p," min timestep: ",plane_min_timestep[p]
        
        self.plane_min_timestep = plane_min_timestep
        
        plane_max_timestep = {}
        
        for p in PLANE:
          plane_max_timestep[p] = 0
          
          for r in REQUEST:
            if REQUEST[r].passengers <= PLANE[p].seats:
              aux = latest_arrival_timesteps[r] + max(direct_flight_timesteps[p,r],max_refuel_flight_timesteps[p,r])
              plane_max_timestep[p] = int(max(plane_max_timestep[p],aux))
        
          #print "plane ",p," max timestep: ",plane_max_timestep[p]
        
        self.plane_max_timestep = plane_max_timestep
        
        TRIP0 = {}
        
        for i,j in TRIP:
          if i != j:
            TRIP0[i,j] = TRIP[i,j]
        
        self.TRIP0 = TRIP0
        
        REQUEST_TRIP = {}
        
        for r in REQUEST:
          REQUEST_TRIP[r] = {}
        
          for i,j in TRIP:
            if i != REQUEST[r].request_arrival and j != REQUEST[r].request_departure:
              REQUEST_TRIP[r][i,j] = TRIP[i,j]
        
        self.REQUEST_TRIP = REQUEST_TRIP
        
        REQUEST_TRIP0 = {}
        
        for r in REQUEST:
          REQUEST_TRIP0[r] = {}
        
          for i,j in TRIP0:
            if i != REQUEST[r].request_arrival and j != REQUEST[r].request_departure:
              REQUEST_TRIP0[r][i,j] = TRIP0[i,j]
        
        self.REQUEST_TRIP0 = REQUEST_TRIP0
        
        min_refuel_trip = {}
        
        for i in AIRPORT:
          for p in PLANE:
            min_refuel_trip[i,p] = 99999
            for j in AIRPORT:
              if (i,j) in TRIP:
                if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '1':
                  min_refuel_trip[i,p] = min(min_refuel_trip[i,p], fuelconsumption[i,j,p])
        
        self.min_refuel_trip = min_refuel_trip
        
        max_turnover_timesteps = {}
        for r in REQUEST:
          max_turnover_timesteps[r] = 0
          for p in PLANE:
            max_turnover_timesteps[r] = max(max_turnover_timesteps[r], turnover_timesteps[REQUEST[r].request_departure,p])
        
        self.max_turnover_timesteps = max_turnover_timesteps
        
        min_timestep = 99999
        max_timestep = 0
        
        for p in PLANE:
          min_timestep = min(min_timestep, plane_min_timestep[p])
          max_timestep = max(max_timestep, plane_max_timestep[p])
        
        
        TIMEFREEREQUESTSOLUTION = {}
        
        for p,r,i,j,hh,mm in REQUEST_SOLUTION:
          TIMEFREEREQUESTSOLUTION[p,r,i,j] = 1
          
        self.TIMEFREEREQUESTSOLUTION = TIMEFREEREQUESTSOLUTION   
        
        TIMEFREEPLANESOLUTION = {}
        
        for p,i,j,hh,mm in PLANE_SOLUTION:
          if (p,i,j) in TIMEFREEPLANESOLUTION:
            TIMEFREEPLANESOLUTION[p,i,j] += 1
          else:
            TIMEFREEPLANESOLUTION[p,i,j] = 1
        
        self.TIMEFREEPLANESOLUTION = TIMEFREEPLANESOLUTION
        
        min_return_time = {}
        for p in PLANE:
            for i in AIRPORT:
                min_out = 10000
                min_in = 10000
                for j in AIRPORT:
                    if (i,j) in TRIP0:
                        if turnover_travel_timesteps[i,j,p] < min_out:
                            min_out = turnover_travel_timesteps[i,j,p]
                        if turnover_travel_timesteps[j,i,p] < min_in:
                            min_in = turnover_travel_timesteps[j,i,p] 
                min_return_time[i,p] = min_in+min_out
    
        self.min_return_time = min_return_time
        
        maxStops = {}
        for i in AIRPORT:
          maxStops[i] = 0
          anyfuel = 0
          for ft in AIRPORT[i].fuel:
            if AIRPORT[i].fuel[ft] == '1':
              anyfuel += 1
              
          if anyfuel == 0:
            for r in REQUEST:
              if REQUEST[r].request_departure == i or REQUEST[r].request_arrival == i:
                maxStops[i] += 1
            
            for p in PLANE:
              if PLANE[p].plane_departure == i:
                maxStops[i] += 1
              if PLANE[p].destination == i:
                maxStops[i] += 1 
          else:
            maxStops[i] = 5
    
                
        AirportNum2 = {}
        for i in AIRPORT:
            for p in PLANE:
                AirportNum2[p,i] = range(1,1+maxStops[i])
                
        self.max_airport_num = AirportNum2
        
        AirportNum = {}
        for i in AIRPORT:
            for p in PLANE:
                AirportNum[p,i] = [1]

        self.current_airport_num = AirportNum

class __AIRLINEMIP__(object):
    
    def __init__(self,DATA,cuttingPlanes={},full=0,p={}):
        self.DATA = DATA
        if full:
            PLANE = p
        else:
            PLANE = DATA.PLANE
        REQUEST = DATA.REQUEST
        REQUEST_TRIP0 = DATA.REQUEST_TRIP0
        AIRPORT = DATA.AIRPORT
        TRIP = DATA.TRIP
        TRIP0 = DATA.TRIP0
        #PLANE_SOLUTION = DATA.PLANE_SOLUTION
        #REQUEST_SOLUTION = DATA.REQUEST_SOLUTION
        latest_arrival_timesteps = DATA.latest_arrival_timesteps
        earliest_departure_timesteps =  DATA.earliest_departure_timesteps
        max_turnover_timesteps = DATA.max_turnover_timesteps
        self.full = full
        if full:
            AirportNum = DATA.max_airport_num
        else:
            AirportNum = DATA.current_airport_num
        turnover_travel_timesteps = DATA.turnover_travel_timesteps
        travelcost = DATA.travelcost
        plane_min_timestep = DATA.plane_min_timestep
        plane_max_timestep = DATA.plane_max_timestep
        fuelconsumption = DATA.fuelconsumption
        min_return_time = DATA.min_return_time
        max_trip_fuel = DATA.max_trip_fuel
        max_trip_payload = DATA.max_trip_payload
        min_refuel_trip = DATA.min_refuel_trip
        
        possiblePaths = {}
        for r in REQUEST:
            maxDuration = latest_arrival_timesteps[r]-earliest_departure_timesteps[r]+max_turnover_timesteps[r]
            maxPathLength = (1 + REQUEST[r].max_detour) * TRIP0[REQUEST[r].request_departure,REQUEST[r].request_arrival].distance
            ori = REQUEST[r].origin
            desti = REQUEST[r].destination
            for p in PLANE:
                possiblePaths[r,p]=[]
                for oriInd in AirportNum[p,ori]:
                    for destiInd in AirportNum[p,desti]:
                        possiblePaths[r,p] += [__REQUESTPATH__([(ori,oriInd),(desti,destiInd)],0,turnover_travel_timesteps[ori,desti,p],TRIP[ori,desti].distance)]
                        for i in AIRPORT:
                            if i!=ori and i != desti:
                                pathLength1 = TRIP[ori,i].distance + TRIP[i,desti].distance
                                pathDuration1 = turnover_travel_timesteps[ori,i,p] + turnover_travel_timesteps[i,desti,p]
                                for ii in AirportNum[p,i]:
                                    if  pathLength1 <= maxPathLength:
                                        if  pathDuration1 <= maxDuration:
                                            possiblePaths[r,p] += [__REQUESTPATH__([(ori,oriInd),(i,ii),(desti,destiInd)],0,pathDuration1,pathLength1)]
                                    for j in AIRPORT:
                                        if j!= ori and j != desti and i!=j:
                                            pathLength = TRIP[ori,i].distance + TRIP[i,j].distance + TRIP[j,desti].distance
                                            pathDuration = turnover_travel_timesteps[ori,i,p] + turnover_travel_timesteps[i,j,p] + turnover_travel_timesteps[j,desti,p]
                                            if pathLength <= maxPathLength:
                                                if pathDuration <= maxDuration:
                                                    for jj in AirportNum[p,j]:
                                                        possiblePaths[r,p] += [__REQUESTPATH__([(ori,oriInd),(i,ii),(j,jj),(desti,destiInd)],0,pathDuration,pathLength)]
    
        self.possiblePaths = possiblePaths
        
        model = cplex.Cplex()
        self.model = model
        
        
        number_of_variables = 0
        number_of_constraints = 0
        number_of_nonzeros = 0
        
        # VARIABLES
        
        x = {}
        self.x = x
        for r in REQUEST:
            for p in PLANE:
                for pathInd in range(len(possiblePaths[r,p])):
                    x[r,p,pathInd] = "x#" + r + "_" + p + "_" + str(pathInd)
                    model.variables.add(obj = [0], names = [x[r,p,pathInd]], types = ["B"])
                    number_of_variables += 1
        
        if full:
            r2 = {}
            self.r2 = r2
            for r in REQUEST:
                r2[r] = "r_ass" + r + "_"
                model.variables.add(names = [r2[r]], types = ["B"])
        
        y = {}
        self.y = y

         
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                      if n1 == AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1]:
                          y[i,j,p,n1,n2] = "y#" + i  + "_" + j  + "_" + p + "_" + str(n1) + "_" + str(n2)
                          model.variables.add(obj = [travelcost[i,j,p]], names = [y[i,j,p,n1,n2]], lb = [0], types = ["I"])
                          number_of_variables += 1
                      else:
                          y[i,j,p,n1,n2] = "y#" + i + "_" + j + "_" + p  + "_" + str(n1) + "_" + str(n2)
                          model.variables.add(obj = [travelcost[i,j,p]], names = [y[i,j,p,n1,n2]], lb = [0], types = ["B"])
                          number_of_variables += 1
                if full:
                    y[i,j,p] = "y#" + i + "_" + j + "_" + p
                    model.variables.add(names = [y[i,j,p]], types = ["I"])
        
        
        y_dep = {}
        self.y_dep = y_dep
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
              y_dep[p,n1] = "y_dep#" + p + str(n1)
              model.variables.add(names = [y_dep[p,n1]], lb = [0], ub = [1], types = ["B"])
              number_of_variables += 1
        
        y_arr = {}
        self.y_arr = y_arr
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].destination]:
              y_arr[p,n1] = "y_arr#" + p + str(n1)
              model.variables.add(names = [y_arr[p,n1]], lb = [0], ub = [1],types = ["B"])
              number_of_variables += 1
        
        f = {}
        self.f = f
        
        for i,j in TRIP0:
          for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                    f[i,j,p,n1,n2] = "f#" + i + "_" + j + "_" + p + "_" + str(n1) + "_" + str(n2)
                    model.variables.add(names = [f[i,j,p,n1,n2]], lb = [0], types = ["C"])
                    number_of_variables += 1
        
        f_dep = {}
        self.f_dep = {}
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
              f_dep[p,n1] = "f_dep#" + p + str(n1)
              #model.variables.add(names = [f_dep[p,n1]], lb = [PLANE[p].departure_min_fuel], ub = [PLANE[p].departure_max_fuel], types = ["C"])
              model.variables.add(names = [f_dep[p,n1]], lb = [0.0], ub = [PLANE[p].departure_max_fuel], types = ["C"])
              number_of_variables += 1
        
        f_arr = {}
        self.f_arr = f_arr
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].destination]:
              f_arr[p,n1] = "f_arr#" + p + str(n1)
              #model.variables.add(names = [f_arr[p,n1]], lb = [PLANE[p].arrival_min_fuel], ub = [PLANE[p].arrival_max_fuel], types = ["C"])
              model.variables.add(names = [f_arr[p,n1]], lb = [0.0], ub = [PLANE[p].arrival_max_fuel], types = ["C"])
              number_of_variables += 1
        
        d = {}
        self.d = d
        
        for i,j in TRIP0:
          for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                    d[i,j,p,n1,n2] = "t#" + i + str(n1) + "_" + j + str(n2) + "_" + p 
                    model.variables.add(names = [d[i,j,p,n1,n2]], lb = [0], types = ["C"])
                    number_of_variables += 1
        
        d_dep = {}
        self.d_dep = d_dep
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
              d_dep[p,n1] = "t_dep#" + p + str(n1)
              model.variables.add(names = [d_dep[p,n1]], lb = [0], ub = [plane_min_timestep[p]], types = ["C"])
              number_of_variables += 1
        
        d_arr = {}
        self.d_arr = d_arr
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].destination]:
              d_arr[p,n1] = "t_arr#" + p + str(n1)
              model.variables.add(names = [d_arr[p,n1]], lb = [0], ub = [plane_max_timestep[p]], types = ["C"])
              number_of_variables += 1
        
        w = {}
        self.w = w
        
        for i,j in TRIP0:
          for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                    w[i,j,p,n1,n2] = "w#" + i + str(n1) + "_" + j + str(n2) + "_" + p
                    model.variables.add(names = [w[i,j,p,n1,n2]], lb = [0], types = ["C"])
                    number_of_variables += 1
        
        self.name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
        
        # OBJECTIVE 
        
        model.objective.set_sense(model.objective.sense.minimize)
        
        # CONSTRAINTS
        
        if full:
            for i,j in TRIP0:
                for p in PLANE:
                    thevars = [ y[i,j,p] ]
                    thecoefs = [-1.0]
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                            thevars += [ y[i,j,p,n1,n2]]
                            thecoefs += [ 1.0  ]
                    
                    model.linear_constraints.add(names = ["set arc_" + i +  '_' + j + "_" + p], 
                                                         lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
        
    
        
        #lower airports first
        for p in PLANE:
            for j in AIRPORT:
                 for n1 in AirportNum[p,j]:
                     if n1 != AirportNum[p,j][-1]:
                         thevars = [y[i,j,p,n2,n1] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         model.linear_constraints.add(names = ["max_one_ingoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
                         number_of_constraints += 1
                         number_of_nonzeros += len(thevars)
                         
                         thevars = [y[j,i,p,n1,n2] for i in AIRPORT if (j,i) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         model.linear_constraints.add(names = ["max_one_outgoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
                         number_of_constraints += 1
                         number_of_nonzeros += len(thevars)
        
        for p in PLANE:
            for j in AIRPORT:
                 for n1ind,n1 in enumerate(AirportNum[p,j]):
                     if n1ind != 0:
                         thevars = [y[i,j,p,n2,n1] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         thevars += [y[j,i,p,AirportNum[p,j][n1ind-1],n2] for i in AIRPORT if (j,i) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs += [-5.0 for i in AIRPORT if (j,i) in TRIP0 for n2 in AirportNum[p,i]]
                         
                         model.linear_constraints.add(names = ["bounded by previous outgoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                         number_of_constraints += 1
                         number_of_nonzeros += len(thevars)
                         
                         
                         thevars = [y[j,i,p,n1,n2] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         thevars += [y[j,i,p,AirportNum[p,j][n1ind-1],n2] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs += [-5.0 for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         
                         model.linear_constraints.add(names = ["bounded by previous outgoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                         number_of_constraints += 1
                         number_of_nonzeros += len(thevars)
                         
                         if cuttingPlanes.has_key('time_order_cut'):
                             for i in AIRPORT:
                                 if (i,j) in TRIP0:
                                     for n2 in AirportNum[p,i]:
                                         thevars = [d[j,i,p,n1,n2],y[j,i,p,n1,n2]]
                                         thecoefs = [1.0,-plane_max_timestep[p]]
                                         
                                         thevars += [d[j,k,p,AirportNum[p,j][n1ind-1],n2] for k in AIRPORT if (k,j) in TRIP0 for n2 in AirportNum[p,k]]
                                         thecoefs += [-1.0 for k in AIRPORT if (k,j) in TRIP0 for n2 in AirportNum[p,k]]
                                         
                                         model.linear_constraints.add(names = ["time bounded by previous outgoing" + p + "_" + j + str(n1)], 
                                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                                               senses = ["G"], rhs = [min_return_time[i,p]-plane_max_timestep[p]])
                                         number_of_constraints += 1
                                         number_of_nonzeros += len(thevars)
                        
        for p in PLANE:
            if len(AirportNum[p,PLANE[p].origin]) > 1:
                thevars = [y[i,PLANE[p].origin,p,n2,1] for i in AIRPORT if (i,PLANE[p].origin) in TRIP0 for n2 in AirportNum[p,i]]
                thecoefs = [1.0]*len(thevars)
                
                model.linear_constraints.add(names = ["no return to start" + p ],lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
                thevars = [y_dep[p,n1],f_dep[p,n1]]
                thecoefs = [-PLANE[p].departure_min_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fueldep_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
                
                thevars = [y_dep[p,n1],f_dep[p,n1]]
                thecoefs = [-PLANE[p].departure_max_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fueldep2_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
                
                thevars = [y_dep[p,n1],d_dep[p,n1]]
                thecoefs = [-plane_min_timestep[p],1.0]
                
                model.linear_constraints.add(names = ["plane_timedep_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
                
                thevars = [y_dep[p,n1],d_dep[p,n1]]
                thecoefs = [-plane_min_timestep[p],1.0]
                
                model.linear_constraints.add(names = ["plane_timedep2_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
            
            for n1 in AirportNum[p,PLANE[p].destination]:
                thevars = [y_arr[p,n1],f_arr[p,n1]]
                thecoefs = [-PLANE[p].arrival_min_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fuelarr_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
                
                thevars = [y_arr[p,n1],f_arr[p,n1]]
                thecoefs = [-PLANE[p].arrival_max_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fuelarr2_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
                
                thevars = [y_arr[p,n1],d_arr[p,n1]]
                thecoefs = [-plane_max_timestep[p],1.0]
                
                model.linear_constraints.add(names = ["plane_timearr_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
        
        # for each request a path must be chosen
        if full:
            for r in REQUEST:
                thevars = []
                thecoefs = []
                for p in PLANE:
                    for pathInd in range(len(possiblePaths[r,p])):
                        thevars.append(x[r,p,pathInd])
                        thecoefs.append(1.0)
                thevars += [r2[r]]
                thecoefs += [-1.0]
                model.linear_constraints.add(names = ["request_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
        else:
            for r in REQUEST:
                thevars = []
                thecoefs = []
                for p in PLANE:
                    for pathInd in range(len(possiblePaths[r,p])):
                        thevars.append(x[r,p,pathInd])
                        thecoefs.append(1.0)
                
                model.linear_constraints.add(names = ["request_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
                number_of_constraints += 1
                number_of_nonzeros += len(thevars)
        
        
        
        # airplane flow
        
        for p in PLANE:
            thevars = []
            thecoefs = []
            for n1 in AirportNum[p,PLANE[p].origin]:
                thevars.append(y_dep[p,n1])
                thecoefs.append(1.0)
            
            model.linear_constraints.add(names = ["plane_one_dep_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
            number_of_constraints += 1
            number_of_nonzeros += len(thevars)
        
        
        for p in PLANE:
          thevars = []
          thecoefs = []
          for n1 in AirportNum[p,PLANE[p].destination]:
            thevars.append(y_arr[p,n1])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
                
          model.linear_constraints.add(names = ["plane_one_arr_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
          number_of_constraints += 1
        
        for p in PLANE:
          for j in AIRPORT:
              for n2 in AirportNum[p,j]:
                #print p,i
                rhs_value = 0.0
                thevars = []
                thecoefs = []
                    
                for i in AIRPORT:
                  if (i,j) in TRIP0:
                      for n1 in AirportNum[p,i]:
                        thevars.append(y[i,j,p,n1,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                    
                if j == PLANE[p].plane_departure:
                    thevars.append(y_dep[p,n2])
                    #rhs_value += -1.0
                    thecoefs.append(1.0)
                    number_of_nonzeros += 1
                  
                for k in AIRPORT:
                  if (j,k) in TRIP0:
                      for n1 in AirportNum[p,k]:
                        thevars.append(y[j,k,p,n2,n1])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                
                if j == PLANE[p].plane_arrival:
                  #rhs_value += 1.0
                  thevars.append(y_arr[p,n2])
                  thecoefs.append(-1.0)
                  number_of_nonzeros += 1
                
                model.linear_constraints.add(names = ["plane_flow_" + p + "_" + j + str(n2)], 
                                                      lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [rhs_value])
                number_of_constraints += 1
        
        
        # seat limit
        
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]:
                        thevars = [y[i,j,p,n1,n2]]
                        thecoefs = [-PLANE[p].seats]
                        number_of_nonzeros += 1
                        for r in REQUEST:
                            thevars+=[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]
                            thecoefs += [REQUEST[r].passengers for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]
                          
                        model.linear_constraints.add(names = ["seatlimit_"  + p + "_" + i + str(n1) + "_" + j + str(n2)], 
                                                              lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                        number_of_constraints += 1
        
        # fueling constraints
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    #print i,j,p
                    thevars = [f[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                    thecoefs = [1.0,-max_trip_fuel[i,j,p]]
                    number_of_nonzeros += 2
                    
                    model.linear_constraints.add(names = ["noflight_nofuel_" + i + str(n1) + "_" + j + "_" + p + str(n2)], 
                                                          lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        
        for j in AIRPORT:
            for p in PLANE:
                for n2 in AirportNum[p,j]:
                    thevars = []
                    thecoefs = []
                            
                    for i in AIRPORT:
                        if (i,j) in TRIP0:
                          for n1 in AirportNum[p,i]:
                            thevars.append(f[i,j,p,n1,n2])
                            thecoefs.append(1.0)
                            number_of_nonzeros += 1
                            
                    if j == PLANE[p].plane_departure:
                        thevars.append(f_dep[p,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                            
                    for k in AIRPORT:
                        if (j,k) in TRIP0:
                            for n1 in AirportNum[p,k]:
                                thevars.append(f[j,k,p,n2,n1])
                                thecoefs.append(-1.0)
                                number_of_nonzeros += 1
                                
                                thevars.append(y[j,k,p,n2,n1])
                                thecoefs.append(-fuelconsumption[j,k,p])
                                number_of_nonzeros += 1
                            
                    if j == PLANE[p].plane_arrival:
                        thevars.append(f_arr[p,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                    
                    if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '0':
                        model.linear_constraints.add(names = ["fuelconsumption_" + j + "_" + p + str(n2)], 
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                        number_of_constraints += 1
                    else:
                        model.linear_constraints.add(names = ["refueling_" + j + "_" + p + str(n2)], 
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                        number_of_constraints += 1
        
        # time constraints
        
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    #print i,j,p
                    thevars = [d[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                    thecoefs = [1.0,-plane_max_timestep[p]]
                    number_of_nonzeros += 2
                    
                    model.linear_constraints.add(names = ["noflight_notime_" + i + str(n1) + "_" + j + str(n2) + "_" + p], 
                                                 lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]:
                        thevars = [d[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                        thecoefs = [1.0,-plane_min_timestep[p]]
                        
                        model.linear_constraints.add(names = ["aflight_atime_" + i + str(n1) + "_" + j + str(n2) + "_" + p], 
                                                     lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                        
                        number_of_constraints += 1
                        number_of_nonzeros += len(thevars)
        
        for j in AIRPORT:
            for p in PLANE:
                for n2 in AirportNum[p,j]:
                    thevars = []
                    thecoefs = []
                    
                    for i in AIRPORT:
                        if (i,j) in TRIP0:
                            for n1 in AirportNum[p,i]:
                                thevars.append(d[i,j,p,n1,n2])
                                thecoefs.append(1.0)
                                number_of_nonzeros += 1
                                
                                thevars.append(y[i,j,p,n1,n2])
                                thecoefs.append(turnover_travel_timesteps[i,j,p])
                                number_of_nonzeros += 1
                        
                    if j == PLANE[p].plane_departure:
                        thevars.append(d_dep[p,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                            
                    for k in AIRPORT:
                        if (j,k) in TRIP0:
                            for n1 in AirportNum[p,k]:
                                thevars.append(d[j,k,p,n2,n1])
                                thecoefs.append(-1.0)
                                number_of_nonzeros += 1
                    
                    if j == PLANE[p].plane_arrival:
                        thevars.append(d_arr[p,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                    
                    model.linear_constraints.add(names = ["timeconsumption_" + j + str(n2) + "_" + p], 
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        for r in REQUEST:
            for p in PLANE:
                for i,j in REQUEST_TRIP0[r]:
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                            thevars = [d[i,j,p,n1,n2]]+[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]
                            thecoefs = [1]+[-plane_max_timestep[p]]*(len(thevars)-1)
                            rhs = earliest_departure_timesteps[r]-plane_max_timestep[p]-max_turnover_timesteps[r]
                            
                                    
                            model.linear_constraints.add(names = ["timewindow2_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                         lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [rhs])
                            
                            number_of_constraints += 1
                            number_of_nonzeros += len(thevars)
                            if not( n1==AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1] ):
                                thecoefs = [1]+[plane_max_timestep[p]]*(len(thevars)-1)
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]#+turnover_timesteps[i,p]#TODO: Verify this timewindow
                                        
                                model.linear_constraints.add(names = ["timewindow1_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                
                                number_of_constraints += 1
                                number_of_nonzeros += len(thevars)
                            else:
                                thevars = [d[i,j,p,n1,n2]]+[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]+[y[i,j,p,n1,n2]]
                                thecoefs = [1]+[2*plane_max_timestep[p]]*(len(thevars)-2)+[-plane_max_timestep[p]]
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]#+turnover_timesteps[i,p]#TODO: Verify this timewindow
                                        
                                model.linear_constraints.add(names = ["timewindow1_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                number_of_constraints += 1
                                number_of_nonzeros += len(thevars)
        
        # weight limit (=max fuel)
        
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]:
                        thevars = [w[i,j,p,n1,n2]]
                        thecoefs = [1.0]
                        number_of_nonzeros += 1
                        
                        for r in REQUEST:
                            for pathInd in range(len(possiblePaths[r,p])):
                                if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)]):
                                    thevars.append(x[r,p,pathInd])
                                    thecoefs.append(-REQUEST[r].weight)
                                    number_of_nonzeros += 1
                        
                        thevars.append(f[i,j,p,n1,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                            
                        model.linear_constraints.add(names = ["computeweight_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                              lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                        number_of_constraints += 1
        
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    #print i,j,p
                    thevars = [w[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                    thecoefs = [1.0,-max_trip_payload[i,j,p]]
                    number_of_nonzeros += 2
                
                    model.linear_constraints.add(names = ["weightlimit_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                 lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        
        
        # minimum number of fuelstops
        if cuttingPlanes.has_key('fuelstop_cut'):
            for p in PLANE:
              if PLANE[p].departure_max_fuel - fuelconsumption[PLANE[p].plane_departure,PLANE[p].plane_arrival,p] < PLANE[p].arrival_min_fuel:
                #print p
                thevars = []
                thecoefs = []
            
                for i,j in TRIP0:
                    if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '1':
                        for n1 in AirportNum[p,i]:
                            for n2 in AirportNum[p,j]:
                                thevars.append(y[i,j,p,n1,n2])
                                thecoefs.append(1.0)
                                number_of_nonzeros += 1
                    
                model.linear_constraints.add(names = ["minfuelstops_in_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [1.0])
                number_of_constraints += 1
            
            for p in PLANE:    
              if PLANE[p].departure_max_fuel - fuelconsumption[PLANE[p].plane_departure,PLANE[p].plane_arrival,p] < PLANE[p].arrival_min_fuel:
                #print p
                thevars = []
                thecoefs = []
            
                for i,j in TRIP0:
                  if AIRPORT[i].fuel[PLANE[p].required_fueltype] == '1':
                      for n1 in AirportNum[p,i]:
                          for n2 in AirportNum[p,j]:
                            thevars.append(y[i,j,p,n1,n2])
                            thecoefs.append(1.0)
                            number_of_nonzeros += 1
                    
                model.linear_constraints.add(names = ["minfuelstops_out_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [1.0])
                number_of_constraints += 1
           
        
        # maximum number of arrivals/departures per airport
        if cuttingPlanes.has_key('max_visit_cut'):
            for i in AIRPORT:
              anyfuel = 0
              for ft in AIRPORT[i].fuel:
                if AIRPORT[i].fuel[ft] == '1':
                  anyfuel += 1
                  
              if anyfuel == 0:
                thevars = []
                thecoefs = []
              
                for j in AIRPORT:
                  if (i,j) in TRIP0:
                      for p in PLANE:
                        for n1 in AirportNum[p,i]:
                            for n2 in AirportNum[p,j]:
                            
                              thevars.append(y[i,j,p,n1,n2])
                              thecoefs.append(1.0)
                              number_of_nonzeros += 1
            
                rhs_value = 0
                
                for r in REQUEST:
                  if REQUEST[r].request_departure == i or REQUEST[r].request_arrival == i:
                    rhs_value += 1
                
                for p in PLANE:
                  if PLANE[p].plane_departure == i:
                    rhs_value += 1
                
                model.linear_constraints.add(names = ["maxpickup_out_" + i], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
                number_of_constraints += 1
              else:
                thevars = []
                thecoefs = []
              
                for j in AIRPORT:
                  if (i,j) in TRIP0:
                      for p in PLANE:
                        for n1 in AirportNum[p,i]:
                            for n2 in AirportNum[p,j]:
                            
                              thevars.append(y[i,j,p,n1,n2])
                              thecoefs.append(1.0)
                              number_of_nonzeros += 1
            
                rhs_value = 3
                
                #model.linear_constraints.add(names = ["maxpickup_out_" + i], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
                number_of_constraints += 1
            
            for j in AIRPORT:
              anyfuel = 0
              for ft in AIRPORT[j].fuel:
                if AIRPORT[j].fuel[ft] == '1':
                  anyfuel += 1
                  
              if anyfuel == 0:
                thevars = []
                thecoefs = []
              
                for i in AIRPORT:
                  if (i,j) in TRIP0:
                      for p in PLANE:
                        for n1 in AirportNum[p,i]:
                            for n2 in AirportNum[p,j]:
                            
                              thevars.append(y[i,j,p,n1,n2])
                              thecoefs.append(1.0)
                              number_of_nonzeros += 1
            
                rhs_value = 0
                
                for r in REQUEST:
                  if REQUEST[r].request_departure == j or REQUEST[r].request_arrival == j:
                    rhs_value += 1
                
                for p in PLANE:
                  if PLANE[p].plane_arrival == j:
                    rhs_value += 1
                    
                model.linear_constraints.add(names = ["maxpickup_in_" + j], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
                number_of_constraints += 1
              else:
                thevars = []
                thecoefs = []
              
                for i in AIRPORT:
                  if (i,j) in TRIP0:
                      for p in PLANE:
                        for n1 in AirportNum[p,i]:
                            for n2 in AirportNum[p,j]:
                            
                              thevars.append(y[i,j,p,n1,n2])
                              thecoefs.append(1.0)
                              number_of_nonzeros += 1
            
                rhs_value = 3
                    
                #model.linear_constraints.add(names = ["maxpickup_in_" + j], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
                number_of_constraints += 1
        
        #a copy has to be visited for fuel supply or to pickup / deliver a request
        if cuttingPlanes.has_key('useless_detour_cut'):
            for p in PLANE:
                for j in AIRPORT:
                    if AIRPORT[j].fuel[PLANE[p].required_fueltype] != '1' and j != PLANE[p].destination:
                        for n2 in AirportNum[p,j]:
                            for i in AIRPORT:
                                if (i,j) in TRIP0:
                                    for n1 in AirportNum[p,i]:
                                        thevars = [y[i,j,p,n1,n2]] + [x[r,p,pathInd] for r in REQUEST if REQUEST[r].destination == j 
                                                    for pathInd in range(len(possiblePaths[r,p])) 
                                                    if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                            x[r,p,pathInd] for r in REQUEST if REQUEST[r].origin == j 
                                                            for pathInd in range(len(possiblePaths[r,p]))
                                                            if possiblePaths[r,p][pathInd].v[0][1] == n2
                                                            ]
                                        
                                        thecoefs = [1.0] + [-1.0 for r in REQUEST if REQUEST[r].destination == j 
                                                    for pathInd in range(len(possiblePaths[r,p])) 
                                                    if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                            -1.0 for r in REQUEST if REQUEST[r].origin == j 
                                                            for pathInd in range(len(possiblePaths[r,p]))
                                                            if possiblePaths[r,p][pathInd].v[0][1] == n2
                                                            ]
                                        
                                        model.linear_constraints.add(names = ["musthaverequest_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                     lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
            
            
            
            for p in PLANE:
                for i in AIRPORT:
                    if AIRPORT[i].fuel[PLANE[p].required_fueltype] != '1':
                        for n1 in AirportNum[p,i]:
                            if not (i == PLANE[p].origin and n1==1):
                                for j in AIRPORT:
                                    if (i,j) in TRIP0:
                                        for n2 in AirportNum[p,j]:
                                            thevars = [y[i,j,p,n1,n2]] + [x[r,p,pathInd] for r in REQUEST if REQUEST[r].origin == i 
                                                        for pathInd in range(len(possiblePaths[r,p])) 
                                                        if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                                x[r,p,pathInd] for r in REQUEST if REQUEST[r].destination == i 
                                                                for pathInd in range(len(possiblePaths[r,p]))
                                                                if possiblePaths[r,p][pathInd].v[-1][1] == n1
                                                                ]
                                            
                                            thecoefs = [1.0] + [-1.0 for r in REQUEST if REQUEST[r].origin == i 
                                                        for pathInd in range(len(possiblePaths[r,p])) 
                                                        if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                                -1.0 for r in REQUEST if REQUEST[r].destination == i 
                                                                for pathInd in range(len(possiblePaths[r,p]))
                                                                if possiblePaths[r,p][pathInd].v[-1][1] == n1
                                                                ]
                                            
                                            model.linear_constraints.add(names = ["musthaverequest_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                         lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])                        
                
            
        # minimum amount of fuel for detour to refueling airport
        if cuttingPlanes.has_key('min_fuel_cut'):
            for i,j in TRIP0:
                for p in PLANE:
                  for n1 in AirportNum[p,i]:
                      for n2 in AirportNum[p,j]:
                      
                        thevars = [y[i,j,p,n1,n2],f[i,j,p,n1,n2]]
                        thecoefs = [min_refuel_trip[j,p],-1.0]
                        number_of_nonzeros += 2
                        
                        model.linear_constraints.add(names = ["minfuel_" + i + str(n1) + "_" + j + str(n2) + "_" + p], 
                                                              lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0])
                        number_of_constraints += 1
         
        model = cplex.Cplex ()
        self.model = model
        # VARIABLES
        timeflow = 1
        x = {}
        number_of_variables = 0
        
        for r in REQUEST:
            for p in PLANE:
                for pathInd in range(len(possiblePaths[r,p])):
                    x[r,p,pathInd] = "x#" + r + "_" + p + "_" + str(pathInd)
                    model.variables.add(obj = [0], names = [x[r,p,pathInd]], types = ["B"])
                    number_of_variables += 1
        
        y = {}
        
        
        for i,j in TRIP0:
          for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                      if n1 == AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1]:
                          y[i,j,p,n1,n2] = "y#" + i  + "_" + j  + "_" + p + "_" + str(n1) + "_" + str(n2)
                          model.variables.add(obj = [travelcost[i,j,p]], names = [y[i,j,p,n1,n2]], lb = [0], types = ["I"])
                          number_of_variables += 1
                      else:
                          y[i,j,p,n1,n2] = "y#" + i + "_" + j + "_" + p  + "_" + str(n1) + "_" + str(n2)
                          model.variables.add(obj = [travelcost[i,j,p]], names = [y[i,j,p,n1,n2]], lb = [0], types = ["B"])
                          number_of_variables += 1
        
        y_dep = {}
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
              y_dep[p,n1] = "y_dep#" + p + str(n1)
              model.variables.add(names = [y_dep[p,n1]], lb = [0], ub = [1], types = ["B"])
              number_of_variables += 1
        
        y_arr = {}
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].destination]:
              y_arr[p,n1] = "y_arr#" + p + str(n1)
              model.variables.add(names = [y_arr[p,n1]], lb = [0], ub = [1],types = ["B"])
              number_of_variables += 1
        
        f = {}
        
        for i,j in TRIP0:
          for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                    f[i,j,p,n1,n2] = "f#" + i + "_" + j + "_" + p + "_" + str(n1) + "_" + str(n2)
                    model.variables.add(names = [f[i,j,p,n1,n2]], lb = [0], types = ["C"])
                    number_of_variables += 1
        
        f_dep = {}
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
              f_dep[p,n1] = "f_dep#" + p + str(n1)
              #model.variables.add(names = [f_dep[p,n1]], lb = [PLANE[p].departure_min_fuel], ub = [PLANE[p].departure_max_fuel], types = ["C"])
              model.variables.add(names = [f_dep[p,n1]], lb = [0.0], ub = [PLANE[p].departure_max_fuel], types = ["C"])
              number_of_variables += 1
        
        f_arr = {}
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].destination]:
              f_arr[p,n1] = "f_arr#" + p + str(n1)
              #model.variables.add(names = [f_arr[p,n1]], lb = [PLANE[p].arrival_min_fuel], ub = [PLANE[p].arrival_max_fuel], types = ["C"])
              model.variables.add(names = [f_arr[p,n1]], lb = [0.0], ub = [PLANE[p].arrival_max_fuel], types = ["C"])
              number_of_variables += 1
        
        if timeflow:
            d = {}
            
            for i,j in TRIP0:
              for p in PLANE:
                  for n1 in AirportNum[p,i]:
                      for n2 in AirportNum[p,j]:
                        d[i,j,p,n1,n2] = "t#" + i + str(n1) + "_" + j + str(n2) + "_" + p 
                        model.variables.add(names = [d[i,j,p,n1,n2]], lb = [0], types = ["C"])
                        number_of_variables += 1
            
            d_dep = {}
            
            for p in PLANE:
                for n1 in AirportNum[p,PLANE[p].origin]:
                  d_dep[p,n1] = "t_dep#" + p + str(n1)
                  model.variables.add(names = [d_dep[p,n1]], lb = [0], ub = [plane_min_timestep[p]], types = ["C"])
                  number_of_variables += 1
            
            d_arr = {}
            
            for p in PLANE:
                for n1 in AirportNum[p,PLANE[p].destination]:
                  d_arr[p,n1] = "t_arr#" + p + str(n1)
                  model.variables.add(names = [d_arr[p,n1]], lb = [0], ub = [plane_max_timestep[p]], types = ["C"])
                  number_of_variables += 1
        
        w = {}
        
        for i,j in TRIP0:
          for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                    w[i,j,p,n1,n2] = "w#" + i + str(n1) + "_" + j + str(n2) + "_" + p
                    model.variables.add(names = [w[i,j,p,n1,n2]], lb = [0], types = ["C"])
                    number_of_variables += 1
        
        print "number of variables: ",number_of_variables
        
        name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
        x_indices = [name2idx[name] for name in x.values() ]
        y_indices = [ name2idx[name] for name in y.values()]
        cb_indices = x_indices+y_indices
        # OBJECTICE 
        
        model.objective.set_sense(model.objective.sense.minimize)
        
        
        # CONSTRAINTS
        
        number_of_constraints = 0
        number_of_nonzeros = 0
        
        #lower airports first
        for p in PLANE:
            for j in AIRPORT:
                 for n1 in AirportNum[p,j]:
                     if n1 != AirportNum[p,j][-1]:
                         thevars = [y[i,j,p,n2,n1] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         model.linear_constraints.add(names = ["max_one_ingoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
                         number_of_constraints += 1
                
                         thevars = [y[j,i,p,n1,n2] for i in AIRPORT if (j,i) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         model.linear_constraints.add(names = ["max_one_outgoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
                         number_of_constraints += 1
        
        for p in PLANE:
            for j in AIRPORT:
                 for n1ind,n1 in enumerate(AirportNum[p,j]):
                     if n1ind != 0:
                         thevars = [y[i,j,p,n2,n1] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         thevars += [y[j,i,p,AirportNum[p,j][n1ind-1],n2] for i in AIRPORT if (j,i) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs += [-5.0 for i in AIRPORT if (j,i) in TRIP0 for n2 in AirportNum[p,i]]
                         
                         model.linear_constraints.add(names = ["bounded by previous outgoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                         number_of_constraints += 1
                         
                         thevars = [y[j,i,p,n1,n2] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs = [1.0]*len(thevars)
                         
                         thevars += [y[j,i,p,AirportNum[p,j][n1ind-1],n2] for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         thecoefs += [-5.0 for i in AIRPORT if (i,j) in TRIP0 for n2 in AirportNum[p,i]]
                         
                         model.linear_constraints.add(names = ["bounded by previous outgoing" + p + "_" + j + str(n1)], 
                                                               lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                         number_of_constraints += 1
                         #"""
                         for i in AIRPORT:
                             if (i,j) in TRIP0:
                                 for n2 in AirportNum[p,i]:
                                     thevars = [d[j,i,p,n1,n2],y[j,i,p,n1,n2]]
                                     thecoefs = [1.0,-plane_max_timestep[p]]
                                     
                                     thevars += [d[j,k,p,AirportNum[p,j][n1ind-1],n2] for k in AIRPORT if (k,j) in TRIP0 for n2 in AirportNum[p,k]]
                                     thecoefs += [-1.0 for k in AIRPORT if (k,j) in TRIP0 for n2 in AirportNum[p,k]]
                                     
                                     model.linear_constraints.add(names = ["time bounded by previous outgoing" + p + "_" + j + str(n1)], 
                                                                           lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                                           senses = ["G"], rhs = [min_return_time[i,p]-plane_max_timestep[p]])
                                     number_of_constraints += 1
                        
                        #"""
        for p in PLANE:
            if len(AirportNum[p,PLANE[p].origin]) > 1:
                thevars = [y[i,PLANE[p].origin,p,n2,1] for i in AIRPORT if (i,PLANE[p].origin) in TRIP0 for n2 in AirportNum[p,i]]
                thecoefs = [1.0]*len(thevars)
                
                model.linear_constraints.add(names = ["no return to start" + p ],lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
        
        
        for p in PLANE:
            for n1 in AirportNum[p,PLANE[p].origin]:
                thevars = [y_dep[p,n1],f_dep[p,n1]]
                thecoefs = [-PLANE[p].departure_min_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fueldep_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                number_of_constraints += 1
                
                thevars = [y_dep[p,n1],f_dep[p,n1]]
                thecoefs = [-PLANE[p].departure_max_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fueldep2_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                
                if timeflow:
                    thevars = [y_dep[p,n1],d_dep[p,n1]]
                    thecoefs = [-plane_min_timestep[p],1.0]
                    
                    model.linear_constraints.add(names = ["plane_timedep_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                    number_of_constraints += 1
                    
                    thevars = [y_dep[p,n1],d_dep[p,n1]]
                    thecoefs = [-plane_min_timestep[p],1.0]
                    
                    model.linear_constraints.add(names = ["plane_timedep2_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
            
            for n1 in AirportNum[p,PLANE[p].destination]:
                thevars = [y_arr[p,n1],f_arr[p,n1]]
                thecoefs = [-PLANE[p].arrival_min_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fuelarr_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
                number_of_constraints += 1
                
                thevars = [y_arr[p,n1],f_arr[p,n1]]
                thecoefs = [-PLANE[p].arrival_max_fuel,1.0]
                
                model.linear_constraints.add(names = ["plane_fuelarr2_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                number_of_constraints += 1
                
                if timeflow:
                    thevars = [y_arr[p,n1],d_arr[p,n1]]
                    thecoefs = [-plane_max_timestep[p],1.0]
                    
                    model.linear_constraints.add(names = ["plane_timearr_" + p + str(n1)], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        # for each request a path must be chosen
        
        for r in REQUEST:
            thevars = []
            thecoefs = []
            for p in PLANE:
                for pathInd in range(len(possiblePaths[r,p])):
                    thevars.append(x[r,p,pathInd])
                    thecoefs.append(1.0)
                    number_of_nonzeros += 1      
            model.linear_constraints.add(names = ["request_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
            number_of_constraints += 1
        
        
        
        # airplane flow
        
        for p in PLANE:
          thevars = []
          thecoefs = []
          for n1 in AirportNum[p,PLANE[p].origin]:
            thevars.append(y_dep[p,n1])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
                
          model.linear_constraints.add(names = ["plane_one_dep_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
          number_of_constraints += 1
        
        
        for p in PLANE:
          thevars = []
          thecoefs = []
          for n1 in AirportNum[p,PLANE[p].destination]:
            thevars.append(y_arr[p,n1])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
                
          model.linear_constraints.add(names = ["plane_one_arr_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
          number_of_constraints += 1
        
        for p in PLANE:
          for j in AIRPORT:
              for n2 in AirportNum[p,j]:
                #print p,i
                rhs_value = 0.0
                thevars = []
                thecoefs = []
                    
                for i in AIRPORT:
                  if (i,j) in TRIP0:
                      for n1 in AirportNum[p,i]:
                        thevars.append(y[i,j,p,n1,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                    
                if j == PLANE[p].plane_departure:
                    thevars.append(y_dep[p,n2])
                    #rhs_value += -1.0
                    thecoefs.append(1.0)
                    number_of_nonzeros += 1
                  
                for k in AIRPORT:
                  if (j,k) in TRIP0:
                      for n1 in AirportNum[p,k]:
                        thevars.append(y[j,k,p,n2,n1])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                
                if j == PLANE[p].plane_arrival:
                  #rhs_value += 1.0
                  thevars.append(y_arr[p,n2])
                  thecoefs.append(-1.0)
                  number_of_nonzeros += 1
                
                model.linear_constraints.add(names = ["plane_flow_" + p + "_" + j + str(n2)], 
                                                      lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [rhs_value])
                number_of_constraints += 1
        
        
        # seat limit
        
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]:
                        thevars = [y[i,j,p,n1,n2]]
                        thecoefs = [-PLANE[p].seats]
                        number_of_nonzeros += 1
                        for r in REQUEST:
                            thevars+=[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]
                            thecoefs += [REQUEST[r].passengers for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]
                          
                        model.linear_constraints.add(names = ["seatlimit_"  + p + "_" + i + str(n1) + "_" + j + str(n2)], 
                                                              lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                        number_of_constraints += 1
        
        # fueling constraints
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    #print i,j,p
                    thevars = [f[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                    thecoefs = [1.0,-max_trip_fuel[i,j,p]]
                    number_of_nonzeros += 2
                    
                    model.linear_constraints.add(names = ["noflight_nofuel_" + i + str(n1) + "_" + j + "_" + p + str(n2)], 
                                                          lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        
        for j in AIRPORT:
            for p in PLANE:
                for n2 in AirportNum[p,j]:
                    thevars = []
                    thecoefs = []
                            
                    for i in AIRPORT:
                        if (i,j) in TRIP0:
                          for n1 in AirportNum[p,i]:
                            thevars.append(f[i,j,p,n1,n2])
                            thecoefs.append(1.0)
                            number_of_nonzeros += 1
                            
                    if j == PLANE[p].plane_departure:
                        thevars.append(f_dep[p,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                            
                    for k in AIRPORT:
                        if (j,k) in TRIP0:
                            for n1 in AirportNum[p,k]:
                                thevars.append(f[j,k,p,n2,n1])
                                thecoefs.append(-1.0)
                                number_of_nonzeros += 1
                                
                                thevars.append(y[j,k,p,n2,n1])
                                thecoefs.append(-fuelconsumption[j,k,p])
                                number_of_nonzeros += 1
                            
                    if j == PLANE[p].plane_arrival:
                        thevars.append(f_arr[p,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                    
                    if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '0':
                        model.linear_constraints.add(names = ["fuelconsumption_" + j + "_" + p + str(n2)], 
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                        number_of_constraints += 1
                    else:
                        model.linear_constraints.add(names = ["refueling_" + j + "_" + p + str(n2)], 
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                        number_of_constraints += 1
        
        # time constraints
        
        if timeflow:
            for i,j in TRIP0:
                for p in PLANE:
                  for n1 in AirportNum[p,i]:
                      for n2 in AirportNum[p,j]:
                      
                        #print i,j,p
                        thevars = [d[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                        thecoefs = [1.0,-plane_max_timestep[p]]
                        number_of_nonzeros += 2
                        
                        model.linear_constraints.add(names = ["noflight_notime_" + i + str(n1) + "_" + j + str(n2) + "_" + p], 
                                                     lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                        number_of_constraints += 1
            
            for i,j in TRIP0:
                for p in PLANE:
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                          
                            #print i,j,p
                            thevars = [d[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                            thecoefs = [1.0,-plane_min_timestep[p]]
                            
                            model.linear_constraints.add(names = ["aflight_atime_" + i + str(n1) + "_" + j + str(n2) + "_" + p], 
                                                         lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [0.0])
            
            
            for j in AIRPORT:
                for p in PLANE:
                    for n2 in AirportNum[p,j]:
                        thevars = []
                        thecoefs = []
                        
                        for i in AIRPORT:
                            if (i,j) in TRIP0:
                                for n1 in AirportNum[p,i]:
                                    thevars.append(d[i,j,p,n1,n2])
                                    thecoefs.append(1.0)
                                    number_of_nonzeros += 1
                                    
                                    thevars.append(y[i,j,p,n1,n2])
                                    thecoefs.append(turnover_travel_timesteps[i,j,p])
                                    number_of_nonzeros += 1
                            
                        if j == PLANE[p].plane_departure:
                            thevars.append(d_dep[p,n2])
                            thecoefs.append(1.0)
                            number_of_nonzeros += 1
                                
                        for k in AIRPORT:
                            if (j,k) in TRIP0:
                                for n1 in AirportNum[p,k]:
                                    thevars.append(d[j,k,p,n2,n1])
                                    thecoefs.append(-1.0)
                                    number_of_nonzeros += 1
                        
                        if j == PLANE[p].plane_arrival:
                            thevars.append(d_arr[p,n2])
                            thecoefs.append(-1.0)
                            number_of_nonzeros += 1
                        
                        model.linear_constraints.add(names = ["timeconsumption_" + j + str(n2) + "_" + p], 
                                                                lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                        number_of_constraints += 1
            
            for r in REQUEST:
                for p in PLANE:
                    for i,j in REQUEST_TRIP0[r]:
                        for n1 in AirportNum[p,i]:
                            for n2 in AirportNum[p,j]:
                                thevars = [d[i,j,p,n1,n2]]+[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]
                                thecoefs = [1]+[-plane_max_timestep[p]]*(len(thevars)-1)
                                rhs = earliest_departure_timesteps[r]-plane_max_timestep[p]-max_turnover_timesteps[r]
                                number_of_nonzeros += 2
                                        
                                model.linear_constraints.add(names = ["timewindow2_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [rhs])
                                
                                number_of_constraints += 1
                                if not( n1==AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1] ):
                                    thecoefs = [1]+[plane_max_timestep[p]]*(len(thevars)-1)
                                    rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]#+turnover_timesteps[i,p]#TODO: Verify this timewindow
                                    number_of_nonzeros += 2
                                            
                                    model.linear_constraints.add(names = ["timewindow1_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                                 lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                    
                                    number_of_constraints += 1
                                else:
                                    thevars = [d[i,j,p,n1,n2]]+[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]+[y[i,j,p,n1,n2]]
                                    thecoefs = [1]+[2*plane_max_timestep[p]]*(len(thevars)-2)+[-plane_max_timestep[p]]
                                    rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]#+turnover_timesteps[i,p]#TODO: Verify this timewindow
                                    number_of_nonzeros += 2
                                            
                                    model.linear_constraints.add(names = ["timewindow1_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                                 lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])

            """
            for r in REQUEST:
                for p in PLANE:
                    for pathInd in range(len(possiblePaths[r,p])):
                        path=possiblePaths[r,p][pathInd]
                        for vertNum in range(len(path.v)-1):
                            arcTail=path.v[vertNum]
                            arcHead=path.v[vertNum+1]
                            if not( arcTail[1] == AirportNum[p,arcTail[0]][-1] and arcHead[1] == AirportNum[p,arcHead[0]][-1] ):
                                thevars = [d[arcTail[0],arcHead[0],p,arcTail[1],arcHead[1]],x[r,p,pathInd]]
                                thecoefs = [1,plane_max_timestep[p]]
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[arcTail[0],arcHead[0],p]#+turnover_timesteps[i,p]#TODO: Verify this timewindow
                                number_of_nonzeros += 2
                                        
                                model.linear_constraints.add(names = ["timewindow1_" + "_" + r + "_" + p + "_" +str(pathInd) + "_" + str(vertNum)], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                
                                number_of_constraints += 1
                            else:
                                thevars = [d[arcTail[0],arcHead[0],p,arcTail[1],arcHead[1]],x[r,p,pathInd],y[arcTail[0],arcHead[0],p,arcTail[1],arcHead[1]]]
                                thecoefs = [1,2*plane_max_timestep[p],-plane_max_timestep[p]]
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[arcTail[0],arcHead[0],p]#+turnover_timesteps[i,p]#TODO: Verify this timewindow
                                number_of_nonzeros += 3
                                        
                                model.linear_constraints.add(names = ["timewindow1_" + "_" + r + "_" + p + "_" +str(pathInd) + "_" + str(vertNum)], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                
                                number_of_constraints += 1
                                
                            thevars = [d[arcTail[0],arcHead[0],p,arcTail[1],arcHead[1]],x[r,p,pathInd]]    
                            thecoefs = [1,-plane_max_timestep[p]]
                            rhs = earliest_departure_timesteps[r]-plane_max_timestep[p]-max_turnover_timesteps[r]
                            number_of_nonzeros += 2
                            model.linear_constraints.add(names = ["timewindow2_" + "_" + r + "_" + p + "_" +str(pathInd) + "_" + str(vertNum)], 
                                                         lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [rhs])
                            
                            number_of_constraints += 1
        """
        # weight limit (=max fuel)
        
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]:
                        thevars = [w[i,j,p,n1,n2]]
                        thecoefs = [1.0]
                        number_of_nonzeros += 1
                        
                        for r in REQUEST:
                            for pathInd in range(len(possiblePaths[r,p])):
                                if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)]):
                                    thevars.append(x[r,p,pathInd])
                                    thecoefs.append(-REQUEST[r].weight)
                                    number_of_nonzeros += 1
                        
                        thevars.append(f[i,j,p,n1,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                            
                        model.linear_constraints.add(names = ["computeweight_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                              lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                        number_of_constraints += 1
        
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    #print i,j,p
                    thevars = [w[i,j,p,n1,n2],y[i,j,p,n1,n2]]
                    thecoefs = [1.0,-max_trip_payload[i,j,p]]
                    number_of_nonzeros += 2
                
                    model.linear_constraints.add(names = ["weightlimit_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                 lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        # minimum number of fuelstops
        
        for p in PLANE:
          if PLANE[p].departure_max_fuel - fuelconsumption[PLANE[p].plane_departure,PLANE[p].plane_arrival,p] < PLANE[p].arrival_min_fuel:
            #print p
            thevars = []
            thecoefs = []
        
            for i,j in TRIP0:
                if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '1':
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                            thevars.append(y[i,j,p,n1,n2])
                            thecoefs.append(1.0)
                            number_of_nonzeros += 1
                
            model.linear_constraints.add(names = ["minfuelstops_in_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [1.0])
            number_of_constraints += 1
        
        for p in PLANE:    
          if PLANE[p].departure_max_fuel - fuelconsumption[PLANE[p].plane_departure,PLANE[p].plane_arrival,p] < PLANE[p].arrival_min_fuel:
            #print p
            thevars = []
            thecoefs = []
        
            for i,j in TRIP0:
              if AIRPORT[i].fuel[PLANE[p].required_fueltype] == '1':
                  for n1 in AirportNum[p,i]:
                      for n2 in AirportNum[p,j]:
                        thevars.append(y[i,j,p,n1,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                
            model.linear_constraints.add(names = ["minfuelstops_out_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [1.0])
            number_of_constraints += 1
           
        
        # maximum number of arrivals/departures per airport
        
        for i in AIRPORT:
          anyfuel = 0
          for ft in AIRPORT[i].fuel:
            if AIRPORT[i].fuel[ft] == '1':
              anyfuel += 1
              
          if anyfuel == 0:
            thevars = []
            thecoefs = []
          
            for j in AIRPORT:
              if (i,j) in TRIP0:
                  for p in PLANE:
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                        
                          thevars.append(y[i,j,p,n1,n2])
                          thecoefs.append(1.0)
                          number_of_nonzeros += 1
        
            rhs_value = 0
            
            for r in REQUEST:
              if REQUEST[r].request_departure == i or REQUEST[r].request_arrival == i:
                rhs_value += 1
            
            for p in PLANE:
              if PLANE[p].plane_departure == i:
                rhs_value += 1
            
            model.linear_constraints.add(names = ["maxpickup_out_" + i], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
            number_of_constraints += 1
          else:
            thevars = []
            thecoefs = []
          
            for j in AIRPORT:
              if (i,j) in TRIP0:
                  for p in PLANE:
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                        
                          thevars.append(y[i,j,p,n1,n2])
                          thecoefs.append(1.0)
                          number_of_nonzeros += 1
        
            rhs_value = 3
            
            #model.linear_constraints.add(names = ["maxpickup_out_" + i], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
            number_of_constraints += 1
        
        for j in AIRPORT:
          anyfuel = 0
          for ft in AIRPORT[j].fuel:
            if AIRPORT[j].fuel[ft] == '1':
              anyfuel += 1
              
          if anyfuel == 0:
            thevars = []
            thecoefs = []
          
            for i in AIRPORT:
              if (i,j) in TRIP0:
                  for p in PLANE:
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                        
                          thevars.append(y[i,j,p,n1,n2])
                          thecoefs.append(1.0)
                          number_of_nonzeros += 1
        
            rhs_value = 0
            
            for r in REQUEST:
              if REQUEST[r].request_departure == j or REQUEST[r].request_arrival == j:
                rhs_value += 1
            
            for p in PLANE:
              if PLANE[p].plane_arrival == j:
                rhs_value += 1
                
            model.linear_constraints.add(names = ["maxpickup_in_" + j], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
            number_of_constraints += 1
          else:
            thevars = []
            thecoefs = []
          
            for i in AIRPORT:
              if (i,j) in TRIP0:
                  for p in PLANE:
                    for n1 in AirportNum[p,i]:
                        for n2 in AirportNum[p,j]:
                        
                          thevars.append(y[i,j,p,n1,n2])
                          thecoefs.append(1.0)
                          number_of_nonzeros += 1
        
            rhs_value = 3
                
            #model.linear_constraints.add(names = ["maxpickup_in_" + j], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs_value])
            number_of_constraints += 1
        for p in PLANE:
            for j in AIRPORT:
                if AIRPORT[j].fuel[PLANE[p].required_fueltype] != '1' and j != PLANE[p].destination:
                    for n2 in AirportNum[p,j]:
                        for i in AIRPORT:
                            if (i,j) in TRIP0:
                                for n1 in AirportNum[p,i]:
                                    thevars = [y[i,j,p,n1,n2]] + [x[r,p,pathInd] for r in REQUEST if REQUEST[r].destination == j 
                                                for pathInd in range(len(possiblePaths[r,p])) 
                                                if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                        x[r,p,pathInd] for r in REQUEST if REQUEST[r].origin == j 
                                                        for pathInd in range(len(possiblePaths[r,p]))
                                                        if possiblePaths[r,p][pathInd].v[0][1] == n2
                                                        ]
                                    
                                    thecoefs = [1.0] + [-1.0 for r in REQUEST if REQUEST[r].destination == j 
                                                for pathInd in range(len(possiblePaths[r,p])) 
                                                if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                        -1.0 for r in REQUEST if REQUEST[r].origin == j 
                                                        for pathInd in range(len(possiblePaths[r,p]))
                                                        if possiblePaths[r,p][pathInd].v[0][1] == n2
                                                        ]
                                    
                                    model.linear_constraints.add(names = ["musthaverequest_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                 lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
        
        #a copy has to be visited for fuel supply or to pickup / deliver a request
        
        for p in PLANE:
            for i in AIRPORT:
                if AIRPORT[i].fuel[PLANE[p].required_fueltype] != '1':
                    for n1 in AirportNum[p,i]:
                        if not (i == PLANE[p].origin and n1==1):
                            for j in AIRPORT:
                                if (i,j) in TRIP0:
                                    for n2 in AirportNum[p,j]:
                                        thevars = [y[i,j,p,n1,n2]] + [x[r,p,pathInd] for r in REQUEST if REQUEST[r].origin == i 
                                                    for pathInd in range(len(possiblePaths[r,p])) 
                                                    if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                            x[r,p,pathInd] for r in REQUEST if REQUEST[r].destination == i 
                                                            for pathInd in range(len(possiblePaths[r,p]))
                                                            if possiblePaths[r,p][pathInd].v[-1][1] == n1
                                                            ]
                                        
                                        thecoefs = [1.0] + [-1.0 for r in REQUEST if REQUEST[r].origin == i 
                                                    for pathInd in range(len(possiblePaths[r,p])) 
                                                    if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])] + [
                                                            -1.0 for r in REQUEST if REQUEST[r].destination == i 
                                                            for pathInd in range(len(possiblePaths[r,p]))
                                                            if possiblePaths[r,p][pathInd].v[-1][1] == n1
                                                            ]
                                        
                                        model.linear_constraints.add(names = ["musthaverequest_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                     lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])                        
            
            
        # minimum amount of fuel for detour to refueling airport
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    thevars = [y[i,j,p,n1,n2],f[i,j,p,n1,n2]]
                    thecoefs = [min_refuel_trip[j,p],-1.0]
                    number_of_nonzeros += 2
                    
                    model.linear_constraints.add(names = ["minfuel_" + i + str(n1) + "_" + j + str(n2) + "_" + p], 
                                                          lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0])
                    number_of_constraints += 1
        
        for p,i,j in DATA.TIMEFREEPLANESOLUTION:
            thevars = []
            thecoefs = []
            for n1 in AirportNum[p,i]:
                for n2 in AirportNum[p,j]:
                    thevars += [y[i,j,p,n1,n2]]
                    thecoefs += [1.0]
            
            print thevars
            print TIMEFREEPLANESOLUTION[p,i,j]
            model.linear_constraints.add(names = ["fix_airplane_schedule_" + i + "_" + j + "_" + p], 
                                       lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                       senses = ["E"], rhs = [DATA.TIMEFREEPLANESOLUTION[p,i,j]])
        self.number_of_constraints = number_of_constraints
        self.number_of_nonzeros = number_of_nonzeros
        self.number_of_variables = number_of_variables

def readData(directory):
    comment_line = re.compile('#')
    
    print "reading '"+directory+"/airplanes.dat'"

    file = open(directory+"/airplanes.dat", "r")
    airplanes_data = file.read()
    file.close()
    
    entries = re.split("\n+", airplanes_data)
    
    PLANE = {}
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 18:
          ID,cost,seats,plane_departure,departure_min_fuel,departure_max_fuel,plane_arrival,arrival_min_fuel,arrival_max_fuel,required_fueltype,fuel,speed,max_fuel,empty_weight,add_turnover_time,reserve_fuel,contigence_ratio,pilot_weight = datas
          PLANE[ID] = __PLANE__(cost,seats,plane_departure,departure_min_fuel,departure_max_fuel,plane_arrival,arrival_min_fuel,arrival_max_fuel,required_fueltype,fuel,speed,max_fuel,empty_weight,add_turnover_time,reserve_fuel,contigence_ratio,pilot_weight)
    
    
    # --------------------
    # reading airports.dat
    # --------------------
    
    print "reading '"+directory+"/airports.dat'"
    
    file = open(directory+"/airports.dat", "r")
    airports_data = file.read()
    file.close()
    
    entries = re.split("\n+", airports_data)
    
    AIRPORT = {}
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 2:
          ID, turnover_time = datas
          AIRPORT[ID] = __AIRPORT__(turnover_time)
    
    
    # ---------------------
    # reading distances.dat
    # ---------------------
    
    print "reading '"+directory+"/distances.dat'"
    
    file = open(directory+"/distances.dat", "r")
    distances_data = file.read()
    file.close()
    
    entries = re.split("\n+", distances_data)
    
    TRIP = {}
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 3:
          origin, destination, distance = datas
          TRIP[origin,destination] = __TRIP__(distance)
    
    
    # -----------------
    # reading fuels.dat
    # -----------------
    
    print "reading '"+directory+"/fuels.dat'"
    
    file = open(directory+"/fuels.dat", "r")
    fuels_data = file.read()
    file.close()
    
    entries = re.split("\n+", fuels_data)
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 3:
          airport, fuelID, isAvailable = datas
          AIRPORT[airport].fuel[int(fuelID)] = isAvailable
    
    
    # --------------------
    # reading requests.dat
    # --------------------
    
    print "reading '"+directory+"/requests.dat'"
    
    file = open(directory+"/requests.dat", "r")
    requests_data = file.read()
    file.close()
    
    entries = re.split("\n+", requests_data)
    
    REQUEST = {}
    
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 11:
          ID,origin,destination,earliest_departure_time,earliest_departure_day,latest_arrival_time,latest_arrival_day,passengers,weight,max_stops,max_detour = datas
          REQUEST[ID] = __REQUEST__(origin,destination,earliest_departure_time,earliest_departure_day,latest_arrival_time,latest_arrival_day,passengers,weight,max_stops,max_detour)
    
    
    # ------------------------
    # reading timedelta.dat
    # ------------------------
    
    print "reading '"+directory+"/timedelta.dat'"
    
    file = open(directory+"/timedelta.dat", "r")
    timedelta_data = file.read()
    file.close()
    
    entries = re.split("\n+", timedelta_data)
    
    timedelta = 1
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 2:
          ID,timedelta = datas
          timedelta = int(timedelta) # conversion from string to int
    
    
    # ------------------------
    # reading weightlimits.dat
    # ------------------------
    
    print "reading '"+directory+"/weightlimits.dat'"
    
    file = open(directory+"/weightlimits.dat", "r")
    weightlimits_data = file.read()
    file.close()
    
    entries = re.split("\n+", weightlimits_data)
    
    WEIGHTLIMIT = {}
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 4:
          airport, airplane, max_takeoff_weight, max_landing_weight = datas
          WEIGHTLIMIT[airport,airplane] = __WEIGHTLIMIT__(max_takeoff_weight,max_landing_weight)
    
    
    # --------------------------
    # reading columnsolution.dat
    # --------------------------
    
    print "reading '"+directory+"/columnsolution.dat'"
    
    file = open(directory+"/columnsolution.dat", "r")
    solution_data = file.read()
    file.close()
    
    entries = re.split("\n+", solution_data)
    
    PLANE_SOLUTION = {}
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 5:
          plane, origin, destination, hour, minute = datas
          PLANE_SOLUTION[plane, origin, destination, hour, minute] = 1
    
    
    # --------------------------
    # reading columnsolution.dat
    # --------------------------
    
    print "reading '"+directory+"/requestsolution.dat'"
    
    file = open(directory+"/requestsolution.dat", "r")
    solution_data = file.read()
    file.close()
    
    entries = re.split("\n+", solution_data)
    
    REQUEST_SOLUTION = {}
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 6:
          plane, request, origin, destination, hour, minute = datas
          REQUEST_SOLUTION[plane, request, origin, destination, hour, minute] = 1
    
    return __DATA__(AIRPORT,PLANE,REQUEST,TRIP,WEIGHTLIMIT,PLANE_SOLUTION,REQUEST_SOLUTION,timedelta)

def setPrimal(mipmodel):
    TIMEFREEPLANESOLUTION = mipmodel.DATA.TIMEFREEPLANESOLUTION
    TIMEFREEREQUESTSOLUTION = mipmodel.DATA.TIMEFREEREQUESTSOLUTION
    model = mipmodel.model
    AirportNum = mipmodel.DATA.current_airport_num
    y = mipmodel.y
    x = mipmodel.x
    """
    for p,i,j in TIMEFREEPLANESOLUTION:
        thevars = []
        thecoefs = []
        for n1 in AirportNum[p,i]:
            for n2 in AirportNum[p,j]:
                thevars += [y[i,j,p,n1,n2]]
                thecoefs += [1.0]
        
        print thevars
        print TIMEFREEPLANESOLUTION[p,i,j]
        model.linear_constraints.add(names = ["fix_airplane_schedule_" + i + "_" + j + "_" + p], 
                                   lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                   senses = ["E"], rhs = [TIMEFREEPLANESOLUTION[p,i,j]])
    """
    model.solve()
    solution = model.solution
    
    """
    for p,i,j in TIMEFREEPLANESOLUTION:
        model.linear_constraints.delete("fix_airplane_schedule_" + i + "_" + j + "_" + p)
    """
    print "Solution status:", solution.get_status()
        
    if solution.is_primal_feasible():
        print "Primal solution value:", solution.get_objective_value()
        time.sleep(5)
        return solution.get_objective_value()
    else:
        print "Primal solution could not be recovered"
        time.sleep(5)
        blub-8
        return 100000
        


# prepare reading and parsing
DIRECTORIES = {
    #'BUF-AIV': 'Testinstances/A2-BUF_A2-AIV',
    #'BUF-ANT': 'Testinstances/A2-BUF_A2-ANT',
    #'BUF-BEE': 'Testinstances/A2-BUF_A2-BEE',
    #'BUF-BOK': 'Testinstances/A2-BUF_A2-BOK',
    #'BUF-EGL': 'Testinstances/A2-BUF_A2-EGL',
    #'BUF-GNU': 'Testinstances/A2-BUF_A2-GNU',
    #'BUF-JKL': 'Testinstances/A2-BUF_A2-JKL',
    #'BUF-LEO': 'Testinstances/A2-BUF_A2-LEO',
    #'BUF-NAS': 'Testinstances/A2-BUF_A2-NAS',
    #'BUF-OWL': 'Testinstances/A2-BUF_A2-OWL',
    #'BUF-ZEB': 'Testinstances/A2-BUF_A2-ZEB',
    #'EGL-BEE': 'Testinstances/A2-EGL_A2-BEE',
    #'EGL-GNU': 'Testinstances/A2-EGL_A2-GNU',
    #'EGL-LEO': 'Testinstances/A2-EGL_A2-LEO',
    #'GNU-BEE': 'Testinstances/A2-GNU_A2-BEE',
    #'GNU-JKL': 'Testinstances/A2-GNU_A2-JKL',
    #'GNU-LEO': 'Testinstances/A2-GNU_A2-LEO',
    #'LEO-AIV': 'Testinstances/A2-LEO_A2-AIV',
    #'LEO-ANT': 'Testinstances/A2-LEO_A2-ANT',
    #'LEO-BEE': 'Testinstances/A2-LEO_A2-BEE',
    #'LEO-BOK': 'Testinstances/A2-LEO_A2-BOK',
    'LEO-JKL': 'Testinstances/A2-LEO_A2-JKL',
    #'LEO-NAS': 'Testinstances/A2-LEO_A2-NAS',
    #'LEO-OWL': 'Testinstances/A2-LEO_A2-OWL',
    }

cuttingPlanes = {
        'min_fuel_cut':1,
        'useless_detour_cut':1,
        'max_visit_cut':1,
        'time_order_cut':1,
        'fuelstop_cut':1,
        }



SOLUTIONS = {}

for instanceName, directory in DIRECTORIES.iteritems():
    DATA = readData(directory)
    if not ("fullAirplaneMIP" in globals()):
        fullAirplaneMIP = {}
        for p in DATA.PLANE:
            fullAirplaneMIP[p] = __AIRLINEMIP__(DATA,{},1,{p:DATA.PLANE[p]})

    primal_objective=100000
    totallySolved = 0
    while not totallySolved:
        airplaneMIP = __AIRLINEMIP__(DATA,cuttingPlanes)
        primal_objective = setPrimal(airplaneMIP)
        cb = airplaneMIP.model.register_callback(breakIncumbentCallback)
        cb.totallySolved = 0
        cb.MIPMODEL = airplaneMIP
        cb.fullModel = fullAirplaneMIP
        cb.number_of_infeasibles = 0
        cb.bestSolution = primal_objective
        
        
        model = airplaneMIP.model
        
        model.solve()
        if cb.bestSolution < primal_objective - 0.01:
            DATA.TIMEFREEPLANESOLUTION = cb.best_plane_solution
            DATA.TIMEFREEREQUESTSOLUTION = cb.best_request_solution
        totallySolved = cb.totallySolved
        #totallySolved = 1
    
solutionValues=model.solution.get_values()
idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
name2solutionValue = { n : solutionValues[j] for j, n in enumerate(model.variables.get_names()) }
for key,val in airplaneMIP.y.iteritems():
    valStore=solutionValues[name2idx[val]]
    if valStore > 0.5 or valStore < -0.5:
        print(val+" %f" %valStore)


#y#VUM_CBE_0_1_1 1.000000
#y#JAO_VUM_0_1_1 1.000000
#y#ABU_SWI_0_1_1 1.000000
#y#CBE_VUM_0_1_1 1.000000
#y#MOM_MUB_1_1_1 1.000000
#y#SWI_BBK_0_1_1 1.000000
#y#MUB_BBK_1_1_1 1.000000
#y#VUM_ABU_0_1_1 1.000000
#y#VUM_MOM_1_1_1 1.000000
#y#XIG_JAO_0_1_1 1.000000
#y#BBK_XIG_0_1_1 1.000000
#y#BBK_SWI_1_1_1 1.000000

#['y#MUB_BBK_1_1_1']
#['y#BBK_XIG_0_1_1']
#['y#XIG_JAO_0_1_1']
#['y#JAO_VUM_0_1_1']
#['y#MOM_MUB_1_1_1']
#['y#CBE_VUM_0_1_1']
#['y#SWI_BBK_0_1_1']
#['y#VUM_ABU_0_1_1']
#['y#VUM_CBE_0_1_1']
#['y#ABU_SWI_0_1_1']
#['y#VUM_MOM_1_1_1']
#['y#BBK_SWI_1_1_1']
