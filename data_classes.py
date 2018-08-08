import math
import cplex
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
    
    def __init__(self,gap,dualBound,bestValue,loop_iterations,time,instanceName=""):
        self.instanceName = instanceName
        self.dualBound = int(round(dualBound,0))
        self.bestValue = int(round(bestValue,0))
        self.gap = round(gap,2)
        self.loop_iterations = loop_iterations
        self.time = int(min([10800,round(time,0)]))
        self.history = {}
    def toString(self):
        s = self.instanceName + " "
        s += str(self.time) +" , "
        s += str(self.dualBound) +" , "
        s += str(self.bestValue) + " , "
        s += str(self.gap) + "\% , "
        s += str(self.loop_iterations)
        return s
    def printToScreen(self):
        print(self.toString())
    def savePlaneVars(self,y,solutionValues,name2idx):
        self.planeVars = {}
        for key,val in y.iteritems():
            valStore=solutionValues[name2idx[val]]
            if valStore > 0.5 or valStore < -0.5:
                self.planeVars[val] = valStore
    def saveRequestVars(self,x,solutionValues,name2idx):
        self.requestVars = {}
        for key,val in x.iteritems():
            valStore=solutionValues[name2idx[val]]
            if valStore > 0.5 or valStore < -0.5:
                self.planeVars[val] = valStore
    def saveToFile(self,fileName,options=[]):
        file = open(fileName, "a")
        if self.instanceName[0] == 'A':
            lineToAdd = self.instanceName + " & & %d & %d & %.2f \\%% & %d & %.1f \\\\\n" % (self.dualBound,self.bestValue,self.gap,self.time,self.loop_iterations)
        else:
            lineToAdd = self.instanceName + " & & %d & %d & %.2f \\%% & %d & %d\\\\\n" % (self.dualBound,self.bestValue,self.gap,self.time,int(self.loop_iterations))
        
        file.write(lineToAdd)
        file.close()
        

class __DATA__(object):
    
    def __init__(self,AIRPORT,PLANE,REQUEST,TRIP,WEIGHTLIMIT,PLANE_SOLUTION,REQUEST_SOLUTION,timedelta,use_all=0):
        self.AIRPORT = AIRPORT
        self.PLANE = PLANE
        self.REQUEST = REQUEST
        self.TRIP = TRIP
        self.WEIGHTLIMIT = WEIGHTLIMIT
        self.PLANE_SOLUTION = PLANE_SOLUTION
        self.REQUEST_SOLUTION = REQUEST_SOLUTION
        self.timedelta = timedelta
        self.use_all = use_all
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
            maxStops[i] = 2
            for p,i2,j in TIMEFREEPLANESOLUTION:
                if i2==i:
                    maxStops[i] += 0.501
            maxStops[i] = int(round(maxStops[i],1))
    
                
        AirportNum2 = {}
        for i in AIRPORT:
            for p in PLANE:
                AirportNum2[p,i] = range(1,1+maxStops[i])
                
        self.max_airport_num = AirportNum2
        
        AirportNum = {}
        for i in AIRPORT:
            for p in PLANE:
                AirportNum[p,i] = [1]
        if not self.use_all:
            self.current_airport_num = AirportNum
        else:
            self.current_airport_num = AirportNum2





class __AIRLINEMIP__(object):
    
    def __init__(self,DATA,cuttingPlanes={},full=0,p={},pathBased=1,log_file_name="",use_all=0):
        self.DATA = DATA
        self.log_file_name = log_file_name
        self.use_all = use_all
        if pathBased:
            self.generateMIP(cuttingPlanes,full,p)
        else:
            self.generateMIP2(cuttingPlanes,full,p)
    def generateMIP(self,cuttingPlanes,full,p):
        use_all=self.use_all
        DATA=self.DATA
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
            #AirportNum = DATA.max_airport_num
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
        if full:
            model.set_results_stream('reconst.rlog')
            model.set_warning_stream('reconst.wlog')
        else:
            if self.log_file_name != "":
                model.set_results_stream(self.log_file_name)
            
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
        else:
            r2 = {}
            self.r2 = r2
            for p in PLANE:
                for r in REQUEST:
                    r2[r,p] = "r_ass" + r + "_" + p
                    model.variables.add(names = [r2[r,p]], types = ["B"])
        y = {}
        self.y = y

         
        for i,j in TRIP0:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                      if n1 == AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1] and not use_all:
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
                for p in PLANE:
                    thevars = []
                    thecoefs = []
                    for pathInd in range(len(possiblePaths[r,p])):
                        thevars.append(x[r,p,pathInd])
                        thecoefs.append(1.0)
                    thevars += [r2[r,p]]
                    thecoefs += [-1.0]
                
                    model.linear_constraints.add(names = ["request_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                    number_of_constraints += 1
                    number_of_nonzeros += len(thevars)
                thevars = []
                thecoefs = []
                for p in PLANE:
                    thevars += [r2[r,p]]
                    thecoefs += [1.0]
                
                model.linear_constraints.add(names = ["request_assignment_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
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
                            
                    if j == PLANE[p].plane_departure and n2==1:
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
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]
                                        
                                model.linear_constraints.add(names = ["timewindow1_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                
                                number_of_constraints += 1
                                number_of_nonzeros += len(thevars)
                            else:
                                thevars = [d[i,j,p,n1,n2]]+[x[r,p,pathInd] for pathInd in range(len(possiblePaths[r,p])) if possiblePaths[r,p][pathInd].hasArc([(i,n1),(j,n2)])]+[y[i,j,p,n1,n2]]
                                thecoefs = [1]+[2*plane_max_timestep[p]]*(len(thevars)-2)+[-plane_max_timestep[p]]
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]
                                        
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
        
        ArrAirport = {}
        DepAirport = {}
        for i in AIRPORT:
            arrRemove = 0
            depRemove = 0
            for r in REQUEST:
                if REQUEST[r].origin == i:
                    arrRemove = 1
                if REQUEST[r].destination == i:
                    depRemove = 1
            if not arrRemove:
                ArrAirport[i] = AIRPORT[i]
            if not depRemove:
                DepAirport[i] = AIRPORT[i]
            
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
        
        if cuttingPlanes.has_key('arrival_departure_cut'):
            for p in PLANE:
                for j in DepAirport:
                    if AIRPORT[j].fuel[PLANE[p].required_fueltype] != '1' and j != PLANE[p].destination:
                        for n2 in AirportNum[p,j]:
                                thevars = [y[i,j,p,n1,n2] for i in AIRPORT if (i,j) in TRIP0 for n1 in AirportNum[p,i]] + [
                                                    x[r,p,pathInd] for r in REQUEST if REQUEST[r].origin == j 
                                                    for pathInd in range(len(possiblePaths[r,p]))
                                                    if possiblePaths[r,p][pathInd].v[0][1] == n2
                                                    ]
                                
                                thecoefs = [1.0 for i in AIRPORT if (i,j) in TRIP0 for n1 in AirportNum[p,i]]  + [
                                                    -1.0 for r in REQUEST if REQUEST[r].origin == j 
                                                    for pathInd in range(len(possiblePaths[r,p]))
                                                    if possiblePaths[r,p][pathInd].v[0][1] == n2
                                                    ]
                                
                                model.linear_constraints.add(names = ["musthaverequest_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
    
    
            
            for p in PLANE:
                for i in ArrAirport:
                    if AIRPORT[i].fuel[PLANE[p].required_fueltype] != '1':
                        for n1 in AirportNum[p,i]:
                            if not (i == PLANE[p].origin and n1==1):
                                            thevars = [y[i,j,p,n1,n2] for j in AIRPORT 
                                                       if (i,j) in TRIP0 for n2 in AirportNum[p,j]] +  [
                                                                x[r,p,pathInd] for r in REQUEST if REQUEST[r].destination == i 
                                                                for pathInd in range(len(possiblePaths[r,p]))
                                                                if possiblePaths[r,p][pathInd].v[-1][1] == n1
                                                                ]
                                            
                                            thecoefs = [1.0 for j in AIRPORT 
                                                        if (i,j) in TRIP0 for n2 in AirportNum[p,j]] + [
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
        
        
        if cuttingPlanes.has_key('time_conflict_cut'):
            self.number_of_time_conflict_cuts = 0
            for p in PLANE:
                for i,j in TRIP0:
                    for n1 in AirportNum[p,i]:
                        if n1 != AirportNum[p,i][-1]:
                            for n2 in AirportNum[p,j]:
                                for r1 in REQUEST:
                                    for r2 in REQUEST:
                                        if r1!=r2:
                                            for path1Ind in range(len(possiblePaths[r1,p])):
                                                for path2Ind in range(len(possiblePaths[r2,p])):
                                                    if possiblePaths[r1,p][path1Ind].hasArc([(i,n1),(j,n2)]) and possiblePaths[r2,p][path2Ind].hasArc([(i,n1+1),(j,n2)]):
                                                        if (earliest_departure_timesteps[r1] + possiblePaths[r1,p][path1Ind].min_duration + 
                                                            turnover_travel_timesteps[REQUEST[r1].destination,REQUEST[r2].origin,p] + possiblePaths[r2,p][path2Ind].min_duration) > latest_arrival_timesteps[r2]:
                                                            thevars = [x[r1,p,path1Ind],x[r2,p,path2Ind]]
                                                            thecoefs = [1.0,1.0]
                                                            
                                                            number_of_nonzeros += 2
                                                            model.linear_constraints.add(names = ["time_conflict_cut#" + p + "_" + r1 + "_" + str(path1Ind) + "_" + r2 + "_" + str(path2Ind)], 
                                                              lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])

                                                            number_of_constraints += 1
                                                            self.number_of_time_conflict_cuts += 1
        if cuttingPlanes.has_key('request_conflict_cut'):
            self.number_of_request_conflict_cuts = 0
            for p in PLANE:
                for r1 in REQUEST:
                    for pathInd in range(len(possiblePaths[r1,p])):
                        if len(possiblePaths[r1,p][pathInd].v) == 3:
                            for r2 in REQUEST:
                                if REQUEST[r2].origin == possiblePaths[r1,p][pathInd].v[1][0] and REQUEST[r2].destination != possiblePaths[r1,p][pathInd].v[2][0]:
                                    thevars = [x[r1,p,pathInd] ] + [x[r2,p,pathInd2] for pathInd2 in range(len(possiblePaths[r2,p]))
                                        if (earliest_departure_timesteps[r1] + possiblePaths[r1,p][pathInd].min_duration + 
                                                            turnover_travel_timesteps[REQUEST[r1].destination,REQUEST[r2].origin,p]+ possiblePaths[r2,p][pathInd2].min_duration) > latest_arrival_timesteps[r2]
                                        and (earliest_departure_timesteps[r2] + possiblePaths[r1,p][pathInd].min_duration + 
                                                            turnover_travel_timesteps[REQUEST[r2].destination,REQUEST[r1].origin,p]+ possiblePaths[r2,p][pathInd2].min_duration) > latest_arrival_timesteps[r1]
                                       
                                        ]
                                    if len (thevars) > 1:
                                        thecoefs = [1.0]*len(thevars)
                                        number_of_nonzeros += len(thevars)
                                        model.linear_constraints.add(names = ["request_conflict_cut#" + p + "_" + r1 + "_" + str(pathInd) + "_" + r2 + "_" + str(pathInd2)], 
                                                                  lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
    
                                        number_of_constraints += 1
                                        self.number_of_request_conflict_cuts += 1
                        if len(possiblePaths[r1,p][pathInd].v) == 4:
                            for r2 in REQUEST:
                                if (REQUEST[r2].origin == possiblePaths[r1,p][pathInd].v[2][0] and REQUEST[r2].destination != possiblePaths[r1,p][pathInd].v[3][0]
                                    or (REQUEST[r2].origin == possiblePaths[r1,p][pathInd].v[1][0] and 
                                    REQUEST[r2].destination != possiblePaths[r1,p][pathInd].v[3][0] and REQUEST[r2].destination != possiblePaths[r1,p][pathInd].v[2][0])):
                                    thevars = [x[r1,p,pathInd] ] + [x[r2,p,pathInd2] for pathInd2 in range(len(possiblePaths[r2,p]))
                                        if (earliest_departure_timesteps[r1] + possiblePaths[r1,p][pathInd].min_duration + 
                                                            turnover_travel_timesteps[REQUEST[r1].destination,REQUEST[r2].origin,p]+ possiblePaths[r2,p][pathInd2].min_duration) > latest_arrival_timesteps[r1]
                                        and (earliest_departure_timesteps[r2] + possiblePaths[r1,p][pathInd].min_duration + 
                                                            turnover_travel_timesteps[REQUEST[r2].destination,REQUEST[r1].origin,p]+ possiblePaths[r2,p][pathInd2].min_duration) > latest_arrival_timesteps[r1]
                                        
                                        ]
                                    thecoefs = [1.0]*len(thevars)
                                    if len (thevars) > 1:
                                        number_of_nonzeros += len(thevars)
                                        
                                        model.linear_constraints.add(names = ["request_conflict_cut#" + p + "_" + r1 + "_" + str(pathInd) + "_" + r2 + "_" + str(pathInd2)], 
                                                                  lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
    
                                        number_of_constraints += 1
                                        self.number_of_request_conflict_cuts += 1
                                    
        self.number_of_constraints = number_of_constraints
        self.number_of_nonzeros = number_of_nonzeros
        self.number_of_variables = number_of_variables

    def generateMIP2(self,cuttingPlanes,full,p):
        DATA=self.DATA
        use_all=self.use_all
        if full:
            PLANE = p
        else:
            PLANE = DATA.PLANE
        REQUEST = DATA.REQUEST
        REQUEST_TRIP0 = DATA.REQUEST_TRIP0
        AIRPORT = DATA.AIRPORT
        #TRIP = DATA.TRIP
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
        
        
        model = cplex.Cplex()
        self.model = model
        if full:
            model.set_results_stream('reconst.rlog')
            model.set_warning_stream('reconst.wlog')
        else:
            if self.log_file_name != "":
                model.set_results_stream(self.log_file_name)
        
        number_of_variables = 0
        number_of_constraints = 0
        number_of_nonzeros = 0
        
        # VARIABLES
        
        x = {}
        self.x = x
        number_of_variables = 0
        
        for r in REQUEST:
            for p in PLANE:
              for i,j in REQUEST_TRIP0[r]:
                  for n1 in AirportNum[p,i]:
                      for n2 in AirportNum[p,j]:
                          x[i,j,r,p,n1,n2] = "x#" + i + "_" + j + "_" + r + "_" + p + "_" + str(n1) + "_" + str(n2)
                          #model.variables.add(obj = [travelcost[i,j,p]], names = [x[i,j,r,p]], lb = [0], ub = [1], types = ["B"])
                          model.variables.add(obj = [0.0001], names = [x[i,j,r,p,n1,n2]], lb = [0], ub = [1], types = ["B"])
                          number_of_variables += 1
        
        x_dep = {}
        self.x_dep = x_dep
        for r in REQUEST:
          for p in PLANE:
              for n1 in AirportNum[p,REQUEST[r].origin]:
                x_dep[r,p,n1] = "x_dep#" + r + "_" + p + " " + str(n1)
                model.variables.add(names = [x_dep[r,p,n1]], lb = [0], ub = [1], types = ["B"])
                number_of_variables += 1
        
        x_arr = {}
        self.x_arr = x_arr
        for r in REQUEST:
          for p in PLANE:
              for n1 in AirportNum[p,REQUEST[r].destination]:
                x_arr[r,p,n1] = "x_arr#" + r + "_" + p + "_" + str(n1)
                model.variables.add(names = [x_arr[r,p,n1]], lb = [0], ub = [1], types = ["B"])
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
                      if n1 == AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1] and not use_all:
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
            # each chosen request must depart        
            for r in REQUEST:
                  thevars = []
                  thecoefs = []
                  for p in PLANE:
                      for n1 in AirportNum[p,REQUEST[r].origin]:
                        thevars.append(x_dep[r,p,n1])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                  thevars += [r2[r]]
                  thecoefs += [-1]
                  model.linear_constraints.add(names = ["request_one_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                  number_of_constraints += 1
            
            
            # each request must arrive
            
            for r in REQUEST:
              thevars = []
              thecoefs = []
              for p in PLANE:
                  for n1 in AirportNum[p,REQUEST[r].destination]:
                    thevars.append(x_arr[r,p,n1])
                    thecoefs.append(1.0)
                    number_of_nonzeros += 1
              thevars += [r2[r]]
              thecoefs += [-1]  
              model.linear_constraints.add(names = ["request_one_arr_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
              number_of_constraints += 1
              
            
            # request flow
            
            for r in REQUEST:
              for p in PLANE:
                for j in AIRPORT:
                    for n2 in AirportNum[p,j]:
                      thevars = []
                      thecoefs = []
                
                      for i in AIRPORT:
                        if (i,j) in REQUEST_TRIP0[r]:
                            for n1 in AirportNum[p,i]:
                                  thevars.append(x[i,j,r,p,n1,n2])
                                  thecoefs.append(1.0)
                                  number_of_nonzeros += 1
                      
                      if j == REQUEST[r].request_departure:
                        thevars.append(x_dep[r,p,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                        
                      for k in AIRPORT:
                        if (j,k) in REQUEST_TRIP0[r]:
                            for n1 in AirportNum[p,k]:
                              thevars.append(x[j,k,r,p,n2,n1])
                              thecoefs.append(-1.0)
                              number_of_nonzeros += 1
                          
                      if j == REQUEST[r].request_arrival:
                        thevars.append(x_arr[r,p,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                        
                      model.linear_constraints.add(names = ["request_flow_" + r + "_" + p + "_" + j + str(n2)],
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                      number_of_constraints += 1
        else:
            # each request must depart
        
            for r in REQUEST:
                  thevars = []
                  thecoefs = []
                  for p in PLANE:
                      for n1 in AirportNum[p,REQUEST[r].origin]:
                        thevars.append(x_dep[r,p,n1])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                        
                  model.linear_constraints.add(names = ["request_one_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
                  number_of_constraints += 1
            
            
            # each request must arrive
            
            for r in REQUEST:
              thevars = []
              thecoefs = []
              for p in PLANE:
                  for n1 in AirportNum[p,REQUEST[r].destination]:
                    thevars.append(x_arr[r,p,n1])
                    thecoefs.append(1.0)
                    number_of_nonzeros += 1
                
              model.linear_constraints.add(names = ["request_one_arr_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
              number_of_constraints += 1
              
            
            # request flow
            
            for r in REQUEST:
              for p in PLANE:
                for j in AIRPORT:
                    for n2 in AirportNum[p,j]:
                      thevars = []
                      thecoefs = []
                
                      for i in AIRPORT:
                        if (i,j) in REQUEST_TRIP0[r]:
                            for n1 in AirportNum[p,i]:
                                  thevars.append(x[i,j,r,p,n1,n2])
                                  thecoefs.append(1.0)
                                  number_of_nonzeros += 1
                      
                      if j == REQUEST[r].request_departure:
                        thevars.append(x_dep[r,p,n2])
                        thecoefs.append(1.0)
                        number_of_nonzeros += 1
                        
                      for k in AIRPORT:
                        if (j,k) in REQUEST_TRIP0[r]:
                            for n1 in AirportNum[p,k]:
                              thevars.append(x[j,k,r,p,n2,n1])
                              thecoefs.append(-1.0)
                              number_of_nonzeros += 1
                          
                      if j == REQUEST[r].request_arrival:
                        thevars.append(x_arr[r,p,n2])
                        thecoefs.append(-1.0)
                        number_of_nonzeros += 1
                        
                      model.linear_constraints.add(names = ["request_flow_" + r + "_" + p + "_" + j + str(n2)],
                                                            lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                      number_of_constraints += 1

        
        
        
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
                  
                    #print i,j,p
                    thevars = [y[i,j,p,n1,n2]]
                    thecoefs = [-PLANE[p].seats]
                    number_of_nonzeros += 1
                    
                    for r in REQUEST:
                      if (i,j) in REQUEST_TRIP0[r]:
                        thevars.append(x[i,j,r,p,n1,n2])
                        thecoefs.append(REQUEST[r].passengers)
                        number_of_nonzeros += 1
                      
                    model.linear_constraints.add(names = ["seatlimit_" + i + str(n1) + "_" + j + "_" + p + str(n2)], 
                                                          lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
                    number_of_constraints += 1
        
        
        # intermediate stops for requests
        
        for r in REQUEST:
          thevars = []
          thecoefs = []
          #print r
            
          for i,j in REQUEST_TRIP0[r]:
              for p in PLANE:
                  for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]: 
                      thevars.append(x[i,j,r,p,n1,n2])
                      thecoefs.append(1.0)
                      number_of_nonzeros += 1
                    
          model.linear_constraints.add(names = ["intermediatestops_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [REQUEST[r].max_stops + 1])
          number_of_constraints += 1
        
        
        # maximum detour for passengers (compared to direct flight)
        
        for r in REQUEST:
          thevars = []
          thecoefs = []
          
          for i,j in REQUEST_TRIP0[r]:
            for p in PLANE:
                for n1 in AirportNum[p,i]:
                    for n2 in AirportNum[p,j]:
                      thevars.append(x[i,j,r,p,n1,n2])
                      thecoefs.append(TRIP0[i,j].distance)
                      number_of_nonzeros += 1
          
          model.linear_constraints.add(names = ["maxdetour_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [(1 + REQUEST[r].max_detour) * TRIP0[REQUEST[r].request_departure,REQUEST[r].request_arrival].distance])
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
                            thevars = [d[i,j,p,n1,n2],x[i,j,r,p,n1,n2]]
                            thecoefs = [1,-plane_max_timestep[p]]
                            rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]
                            rhs = earliest_departure_timesteps[r]-plane_max_timestep[p]-max_turnover_timesteps[r]
                            number_of_nonzeros += 2
                                    
                            model.linear_constraints.add(names = ["timewindow1_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                         lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["G"], rhs = [rhs])
                            
                            number_of_constraints += 1
                            if not( n1==AirportNum[p,i][-1] and n2 == AirportNum[p,j][-1] ):
                                thevars = [d[i,j,p,n1,n2],x[i,j,r,p,n1,n2]]
                                number_of_constraints += 1
                                thecoefs = [1,plane_max_timestep[p]]
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]
                                number_of_nonzeros += 2
                                        
                                model.linear_constraints.add(names = ["timewindow2_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                
                                number_of_constraints += 1
                            else:              
                                thevars = [d[i,j,p,n1,n2],x[i,j,r,p,n1,n2],y[i,j,p,n1,n2]]
                                thecoefs = [1,2*plane_max_timestep[p],-plane_max_timestep[p]]
                                rhs = latest_arrival_timesteps[r]+plane_max_timestep[p]-turnover_travel_timesteps[i,j,p]
                                number_of_nonzeros += 3
                                        
                                model.linear_constraints.add(names = ["timewindow2_" + i + str(n1) + '_' + j + str(n2) + '_' + r + "_" + p], 
                                                             lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [rhs])
                                
                                number_of_constraints += 1
        
        # weight limit (=max fuel)
        
        for i,j in TRIP0:
            for p in PLANE:
              for n1 in AirportNum[p,i]:
                  for n2 in AirportNum[p,j]:
                  
                    thevars = [w[i,j,p,n1,n2]]
                    thecoefs = [1.0]
                    number_of_nonzeros += 1
                
                    for r in REQUEST:
                      if (i,j) in REQUEST_TRIP0[r]:
                        thevars.append(x[i,j,r,p,n1,n2])
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
                                        thevars = [y[i,j,p,n1,n2]] + [x[i,j,r,p,n1,n2] for r in REQUEST
                                                   if (i,j) in REQUEST_TRIP0[r] 
                                                   if j == REQUEST[r].destination] + [x[j,k,r,p,n2,n3]
                                                            for r in REQUEST if REQUEST[r].origin == j 
                                                            for k in AIRPORT if (j,k) in REQUEST_TRIP0[r] for n3 in AirportNum[p,k]]
                                        
                                        thecoefs = [1.0] + [-1.0 for r in REQUEST
                                                   if (i,j) in REQUEST_TRIP0[r] 
                                                   if j == REQUEST[r].destination] + [-1.0
                                                            for r in REQUEST if REQUEST[r].origin == j 
                                                            for k in AIRPORT if (j,k) in REQUEST_TRIP0[r] for n3 in AirportNum[p,k]]
                                        
                                        model.linear_constraints.add(names = ["musthaverequest1_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
                                                     lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
            
            
            for p in PLANE:
                for i in AIRPORT:
                    if AIRPORT[i].fuel[PLANE[p].required_fueltype] != '1':
                        for n1 in AirportNum[p,i]:
                            if not (i == PLANE[p].origin and n1==1):
                                for j in AIRPORT:
                                    if (i,j) in TRIP0:
                                        for n2 in AirportNum[p,j]:
                                            thevars = [y[i,j,p,n1,n2]] + [x[i,j,r,p,n1,n2] for r in REQUEST if (i,j) in REQUEST_TRIP0[r]
                                                        if i == REQUEST[r].origin ] + [
                                                             x[k,i,r,p,n3,n1]
                                                            for r in REQUEST if REQUEST[r].destination == i 
                                                            for k in AIRPORT if (k,i) in REQUEST_TRIP0[r] for n3 in AirportNum[p,k]]
                                            
                                            thecoefs = [1.0] + [-1.0 for r in REQUEST if (i,j) in REQUEST_TRIP0[r]
                                                        if i == REQUEST[r].origin] + [
                                                             -1.0
                                                            for r in REQUEST if REQUEST[r].destination == i 
                                                            for k in AIRPORT if (k,i) in REQUEST_TRIP0[r] for n3 in AirportNum[p,k]]
                                            
                                            model.linear_constraints.add(names = ["musthaverequest2_" + i + str(n1) + '_' + j + str(n2) + "_" + p], 
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
        
                                                            
                                
        if full:
            model.set_results_stream('reconst.rlog')
            model.set_warning_stream('reconst.wlog')
        self.number_of_constraints = number_of_constraints
        self.number_of_nonzeros = number_of_nonzeros
        self.number_of_variables = number_of_variables
