#! /usr/bin/python

import re
import cplex
import math
import time
import sys
from operator import itemgetter
from cplex.callbacks import IncumbentCallback
from cplex.callbacks import LazyConstraintCallback
from cplex.callbacks import MIPInfoCallback
from cplex.exceptions import CplexSolverError
from sets import Set 

EPSILON = 1e-6


# ---------
# callbacks
# ---------

class CountNodesCallback(MIPInfoCallback):
  
  def __call__(self):
    
    self.number_of_nodes = self.get_num_nodes()
    self.best_obj_val = self.get_best_objective_value()
    self.mip_gap = self.get_MIP_relative_gap()
    
    return
    

class CheckSolutionMIPCallback(IncumbentCallback):

  def __call__(self):
    
    self.number_of_calls += 1

    #print "Check Solution with MIP (IncumbentCallback) ",self.number_of_calls

    # get incumbent values
    
    solution_source = self.get_solution_source()
    
    #print "solution source: ",solution_source
    
    all_values = self.get_values()
    
    #print all_values
    
    all_values_int = [int(round(val)) for val in all_values]
    
    all_values_int_tuple = tuple(all_values_int)
    
    #print all_values_int_tuple
    
    if all_values_int_tuple in self.solution_pool:
      #print "DIE HATTEN WIR SCHON"
      self.reject()
      return
    else:
      #print "DIE IST NEU"
      self.solution_pool.append(all_values_int_tuple)
      self.number_of_calls_with_new_solution += 1
      
    # get incumbent solution

    value_of_y = {}

    for i,j in TRIP0:
      for p in PLANE:
        value_of_y[i,j,p] = int(round(self.get_values(y[i,j,p])))
        #if value_of_y[i,j,p] > 0:
          #print "  from",i,"to",j,"plane",p,"   (",value_of_y[i,j,p],")"

    value_of_x = {}

    for r in REQUEST:
      for i,j in REQUEST_TRIP0[r]:
        for p in PLANE:
          value_of_x[i,j,r,p] = int(round(self.get_values(x[i,j,r,p])))
          #if value_of_x[i,j,r,p] > 0:
          #  print "  from",i,"to",j,"request",r,"plane",p
            
    value_of_x_dep = {}
    
    for r in REQUEST:
      for p in PLANE:
        value_of_x_dep[r,p] = int(round(self.get_values(x_dep[r,p])))
        #if value_of_x_dep[r,p] > 0:
          #print "  request",r,"plane",p 
    
    value_of_x_arr = {}
        
    for r in REQUEST:
      for p in PLANE:
        value_of_x_arr[r,p] = int(round(self.get_values(x_arr[r,p])))
        #if value_of_x_arr[r,p] > 0:
          #print "  request",r,"plane",p
    
    # print data on screen
    
    #print
    #
    #for p in PLANE:
    #  print "# plane",p
    #  for i,j in TRIP0:
    #    if value_of_y[i,j,p] > 0:
    #      print p,i,j,value_of_y[i,j,p]
    #
    #print
    #
    #for p in PLANE:
    #  print "# plane",p
    #  for r in REQUEST:
    #    if value_of_x_arr[r,p] > 0:
    #      print p,r;
    #      
    #print
    #
    #for p in PLANE:
    #  print "# plane",p
    #  for r in REQUEST:
    #    for i,j in REQUEST_TRIP0[r]:
    #      if value_of_x[i,j,r,p] > 0:
    #        print p,r,i,j
    #
    #print "\n----------------------------------\n\n"
    
    # define sets and parameters from incumbent 
    
    INCUMBENT_TRIP = {}
    trips_between_airports = {}
    
    for p in PLANE:
      INCUMBENT_TRIP[p] = {}
      trips_between_airports[p] = {}
      
      for i,j in TRIP0:
        if value_of_y[i,j,p] > 0:
          INCUMBENT_TRIP[p][i,j] = 1
          trips_between_airports[p][i,j] = value_of_y[i,j,p]
    
                
    INCUMBENT_AIRPORT = {}
    
    for p in PLANE:
      INCUMBENT_AIRPORT[p] = {}
      for i,j in INCUMBENT_TRIP[p]:
        INCUMBENT_AIRPORT[p][i] = 1
        INCUMBENT_AIRPORT[p][j] = 1
    
    
    INCUMBENT_TRIP_PLUS = {}
    
    for p in PLANE:
      INCUMBENT_TRIP_PLUS[p] = {}
      for i,j in INCUMBENT_TRIP[p]:
        INCUMBENT_TRIP_PLUS[p][i,j] = 1
      
      for i in INCUMBENT_AIRPORT[p]:
        INCUMBENT_TRIP_PLUS[p][i,i] = 1
    
    
    INCUMBENT_REQUEST = {}
    
    for p in PLANE:
      INCUMBENT_REQUEST[p] = {}
      
      for r in REQUEST:
        if value_of_x_arr[r,p] > 0:
          INCUMBENT_REQUEST[p][r] = 1
    
    
    INCUMBENT_REQUEST_AIRPORT = {}
    
    for p in PLANE:
      for r in REQUEST:
        INCUMBENT_REQUEST_AIRPORT[p,r] = {}
        
        for i,j in REQUEST_TRIP0[r]:
          if value_of_x[i,j,r,p] > 0:
            INCUMBENT_REQUEST_AIRPORT[p,r][i] = 1
            INCUMBENT_REQUEST_AIRPORT[p,r][j] = 1
    
    
    INCUMBENT_REQUEST_TRIP_PLUS = {}

    for p in PLANE:
      for r in REQUEST:
        INCUMBENT_REQUEST_TRIP_PLUS[p,r] = {}
    
        for i,j in REQUEST_TRIP0[r]:
          if value_of_x[i,j,r,p] > 0:
            INCUMBENT_REQUEST_TRIP_PLUS[p,r][i,j] = 1
          
        for i in INCUMBENT_REQUEST_AIRPORT[p,r]:
          if i != REQUEST[r].request_departure and i != REQUEST[r].request_arrival:
            INCUMBENT_REQUEST_TRIP_PLUS[p,r][i,i] = 1
    
    
    # INCUMBENT CHECKER MODEL
    
    for p in PLANE:
    
      #print "checking plane ",p
      
      # set up MIP model
        
      incumb_model = cplex.Cplex()
        
                
      # Variables
    
      incumbent_x = {}
      number_of_incumbent_x = 0
      thebinaryvars = []

      for r in INCUMBENT_REQUEST[p]:
        for i,j in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
          for t in TIMESTEP:
            if t in REQUEST_TIMESTEP[r]:
              incumbent_x[i,j,t,r] = "x#" + i + "_" + j + "_" + str(t) + "_" + r
              ##print incumbent_x[i,j,t,r] 
              thebinaryvars.append(incumbent_x[i,j,t,r])
              number_of_incumbent_x += 1
                    
      #print "number of 'x' variables:",number_of_incumbent_x
            
      incumbent_x_dep = {}
      number_of_incumbent_x_dep = 0
      
      for r in INCUMBENT_REQUEST[p]:
        for t in TIMESTEP:
          if t in REQUEST_TIMESTEP[r]:
            incumbent_x_dep[t,r] = "x_dep#" + str(t) + "_" + r
            thebinaryvars.append(incumbent_x_dep[t,r])
            number_of_incumbent_x_dep += 1
                  
      #print "number of 'x_dep' variables:",number_of_incumbent_x_dep

      incumbent_x_arr = {}
      number_of_incumbent_x_arr = 0

      for r in INCUMBENT_REQUEST[p]:
        for t in TIMESTEP:
          if t in REQUEST_TIMESTEP[r]:
            incumbent_x_arr[t,r] = "x_arr#" + str(t) + "_" + r
            thebinaryvars.append(incumbent_x_arr[t,r])
            number_of_incumbent_x_arr += 1
            
      #print "number of 'x_arr' variables:",number_of_incumbent_x_arr
      
      incumbent_y = {}
      number_of_incumbent_y = 0

      for i,j in INCUMBENT_TRIP_PLUS[p]:
        for t in PLANE_TIMESTEP[p]:
          incumbent_y[i,j,t] = "y#" + i + "_" + j + "_" + str(t)
          thebinaryvars.append(incumbent_y[i,j,t])
          number_of_incumbent_y += 1
          
      #print "number of 'y' variables:",number_of_incumbent_y

      incumbent_y_dep = {}
      number_of_incumbent_y_dep = 0
      
      for t in PLANE_TIMESTEP[p]:
        incumbent_y_dep[t] = "y_dep#" + str(t)
        thebinaryvars.append(incumbent_y_dep[t])
        number_of_incumbent_y_dep += 1
        
      #print "number of 'y_dep' variables:",number_of_incumbent_y_dep
      
      incumbent_y_arr = {}
      number_of_incumbent_y_arr = 0
      
      for t in PLANE_TIMESTEP[p]:
        incumbent_y_arr[t] = "y_arr#" + str(t)
        thebinaryvars.append(incumbent_y_arr[t])
        number_of_incumbent_y_arr += 1
        
      #print "number of 'y_arr' variables:",number_of_incumbent_y_arr
      
      incumbent_f = {}
      number_of_incumbent_f = 0
      thecontvars = []

      for i,j in INCUMBENT_TRIP_PLUS[p]:
        for t in PLANE_TIMESTEP[p]:
          incumbent_f[i,j,t] = "f#" + i + "_" + j + "_" + str(t)
          thecontvars.append(incumbent_f[i,j,t])
          number_of_incumbent_f += 1

      #print "number of 'f' variables:",number_of_incumbent_f
      
      incumbent_f_dep = {}
      number_of_incumbent_f_dep = 0

      for t in PLANE_TIMESTEP[p]:
        incumbent_f_dep[t] = "f_dep#" + str(t)
        thecontvars.append(incumbent_f_dep[t])
        number_of_incumbent_f_dep += 1

      #print "number of 'f_dep' variables:",number_of_incumbent_f_dep
      
      incumbent_f_arr = {}
      number_of_incumbent_f_arr = 0
      
      for t in PLANE_TIMESTEP[p]:
        incumbent_f_arr[t] = "f_arr#" + str(t)
        thecontvars.append(incumbent_f_arr[t])
        number_of_incumbent_f_arr += 1
        
      #print "number of 'f_arr' variables:",number_of_incumbent_f_arr

      incumbent_w = {}
      number_of_incumbent_w = 0

      for t in PLANE_TIMESTEP[p]:
        incumbent_w[t] = "w#" + str(t)
        thecontvars.append(incumbent_w[t])
        number_of_incumbent_w += 1
        
      #print "number of 'w' variables:",number_of_incumbent_w

      # add all variables to model
      
      incumb_model.variables.add(names = thebinaryvars, lb = [0] * len(thebinaryvars), ub = [1] * len(thebinaryvars), types = ["B"] * len(thebinaryvars))

      incumb_model.variables.add(names = thecontvars, lb = [0] * len(thecontvars), types = ["C"] * len(thecontvars))


      # Constraints
      
      thenames = []
      thelin_exprs = []
      thesenses = []
      therhss = []
      
      # schedule of requests
      
      number_of_request_one_dep = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
        for t in REQUEST_TIMESTEP[r]:
          thevars.append(incumbent_x_dep[t,r])
          thecoefs.append(1.0)
          
        if len(thevars) > 0:
          thenames.append("request_one_dep_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(1.0)
          number_of_request_one_dep += 1

      #print "number of 'each request must depart' constraints:",number_of_request_one_dep

      #
      
      number_of_request_one_arr = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
        for t in REQUEST_TIMESTEP[r]:
          thevars.append(incumbent_x_arr[t,r])
          thecoefs.append(1.0);
        
        if len(thevars) > 0:
          thenames.append("request_one_arr_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(1.0)
          number_of_request_one_arr += 1
        
      #print "number of 'each request must arrive' constraints:",number_of_request_one_arr

      # 
      
      number_of_request_flow = 0
      
      for j in AIRPORT:
        for r in INCUMBENT_REQUEST[p]:
          for t in REQUEST_TIMESTEP[r]:
            thevars = []
            thecoefs = []

            for i in AIRPORT:
              if (i,j) in REQUEST_TRIP[r] and (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
                thevars.append(incumbent_x[i,j,t,r])
                thecoefs.append(1.0)
      
            if j == REQUEST[r].request_departure:
              thevars.append(incumbent_x_dep[t,r])
              thecoefs.append(1.0)
        
            for k in AIRPORT:
              if (j,k) in REQUEST_TRIP[r] and (j,k) in INCUMBENT_REQUEST_TRIP_PLUS[p,r] and (t + turnover_travel_timesteps[j,k,p]) in REQUEST_TIMESTEP[r]:
                thevars.append(incumbent_x[j,k,(t + turnover_travel_timesteps[j,k,p]),r])
                thecoefs.append(-1.0)
          
            if j == REQUEST[r].request_arrival:
              thevars.append(incumbent_x_arr[t,r])
              thecoefs.append(-1.0)
        
            if len(thevars) > 0:
              thenames.append("request_flow_" + j + "_" + r + "_" + str(t))
              thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
              thesenses.append("E")
              therhss.append(0.0)

              number_of_request_flow += 1

      #print "number of 'request flow' constraints:",number_of_request_flow


      # schedule of planes
            
      thevars = []
      thecoefs = []
      
      for t in PLANE_TIMESTEP[p]:
        thevars.append(incumbent_y_dep[t])
        thecoefs.append(1.0)
      
      thenames.append("plane_one_dep")
      thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
      thesenses.append("E")
      therhss.append(1.0)

      #print "number of 'airplane one departure' constraints: 1"
      
      #
      
      thevars = []
      thecoefs = []
      
      for t in PLANE_TIMESTEP[p]:
        thevars.append(incumbent_y_arr[t])
        thecoefs.append(1.0)
      
      thenames.append("plane_one_arr")
      thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
      thesenses.append("E")
      therhss.append(1.0)
      
      #print "number of 'airplane one arrival' constraints: 1"
      
      #
      
      number_of_plane_flow = 0
      
      for j in AIRPORT:
        for t in PLANE_TIMESTEP[p]:
          #print p,i
          thevars = []
          thecoefs = []
        
          for i in AIRPORT:
            if (i,j) in INCUMBENT_TRIP_PLUS[p]:
              thevars.append(incumbent_y[i,j,t])
              thecoefs.append(1.0)
        
          if j == PLANE[p].plane_departure:
            thevars.append(incumbent_y_dep[t])
            thecoefs.append(1.0)
      
          for k in AIRPORT:
            if (j,k) in INCUMBENT_TRIP_PLUS[p] and (t + turnover_travel_timesteps[j,k,p]) in PLANE_TIMESTEP[p]:
              thevars.append(incumbent_y[j,k,(t + turnover_travel_timesteps[j,k,p])])
              thecoefs.append(-1.0)
    
          if j == PLANE[p].plane_arrival:
            thevars.append(incumbent_y_arr[t])
            thecoefs.append(-1.0)
    
          if len(thevars) > 0:
            thenames.append("plane_flow_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            thesenses.append("E")
            therhss.append(0.0)

            number_of_plane_flow += 1
  
      #print "number of 'plane flow' constraints:",number_of_plane_flow
      

      # seat limit of planes
      
      number_of_seat_limit = 0
      
      for i,j in INCUMBENT_TRIP[p]:
        for t in PLANE_TIMESTEP[p]:
          #print i,j,p
          thevars = [incumbent_y[i,j,t]]
          thecoefs = [-PLANE[p].seats]
    
          for r in INCUMBENT_REQUEST[p]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r] and t in REQUEST_TIMESTEP[r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(REQUEST[r].passengers)
      
          if len(thevars) > 0:
            thenames.append("seatlimit_" + i + "_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            thesenses.append("L")
            therhss.append(0.0)

            number_of_seat_limit += 1

      #print "number of 'seat limit' constraints:",number_of_seat_limit

      
      # intermediate stop limit for passengers
      
      number_of_intermediate_stops = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
        #print r
    
        for i,j in REQUEST_TRIP[r]:
          for t in REQUEST_TIMESTEP[r]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(1.0);
            
        if len(thevars) > 0:
          thenames.append("intermediate_stops_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("L")
          therhss.append(REQUEST[r].max_stops + 1)

          number_of_intermediate_stops += 1
          
      #print "number of 'intermediate stops' constraints:",number_of_intermediate_stops
      

      # maximum detour for passengers (compared to direct flight)
      
      number_of_maxdetour = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
  
        for i,j in REQUEST_TRIP[r]:
          for t in REQUEST_TIMESTEP[r]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(TRIP[i,j].distance)
      
        if len(thevars) > 0:
          thenames.append("max_detour_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("L")
          therhss.append((1 + REQUEST[r].max_detour) * TRIP0[REQUEST[r].request_departure,REQUEST[r].request_arrival].distance)

          number_of_maxdetour += 1
          
      #print "number of 'max detour' constraints:",number_of_maxdetour
      

      # fueling constraints
      
      number_of_noflight_nofuel = 0
      
      for i,j in INCUMBENT_TRIP_PLUS[p]:
        for t in PLANE_TIMESTEP[p]:
          #print i,j,p
          thevars = [incumbent_f[i,j,t],incumbent_y[i,j,t]]
          thecoefs = [1.0,-max_trip_fuel[i,j,p]]
          
          if len(thevars) > 0:
            thenames.append("noflight_nofuel_" + i + "_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            thesenses.append("L")
            therhss.append(0.0)

            number_of_noflight_nofuel += 1
            
      #print "number of 'no flight, no fuel' constraints:",number_of_noflight_nofuel
      
      #
      
      number_of_fuelconsumption = 0
      number_of_refueling = 0

      for j in AIRPORT:
        for t in PLANE_TIMESTEP[p]:
          ##print j,p
          thevars = []
          thecoefs = []
            
          for i in AIRPORT:
            if (i,j) in INCUMBENT_TRIP_PLUS[p]:
              thevars.append(incumbent_f[i,j,t])
              thecoefs.append(1.0)
            
          if j == PLANE[p].plane_departure:
            thevars.append(incumbent_f_dep[t])
            thecoefs.append(1.0)
            
          for k in AIRPORT:
            if (j,k) in INCUMBENT_TRIP_PLUS[p] and (t + turnover_travel_timesteps[j,k,p]) in PLANE_TIMESTEP[p]:
              thevars.append(incumbent_f[j,k,(t + turnover_travel_timesteps[j,k,p])])
              thecoefs.append(-1.0)
        
              thevars.append(incumbent_y[j,k,(t + turnover_travel_timesteps[j,k,p])])
              thecoefs.append(-fuelconsumption[j,k,p])
              
          if j == PLANE[p].plane_arrival:
            thevars.append(incumbent_f_arr[t])
            thecoefs.append(-1.0)
          
          if len(thevars) > 0:
            thenames.append("fuelconsumption_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            therhss.append(0.0)

            if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '0':
              thesenses.append("E")
              number_of_fuelconsumption += 1
            else:
              thesenses.append("L")
              number_of_refueling += 1

      #print "number of 'fuelconsumption' constraints:",number_of_fuelconsumption
      #print "number of 'refueling' constraints:",number_of_refueling

      
      # fuel limits at departure and arrival nodes
      
      number_of_departure_fuel_min = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_dep[t],incumbent_f_dep[t]]
        thecoefs = [PLANE[p].departure_min_fuel, -1.0]
        thenames.append("departure_fuel_min_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_departure_fuel_min += 1
        
      #print "number of 'fuel min limit at departure' constraints:",number_of_departure_fuel_min
      
      #
      
      number_of_departure_fuel_max = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_dep[t],incumbent_f_dep[t]]
        thecoefs = [-PLANE[p].departure_max_fuel, 1.0]
        thenames.append("departure_fuel_max_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_departure_fuel_max += 1
        
      #print "number of 'fuel max limit at departure' constraints:",number_of_departure_fuel_max
      
      #
      
      number_of_arrival_fuel_min = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_arr[t],incumbent_f_arr[t]]
        thecoefs = [PLANE[p].arrival_min_fuel, -1.0]
        thenames.append("arrival_fuel_min_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_arrival_fuel_min += 1
        
      #print "number of 'fuel min limit at arrival' constraints:",number_of_arrival_fuel_min
      
      #
      
      number_of_arrival_fuel_max = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_arr[t],incumbent_f_arr[t]]
        thecoefs = [-PLANE[p].arrival_max_fuel, 1.0]
        thenames.append("arrival_fuel_max_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_arrival_fuel_max += 1
        
      #print "number of 'fuel max limit at arrival' constraints:",number_of_arrival_fuel_max
      
      
      # number of trips between airports

      number_of_trips_between_airports = 0
      
      for i,j in INCUMBENT_TRIP[p]:
        thevars = []
        thecoefs = []
        for t in PLANE_TIMESTEP[p]:
          thevars.append(incumbent_y[i,j,t])
          thecoefs.append(1.0)
        if len(thevars) > 0:
          thenames.append("trips_between_airports_" + i + "_" + j)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(trips_between_airports[p][i,j])

          number_of_trips_between_airports += 1
          
      #print "number of 'trips between airports' constraints:",number_of_trips_between_airports
      

      # weight limits
      
      number_of_computeweight = 0
      
      for t in PLANE_TIMESTEP[p]:
        #print i,j,p
        thevars = [incumbent_w[t]]
        thecoefs = [1.0]

        for i,j in INCUMBENT_TRIP[p]:
          for r in INCUMBENT_REQUEST[p]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r] and t in REQUEST_TIMESTEP[r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(-REQUEST[r].weight)

          thevars.append(incumbent_f[i,j,t])
          thecoefs.append(-1.0)
        
        if len(thevars) > 0:
          thenames.append("computeweight_" + str(t))
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(0.0)

          number_of_computeweight += 1
            
      #print "number of 'computeweight' constraints:",number_of_computeweight
      
      #
      
      number_of_weightlimit = 0
      
      for t in PLANE_TIMESTEP[p]:
        #print i,j,p
        thevars = [incumbent_w[t]]
        thecoefs = [1.0]

        for i,j in INCUMBENT_TRIP[p]:
          thevars.append(incumbent_y[i,j,t])
          thecoefs.append(-max_trip_payload[i,j,p])

        if len(thevars) > 0:
          thenames.append("weightlimit_" + str(t))
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("L")
          therhss.append(0.0)

          number_of_weightlimit += 1
            
      #print "number of 'weightlimit' constraints:",number_of_weightlimit
    
    
      # add all constraints
      
      incumb_model.linear_constraints.add(names = thenames, lin_expr = thelin_exprs, senses = thesenses, rhs = therhss)
      
      
      # OBJECTIVE FUNCTION 

      incumb_model.objective.set_sense(model.objective.sense.minimize)


      # output model

      #incumb_model.write("INCMOD/incumbent_model." + p + "." + str(self.number_of_calls) + ".lp")

      # surpress output 
            
      incumb_model.set_results_stream(None,fn=None)
            
      # solve model
      
      cb_t0 = time.clock()

      incumb_model.solve()

      self.callback_time += time.clock() - cb_t0
    
      # solution interpretation

      incumb_solution = incumb_model.solution

      if not incumb_solution.is_primal_feasible():
        #print "solution for plane",p,"infeasible"
        self.reject()
        #print time.clock() - cb_t0
        
        # clean up & return
        
        del incumb_model;
        del thevars;
        del thecoefs;
        del thenames;
        del thesenses;
        del therhss;
        del thelin_exprs;
        del incumb_solution;
        
        return
      
      #incumb_model.write("incumbent_model." + p + ".lp")


    #print "feasible solution found at call: ",self.number_of_calls
    
    #if self.number_of_calls > 1000:
    #  exit()
    #print time.clock() - cb_t0

    del incumb_model;
    del thevars;
    del thecoefs;
    del thenames;
    del thesenses;
    del therhss;
    del thelin_exprs;
    del incumb_solution;
        
    return

        
class InfeasibleScheduleMIPCallback(LazyConstraintCallback):

  def __call__(self):
    
    self.number_of_calls += 1

    #print "Check Solution with MIP (IncumbentCallback) ",self.number_of_calls
      
    # get incumbent solution

    value_of_y = {}

    for i,j in TRIP0:
      for p in PLANE:
        value_of_y[i,j,p] = int(round(self.get_values(y[i,j,p])))
        #if value_of_y[i,j,p] > 0:
          #print "  from",i,"to",j,"plane",p,"   (",value_of_y[i,j,p],")"

    value_of_x = {}

    for r in REQUEST:
      for i,j in REQUEST_TRIP0[r]:
        for p in PLANE:
          value_of_x[i,j,r,p] = int(round(self.get_values(x[i,j,r,p])))
          #if value_of_x[i,j,r,p] > 0:
          #  print "  from",i,"to",j,"request",r,"plane",p
            
    value_of_x_dep = {}
    
    for r in REQUEST:
      for p in PLANE:
        value_of_x_dep[r,p] = int(round(self.get_values(x_dep[r,p])))
        #if value_of_x_dep[r,p] > 0:
          #print "  request",r,"plane",p 
    
    value_of_x_arr = {}
        
    for r in REQUEST:
      for p in PLANE:
        value_of_x_arr[r,p] = int(round(self.get_values(x_arr[r,p])))
        #if value_of_x_arr[r,p] > 0:
          #print "  request",r,"plane",p
    
    # print data on screen
    
    #print
    #
    #for p in PLANE:
    #  print "# plane",p
    #  for i,j in TRIP0:
    #    if value_of_y[i,j,p] > 0:
    #      print p,i,j,value_of_y[i,j,p]
    #
    #print
    #
    #for p in PLANE:
    #  print "# plane",p
    #  for r in REQUEST:
    #    if value_of_x_arr[r,p] > 0:
    #      print p,r;
    #      
    #print
    #
    #for p in PLANE:
    #  print "# plane",p
    #  for r in REQUEST:
    #    for i,j in REQUEST_TRIP0[r]:
    #      if value_of_x[i,j,r,p] > 0:
    #        print p,r,i,j
    #
    #print "\n----------------------------------\n\n"
    
    # define sets and parameters from incumbent 
    
    INCUMBENT_TRIP = {}
    trips_between_airports = {}
    
    for p in PLANE:
      INCUMBENT_TRIP[p] = {}
      trips_between_airports[p] = {}
      
      for i,j in TRIP0:
        if value_of_y[i,j,p] > 0:
          INCUMBENT_TRIP[p][i,j] = 1
          trips_between_airports[p][i,j] = value_of_y[i,j,p]
    
                
    INCUMBENT_AIRPORT = {}
    
    for p in PLANE:
      INCUMBENT_AIRPORT[p] = {}
      for i,j in INCUMBENT_TRIP[p]:
        INCUMBENT_AIRPORT[p][i] = 1
        INCUMBENT_AIRPORT[p][j] = 1
    
    
    INCUMBENT_TRIP_PLUS = {}
    
    for p in PLANE:
      INCUMBENT_TRIP_PLUS[p] = {}
      for i,j in INCUMBENT_TRIP[p]:
        INCUMBENT_TRIP_PLUS[p][i,j] = 1
      
      for i in INCUMBENT_AIRPORT[p]:
        INCUMBENT_TRIP_PLUS[p][i,i] = 1
    
    
    INCUMBENT_REQUEST = {}
    
    for p in PLANE:
      INCUMBENT_REQUEST[p] = {}
      
      for r in REQUEST:
        if value_of_x_arr[r,p] > 0:
          INCUMBENT_REQUEST[p][r] = 1
    
    
    INCUMBENT_REQUEST_AIRPORT = {}
    
    for p in PLANE:
      for r in REQUEST:
        INCUMBENT_REQUEST_AIRPORT[p,r] = {}
        
        for i,j in REQUEST_TRIP0[r]:
          if value_of_x[i,j,r,p] > 0:
            INCUMBENT_REQUEST_AIRPORT[p,r][i] = 1
            INCUMBENT_REQUEST_AIRPORT[p,r][j] = 1
    
    
    INCUMBENT_REQUEST_TRIP_PLUS = {}

    for p in PLANE:
      for r in REQUEST:
        INCUMBENT_REQUEST_TRIP_PLUS[p,r] = {}
    
        for i,j in REQUEST_TRIP0[r]:
          if value_of_x[i,j,r,p] > 0:
            INCUMBENT_REQUEST_TRIP_PLUS[p,r][i,j] = 1
          
        for i in INCUMBENT_REQUEST_AIRPORT[p,r]:
          if i != REQUEST[r].request_departure and i != REQUEST[r].request_arrival:
            INCUMBENT_REQUEST_TRIP_PLUS[p,r][i,i] = 1
    
    
    # INCUMBENT CHECKER MODEL
    
    for p in PLANE:
    
      #print "checking plane ",p
      
      # set up MIP model
        
      incumb_model = cplex.Cplex()
        
                
      # Variables
    
      incumbent_x = {}
      number_of_incumbent_x = 0
      thebinaryvars = []

      for r in INCUMBENT_REQUEST[p]:
        for i,j in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
          for t in TIMESTEP:
            if t in REQUEST_TIMESTEP[r]:
              incumbent_x[i,j,t,r] = "x#" + i + "_" + j + "_" + str(t) + "_" + r
              ##print incumbent_x[i,j,t,r] 
              thebinaryvars.append(incumbent_x[i,j,t,r])
              number_of_incumbent_x += 1
                    
      #print "number of 'x' variables:",number_of_incumbent_x
            
      incumbent_x_dep = {}
      number_of_incumbent_x_dep = 0
      
      for r in INCUMBENT_REQUEST[p]:
        for t in TIMESTEP:
          if t in REQUEST_TIMESTEP[r]:
            incumbent_x_dep[t,r] = "x_dep#" + str(t) + "_" + r
            thebinaryvars.append(incumbent_x_dep[t,r])
            number_of_incumbent_x_dep += 1
                  
      #print "number of 'x_dep' variables:",number_of_incumbent_x_dep

      incumbent_x_arr = {}
      number_of_incumbent_x_arr = 0

      for r in INCUMBENT_REQUEST[p]:
        for t in TIMESTEP:
          if t in REQUEST_TIMESTEP[r]:
            incumbent_x_arr[t,r] = "x_arr#" + str(t) + "_" + r
            thebinaryvars.append(incumbent_x_arr[t,r])
            number_of_incumbent_x_arr += 1
            
      #print "number of 'x_arr' variables:",number_of_incumbent_x_arr
      
      incumbent_y = {}
      number_of_incumbent_y = 0

      for i,j in INCUMBENT_TRIP_PLUS[p]:
        for t in PLANE_TIMESTEP[p]:
          incumbent_y[i,j,t] = "y#" + i + "_" + j + "_" + str(t)
          thebinaryvars.append(incumbent_y[i,j,t])
          number_of_incumbent_y += 1
          
      #print "number of 'y' variables:",number_of_incumbent_y

      incumbent_y_dep = {}
      number_of_incumbent_y_dep = 0
      
      for t in PLANE_TIMESTEP[p]:
        incumbent_y_dep[t] = "y_dep#" + str(t)
        thebinaryvars.append(incumbent_y_dep[t])
        number_of_incumbent_y_dep += 1
        
      #print "number of 'y_dep' variables:",number_of_incumbent_y_dep
      
      incumbent_y_arr = {}
      number_of_incumbent_y_arr = 0
      
      for t in PLANE_TIMESTEP[p]:
        incumbent_y_arr[t] = "y_arr#" + str(t)
        thebinaryvars.append(incumbent_y_arr[t])
        number_of_incumbent_y_arr += 1
        
      #print "number of 'y_arr' variables:",number_of_incumbent_y_arr
      
      incumbent_f = {}
      number_of_incumbent_f = 0
      thecontvars = []

      for i,j in INCUMBENT_TRIP_PLUS[p]:
        for t in PLANE_TIMESTEP[p]:
          incumbent_f[i,j,t] = "f#" + i + "_" + j + "_" + str(t)
          thecontvars.append(incumbent_f[i,j,t])
          number_of_incumbent_f += 1

      #print "number of 'f' variables:",number_of_incumbent_f
      
      incumbent_f_dep = {}
      number_of_incumbent_f_dep = 0

      for t in PLANE_TIMESTEP[p]:
        incumbent_f_dep[t] = "f_dep#" + str(t)
        thecontvars.append(incumbent_f_dep[t])
        number_of_incumbent_f_dep += 1

      #print "number of 'f_dep' variables:",number_of_incumbent_f_dep
      
      incumbent_f_arr = {}
      number_of_incumbent_f_arr = 0
      
      for t in PLANE_TIMESTEP[p]:
        incumbent_f_arr[t] = "f_arr#" + str(t)
        thecontvars.append(incumbent_f_arr[t])
        number_of_incumbent_f_arr += 1
        
      #print "number of 'f_arr' variables:",number_of_incumbent_f_arr

      incumbent_w = {}
      number_of_incumbent_w = 0

      for t in PLANE_TIMESTEP[p]:
        incumbent_w[t] = "w#" + str(t)
        thecontvars.append(incumbent_w[t])
        number_of_incumbent_w += 1
        
      #print "number of 'w' variables:",number_of_incumbent_w

      # add all variables to model
      
      incumb_model.variables.add(names = thebinaryvars, lb = [0] * len(thebinaryvars), ub = [1] * len(thebinaryvars), types = ["B"] * len(thebinaryvars))

      incumb_model.variables.add(names = thecontvars, lb = [0] * len(thecontvars), types = ["C"] * len(thecontvars))


      # Constraints
      
      thenames = []
      thelin_exprs = []
      thesenses = []
      therhss = []
      
      # schedule of requests
      
      number_of_request_one_dep = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
        for t in REQUEST_TIMESTEP[r]:
          thevars.append(incumbent_x_dep[t,r])
          thecoefs.append(1.0)
          
        if len(thevars) > 0:
          thenames.append("request_one_dep_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(1.0)
          number_of_request_one_dep += 1

      #print "number of 'each request must depart' constraints:",number_of_request_one_dep

      #
      
      number_of_request_one_arr = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
        for t in REQUEST_TIMESTEP[r]:
          thevars.append(incumbent_x_arr[t,r])
          thecoefs.append(1.0);
        
        if len(thevars) > 0:
          thenames.append("request_one_arr_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(1.0)
          number_of_request_one_arr += 1
        
      #print "number of 'each request must arrive' constraints:",number_of_request_one_arr

      # 
      
      number_of_request_flow = 0
      
      for j in AIRPORT:
        for r in INCUMBENT_REQUEST[p]:
          for t in REQUEST_TIMESTEP[r]:
            thevars = []
            thecoefs = []

            for i in AIRPORT:
              if (i,j) in REQUEST_TRIP[r] and (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
                thevars.append(incumbent_x[i,j,t,r])
                thecoefs.append(1.0)
      
            if j == REQUEST[r].request_departure:
              thevars.append(incumbent_x_dep[t,r])
              thecoefs.append(1.0)
        
            for k in AIRPORT:
              if (j,k) in REQUEST_TRIP[r] and (j,k) in INCUMBENT_REQUEST_TRIP_PLUS[p,r] and (t + turnover_travel_timesteps[j,k,p]) in REQUEST_TIMESTEP[r]:
                thevars.append(incumbent_x[j,k,(t + turnover_travel_timesteps[j,k,p]),r])
                thecoefs.append(-1.0)
          
            if j == REQUEST[r].request_arrival:
              thevars.append(incumbent_x_arr[t,r])
              thecoefs.append(-1.0)
        
            if len(thevars) > 0:
              thenames.append("request_flow_" + j + "_" + r + "_" + str(t))
              thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
              thesenses.append("E")
              therhss.append(0.0)

              number_of_request_flow += 1

      #print "number of 'request flow' constraints:",number_of_request_flow


      # schedule of planes
            
      thevars = []
      thecoefs = []
      
      for t in PLANE_TIMESTEP[p]:
        thevars.append(incumbent_y_dep[t])
        thecoefs.append(1.0)
      
      thenames.append("plane_one_dep")
      thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
      thesenses.append("E")
      therhss.append(1.0)

      #print "number of 'airplane one departure' constraints: 1"
      
      #
      
      thevars = []
      thecoefs = []
      
      for t in PLANE_TIMESTEP[p]:
        thevars.append(incumbent_y_arr[t])
        thecoefs.append(1.0)
      
      thenames.append("plane_one_arr")
      thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
      thesenses.append("E")
      therhss.append(1.0)
      
      #print "number of 'airplane one arrival' constraints: 1"
      
      #
      
      number_of_plane_flow = 0
      
      for j in AIRPORT:
        for t in PLANE_TIMESTEP[p]:
          #print p,i
          thevars = []
          thecoefs = []
        
          for i in AIRPORT:
            if (i,j) in INCUMBENT_TRIP_PLUS[p]:
              thevars.append(incumbent_y[i,j,t])
              thecoefs.append(1.0)
        
          if j == PLANE[p].plane_departure:
            thevars.append(incumbent_y_dep[t])
            thecoefs.append(1.0)
      
          for k in AIRPORT:
            if (j,k) in INCUMBENT_TRIP_PLUS[p] and (t + turnover_travel_timesteps[j,k,p]) in PLANE_TIMESTEP[p]:
              thevars.append(incumbent_y[j,k,(t + turnover_travel_timesteps[j,k,p])])
              thecoefs.append(-1.0)
    
          if j == PLANE[p].plane_arrival:
            thevars.append(incumbent_y_arr[t])
            thecoefs.append(-1.0)
    
          if len(thevars) > 0:
            thenames.append("plane_flow_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            thesenses.append("E")
            therhss.append(0.0)

            number_of_plane_flow += 1
  
      #print "number of 'plane flow' constraints:",number_of_plane_flow
      

      # seat limit of planes
      
      number_of_seat_limit = 0
      
      for i,j in INCUMBENT_TRIP[p]:
        for t in PLANE_TIMESTEP[p]:
          #print i,j,p
          thevars = [incumbent_y[i,j,t]]
          thecoefs = [-PLANE[p].seats]
    
          for r in INCUMBENT_REQUEST[p]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r] and t in REQUEST_TIMESTEP[r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(REQUEST[r].passengers)
      
          if len(thevars) > 0:
            thenames.append("seatlimit_" + i + "_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            thesenses.append("L")
            therhss.append(0.0)

            number_of_seat_limit += 1

      #print "number of 'seat limit' constraints:",number_of_seat_limit

      
      # intermediate stop limit for passengers
      
      number_of_intermediate_stops = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
        #print r
    
        for i,j in REQUEST_TRIP[r]:
          for t in REQUEST_TIMESTEP[r]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(1.0);
            
        if len(thevars) > 0:
          thenames.append("intermediate_stops_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("L")
          therhss.append(REQUEST[r].max_stops + 1)

          number_of_intermediate_stops += 1
          
      #print "number of 'intermediate stops' constraints:",number_of_intermediate_stops
      

      # maximum detour for passengers (compared to direct flight)
      
      number_of_maxdetour = 0
      
      for r in INCUMBENT_REQUEST[p]:
        thevars = []
        thecoefs = []
  
        for i,j in REQUEST_TRIP[r]:
          for t in REQUEST_TIMESTEP[r]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(TRIP[i,j].distance)
      
        if len(thevars) > 0:
          thenames.append("max_detour_" + r)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("L")
          therhss.append((1 + REQUEST[r].max_detour) * TRIP0[REQUEST[r].request_departure,REQUEST[r].request_arrival].distance)

          number_of_maxdetour += 1
          
      #print "number of 'max detour' constraints:",number_of_maxdetour
      

      # fueling constraints
      
      number_of_noflight_nofuel = 0
      
      for i,j in INCUMBENT_TRIP_PLUS[p]:
        for t in PLANE_TIMESTEP[p]:
          #print i,j,p
          thevars = [incumbent_f[i,j,t],incumbent_y[i,j,t]]
          thecoefs = [1.0,-max_trip_fuel[i,j,p]]
          
          if len(thevars) > 0:
            thenames.append("noflight_nofuel_" + i + "_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            thesenses.append("L")
            therhss.append(0.0)

            number_of_noflight_nofuel += 1
            
      #print "number of 'no flight, no fuel' constraints:",number_of_noflight_nofuel
      
      #
      
      number_of_fuelconsumption = 0
      number_of_refueling = 0

      for j in AIRPORT:
        for t in PLANE_TIMESTEP[p]:
          ##print j,p
          thevars = []
          thecoefs = []
            
          for i in AIRPORT:
            if (i,j) in INCUMBENT_TRIP_PLUS[p]:
              thevars.append(incumbent_f[i,j,t])
              thecoefs.append(1.0)
            
          if j == PLANE[p].plane_departure:
            thevars.append(incumbent_f_dep[t])
            thecoefs.append(1.0)
            
          for k in AIRPORT:
            if (j,k) in INCUMBENT_TRIP_PLUS[p] and (t + turnover_travel_timesteps[j,k,p]) in PLANE_TIMESTEP[p]:
              thevars.append(incumbent_f[j,k,(t + turnover_travel_timesteps[j,k,p])])
              thecoefs.append(-1.0)
        
              thevars.append(incumbent_y[j,k,(t + turnover_travel_timesteps[j,k,p])])
              thecoefs.append(-fuelconsumption[j,k,p])
              
          if j == PLANE[p].plane_arrival:
            thevars.append(incumbent_f_arr[t])
            thecoefs.append(-1.0)
          
          if len(thevars) > 0:
            thenames.append("fuelconsumption_" + j + "_" + str(t))
            thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
            therhss.append(0.0)

            if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '0':
              thesenses.append("E")
              number_of_fuelconsumption += 1
            else:
              thesenses.append("L")
              number_of_refueling += 1

      #print "number of 'fuelconsumption' constraints:",number_of_fuelconsumption
      #print "number of 'refueling' constraints:",number_of_refueling

      
      # fuel limits at departure and arrival nodes
      
      number_of_departure_fuel_min = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_dep[t],incumbent_f_dep[t]]
        thecoefs = [PLANE[p].departure_min_fuel, -1.0]
        thenames.append("departure_fuel_min_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_departure_fuel_min += 1
        
      #print "number of 'fuel min limit at departure' constraints:",number_of_departure_fuel_min
      
      #
      
      number_of_departure_fuel_max = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_dep[t],incumbent_f_dep[t]]
        thecoefs = [-PLANE[p].departure_max_fuel, 1.0]
        thenames.append("departure_fuel_max_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_departure_fuel_max += 1
        
      #print "number of 'fuel max limit at departure' constraints:",number_of_departure_fuel_max
      
      #
      
      number_of_arrival_fuel_min = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_arr[t],incumbent_f_arr[t]]
        thecoefs = [PLANE[p].arrival_min_fuel, -1.0]
        thenames.append("arrival_fuel_min_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_arrival_fuel_min += 1
        
      #print "number of 'fuel min limit at arrival' constraints:",number_of_arrival_fuel_min
      
      #
      
      number_of_arrival_fuel_max = 0
            
      for t in PLANE_TIMESTEP[p]:
        thevars = [incumbent_y_arr[t],incumbent_f_arr[t]]
        thecoefs = [-PLANE[p].arrival_max_fuel, 1.0]
        thenames.append("arrival_fuel_max_" + str(t))
        thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
        thesenses.append("L")
        therhss.append(0.0)

        number_of_arrival_fuel_max += 1
        
      #print "number of 'fuel max limit at arrival' constraints:",number_of_arrival_fuel_max
      
      
      # number of trips between airports

      number_of_trips_between_airports = 0
      
      for i,j in INCUMBENT_TRIP[p]:
        thevars = []
        thecoefs = []
        for t in PLANE_TIMESTEP[p]:
          thevars.append(incumbent_y[i,j,t])
          thecoefs.append(1.0)
        if len(thevars) > 0:
          thenames.append("trips_between_airports_" + i + "_" + j)
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(trips_between_airports[p][i,j])

          number_of_trips_between_airports += 1
          
      #print "number of 'trips between airports' constraints:",number_of_trips_between_airports
      

      # weight limits
      
      number_of_computeweight = 0
      
      for t in PLANE_TIMESTEP[p]:
        #print i,j,p
        thevars = [incumbent_w[t]]
        thecoefs = [1.0]

        for i,j in INCUMBENT_TRIP[p]:
          for r in INCUMBENT_REQUEST[p]:
            if (i,j) in INCUMBENT_REQUEST_TRIP_PLUS[p,r] and t in REQUEST_TIMESTEP[r]:
              thevars.append(incumbent_x[i,j,t,r])
              thecoefs.append(-REQUEST[r].weight)

          thevars.append(incumbent_f[i,j,t])
          thecoefs.append(-1.0)
        
        if len(thevars) > 0:
          thenames.append("computeweight_" + str(t))
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("E")
          therhss.append(0.0)

          number_of_computeweight += 1
            
      #print "number of 'computeweight' constraints:",number_of_computeweight
      
      #
      
      number_of_weightlimit = 0
      
      for t in PLANE_TIMESTEP[p]:
        #print i,j,p
        thevars = [incumbent_w[t]]
        thecoefs = [1.0]

        for i,j in INCUMBENT_TRIP[p]:
          thevars.append(incumbent_y[i,j,t])
          thecoefs.append(-max_trip_payload[i,j,p])

        if len(thevars) > 0:
          thenames.append("weightlimit_" + str(t))
          thelin_exprs.append(cplex.SparsePair(thevars,thecoefs))
          thesenses.append("L")
          therhss.append(0.0)

          number_of_weightlimit += 1
            
      #print "number of 'weightlimit' constraints:",number_of_weightlimit
    
    
      # add all constraints
      
      incumb_model.linear_constraints.add(names = thenames, lin_expr = thelin_exprs, senses = thesenses, rhs = therhss)
      
      
      # OBJECTIVE FUNCTION 

      incumb_model.objective.set_sense(model.objective.sense.minimize)


      # output model

      #incumb_model.write("INCMOD/incumbent_model." + p + "." + str(self.number_of_calls) + ".lp")

      # surpress output 
            
      incumb_model.set_results_stream(None,fn=None)
            
      # solve model
      
      cb_t0 = time.clock()

      incumb_model.solve()

      self.callback_time += time.clock() - cb_t0
    
      # solution interpretation

      incumb_solution = incumb_model.solution

      if not incumb_solution.is_primal_feasible():
        #print "solution for plane",p,"infeasible"
        #print time.clock() - cb_t0
        
        # build cut: 
        # sum_{i : val(x_i) = 1} x_i + sum_{i : val(x_i) = 0} (1 - x_i) <= card({i : val(x_i) = 1} u {i : val(x_i) = 0}) - 1
        # <=> 
        # sum_{i : val(x_i) = 1} x_i - sum_{i : val(x_i) = 0} x_i <= card({i : val(x_i) = 1}) - 1
        
        therhs = 0
        thevars = []
        thecoefs = []
        
        for i,j in TRIP0:
          for p in PLANE:
            if value_of_y[i,j,p] > 0:
              k = value_of_y[i,j,p]
              thevars.append(eta[i,j,p,k])
              thecoefs.append(1.0)
              therhs += 1.0
            else:
              for k in range(1,3):
                thevars.append(eta[i,j,p,k])
                thecoefs.append(-1.0)
              therhs += 0.0

        for r in REQUEST:
          for i,j in REQUEST_TRIP0[r]:
            for p in PLANE:
              if value_of_x[i,j,r,p] > 0:
                thevars.append(x[i,j,r,p])
                thecoefs.append(1.0)
                therhs += 1.0
              #else:
              #  thevars.append(x[i,j,r,p])
              #  thecoefs.append(-1.0)
              #  therhs += 0.0
                
        for r in REQUEST:
          for p in PLANE:
            if value_of_x_dep[r,p] > 0:
              thevars.append(x_dep[r,p])
              thecoefs.append(1.0)
              therhs += 1.0
            #else:
            #  thevars.append(x_dep[r,p])
            #  thecoefs.append(-1.0)
            #  therhs += 0.0
              
        for r in REQUEST:
          for p in PLANE:
            if value_of_x_arr[r,p] > 0:
              thevars.append(x_arr[r,p])
              thecoefs.append(1.0)
              therhs += 1.0
            #else:
            #  thevars.append(x_arr[r,p])
            #  thecoefs.append(-1.0)
            #  therhs += 0.0

        #print thevars
        #print thecoefs
        #print therhs
        
        self.add(constraint = [thevars,thecoefs], sense = "L",rhs = therhs - 1.0)
      
      #incumb_model.write("incumbent_model." + p + ".lp")


    #print "feasible solution found at call: ",self.number_of_calls
    
    #if self.number_of_calls > 1000:
    #  exit()
    #print time.clock() - cb_t0
        
    return


# -------
# classes
# -------

class __PLANE__(object):
  def __init__(self,cost=None,seats=None,plane_departure=None,departure_min_fuel=None,departure_max_fuel=None,plane_arrival=None,arrival_min_fuel=None,arrival_max_fuel=None,required_fueltype=None,fuel=None,speed=None,max_fuel=None,empty_weight=None,add_turnover_time=None,reserve_fuel=None,contigence_ratio=None,pilot_weight=None):
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

class __AIRPORT__(object):
  def __init__(self,turnover_time=None):
    self.turnover_time = int(turnover_time)
    self.fuel = {}

class __REQUEST__(object):
  def __init__(self,request_departure=None,request_arrival=None,earliest_departure_time=None,earliest_departure_day=None,latest_arrival_time=None,latest_arrival_day=None,passengers=None,weight=None,max_stops=None,max_detour=None):
    self.request_departure = request_departure
    self.request_arrival = request_arrival
    self.earliest_departure_time = int(earliest_departure_time)
    self.earliest_departure_day = int(earliest_departure_day)
    self.latest_arrival_time = int(latest_arrival_time)
    self.latest_arrival_day = int(latest_arrival_day)
    self.passengers = int(passengers)
    self.weight = float(weight)
    self.max_stops = int(max_stops)
    self.max_detour = float(max_detour) 

    self.earliest_departure = 1440 * (self.earliest_departure_day - 1) + self.earliest_departure_time
    self.latest_arrival = 1440 * (self.latest_arrival_day - 1) + self.latest_arrival_time

class __WEIGHTLIMIT__(object):
  def __init__(self,max_takeoff_weight=None,max_landing_weight=None):
    self.max_takeoff_weight = float(max_takeoff_weight)
    self.max_landing_weight = float(max_landing_weight)

class __TRIP__(object):
  def __init__(self,distance=None):
    self.distance = float(distance)
    
    
# prepare reading and parsing

comment_line = re.compile('#');

# prepare reading and parsing
DIRECTORIES = {sys.argv[1]: sys.argv[2]
    }

strategy = 99

fileName='resultsTF.txt'
# ---------------------
# reading airplanes.dat
# ---------------------
for direcStr,directory in sorted(DIRECTORIES.iteritems()):
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
          print datas
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
          print datas
          ID, turnover_time = datas
          AIRPORT[ID] = __AIRPORT__(turnover_time)
    
    
    # --------------------------
    # reading columnsolution.dat
    # --------------------------
    
    print "reading '"+directory+"/columnsolution.dat'"
    
    file = open(directory+"/columnsolution.dat", "r")
    columnsolution_data = file.read()
    file.close()
    
    entries = re.split("\n+", columnsolution_data)
    
    COLUMNSOLUTION = Set()
    
    for line in entries:
      if comment_line.search(line) == None:
        datas = re.split("\s+", line)
        if len(datas) == 5:
          print datas
          plane, origin, destination, hour, minute = datas
          COLUMNSOLUTION.add((plane, origin, destination, hour, minute))
    
    #print Columnsolution
    
    
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
          print datas
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
          print datas
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
          print datas
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
          print datas
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
          print datas
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
          print datas
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
          print datas
          plane, request, origin, destination, hour, minute = datas
          REQUEST_SOLUTION[plane, request, origin, destination, hour, minute] = 1
    
    
    # --------------------------------
    # generating further instance data
    # --------------------------------
    
    turnover_timesteps = {}
    
    for i in AIRPORT:
      for p in PLANE:
        turnover_timesteps[i,p] = int(max(1,math.ceil((AIRPORT[i].turnover_time + PLANE[p].add_turnover_time) / timedelta)))
    
    
    travelcost = {}
    
    for p in PLANE:
      for i, j in TRIP:
        travelcost[i,j,p] = TRIP[i,j].distance * PLANE[p].cost
    
    
    travel_time = {}
    
    for p in PLANE:
      for i, j in TRIP:
        travel_time[i,j,p] = int(math.floor(TRIP[i,j].distance / ((PLANE[p].speed / 60) * 5)) * 5)
    
    
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
    
    
    max_takeoff_payload = {}
    
    for p in PLANE:
      for i in AIRPORT:
        max_takeoff_payload[i,p] = WEIGHTLIMIT[i,p].max_takeoff_weight - PLANE[p].empty_weight - PLANE[p].reserve_fuel - PLANE[p].pilot_weight
    
    
    max_landing_payload = {}
    
    for p in PLANE:
      for i in AIRPORT:
        max_landing_payload[i,p] = WEIGHTLIMIT[i,p].max_landing_weight - PLANE[p].empty_weight - PLANE[p].reserve_fuel - PLANE[p].pilot_weight
    
    
    fuelconsumption = {}
    
    for p in PLANE:
      for i, j in TRIP:
        fuelconsumption[i,j,p] = math.ceil(travel_time[i,j,p] * PLANE[p].fuel * PLANE[p].speed * PLANE[p].contigence_ratio / 60.0);
    
    
    max_trip_payload = {}
    
    for p in PLANE:
      for i, j in TRIP:
        max_trip_payload[i,j,p] = min(max_takeoff_payload[i,p] + fuelconsumption[i,j,p], max_landing_payload[j,p])
    
    
    max_trip_fuel = {}
    
    for p in PLANE:
      for i, j in TRIP:
        max_trip_fuel[i,j,p] = PLANE[p].max_fuel - fuelconsumption[i,j,p] - PLANE[p].reserve_fuel
    
    
    earliest_departure_travel_timesteps = {}
    
    for p in PLANE:
      for i, j in TRIP:
        for r in REQUEST:
          earliest_departure_travel_timesteps[i,j,p,r] = math.ceil((REQUEST[r].earliest_departure_time + 60 * TRIP[i,j].distance / PLANE[p].speed) / timedelta)
    
    
    earliest_departure_timesteps = {}
    
    for r in REQUEST:
      earliest_departure_timesteps[r] = int(math.ceil((REQUEST[r].earliest_departure_time + 1440 * (REQUEST[r].earliest_departure_day - 1)) / timedelta))
      
    
    latest_arrival_timesteps = {}
    
    for r in REQUEST:
      latest_arrival_timesteps[r] = int(math.floor((REQUEST[r].latest_arrival_time + 1440 * (REQUEST[r].latest_arrival_day - 1)) / timedelta))
    
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
    
    plane_max_timestep = {}
    
    for p in PLANE:
      plane_max_timestep[p] = 0
      
      for r in REQUEST:
        if REQUEST[r].passengers <= PLANE[p].seats:
          aux = latest_arrival_timesteps[r] + max(direct_flight_timesteps[p,r],max_refuel_flight_timesteps[p,r])
          plane_max_timestep[p] = int(max(plane_max_timestep[p],aux))
    
      #print "plane ",p," max timestep: ",plane_max_timestep[p]
    
    
    TRIP0 = {}
    
    for i,j in TRIP:
      if i != j:
        TRIP0[i,j] = TRIP[i,j]
    
    
    REQUEST_TRIP = {}
    
    for r in REQUEST:
      REQUEST_TRIP[r] = {}
    
      for i,j in TRIP:
        if i != REQUEST[r].request_arrival and j != REQUEST[r].request_departure:
          REQUEST_TRIP[r][i,j] = TRIP[i,j]
    
    
    REQUEST_TRIP0 = {}
    
    for r in REQUEST:
      REQUEST_TRIP0[r] = {}
    
      for i,j in TRIP0:
        if i != REQUEST[r].request_arrival and j != REQUEST[r].request_departure:
          REQUEST_TRIP0[r][i,j] = TRIP0[i,j]
    
    
    min_refuel_trip = {}
    
    for i in AIRPORT:
      for p in PLANE:
        min_refuel_trip[i,p] = 99999
        for j in AIRPORT:
          if (i,j) in TRIP:
            if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '1':
              min_refuel_trip[i,p] = min(min_refuel_trip[i,p], fuelconsumption[i,j,p])
    
    
    plane_solution_arrival_time = {}
    plane_solution_arrival_timestep = {}
    
    for p,i,j,hh,mm in PLANE_SOLUTION:
      plane_solution_arrival_time[p,i,j,hh,mm] = 60 * int(hh) + int(mm);
      plane_solution_arrival_timestep[p,i,j,hh,mm] = math.ceil(plane_solution_arrival_time[p,i,j,hh,mm] / timedelta)
    
    
    request_solution_arrival_time = {}
    request_solution_arrival_timestep = {}
    
    for p,r,i,j,hh,mm in REQUEST_SOLUTION:
      request_solution_arrival_time[p,r,i,j,hh,mm] = 60 * int(hh) + int(mm);
      request_solution_arrival_timestep[p,r,i,j,hh,mm] = math.ceil(request_solution_arrival_time[p,r,i,j,hh,mm] / timedelta)
      
      
    PLANE_TIMESTEP = {}
    
    for p in PLANE:
      PLANE_TIMESTEP[p] = {}
      
      for t in range(plane_min_timestep[p], plane_max_timestep[p] + 1):
        PLANE_TIMESTEP[p][t] = 1
        
    
    REQUEST_TIMESTEP = {}
    
    for r in REQUEST:
      max_turnover_timesteps = 0
      for p in PLANE:
        max_turnover_timesteps = max(max_turnover_timesteps, turnover_timesteps[REQUEST[r].request_departure,p])
      
      REQUEST_TIMESTEP[r] = range(earliest_departure_timesteps[r] - max_turnover_timesteps, latest_arrival_timesteps[r] + 1)
    
    
    min_timestep = 99999
    max_timestep = 0
    
    for p in PLANE:
      min_timestep = min(min_timestep, plane_min_timestep[p])
      max_timestep = max(max_timestep, plane_max_timestep[p])
      
    TIMESTEP = range(min_timestep, max_timestep + 1)
    
    
    TIMEFREEPLANESOLUTION = {}
    
    for p,i,j,hh,mm in PLANE_SOLUTION:
      TIMEFREEPLANESOLUTION[p,i,j] = 1
      
    
    TIMEFREEREQUESTSOLUTION = {}
    
    for p,r,i,j,hh,mm in REQUEST_SOLUTION:
      TIMEFREEREQUESTSOLUTION[p,r,i,j] = 1
      
       
    
    multiple_arc_use = {}
    
    for p,i,j,hh,mm in PLANE_SOLUTION:
      if (p,i,j) in multiple_arc_use:
        multiple_arc_use[p,i,j] += 1
      else:
        multiple_arc_use[p,i,j] = 1
    
    
    # ----------------
    # MODEL GENERATION
    # ----------------
    
    model = cplex.Cplex()
    
    
    # VARIABLES
    
    x = {}
    number_of_variables = 0
    
    for r in REQUEST:
      for i,j in REQUEST_TRIP0[r]:
        for p in PLANE:
          x[i,j,r,p] = "x#" + i + "_" + j + "_" + r + "_" + p
          #model.variables.add(obj = [travelcost[i,j,p]], names = [x[i,j,r,p]], lb = [0], ub = [1], types = ["B"])
          model.variables.add(obj = [0.0001], names = [x[i,j,r,p]], lb = [0], ub = [1], types = ["B"])
          number_of_variables += 1
    
    x_dep = {}
    
    for r in REQUEST:
      for p in PLANE:
        x_dep[r,p] = "x_dep#" + r + "_" + p
        model.variables.add(names = [x_dep[r,p]], lb = [0], ub = [1], types = ["B"])
        number_of_variables += 1
    
    x_arr = {}
    
    for r in REQUEST:
      for p in PLANE:
        x_arr[r,p] = "x_arr#" + r + "_" + p
        model.variables.add(names = [x_arr[r,p]], lb = [0], ub = [1], types = ["B"])
        number_of_variables += 1
    
    y = {}
    eta = {}
    
    for i,j in TRIP0:
      for p in PLANE:
        y[i,j,p] = "y#" + i + "_" + j + "_" + p
        if strategy == '1':
          model.variables.add(obj = [travelcost[i,j,p]], names = [y[i,j,p]], lb = [0], types = ["C"])
          number_of_variables += 1
          
          for k in range(1,3):
            eta[i,j,p,k] = "eta#" + i + "_" + j + "_" + p + "_" + str(k)
            model.variables.add(names = [eta[i,j,p,k]], lb = [0], ub = [1], types = ["B"])
            number_of_variables += 1
        else:
          model.variables.add(obj = [travelcost[i,j,p]], names = [y[i,j,p]], lb = [0], types = ["I"])
          number_of_variables += 1
    
    f = {}
    
    for i,j in TRIP0:
      for p in PLANE:
        f[i,j,p] = "f#" + i + "_" + j + "_" + p
        model.variables.add(names = [f[i,j,p]], lb = [0], types = ["C"])
        number_of_variables += 1
    
    f_dep = {}
    
    for p in PLANE:
      f_dep[p] = "f_dep#" + p
      model.variables.add(names = [f_dep[p]], lb = [PLANE[p].departure_min_fuel], ub = [PLANE[p].departure_max_fuel], types = ["C"])
      number_of_variables += 1
    
    f_arr = {}
    
    for p in PLANE:
      f_arr[p] = "f_arr#" + p
      model.variables.add(names = [f_arr[p]], lb = [PLANE[p].arrival_min_fuel], ub = [PLANE[p].arrival_max_fuel], types = ["C"])
      number_of_variables += 1
    
    w = {}
    
    for i,j in TRIP0:
      for p in PLANE:
        w[i,j,p] = "w#" + i + "_" + j + "_" + p
        model.variables.add(names = [w[i,j,p]], lb = [0], types = ["C"])
        number_of_variables += 1
    
    print "number of variables: ",number_of_variables
    
    
    # OBJECTICE 
    
    model.objective.set_sense(model.objective.sense.minimize)
    
    
    # CONSTRAINTS
    
    number_of_constraints = 0
    number_of_nonzeros = 0
    
    # auxiliary constraints for y-eta
    
    if strategy == '1':
      for i,j in TRIP0:
        for p in PLANE:
          thevars = [y[i,j,p]]
          thecoefs = [1.0]
          number_of_nonzeros += 1
          
          for k in range(1,3):
            thevars.append(eta[i,j,p,k])
            thecoefs.append(-k)
            number_of_nonzeros += 1
            
          model.linear_constraints.add(names = ["y_eta_coupling_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
          number_of_constraints += 1
          
          thevars = []
          thecoefs = []
          
          for k in range(1,3):
            thevars.append(eta[i,j,p,k])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
            
          model.linear_constraints.add(names = ["eta_selection_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [1.0])
          number_of_constraints += 1
          
          
    # each request must depart
    
    for r in REQUEST:
      thevars = []
      thecoefs = []
      for p in PLANE:
        thevars.append(x_dep[r,p])
        thecoefs.append(1.0)
        number_of_nonzeros += 1
        
      model.linear_constraints.add(names = ["request_one_dep_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
      number_of_constraints += 1
    
    
    # each request must arrive
    
    for r in REQUEST:
      thevars = []
      thecoefs = []
      for p in PLANE:
        thevars.append(x_arr[r,p])
        thecoefs.append(1.0)
        number_of_nonzeros += 1
        
      model.linear_constraints.add(names = ["request_one_arr_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
      number_of_constraints += 1
      
    
    # request flow
    
    for r in REQUEST:
      for p in PLANE:
        for j in AIRPORT:
          thevars = []
          thecoefs = []
    
          for i in AIRPORT:
            if (i,j) in REQUEST_TRIP0[r]:
              thevars.append(x[i,j,r,p])
              thecoefs.append(1.0)
              number_of_nonzeros += 1
          
          if j == REQUEST[r].request_departure:
            thevars.append(x_dep[r,p])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
            
          for k in AIRPORT:
            if (j,k) in REQUEST_TRIP0[r]:
              thevars.append(x[j,k,r,p])
              thecoefs.append(-1.0)
              number_of_nonzeros += 1
              
          if j == REQUEST[r].request_arrival:
            thevars.append(x_arr[r,p])
            thecoefs.append(-1.0)
            number_of_nonzeros += 1
            
          model.linear_constraints.add(names = ["request_flow_" + r + "_" + p + "_" + j], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
          number_of_constraints += 1
              
    
    # airplane flow
    
    for p in PLANE:
      for j in AIRPORT:
        #print p,i
        rhs_value = 0.0
        thevars = []
        thecoefs = []
            
        for i in AIRPORT:
          if (i,j) in TRIP0:
            thevars.append(y[i,j,p])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
            
        if j == PLANE[p].plane_departure:
          rhs_value += -1.0
          
        for k in AIRPORT:
          if (j,k) in TRIP0:
            thevars.append(y[j,k,p])
            thecoefs.append(-1.0)
            number_of_nonzeros += 1
        
        if j == PLANE[p].plane_arrival:
          rhs_value += 1.0
        
        model.linear_constraints.add(names = ["plane_flow_" + p + "_" + j], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [rhs_value])
        number_of_constraints += 1
    
                  
    # seat limit
    
    for i,j in TRIP0:
      for p in PLANE:
        #print i,j,p
        thevars = [y[i,j,p]]
        thecoefs = [-PLANE[p].seats]
        number_of_nonzeros += 1
        
        for r in REQUEST:
          if (i,j) in REQUEST_TRIP0[r]:
            thevars.append(x[i,j,r,p])
            thecoefs.append(REQUEST[r].passengers)
            number_of_nonzeros += 1
          
        model.linear_constraints.add(names = ["seatlimit_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
        number_of_constraints += 1
    
    
    # intermediate stops for requests
    
    for r in REQUEST:
      thevars = []
      thecoefs = []
      #print r
        
      for i,j in REQUEST_TRIP0[r]:
        for p in PLANE:
          thevars.append(x[i,j,r,p])
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
          thevars.append(x[i,j,r,p])
          thecoefs.append(TRIP0[i,j].distance)
          number_of_nonzeros += 1
          
      model.linear_constraints.add(names = ["maxdetour_" + r], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [(1 + REQUEST[r].max_detour) * TRIP0[REQUEST[r].request_departure,REQUEST[r].request_arrival].distance])
      number_of_constraints += 1
      
    
    # fueling constraints
    
    for i,j in TRIP0:
      for p in PLANE:
        #print i,j,p
        thevars = [f[i,j,p],y[i,j,p]]
        thecoefs = [1.0,-max_trip_fuel[i,j,p]]
        number_of_nonzeros += 2
        
        model.linear_constraints.add(names = ["noflight_nofuel_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
        number_of_constraints += 1
    
    
    for j in AIRPORT:
      for p in PLANE:
        #print j,p
        thevars = []
        thecoefs = []
                
        for i in AIRPORT:
          if (i,j) in TRIP0:
            thevars.append(f[i,j,p])
            thecoefs.append(1.0)
            number_of_nonzeros += 1
                
        if j == PLANE[p].plane_departure:
          thevars.append(f_dep[p])
          thecoefs.append(1.0)
          number_of_nonzeros += 1
                
        for k in AIRPORT:
          if (j,k) in TRIP0:
            thevars.append(f[j,k,p])
            thecoefs.append(-1.0)
            number_of_nonzeros += 1
            
            thevars.append(y[j,k,p])
            thecoefs.append(-fuelconsumption[j,k,p])
            number_of_nonzeros += 1
                
        if j == PLANE[p].plane_arrival:
          thevars.append(f_arr[p])
          thecoefs.append(-1.0)
          number_of_nonzeros += 1
                
        if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '0':
          model.linear_constraints.add(names = ["fuelconsumption_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
          number_of_constraints += 1
        else:
          model.linear_constraints.add(names = ["refueling_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
          number_of_constraints += 1
    
    
    # weight limit (=max fuel)
    
    for i,j in TRIP0:
      for p in PLANE:
        #print i,j,p
        thevars = [w[i,j,p]]
        thecoefs = [1.0]
        number_of_nonzeros += 1
    
        for r in REQUEST:
          if (i,j) in REQUEST_TRIP0[r]:
            thevars.append(x[i,j,r,p])
            thecoefs.append(-REQUEST[r].weight)
            number_of_nonzeros += 1
    
        thevars.append(f[i,j,p])
        thecoefs.append(-1.0)
        number_of_nonzeros += 1
            
        model.linear_constraints.add(names = ["computeweight_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
        number_of_constraints += 1
    
    
    for i,j in TRIP0:
      for p in PLANE:
        #print i,j,p
        thevars = [w[i,j,p],y[i,j,p]]
        thecoefs = [1.0,-max_trip_payload[i,j,p]]
        number_of_nonzeros += 2
    
        model.linear_constraints.add(names = ["weightlimit_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0.0])
        number_of_constraints += 1
    
    
    # minimum number of fuelstops
    
    for p in PLANE:
      if PLANE[p].departure_max_fuel - fuelconsumption[PLANE[p].plane_departure,PLANE[p].plane_arrival,p] < PLANE[p].arrival_min_fuel:
        #print p
        thevars = []
        thecoefs = []
    
        for i,j in TRIP0:
          if AIRPORT[j].fuel[PLANE[p].required_fueltype] == '1':
            thevars.append(y[i,j,p])
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
            thevars.append(y[i,j,p])
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
              thevars.append(y[i,j,p])
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
              thevars.append(y[i,j,p])
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
    
    
    # minimum amount of fuel for detour to refueling airport
    
    for i,j in TRIP0:
      for p in PLANE:
        thevars = [y[i,j,p],f[i,j,p]]
        thecoefs = [min_refuel_trip[j,p],-1.0]
        number_of_nonzeros += 2
        
        model.linear_constraints.add(names = ["minfuel_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["L"], rhs = [0])
        number_of_constraints += 1
    
    print "number of constraints: ",number_of_constraints
    print "number of non-zeros: ",number_of_nonzeros
    
    
    # fix heuristic solution variables
    
    for p,i,j in TIMEFREEPLANESOLUTION:
      thevars = [y[i,j,p]]
      thecoefs = [1.0]
    
      model.linear_constraints.add(names = ["fix_airplane_schedule_" + i + "_" + j + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [multiple_arc_use[p,i,j]])
    
    
    for p,r,i,j in TIMEFREEREQUESTSOLUTION:
      if (i,j,r,p) in x and j != REQUEST[r].request_arrival and i != REQUEST[r].request_departure:
        thevars = [x[i,j,r,p]]
        thecoefs = [1.0]
    
        model.linear_constraints.add(names = ["fix_request_schedule_" + i + "_" + j + "_" + r + "_" + p], lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
    
    
    # output model
    
    model.write("model_fixed.lp")
    
    
    # set parameters
    
    model.parameters.mip.strategy.heuristicfreq.set(-1)
    model.parameters.emphasis.mip.set(3) 
     
    # solve model
           
    model.solve()
    
    
    # solution interpretation
    
    solution = model.solution
    
    print "Solution status:", solution.get_status()
    
    if solution.is_primal_feasible():
        primal_val=solution.get_objective_value()
        print "Primal solution value:", solution.get_objective_value()
        solution.write("model_fixed.sol")
    else:
        print "No solution available."
        exit()
    
    
    # delete fix_schedule constraints
    
    for p,i,j in TIMEFREEPLANESOLUTION:
      model.linear_constraints.delete("fix_airplane_schedule_" + i + "_" + j + "_" + p)
    
    for p,r,i,j in TIMEFREEREQUESTSOLUTION:
      if (i,j,r,p) in x and j != REQUEST[r].request_arrival and i != REQUEST[r].request_departure:
        model.linear_constraints.delete("fix_request_schedule_" + i + "_" + j + "_" + r + "_" + p)
    
    
    # set callbacks
    
    #incumbent_cb = model.register_callback(CheckSolutuionCallback)
    #lazyconstraint_cb = model.register_callback(SubtourEliminationCallback)
    
    if strategy == '1':
      print "Strategy: lazy constraints"
      lazyconstraint_cb = model.register_callback(InfeasibleScheduleMIPCallback)
      lazyconstraint_cb.number_of_calls = 0
      lazyconstraint_cb.callback_time = 0
    else:
      print "Strategy: incumbent branching"
      incumbent_cb = model.register_callback(CheckSolutionMIPCallback)
      incumbent_cb.number_of_calls = 0
      incumbent_cb.number_of_calls_with_new_solution = 0
      incumbent_cb.callback_time = 0
      incumbent_cb.solution_pool = []
    
    
    mipinfo_cb = model.register_callback(CountNodesCallback)
    mipinfo_cb.number_of_nodes = 0
    mipinfo_cb.mip_gap=100
    mipinfo_cb.best_obj_val = primal_val
    
    # set time limit
    
    model.parameters.timelimit.set(10800) # 10800 = 3h, 86400 = one day (24h)
    model.parameters.mip.tolerances.mipgap.set(0.0)
    #model.parameters.mip.strategy.file.set(2) # node file on disk
    #model.parameters.workmem.set(4096.0) # working memory
    
    # solve again
    
    #model.write("model.lp")
    log_file_name = "logs/TF/"+direcStr
    model.set_results_stream(log_file_name)
    model.set_log_stream(log_file_name+"vars")
    t0 = time.clock()
    model.parameters.workmem.set(8192)
    model.solve()
    
    # report solution
    
    print "total time:",time.clock() - t0
    
    if strategy == '1':
      print "total calls of incumbent callback:",lazyconstraint_cb.number_of_calls
      print "callback time:",lazyconstraint_cb.callback_time
    else:
      print "total calls of incumbent callback:",incumbent_cb.number_of_calls
      print "calls of incumbent callback with new solutions:",incumbent_cb.number_of_calls_with_new_solution
      print "callback time:",incumbent_cb.callback_time
    
    #print "MIP gap:",mipinfo_cb.mip_gap
    #print "Dual bound value:",mipinfo_cb.best_obj_val
    
    #print "number of nodes:",mipinfo_cb.number_of_nodes
    
    solution = model.solution
    
    print "Solution status:", solution.get_status()
    
    
    if solution.is_primal_feasible():
        print "Final solution value:", solution.get_objective_value()
        lineToAdd = direcStr + " & %d & %d & %.2f \\%% & %d \\\\\n" % (mipinfo_cb.best_obj_val,  
                                                                       solution.get_objective_value(),
                                                                       mipinfo_cb.mip_gap*100,time.clock() - t0)
    else:
        print "No solution available."
    file = open(fileName, "a")
    file.write(lineToAdd)
    file.close()
    


