import re
from data_classes import __PLANE__, __AIRPORT__, __REQUEST__, __WEIGHTLIMIT__ 
from data_classes import __TRIP__, __AIRLINEMIP__,__DATA__,__SOLUTION__,__REQUESTPATH__
import cplex
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 17:23:06 2018

@author: fabiangnegel
"""

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
    #TIMEFREEREQUESTSOLUTION = mipmodel.DATA.TIMEFREEREQUESTSOLUTION
    model = mipmodel.model
    AirportNum = mipmodel.DATA.current_airport_num
    y = mipmodel.y
    #x = mipmodel.x
    
    for p,i,j in TIMEFREEPLANESOLUTION:
        thevars = []
        thecoefs = []
        for n1 in AirportNum[p,i]:
            for n2 in AirportNum[p,j]:
                thevars += [y[i,j,p,n1,n2]]
                thecoefs += [1.0]
        
        model.linear_constraints.add(names = ["fix_airplane_schedule_" + i + "_" + j + "_" + p], 
                                   lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                   senses = ["E"], rhs = [TIMEFREEPLANESOLUTION[p,i,j]])
    
    model.solve()
    solution = model.solution
    
    
    
    print "Solution status:", solution.get_status()
        
    if solution.is_primal_feasible():
        print "Primal solution value:", solution.get_objective_value()
        obj = solution.get_objective_value()
        for p,i,j in TIMEFREEPLANESOLUTION:
            model.linear_constraints.delete("fix_airplane_schedule_" + i + "_" + j + "_" + p)
        return obj
    else:
        print "Primal solution could not be recovered"
        for p,i,j in TIMEFREEPLANESOLUTION:
            model.linear_constraints.delete("fix_airplane_schedule_" + i + "_" + j + "_" + p)
        return 100000
        

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
