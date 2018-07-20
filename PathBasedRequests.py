#import re
import time
import sys

#from operator import itemgetter
if not 'cplex' in globals():
    import cplex
    from cplex.callbacks import IncumbentCallback
    from cplex.callbacks import LazyConstraintCallback
    from cplex.callbacks import MIPInfoCallback
    #from cplex.callbacks import UserCutCallback
    #from cplex.exceptions import CplexSolverError


from auxiliary_functions import solToPaths, solToAirports, arcAmount,readData,setPrimal
#from sets import Set
#from data_classes import __PLANE__, __AIRPORT__, __REQUEST__, __WEIGHTLIMIT__ 
#from data_classes import __TRIP__, __AIRLINEMIP__,__DATA__,__SOLUTION__,__REQUESTPATH__
from data_classes import  __AIRLINEMIP__,__SOLUTION__

EPSILON = 1e-6


# ---------
# callbacks
# ---------


class RemoveIncumbentCallback(LazyConstraintCallback):
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
    REQUEST = self.MIPMODEL.DATA.REQUEST
    
    #AirportNum = self.MIPMODEL.DATA.current_airport_num
    fullModelobj = 0
    for p in PLANE:
        r2 = self.fullModel[p].r2
        fullModel = self.fullModel[p].model
        y2 = self.fullModel[p].y

        
        assignedRequests[p] = {}
        

        
        
        #read the y-variables that are set for p
        yString=[]
        flyTime[p]=0
        for key,val in y.iteritems():
            if key[2]==p:
                valStore=all_values[name2idx[val]]
                for k in range(10): #account for y variables that are not binary
                    if valStore > 0.1+k: 
                        yString+=[val]
                        flyTime[p]+=turnover_travel_timesteps[key[0],key[1],key[2]]
                    else:
                        break
        
        
        paths[p]=solToPaths(yString)
    
    
            
        if paths[p] == []:
            continue
        
        for i,j in TRIP0:
            if arcAmount(paths[p][0],[i,j]) < 0.5:
                fullModel.variables.set_upper_bounds( [(y2[i,j,p],1.0) ] )
                fullModel.variables.set_lower_bounds( [(y2[i,j,p],arcAmount(paths[p][0],[i,j])) ] )
            else:
                fullModel.variables.set_upper_bounds( [(y2[i,j,p],5) ] )#TODO 5 korrigieren
                fullModel.variables.set_lower_bounds( [(y2[i,j,p],arcAmount(paths[p][0],[i,j])) ] )
        

            
        for r in REQUEST:
            assignedRequests[p][r] = 0
        
        #find assigned requests and set arcs
        for key,val in x.iteritems():
            if key[1]==p:
                valStore=all_values[name2idx[val]]
                if valStore > 0.1:
                    assignedRequests[p][key[0]] = 1


        
        for r,assigned in assignedRequests[p].iteritems():
            fullModel.variables.set_upper_bounds( [(r2[r], assigned)])
            fullModel.variables.set_lower_bounds( [(r2[r], assigned)])
        
        #converts the analyzed y variables to a set of longest paths
        
        
        
        fullModel.solve()
        if not fullModel.solution.is_primal_feasible():
            self.number_of_infeasibles += 1
            #print "\n number of infeasibles: " +str(self.number_of_infeasibles) +"\n"
            self.totallySolved = 0
        else:
            fullModelobj += fullModel.solution.get_objective_value()
        
            
    if self.totallySolved:
        if fullModelobj < self.bestSolution:
            self.bestSolution = fullModelobj
            if abs(self.get_objective_value()-fullModelobj) < 1.0:
                self.best_plane_solution = {}
                self.best_request_solution = {}
                for key,val in y.iteritems():
                    valStore=all_values[name2idx[val]]
                    if valStore > 0.1: 
                        if (key[2],key[0],key[1]) in self.best_plane_solution:
                            self.best_plane_solution[key[2],key[0],key[1]] += valStore
                        else:
                            self.best_plane_solution[key[2],key[0],key[1]] = 1
    else:
        thevars = []
        thecoefs = []
        rhsval = 0
        for key,val in self.MIPMODEL.r2.iteritems():
            valStore=all_values[name2idx[val]]
            if valStore > 0.1:
                thevars += [val]
                thecoefs += [1.0]
                rhsval += valStore
        for key,val in y.iteritems():
            if len(key) > 3:
                valStore=all_values[name2idx[val]]
                if valStore > 0.1:
                    thevars += [val]
                    thecoefs += [1.0]
                    rhsval += valStore
        self.add(constraint = cplex.SparsePair(thevars,thecoefs), sense = "L", rhs = rhsval-1)
    return


class breakIncumbentCallback3(IncumbentCallback):
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
    REQUEST = self.MIPMODEL.DATA.REQUEST
    
    AirportNum = self.MIPMODEL.DATA.current_airport_num
    
    for p in PLANE:
        r2 = self.fullModel[p].r2
        fullModel = self.fullModel[p].model
        y2 = self.fullModel[p].y

        
        assignedRequests[p] = {}
        

        
        
        #read the y-variables that are set for p
        yString=[]
        flyTime[p]=0
        for key,val in y.iteritems():
            if key[2]==p:
                valStore=all_values[name2idx[val]]
                for k in range(10): #account for y variables that are not binary
                    if valStore > 0.1+k: 
                        yString+=[val]
                        flyTime[p]+=turnover_travel_timesteps[key[0],key[1],key[2]]
                    else:
                        break
        
        
        paths[p]=solToPaths(yString)
    
    
            
        if paths[p] == []:
            continue
        
        for i,j in TRIP0:
            fullModel.variables.set_upper_bounds( [(y2[i,j,p],arcAmount(paths[p][0],[i,j])) ] )
            fullModel.variables.set_lower_bounds( [(y2[i,j,p],arcAmount(paths[p][0],[i,j])) ] )
        
        

            
        for r in REQUEST:
            assignedRequests[p][r] = 0
        
        #find assigned requests and set arcs
        for key,val in x.iteritems():
            if key[1]==p:
                valStore=all_values[name2idx[val]]
                if valStore > 0.1:
                    assignedRequests[p][key[0]] = 1


        
        for r,assigned in assignedRequests[p].iteritems():
            fullModel.variables.set_upper_bounds( [(r2[r], assigned)])
            fullModel.variables.set_lower_bounds( [(r2[r], assigned)])
        
        #converts the analyzed y variables to a set of longest paths
        airports=solToAirports(yString,p,PLANE)
        
        
        
        fullModel.solve()
        if not fullModel.solution.is_primal_feasible():
            self.number_of_infeasibles += 1
            print "\n number of infeasibles: " +str(self.number_of_infeasibles) +"\n"
            self.totallySolved = 0
            for s in airports:
                if AirportNum[p,s][-1] < airports[s]:
                    AirportNum[p,s].append(AirportNum[p,s][-1]+1)
            #if self.number_of_infeasibles < 5:
            #    self.reject()
            return
            
    if self.totallySolved:
        if self.get_objective_value() < self.bestSolution:
            self.bestSolution = self.get_objective_value()
            self.best_plane_solution = {}
            self.best_request_solution = {}
            for key,val in y.iteritems():
                valStore=all_values[name2idx[val]]
                if valStore > 0.1: 
                    if (key[2],key[0],key[1]) in self.best_plane_solution:
                        self.best_plane_solution[key[2],key[0],key[1]] += valStore
                    else:
                        self.best_plane_solution[key[2],key[0],key[1]] = 1
    
    return


class CountNodesCallback(MIPInfoCallback):

    def __call__(self):

        self.best_obj_val = self.get_best_objective_value()
        self.mip_gap = self.get_MIP_relative_gap()

        return


class breakIncumbentCallback2(IncumbentCallback):
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
        

        
        
        #read the y-variables that are set for p
        yString=[]
        flyTime[p]=0
        for key,val in y.iteritems():
            if key[2]==p:
                valStore=all_values[name2idx[val]]
                for k in range(10): #account for y variables that are not binary
                    if valStore > 0.1+k: 
                        yString+=[val]
                        flyTime[p]+=turnover_travel_timesteps[key[0],key[1],key[2]]
                    else:
                        break
        
        
        paths[p]=solToPaths(yString)
    
    
            
        if paths[p] == []:
            continue
        
        for i,j in TRIP0:
            fullModel.variables.set_upper_bounds( [(y2[i,j,p],arcAmount(paths[p][0],[i,j])) ] )
            fullModel.variables.set_lower_bounds( [(y2[i,j,p],arcAmount(paths[p][0],[i,j])) ] )
        
        
        requestArcs = {}
        for i,j in TRIP:
            requestArcs[i,j] = 0
            
        for r in REQUEST:
            assignedRequests[p][r] = 0
        
        xString = {}
        #find assigned requests and set arcs
        for key,val in x.iteritems():
            if key[3]==p:
                valStore=all_values[name2idx[val]]
                if valStore > 0.1:
                    if assignedRequests[p][key[2]] == 0:
                        xString[key[2]]=[]
                    assignedRequests[p][key[2]] = 1
                    requestArcs[key[0],key[1]] = 1
                    if key[0]!=key[1]:
                        xString[key[2]]+=["x#" + key[0] + "_" + key[1] + "_" + key[2] + "_" + key[3]]
        
        
        for r,assigned in assignedRequests[p].iteritems():
            fullModel.variables.set_upper_bounds( [(r2[r], assigned)])
            fullModel.variables.set_lower_bounds( [(r2[r], assigned)])
        
        #converts the analyzed y variables to a set of longest paths
        airports=solToAirports(yString,p,PLANE)
        
        
        
        fullModel.solve()
        if not fullModel.solution.is_primal_feasible():
            self.number_of_infeasibles += 1
            self.totallySolved = 0
            for s in airports:
                if AirportNum[p,s][-1] < airports[s]:
                    AirportNum[p,s].append(AirportNum[p,s][-1]+1)
            #if self.number_of_infeasibles < 5:
            #    self.reject()
            return
            
    if self.totallySolved:
        if self.get_objective_value() < self.bestSolution:
            self.bestSolution = self.get_objective_value()
            self.best_plane_solution = {}
            self.best_request_solution = {}
            for key,val in y.iteritems():
                valStore=all_values[name2idx[val]]
                if valStore > 0.1: 
                    if (key[2],key[0],key[1]) in self.best_plane_solution:
                        self.best_plane_solution[key[2],key[0],key[1]] += 1
                    else:
                        self.best_plane_solution[key[2],key[0],key[1]] = 1
            
            self.best_request_solution = {}
            for key,val in x.iteritems():
                valStore=all_values[name2idx[val]]
                if valStore > 0.1: 
                    self.best_request_solution[key[3],key[2],key[0],key[1]] = 1
                        
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


# prepare reading and parsing
DIRECTORIES = {
    'BUF-AIV': 'Testinstances/A2-BUF_A2-AIV',
    'BUF-ANT': 'Testinstances/A2-BUF_A2-ANT',
    'BUF-BEE': 'Testinstances/A2-BUF_A2-BEE',
    'BUF-BOK': 'Testinstances/A2-BUF_A2-BOK',
    'BUF-EGL': 'Testinstances/A2-BUF_A2-EGL',
    'BUF-GNU': 'Testinstances/A2-BUF_A2-GNU',
    'BUF-JKL': 'Testinstances/A2-BUF_A2-JKL',
    'BUF-LEO': 'Testinstances/A2-BUF_A2-LEO',
    'BUF-NAS': 'Testinstances/A2-BUF_A2-NAS',
    'BUF-OWL': 'Testinstances/A2-BUF_A2-OWL',
    'BUF-ZEB': 'Testinstances/A2-BUF_A2-ZEB',
    'EGL-BEE': 'Testinstances/A2-EGL_A2-BEE',
    'EGL-GNU': 'Testinstances/A2-EGL_A2-GNU',
    'EGL-LEO': 'Testinstances/A2-EGL_A2-LEO',
    'GNU-BEE': 'Testinstances/A2-GNU_A2-BEE',
    'GNU-JKL': 'Testinstances/A2-GNU_A2-JKL',
    'GNU-LEO': 'Testinstances/A2-GNU_A2-LEO',
    'LEO-AIV': 'Testinstances/A2-LEO_A2-AIV',
    'LEO-ANT': 'Testinstances/A2-LEO_A2-ANT',
    'LEO-BEE': 'Testinstances/A2-LEO_A2-BEE',
    'LEO-BOK': 'Testinstances/A2-LEO_A2-BOK',
    'LEO-JKL': 'Testinstances/A2-LEO_A2-JKL',
    'LEO-NAS': 'Testinstances/A2-LEO_A2-NAS',
    'LEO-OWL': 'Testinstances/A2-LEO_A2-OWL',
    }

use_all = sys.argv[3]
"""
if use_all:
    DIRECTORIES = {
        'BUF-AIV': 'Testinstances/A2-BUF_A2-AIV',
        'LEO-AIV': 'Testinstances/A2-LEO_A2-AIV',
        'LEO-ANT': 'Testinstances/A2-LEO_A2-ANT',
        'LEO-JKL': 'Testinstances/A2-LEO_A2-JKL',
        'LEO-NAS': 'Testinstances/A2-LEO_A2-NAS',
        'LEO-OWL': 'Testinstances/A2-LEO_A2-OWL',
        #'BUF-ZEB': 'Testinstances/A2-BUF_A2-ZEB',
        'EGL-BEE': 'Testinstances/A2-EGL_A2-BEE',
        #'EGL-GNU': 'Testinstances/A2-EGL_A2-GNU',
        }

"""

cuttingPlanes = {
        'min_fuel_cut':1,
        #'useless_detour_cut':1,
        'max_visit_cut':1,
        #'time_order_cut':1,
        'fuelstop_cut':1,
        #'arrival_departure_cut':1,
        #'request_conflict_cut':1,
        #'time_conflict_cut':1,
        }
pathBasedBoolean = int(sys.argv[1])

if int(sys.argv[2])==2:
    cuttingPlanes['useless_detour_cut']=1
if int(sys.argv[2])==3:
    cuttingPlanes['time_order_cut']=1
if int(sys.argv[2])==4:
    cuttingPlanes['request_conflict_cut']=1
if int(sys.argv[2])==5:
    cuttingPlanes['useless_detour_cut']=1
    cuttingPlanes['time_order_cut']=1
    cuttingPlanes['request_conflict_cut']=1


differentFull = 1
lazyCb = 0

SOLUTIONS = {}
maxLoopIterations = 0
for instanceName, directory in DIRECTORIES.iteritems():
    dualBound=0
    DATA = readData(directory,use_all)
    #continue
    if not ("fullAirplaneMIP" in globals() ) or 1:
        fullAirplaneMIP = {}
        fullModelCuts = {}
        if lazyCb:
            fullModelCuts = cuttingPlanes
        for p in DATA.PLANE:
            if pathBasedBoolean:
                if differentFull:
                    fullAirplaneMIP[p] = __AIRLINEMIP__(DATA,fullModelCuts,1,{p:DATA.PLANE[p]},0,log_file_name ="")
                else:
                    fullAirplaneMIP[p] = __AIRLINEMIP__(DATA,fullModelCuts,1,{p:DATA.PLANE[p]},1,log_file_name ="")
            else:
                fullAirplaneMIP[p] = __AIRLINEMIP__(DATA,fullModelCuts,1,{p:DATA.PLANE[p]},0,log_file_name = "")
    primal_objective=100000
    totallySolved = 0
    timeLimit = 10800
    t0 = time.time()
    time_used = 0
    SOLUTIONS[instanceName] = __SOLUTION__(100,0,100000,0,10800,instanceName)
    #while not totallySolved and not lazyCb:
    if not use_all:
        while not totallySolved:
            #if SOLUTIONS[instanceName].loop_iterations > 0:
            #    lazyCb=1
            SOLUTIONS[instanceName].loop_iterations += 1
            if pathBasedBoolean:
                airplaneMIP = __AIRLINEMIP__(DATA,cuttingPlanes,
                                             log_file_name = "logs/PB%d/" % int(sys.argv[2]) + instanceName + "%d" % SOLUTIONS[instanceName].loop_iterations)
            else:
                airplaneMIP = __AIRLINEMIP__(DATA,cuttingPlanes,pathBased=0,
                                             log_file_name = "logs/notPB%d/" % int(sys.argv[2]) + instanceName + "%d" % SOLUTIONS[instanceName].loop_iterations)
            primal_objective = setPrimal(airplaneMIP)
            
            if pathBasedBoolean:
                if differentFull:
                    if lazyCb:
                        cb = airplaneMIP.model.register_callback(RemoveIncumbentCallback)
                    else:
                        cb = airplaneMIP.model.register_callback(breakIncumbentCallback3)
                else:
                    cb = airplaneMIP.model.register_callback(breakIncumbentCallback)
            else:
                cb = airplaneMIP.model.register_callback(breakIncumbentCallback2)
            cb.totallySolved = 0
            cb.MIPMODEL = airplaneMIP
            cb.fullModel = fullAirplaneMIP
            cb.number_of_infeasibles = 0
            SOLUTIONS[instanceName].bestValue = round(primal_objective,0) 
            cb.bestSolution = primal_objective
            
            model = airplaneMIP.model
            infoCB = model.register_callback(CountNodesCallback)
            model.parameters.timelimit.set(timeLimit) # 10800 = 3h, 86400 = one day (24h)
            model.parameters.mip.tolerances.mipgap.set(0.0)
            #model.parameters.mip.strategy.file.set(2)
            #model.parameters.emphasis.mip.set(2)
            t0used = time.time()
            model.solve()
            time_used += -t0used+time.time()
            if cb.bestSolution < primal_objective - 0.01:
                DATA.TIMEFREEPLANESOLUTION = cb.best_plane_solution
                DATA.TIMEFREEREQUESTSOLUTION = cb.best_request_solution
                SOLUTIONS[instanceName].bestValue = round(cb.bestSolution,0) 
            totallySolved = cb.totallySolved
            #totallySolved = 1
            timeLimit = 10800-(time.time()-t0)
            
            solution = model.solution
            if model.solution.get_status() == 107:
                print("Timed out stopping loop")
                dualBound =round( max([dualBound,infoCB.best_obj_val]),0)
                bestValue = round(cb.bestSolution,0)
                #TODO: Properly update the dual gap
                #if solution.MIP.get_best_objective() > bestDualBound[instanceName]:
                #     bestDualBound[instanceName] = solution.MIP.get_best_objective()
                break
            else:
                SOLUTIONS[instanceName].dualBound = round(solution.get_objective_value(),0)
                bestValue = round(cb.bestSolution,0)
                dualBound = round(solution.get_objective_value(),0)
                loops = SOLUTIONS[instanceName].loop_iterations
                gap = round(100 - 100*dualBound/bestValue,2)
                SOLUTIONS[instanceName].history[loops] = __SOLUTION__(gap,dualBound,bestValue,loops,time.time()-t0,instanceName)
                if loops > maxLoopIterations:
                    maxLoopIterations = loops
        
        SOLUTIONS[instanceName].gap = round(100- 100*SOLUTIONS[instanceName].dualBound/SOLUTIONS[instanceName].bestValue,2)
        SOLUTIONS[instanceName].time = int(min([10800,round(time.time()-t0,0)]))
        SOLUTIONS[instanceName].printToScreen()
        SOLUTIONS[instanceName].savePlaneVars(airplaneMIP.y,model.solution.get_values(),
                 { n : j for j, n in enumerate(model.variables.get_names()) })
    else:
        if pathBasedBoolean:
            airplaneMIP = __AIRLINEMIP__(DATA,cuttingPlanes,
                                         log_file_name = "logs/PB%d/" % int(sys.argv[2]) + instanceName + "%d" % SOLUTIONS[instanceName].loop_iterations)
        else:
            airplaneMIP = __AIRLINEMIP__(DATA,cuttingPlanes,pathBased=0,
                                         log_file_name = "logs/notPB%d/" % int(sys.argv[2]) + instanceName + "%d" % SOLUTIONS[instanceName].loop_iterations)
        primal_objective = setPrimal(airplaneMIP)
        SOLUTIONS[instanceName].bestValue = round(primal_objective,0) 
        
        model = airplaneMIP.model
        infoCB = model.register_callback(CountNodesCallback)
        model.parameters.timelimit.set(timeLimit) # 10800 = 3h, 86400 = one day (24h)
        model.parameters.mip.tolerances.mipgap.set(0.0)
        #model.parameters.mip.strategy.file.set(2)
        #model.parameters.emphasis.mip.set(2)
        t0 = time.time()
        model.solve()
        solution = model.solution
        if model.solution.get_status() == 107:
            bestValue = round(solution.get_objective_value(),0)
            dualBound = infoCB.best_obj_val
            gap = 100*infoCB.mip_gap
        else:
            
            bestValue = round(solution.get_objective_value(),0)
            dualBound = round(solution.get_objective_value(),0)
            gap = 0.00
        
        SOLUTIONS[instanceName] = __SOLUTION__(gap,dualBound,bestValue,1,time.time()-t0,instanceName)




SOLUTIONS["Average"]=__SOLUTION__(sum([sol.gap for sol in SOLUTIONS.itervalues()])/float(len(SOLUTIONS)),

                                  sum([sol.dualBound for sol in SOLUTIONS.itervalues()])/float(len(SOLUTIONS)),
                                  sum([sol.bestValue for sol in SOLUTIONS.itervalues()])/float(len(SOLUTIONS)),
                                  sum([sol.loop_iterations for sol in SOLUTIONS.itervalues()])/float(len(SOLUTIONS)),
                                  sum([sol.time for sol in SOLUTIONS.itervalues()])/float(len(SOLUTIONS)),"Average")

for i in range(1,1+maxLoopIterations):
    bestValues = 0.0
    dualBounds = 0.0
    times = 0.0
    for sol in SOLUTIONS.itervalues():
        if sol.history.has_key(i):
            bestValues +=  sol.history[i].bestValue
            dualBounds += sol.history[i].dualBound
            times += sol.history[i].time
        else:
            for adder in range(1,i):
                if sol.history.has_key(i-adder):
                    bestValues +=  sol.history[i-adder].bestValue
                    dualBounds += sol.history[i-adder].dualBound
                    times += sol.history[i-adder].time
                    break
        
    SOLUTIONS["Average_iteration_%d" % i ]=__SOLUTION__(round(100 - 100*(dualBounds/float(len(SOLUTIONS)-i))/(bestValues/float(len(SOLUTIONS)-i)),2),

                                  dualBounds/float(len(SOLUTIONS)-i),
                                  bestValues/float(len(SOLUTIONS)-i),
                                  i,
                                  times/float(len(SOLUTIONS)-i),"Average_iteration_%d" % i)

cuttingPlanesStr = ""
for key in cuttingPlanes:
    cuttingPlanesStr += " " + key
commentLine = "# " + time.ctime(time.time()) + "\n"
commentLine += "# pathBased %d, Cutting planes: " % pathBasedBoolean +cuttingPlanesStr +"\n \n"
if pathBasedBoolean:
    fileName = "resultsPB.txt"
else:
    fileName = "resultsOldModel.txt"
if use_all:
    fileName = "full_model_"+fileName

file = open(fileName, "a")
file.write(commentLine)
file.close()
for key, sol in sorted(SOLUTIONS.iteritems()):
    sol.saveToFile(fileName)


"""
solutionValues=model.solution.get_values()
idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
name2solutionValue = { n : solutionValues[j] for j, n in enumerate(model.variables.get_names()) }

for key,val in airplaneMIP.y.iteritems():
    valStore=solutionValues[name2idx[val]]
    if valStore > 0.5 or valStore < -0.5:
        print(val+" %f" %valStore)
for key,val in airplaneMIP.x.iteritems():
    valStore=solutionValues[name2idx[val]]
    if valStore > 0.5 or valStore < -0.5:
        print(val+" %f" %valStore)
"""