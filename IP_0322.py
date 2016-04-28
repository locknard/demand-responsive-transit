#coding=utf-8
import pulp as lp
import numpy as np
import math
import cplex
import copy
import os
import sys
from PyQt4 import QtGui,QtCore
#常量
side_length=4
optimal_interval=15 #5*2=10 min
candidate_interval=[2,3,4,5,6,8,10,20,30,40]
experiment=10
default_pos=6
vehicle_num=4
max_vehicle=15
min_vehicle=1
if len(sys.argv)>1:
    experiment=sys.argv[1]
    side_length=sys.argv[2]
    optimal_interval=sys.argv[3]
    vehicle_num=sys.argv[4]
#global statistics
unfulfilled=0
iteration=0
# if not os.path.exists('experiment_%d' %experiment):
#     os.makedirs('experiment_%d' %experiment)
#这部分以后用
# while(True):
#     if os.path.exists('experiment_%d' %experiment):
#         experiment+=1
#     else:
#         os.makedirs('experiment_%d' %experiment)
#         break

class Node:
    node_index=0
    time_index=0#在时间轴上的坐标
    station_index=0#在站点轴上的坐标
    time=0#timeindex对应的时间点
    in_node=[]#在构建的网格中，可通过弧到达当前节点的节点
    out_node=[]#在构建的网络中，可通过当前节点直接到达的节点
    def __init__(self,time_index,station_index):
        global max_node_index,core_node_list
        self.time_index=time_index
        self.station_index=station_index
        self.node_index=max_node_index
        max_node_index+=1
        core_node_list.append(self)
    def __getitem__(self, item):
        return self
    def __str__(self):
        str='''time_index:%d
station_index:%d
in_node:%s
out_node:%s '''
        in_node_str=''.join('[%d,%d]' % (node.time_index,node.station_index) for node in self.in_node)
        out_node_str=''.join('[%d,%d]' % (node.time_index,node.station_index) for node in self.out_node)
        return str % (self.time_index,self.station_index,in_node_str,out_node_str)
    def __repr__(self):
        return ('Node %d'% self.node_index)+str([self.time_index,self.station_index])
class Passenger:
    index=0
    o_station_index=0
    d_station_index=0
    o_time_index=0
    d_time_index=0
    board_bus=0
    current_pos=0
    o_station=0
    d_station=0
    AO=[]
    AD=[]
    def __init__(self,index,ot,os,dt,ds):
        global core_passenger_list
        self.index=index
        self.o_time_index=ot
        self.o_station_index=os
        self.d_station_index=ds
        self.d_time_index=dt
        core_passenger_list.append(self)
    def __getitem__(self, item):
        return self
    def __repr__(self):
        return 'Passenger: %d' % self.index
    def __str__(self):
        return '''Passenger: %d
Vehicle: %d
o: [%d,%d]
d: [%d,%d]
''' % (self.index,self.board_bus,self.o_time_index,self.o_station_index,self.d_time_index,self.d_station_index)
class Vehicle:
    index=0
    start_pos=0
    new_pos=-1
    def __init__(self,start_pos):
        global core_vehicle_list,max_vehicle_index
        self.index=max_vehicle_index
        self.start_pos=start_pos
        max_vehicle_index+=1
        core_vehicle_list.append(self)
    def __getitem__(self, item):
        return self
    def __repr__(self):
        return 'Vehicle: %d'% self.index
    def __str__(self):
        return 'Vehicle: %d'% self.index
class Arc:
    i=0
    j=0
    x_list=[]
    y_list=[]
    def __init__(self,node_i,node_j):
        self.i=node_i
        self.j=node_j
    def __getitem__(self, item):
        return self
    def __repr__(self):
        return 'Node: (%d)[%d,%d],(%d)[%d,%d]'% (self.i.node_index,self.i.time_index,self.i.station_index,self.j.node_index,self.j.time_index,self.j.station_index)
class Example(QtGui.QWidget):

    def __init__(self):
        super(Example, self).__init__()
        self.initUI()
    def initUI(self):
        self.setGeometry(300, 300, 280, 170)
        self.setWindowTitle('Route Display')
        self.show()
    def drawPoints(self, qp,sidelength):
        qp.setPen(QtCore.Qt.red)
        size = self.size()
def segment_request(request):
    max_iteration=int(request[:,0].max()//optimal_interval)
    for i in range(max_iteration+1):
        temp=request[np.logical_and(request[:,0]<=optimal_interval*(i+1),request[:,0]>=(optimal_interval*i+1))]
        temp[:,0]=temp[:,0]-optimal_interval*i
        np.save('experiment_%d/new_passenger_%d.npy'% (experiment,i),temp)
    return max_iteration
def get_neighbor_pos(station_num):
    up=0
    down=0
    left=0
    right=0
    if station_num//side_length==0:
        up=1
    elif station_num//side_length==(side_length-1):
        down=1
    if station_num%side_length==0:
        left=1
    elif station_num%side_length==(side_length-1):
        right=1
    if up==1&left==1:
        return [station_num+1,station_num+side_length]
    elif up==1&right==1:
        return [station_num-1,station_num+side_length]
    elif up==1:
        return [station_num-1,station_num+1,station_num+side_length]
    elif down==1&left==1:
        return [station_num-side_length,station_num+1]
    elif down==1&right==1:
        return [station_num-side_length,station_num-1]
    elif down==1:
        return [station_num-side_length,station_num-1,station_num+1]
    elif left==1:
        return [station_num-side_length,station_num+1,station_num+side_length]
    elif right==1:
        return [station_num-side_length,station_num-1,station_num+side_length]
    else:
        return [station_num-side_length,station_num-1,station_num+1,station_num+side_length]
def build_network(station_num,time_horizon):
    time_horizon=int(time_horizon)
    ON=[]
    N=[[] for i in range(time_horizon)]
    DN=[]
    for i in range(station_num):
        ON.append(Node(0,i))
        for j in range(time_horizon):
            N[j].append(Node(j+1,i))
        DN.append(Node(time_horizon+1,i))
    for node in ON:
        node.out_node=[N[0][i] for i in get_neighbor_pos(node.station_index)]
        node.out_node.append(N[0][node.station_index])
    for node in DN:
        node.in_node=[N[-1][i] for i in get_neighbor_pos(node.station_index)]
        node.in_node.append(N[-1][node.station_index])
    for i in range(time_horizon):
        for node in N[i]:
            if i==0:
                node.in_node=[ON[j] for j in get_neighbor_pos(node.station_index)]
                node.in_node.append(ON[node.station_index])
                node.out_node=[N[i+1][j] for j in get_neighbor_pos(node.station_index)]
                node.out_node.append(N[i+1][node.station_index])
            elif i==time_horizon-1:
                node.in_node=[N[i-1][j] for j in get_neighbor_pos(node.station_index)]
                node.in_node.append(N[i-1][node.station_index])
                node.out_node=[DN[j] for j in get_neighbor_pos(node.station_index)]
                node.out_node.append(DN[node.station_index])
            else:
                node.in_node=[N[i-1][j] for j in get_neighbor_pos(node.station_index)]
                node.in_node.append(N[i-1][node.station_index])
                node.out_node=[N[i+1][j] for j in get_neighbor_pos(node.station_index)]
                node.out_node.append(N[i+1][node.station_index])
    return {'ON':ON,'N':N,'DN':DN,'FULL':[ON]+N+[DN]}
def get_city_distance(station_1,station_2):
    x1=station_1//side_length
    x2=station_2//side_length
    y1=station_1%side_length
    y2=station_2%side_length
    return abs(x1-x2)+abs(y1-y2)
def read_onboard_passenger(vehicle_list,iteration=0):
    #读取格式[vehicle_index,d_time,d_station] 0314更新 不使用保存的index
    #Passenger初始化格式:index,ot,os,dt,ds
    global passenger_index,time_horizon,core_boarded_passenger_list
    if iteration==0:
        return []
    passenger_information=np.load('experiment_%d/onboard_passenger_%d.npy'% (experiment,iteration))
    passenger_list=[]
    for passenger in passenger_information:
        start_pos=vehicle_list[passenger[0]].start_pos
        new=Passenger(passenger_index,0,start_pos,passenger[1],passenger[2])
        passenger_list.append(new)
        core_boarded_passenger_list.append(new)
        new.board_bus=passenger[0]
        passenger_index+=1
        time_horizon=max(time_horizon,passenger[1])
    return  passenger_list
def read_vehicle(iteration=0):
    #读取格式[index,start_pos]
    #Vehicle初始化格式相同
    vehicle_list=[Vehicle(-1)]
    if os.path.exists('experiment_%d/vehicle_%d.npy' % (experiment,iteration)):
        vehicle_information=np.load('experiment_%d/vehicle_%d.npy' % (experiment,iteration))
        for vehicle in vehicle_information:
            vehicle_list.append(Vehicle(vehicle[1])) #index,start_pos #0314更新，只接受startpos
    else:
        for i in range(vehicle_num):
            vehicle_list.append(Vehicle(default_pos))
    return vehicle_list
def read_new_passenger(iteration=0):
    #读取格式[ot,os,ds]
    #初始化格式:index,ot,os,dt,ds
    global passenger_index,time_horizon,core_new_passenger_list
    passenger_information=np.load('experiment_%d/new_passenger_%d.npy'% (experiment,iteration)).tolist()
    if os.path.exists('experiment_%d/unsolved_%d.npy'% (experiment,iteration)):
        passenger_information+=np.load('experiment_%d/unsolved_%d.npy'% (experiment,iteration)).tolist()
    passenger_list=[]
    unsolved_list=[]
    for passenger in passenger_information:
        if passenger[0]<=optimal_interval:
            dt=int(passenger[0])+int(math.ceil(1.5*get_city_distance(passenger[1],passenger[2])))
            passenger_list.append(Passenger(passenger_index,int(passenger[0]),int(passenger[1]),dt,int(passenger[2])))
            core_new_passenger_list.append(passenger_list[-1])
            passenger_index+=1
            time_horizon=max(time_horizon,dt)
        else:
            unsolved_list.append([int(passenger[0]-optimal_interval),int(passenger[1]),int(passenger[2])])
    np.save('experiment_%d/unsolved_%d.npy'% (experiment,iteration+1),unsolved_list)
    return passenger_list
def read_determined_passenger(iteration=0):
    #读取格式[os,dt,ds]
    #初始化格式:index,ot,os,dt,ds
    global passenger_index,time_horizon,core_new_passenger_list,core_determined_passenger_list
    if iteration==0:
        return []
    passenger_information=np.load('experiment_%d/determined_passenger_%d.npy'% (experiment,iteration))
    passenger_list=[]
    for passenger in passenger_information:
        passenger_list.append(Passenger(passenger_index,0,passenger[0],passenger[1],passenger[2]))
        core_determined_passenger_list.append(passenger_list[-1])
        passenger_index+=1
        time_horizon=max(time_horizon,passenger[1])
    return passenger_list
def get_node(index):
    global core_node_list
    return core_node_list[index]
def get_related_passenger(node):
    return [i for i in core_passenger_list if node.time_index>=i.o_time_index&node.time_index<=i.d_time_index]
def get_passenger_area(passenger,net,type):#type 1 AO type 2 AD type 3 AOUAD type 4 N/AO/AD type 5 N U o(p) type 6 N/AD
    o=passenger.o_station_index
    ot=passenger.o_time_index
    d=passenger.d_station_index
    dt=passenger.d_time_index
    if type==1:
        return [net['FULL'][i][o] for i in range(ot+1,dt)]
    elif type==2:
        return [net['FULL'][i][d] for i in range(ot+1,dt)]
    elif type==3:
        return [net['FULL'][i][o] for i in range(ot+1,dt)]+[net['FULL'][j][d] for j in range(ot+1,dt)]
    elif type==4:
        return [net['FULL'][t][s] for t in range(ot+1,dt) for s in range(side_length**2) if s!=o and s!=d]
    elif type==5:
        return [net['FULL'][t][s] for t in range(ot+1,dt) for s in range(side_length**2)] + [net['FULL'][ot][o]]
    elif type==6:
        return [net['FULL'][t][s] for t in range(ot+1,dt) for s in range(side_length**2) if s!=d]
    else:
        return []
def get_direct_node(node,net,type):#type1 ci type2 co
    if type==1:
        return net['FULL'][node.time_index-1][node.station_index]
    elif type==2:
        return net['FULL'][node.time_index+1][node.station_index]
    else:
        return []
def get_all_legal_arc(net):
    return [[i,j] for row in net['FULL'][:-1] for i in row for j in i.out_node]
def get_dummy_var(p,net):
    o_node=net['FULL'][p.o_time_index][p.o_station_index]
    d_node=net['FULL'][p.d_time_index][p.d_station_index]
    return var_y(p.index,0,o_node.node_index,d_node.node_index)
def get_T(node,net):
    i=node.time_index
    return [net['FULL'][i][j] for j in range(side_length**2) if j!=node.station_index]
def show_result(my_prob):
    # The optimised objective function value is printed to the screen
    numrows = my_prob.linear_constraints.get_num()
    numcols = my_prob.variables.get_num()
    print(my_prob.solution.status[my_prob.solution.get_status()])
    print("Solution value  = ", my_prob.solution.get_objective_value())
    for i,x in enumerate(my_prob.solution.get_values()):
        if(x!=0 and my_prob.variables.get_names(i)[0:2]=='X_'):
            print "Solution values of ",my_prob.variables.get_names(i),' = ',x
    for i,x in enumerate(my_prob.solution.get_values()):
        if(x!=0 and my_prob.variables.get_names(i)[0:2]=='Y_'):
            print "Solution values of ",my_prob.variables.get_names(i),' = ',x
def my_dicts(name,indexs,lowBound = None,upBound = None,cat = lp.LpContinuous,indexStart=[]):
        if not isinstance(indexs, tuple): indexs = (indexs,)
        if "%" not in name: name += "_%s" * (len(indexs)+1)
        index = indexs[0]
        indexs = indexs[1:]
        d = {}
        if len(indexs) == 0:
            if 'X' in name:
                my_type=1
                q=int(indexStart[0])
                p=-1
            else:
                my_type=2
                p=int(indexStart[0])
                q=int(indexStart[1])
            for i in index:
                d[i]={}
                for j in get_legal_node(i,my_type,p,q):
                    d[i][j.node_index]=lp.LpVariable(name % tuple(indexStart + [str(i)]+[str(j.node_index)]), lowBound, upBound, cat)
        else:
            for i in index:
                d[i] = my_dicts(name, indexs, lowBound, upBound, cat, indexStart + [i])
        return d
def get_legal_node(node_index,type,p=0,q=0):
    #type 1:vehicle
    #type 2:passenger
    global core_node_list,core_passenger_list,core_vehicle_list
    curNode=core_node_list[node_index]
    if type==1:
        return curNode.out_node
    elif type==2:
        curPassenger=core_passenger_list[p]
        curVehicle=core_vehicle_list[q]
        # if curNode.time_index>=curPassenger.d_time_index:
        #     return []
        # if curPassenger.borad_bus!=0 and curPassenger.borad_bus!=q:
        #     return []
        if q==0:
            if len(curNode.out_node)==0:
                return []
            return [curNode.out_node[-1]]
        return curNode.out_node
def build_dummy_var(var_y):
    for p in core_new_passenger_list:
        o_node=net['FULL'][p.o_time_index][p.o_station_index]
        d_node=net['FULL'][p.d_time_index][p.d_station_index]
        var_y[p.index][0][o_node.node_index][d_node.node_index]=lp.LpVariable('Y_%s_%s_%s_%s' % (p.index,0,o_node.node_index,d_node.node_index),0,1,cat='Integer')
def reduce_route(route):
    temp_route=copy.deepcopy(route)
    new_route=copy.deepcopy(route[0])
    unsolved=len(route)-1
    while(unsolved!=0):
        for x,i in enumerate(temp_route):
            if i[0]==new_route[-1]:
                new_route.append(i[1])
                del temp_route[x]
                unsolved-=1
                continue
            if i[1]==new_route[0]:
                new_route.insert(0,i[0])
                del temp_route[x]
                unsolved-=1
                continue
    return new_route
def list_cmp(list1,list2):
    sl1=''.join([str(i) for i in list1])
    sl2=''.join([str(i) for i in list2])
    if sl1.find(sl2)==-1:
        return  True
    else:
        return False
def interpret_result(prob):
    index_list=[i.node_index for i in net['FULL'][optimal_interval]]
    #vehicle
    new_vehicle_list=[]
    vroute_list=[]
    for q in core_vehicle_list[1:]:
        route=[[int(prob.variables.get_names(i).split('_')[2]),int(prob.variables.get_names(i).split('_')[3])] for i,x in enumerate(prob.solution.get_values()) if(x>=0.99 and prob.variables.get_names(i).split('_')[0:2]==['X',str(q.index)])]
        # print route
        route2=reduce_route(route)
        vroute_list.append(route2)
        # print 'Vehicle:%d' % q.index,route2
        for r in route:
            if r[0] in index_list:
                new_vehicle_list.append([q.index,get_node(r[0]).station_index])
                break
    #debug model
    for p in core_passenger_list:
        route=[[int(prob.variables.get_names(i).split('_')[3]),int(prob.variables.get_names(i).split('_')[4])]
                for i,x in enumerate(prob.solution.get_values()) if(x>=0.99 and prob.variables.get_names(i).split('_')[0:2]==['Y',str(p.index)] and prob.variables.get_names(i).split('_')[2]!='0')]
        if len(route)==0:
            continue
        route2=reduce_route(route)
        print 'Passenger %d'% p.index,route2
        for i,x in enumerate(prob.solution.get_values()):
            if (x>=0.99 and prob.variables.get_names(i).split('_')[0:2]==['Y',str(p.index)] and prob.variables.get_names(i).split('_')[2]!='0'):
                cur_vehicle=int(prob.variables.get_names(i).split('_')[2])
                break
        print 'Vehicle %d' %  cur_vehicle,vroute_list[cur_vehicle-1]
        if list_cmp(vroute_list[cur_vehicle-1],route2):
            raise ValueError('Wrong route detected!')
    np.save('experiment_%d/vehicle_%d.npy' % (experiment,iteration),new_vehicle_list)
    #passenger
    new_boarded_list=[]
    new_determined_list=[]
    for p in core_boarded_passenger_list:
        if p.d_time_index<=optimal_interval:
            continue
        else:
            new_boarded_list.append([p.board_bus,p.d_time_index-optimal_interval,p.d_station_index])
    for p in core_new_passenger_list:
        state=[[int(prob.variables.get_names(i).split('_')[2]),int(prob.variables.get_names(i).split('_')[3]),int(prob.variables.get_names(i).split('_')[4])] for i,x in enumerate(prob.solution.get_values()) if(x>=0.99 and prob.variables.get_names(i).split('_')[0:2]==['Y',str(p.index)])]
        station_index=np.unique(np.array(state)[:,1:3].ravel()).tolist()
        o_node=net['FULL'][p.o_time_index][p.o_station_index]
        d_node=net['FULL'][p.d_time_index][p.d_station_index]
        #是否经过dummy0
        if [o_node.node_index,d_node.node_index] in np.array(state)[:,1:3].tolist():
            continue
        #是否已经到达
        if p.d_time_index<=optimal_interval:
            continue
        direct_d_index=net['FULL'][optimal_interval][p.o_station_index].node_index
        for s in state:
            if s[1] in index_list:
                #是否提前到达
                if get_node(s[1]).station_index==p.d_station_index:
                    break
                #是否尚未出发
                if s[0]==0 and get_node(s[1]).station_index==p.o_station_index:
                    new_determined_list.append([p.o_station_index,p.d_time_index-optimal_interval,p.d_station_index])
                    break
                #如果已经出发
                if s[0]!=0:
                    new_boarded_list.append([s[0],p.d_time_index-optimal_interval,p.d_station_index])
                break
    np.save('experiment_%d/determined_passenger_%d.npy'% (experiment,iteration),new_determined_list)
    np.save('experiment_%d/onboard_passenger_%d.npy'% (experiment,iteration),new_boarded_list)
def var_x(p,i,j):
    global core_var_x
    if core_var_x[p][i][j] is 0:
        core_var_x[p][i][j]=lp.LpVariable('X_%d_%d_%d'%(p,i,j),0,1,cat='Integer')
    return core_var_x[p][i][j]
def var_y(p,q,i,j):
    global core_var_y
    if core_var_y[p][q][i][j] is 0:
        core_var_y[p][q][i][j]=lp.LpVariable('Y_%d_%d_%d_%d'%(p,q,i,j),0,1,cat='Integer')
    return core_var_y[p][q][i][j]
def result_stat(my_prob):
    pass
#initialization
request=np.load('experiment_%d/request.npy' % experiment)
max_iteration=segment_request(request)
#main
unfulfilled_stat=[]
for i in range(min_vehicle,max_vehicle+1):
    vehicle_num=i
    unfulfilled=0
    iteration=0
    while iteration<=max_iteration:
        passenger_index=0
        time_horizon=optimal_interval
        max_node_index=0
        max_vehicle_index=0#留出0来
        core_node_list=[]
        core_passenger_list=[]
        core_new_passenger_list=[]
        core_boarded_passenger_list=[]
        core_determined_passenger_list=[]
        core_vehicle_list=[]
        vehicle=read_vehicle(iteration)
        onboard_passenger=read_onboard_passenger(vehicle,iteration)
        new_passenger=read_new_passenger(iteration)
        determined_passenger=read_determined_passenger(iteration)
        if len(core_passenger_list)==0:
            iteration+=1
            np.save('experiment_%d/vehicle_%d.npy' % (experiment,iteration),[])
            np.save('experiment_%d/determined_passenger_%d.npy'% (experiment,iteration),[])
            np.save('experiment_%d/onboard_passenger_%d.npy'% (experiment,iteration),[])
            continue
        net=build_network(side_length*side_length,time_horizon)
        dummy_arc=1
        prob=lp.LpProblem('Iteration_%d'%iteration,lp.LpMinimize);
        #create index for variable
        var_node_index=[i for i in range(max_node_index)]
        la=len(var_node_index)
        var_passenger_index=[i for i in range(passenger_index)]
        lb=len(var_passenger_index)
        var_vehicle_index=[i for i in range(max_vehicle_index)]
        lc=len(var_vehicle_index)
        core_var_x=np.zeros((lc,la,la),object)
        core_var_y=np.zeros((lb,lc,la,la),object)
        # var_x=my_dicts('X',(var_vehicle_index[1:], var_node_index),0,1,cat='Integer')
        # var_y=my_dicts('Y',(var_passenger_index,var_vehicle_index,var_node_index),0,1,cat='Integer')
        # var_x=lp.LpVariable.dicts('X',(var_vehicle_index[1:],var_node_index,var_node_index),0,1,cat='Integer')
        # var_y=lp.LpVariable.dicts('Y',(var_passenger_index,var_vehicle_index,var_node_index,var_node_index),0,1,cat='Integer')
        # build_dummy_var(var_y)
        # var_x=lp.LpVariable.dicts('X',(var_vehicle_index,var_node_index),0,1,cat='Integer')
        #objective
        print 'net built'
        prob += lp.lpSum([get_dummy_var(p,net) for p in core_new_passenger_list])
        #build constraint
        #constraint 1
        for row in net['N']:
            for node in row:
                for v in vehicle[1:]:#Vehicle为步行
                    a=lp.lpSum([var_x(v.index,node.node_index,j.node_index) for j in node.out_node ])
                    b=lp.lpSum([var_x(v.index,h.node_index,node.node_index) for h in node.in_node ])
                    prob += a-b == 0,''
        #constraint 2
        for v in  vehicle[1:]:
            i=net['ON'][v.start_pos]
            a=lp.lpSum([var_x(v.index,i.node_index,j.node_index) for j in i.out_node])
            prob += a==1,''
        #constraint 3
        for v in vehicle[1:]:
            a=lp.lpSum([var_x(v.index,i.node_index,j.node_index) for j in net['DN'] for i in j.in_node])
            prob += a==1,''
        #constraint 4
        for p in core_new_passenger_list:
            for q in vehicle[1:]:
                for i in get_passenger_area(p,net,4):
                    a=lp.lpSum([var_y(p.index,q.index,i.node_index,j.node_index) for j in i.out_node])
                    b=lp.lpSum([var_y(p.index,q.index,h.node_index,i.node_index) for h in i.in_node])
                    prob += a-b==0,''
        for p in core_determined_passenger_list:
            for q in vehicle[1:]:
                for i in get_passenger_area(p,net,4):
                    a=lp.lpSum([var_y(p.index,q.index,i.node_index,j.node_index) for j in i.out_node])
                    b=lp.lpSum([var_y(p.index,q.index,h.node_index,i.node_index) for h in i.in_node])
                    prob += a-b==0,''
        for p in core_boarded_passenger_list:
            for q in vehicle[1:]:
                for i in get_passenger_area(p,net,6):
                    a=lp.lpSum([var_y(p.index,q.index,i.node_index,j.node_index) for j in i.out_node])
                    b=lp.lpSum([var_y(p.index,q.index,h.node_index,i.node_index) for h in i.in_node])
                    prob += a-b==0,''
        #constraint 5
        #5a
        for p in core_new_passenger_list:
            for i in get_passenger_area(p,net,3):
                for q in vehicle[1:]:
                    a=lp.lpSum([var_y(p.index,q.index,g.node_index,i.node_index) for g in i.in_node])
                    b=lp.lpSum([var_y(p.index,q.index,i.node_index,k.node_index) for k in i.out_node])
                    prob+= a+var_y(p.index,0,i.in_node[-1].node_index,i.node_index)-var_y(p.index,0,i.node_index,i.out_node[-1].node_index)-b == 0,''
        #5b
        for p in core_boarded_passenger_list:
            for i in get_passenger_area(p,net,2):
                for q in vehicle[1:]:
                    a=lp.lpSum([var_y(p.index,q.index,g.node_index,i.node_index) for g in i.in_node])
                    b=lp.lpSum([var_y(p.index,q.index,i.node_index,k.node_index) for k in i.out_node])
                    prob+= a+var_y(p.index,0,i.in_node[-1].node_index,i.node_index)-var_y(p.index,0,i.node_index,i.out_node[-1].node_index)-b == 0,''
        #5c
        for p in core_determined_passenger_list:
            for i in get_passenger_area(p,net,3):
                for q in vehicle[1:]:
                    a=lp.lpSum([var_y(p.index,q.index,g.node_index,i.node_index) for g in i.in_node])
                    b=lp.lpSum([var_y(p.index,q.index,i.node_index,k.node_index) for k in i.out_node])
                    prob+= a+var_y(p.index,0,i.in_node[-1].node_index,i.node_index)-var_y(p.index,0,i.node_index,i.out_node[-1].node_index)-b == 0,''
        #constraint 6
        for p in core_new_passenger_list+core_determined_passenger_list:
            for i in get_passenger_area(p,net,1):
                prob+= var_y(p.index,0,i.in_node[-1].node_index,i.node_index) - var_y(p.index,0,i.node_index,i.out_node[-1].node_index) >= 0,''
        #constraint 7
        for p in core_passenger_list:
            for i in get_passenger_area(p,net,2):
                prob+= var_y(p.index,0,i.in_node[-1].node_index,i.node_index) - var_y(p.index,0,i.node_index,i.out_node[-1].node_index) <= 0,''
        #constraint 8
        for p in core_new_passenger_list:
            i=net['FULL'][p.o_time_index][p.o_station_index]
            a=lp.lpSum([var_y(p.index,q.index,i.node_index,j.node_index) for q in vehicle[1:] for j in i.out_node])
            b=lp.lpSum([var_y(p.index,q.index,h.node_index,k.node_index) for q in vehicle for h in get_T(i,net) for k in h.out_node])
            prob+= a + var_y(p.index,0,i.node_index,i.out_node[-1].node_index) + get_dummy_var(p,net)== 1,''
            prob+= b ==0,''
        #constraint 9
        for p in core_determined_passenger_list:
            i=net['FULL'][p.o_time_index][p.o_station_index]
            a=lp.lpSum([var_y(p.index,q.index,i.node_index,j.node_index) for q in vehicle[1:] for j in i.out_node])
            b=lp.lpSum([var_y(p.index,q.index,h.node_index,k.node_index) for q in vehicle for h in get_T(i,net) for k in h.out_node])
            prob+= a + var_y(p.index,0,i.node_index,i.out_node[-1].node_index)== 1,''
            prob+= b ==0,''
        #constraint 10
        for p in core_boarded_passenger_list:
            i=net['FULL'][p.o_time_index][p.o_station_index]
            q=vehicle[p.board_bus]
            a=lp.lpSum([var_y(p.index,q.index,i.node_index,j.node_index) for j in i.out_node])
            b=lp.lpSum([var_y(p.index,q1.index,i.node_index,j.node_index) for j in i.out_node for q1 in vehicle if q1.index!=p.board_bus])
            c=lp.lpSum([var_y(p.index,q2.index,h.node_index,k.node_index) for q2 in vehicle for h in get_T(i,net) for k in h.out_node])
            prob+= a == 1,''
            prob+= b == 0,''
            prob+= c == 0,''
        #constraint 11
        for p in core_new_passenger_list:
            i=net['FULL'][p.d_time_index][p.d_station_index]
            a=lp.lpSum([var_y(p.index,q.index,h.node_index,i.node_index) for q in vehicle[1:] for h in i.in_node])
            b=lp.lpSum([var_y(p.index,q2.index,h.node_index,k.node_index) for q2 in vehicle for k in get_T(i,net) for h in k.in_node])
            prob+= a + var_y(p.index,0,i.in_node[-1].node_index,i.node_index) + get_dummy_var(p,net) == 1,''
            prob+= b==0,''
        #constraint 12
        for p in core_boarded_passenger_list+core_determined_passenger_list:
            i=net['FULL'][p.d_time_index][p.d_station_index]
            a=lp.lpSum([var_y(p.index,q.index,h.node_index,i.node_index) for q in vehicle[1:] for h in i.in_node])
            b=lp.lpSum([var_y(p.index,q2.index,h.node_index,k.node_index) for q2 in vehicle for k in get_T(i,net) for h in k.in_node])
            prob+= a + var_y(p.index,0,i.in_node[-1].node_index,i.node_index) == 1,''
            prob+= b==0,''
        #constraint 13
        for q in vehicle[1:]:
            for p in core_passenger_list:
                for i in get_passenger_area(p,net,5):
                    for j in i.out_node:
                        prob+= var_x(q.index,i.node_index,j.node_index) - var_y(p.index,q.index,i.node_index,j.node_index) >=0,''
        print 'constraint built'
        prob.writeLP('experiment_%d/iteration_%d.lp'%(experiment,iteration))
        my_prob = cplex.Cplex('experiment_%d/iteration_%d.lp'%(experiment,iteration))
        my_prob.parameters.mip.tolerances.relobjdifference.set(0.02)
        my_prob.parameters.mip.tolerances.objdifference.set(2)
        my_prob.parameters.mip.tolerances.mipgap.set(0.05)
        my_prob.parameters.mip.strategy.variableselect.set(3)
        my_prob.parameters.emphasis.mip.set(1)
        print 'Solve iteration %d' % iteration
        my_prob.solve()
        unfulfilled+=my_prob.solution.get_objective_value()
        # interpret
        iteration+=1
        interpret_result(my_prob)
        show_result(my_prob)
        result_stat(my_prob)
    unfulfilled_stat.append(unfulfilled)
    print unfulfilled_stat
print unfulfilled_stat