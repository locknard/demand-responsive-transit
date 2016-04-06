#constraint 4
for row in net['N']:
    for node in row:
        for v in vehicle[1:]:
            for p in get_related_passenger(node):
                a=lp.lpSum([var_y[p.index][v.index][node.node_index][j.node_index] for j in node.out_node])
                b=lp.lpSum([var_y[p.index][v.index][h.node_index][node.node_index] for h in node.in_node])
                prob += a-b==0,''
#constraint 5
for p in core_new_passenger_list:
    for i in get_passenger_area(p,net,3):
        a=lp.lpSum([var_y[p.index][q.index][g.node_index][i.node_index] for q in vehicle[1:] for g in i.in_node[:-1]])
        b=lp.lpSum([var_y[p.index][q.index][k.node_index][i.node_index] for q in vehicle[1:] for k in i.out_node[:-1]])
        prob+= a+var_y[p.index][0][i.in_node[-1].node_index][i.node_index]-var_y[p.index][0][i.out_node[-1].node_index][i.node_index]-b == 0,''
for p in core_boarded_passenger_list:
    for i in get_passenger_area(p,net,2):
        a=lp.lpSum([var_y[p.index][q.index][g.node_index][i.node_index] for q in vehicle[1:] for g in i.in_node[:-1]])
        b=lp.lpSum([var_y[p.index][q.index][k.node_index][i.node_index] for q in vehicle[1:] for k in i.out_node[:-1]])
        prob+= a+var_y[p.index][0][i.in_node[-1].node_index][i.node_index]-var_y[p.index][0][i.out_node[-1].node_index][i.node_index]-b == 0,''
#constraint 6
for p in core_new_passenger_list:
    for i in get_passenger_area(p,net,1):
        prob+= var_y[p.index][0][i.in_node[-1].node_index][i.node_index] - var_y[p.index][0][i.out_node[-1].node_index][i.node_index] >= 0,''
#constraint 7
for p in core_passenger_list:
    for i in get_passenger_area(p,net,2):
        prob+= var_y[p.index][0][i.in_node[-1].node_index][i.node_index] - var_y[p.index][0][i.out_node[-1].node_index][i.node_index] <= 0,''
#constraint 8
for p in core_new_passenger_list:
    i=net['FULL'][p.o_time_index][p.o_station_index]
    a=lp.lpSum([var_y[p.index][q.index][i.node_index][j.node_index] for q in vehicle[1:] for j in i.out_node])
    prob+= a + var_y[p.index][0][i.node_index][i.out_node[-1].node_index] + get_dummy_var(var_y,p,net)== 1,''
#constraint 9
for p in core_boarded_passenger_list:
    i=net['FULL'][p.o_time_index][p.o_station_index]
    q=vehicle[p.board_bus]
    a=lp.lpSum([var_y[p.index][q.index][i.node_index][j.node_index] for j in i.out_node])
    prob+= a == 1,''
#constraint 10
for p in core_passenger_list:
    i=net['FULL'][p.d_time_index][p.d_station_index]
    a=lp.lpSum([var_y[p.index][q.index][h.node_index][i.node_index] for q in vehicle for h in i.in_node])
    prob+= a + get_dummy_var(var_y,p,net)== 1,''
#constraint 11
for q in vehicle[1:]:
    for p in core_passenger_list:
        for ij in get_all_legal_arc(net):
            prob+= var_x[q.index][ij[0].node_index][ij[1].node_index] - var_y[p.index][q.index][ij[0].node_index][ij[1].node_index] >= 0,''