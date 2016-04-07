#coding=utf-8
import numpy as np
import pandas as pd
import math
import scipy.spatial.distance as distance

def cellAxis2Num(x,y,ynum):#(x,y)
    return ynum*x+y
def cellNum2Axis(num,ynum):#(x,y)
    return [num//ynum,num%ynum]
def latlng_2_xy(list):
    return [[(i[0]-down)*math.pi/180.0*RADIUS,(i[1]-left)*math.pi/180.0*RADIUS] for i in list]
def manhattan_ETA(o,d,t0):
    ox=o%side_length
    oy=o//side_length
    dx=d%side_length
    dy=d//side_length
    return t0+np.ceil((abs(dx-ox)+abs(dy-oy))*1.5)
RADIUS=6378.140
initialTime=36000
endTime=43200
down=39.068995
left=115.602595
filename='Reduced20151122'
up_left_point=[39.8935486,116.4616619]
prec=1.0
side_length=5
time_prec=120.0

R=pd.read_csv(filename,index_col=0)
del R['mode']
x_max=max(R['o_x'].max(),R['d_x'].max())
y_max=max(R['o_y'].max(),R['d_y'].max())
x_cell_num=x_max//prec+1
y_cell_num=y_max//prec+1
R2=R.assign(o_cell=cellAxis2Num(R['o_x']//prec,R['o_y']//prec,y_cell_num)).assign(d_cell=cellAxis2Num(R['d_x']//prec,R['d_y']//prec,y_cell_num))
R2=R2[np.logical_and(R2['t0']>initialTime,R2['t0']<endTime)]
xy_point=latlng_2_xy([up_left_point])[0]
ref_cell=cellAxis2Num(xy_point[0]//prec,xy_point[1]//prec,y_cell_num)
ref_mat=[ref_cell+j-i*y_cell_num for j in range(side_length) for i in range(side_length)]
R3=R2[np.logical_and(R2.o_cell.isin(ref_mat),R2.d_cell.isin(ref_mat))]
xy_point_2=cellNum2Axis(ref_cell,y_cell_num)
station_mat=[[xy_point_2[0]-prec/2-i,xy_point_2[1]+prec/2+j] for i in range(side_length) for j in range(side_length)]
oDist=distance.cdist(R3.iloc[:,1:3].values,station_mat)
dDist=distance.cdist(R3.iloc[:,3:5].values,station_mat)
oStation=oDist.argmin(axis=1)
dStation=dDist.argmin(axis=1)
t0=((R3['t0']-initialTime)//120.0).values
newR=pd.DataFrame.from_items([('o',oStation),('d',dStation),('t0',t0)])
np.save('request_%d_%d-%d.npy'% (ref_cell,initialTime/3600,endTime/3600),newR.values)
print 1