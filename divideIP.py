
#coding=utf-8
from pulp import *
import numpy as np
import pandas as pd
import random




class Node:
    timeIndex=0#在时间轴上的坐标
    stationIndex=0#在站点轴上的坐标
    time=0#timeindex对应的时间点
    inNode=[]#在构建的网格中，可通过弧到达当前节点的节点
    outNode=[]#在构建的网络中，可通过当前节点直接到达的节点
    def __init__(self,timeIndex,stationIndex):
        self.timeIndex=timeIndex
        self.stationIndex=stationIndex
    def __getitem__(self, item):
        return self

class Edge:
    empty=True
    originNode=0
    destNode=0
    cv=0#车辆经过弧的费用
    cp=0#乘客经过弧的费用
    xVar=0#所有的x^k_ij，对于当前的ij来说有k个变量
    yVar=0#所有的x^l_ijk，对于当前的ij来说，有l*k个变量，axis0为l，axis1为k
    def __init__(self,originNode,destNode,**kwargs):
        if 'empty' in kwargs:
            self.empty=kwargs['empty']
        self.originNode=originNode
        self.destNode=destNode
    def __getitem__(self, item):
        return self

class Graph:
    nodeList=0
    edgeList=0
    def __init__(self,numOfStation,numOfTime,distanceMatrix,vehicleCostMatrix,customerCostMatrix,shuttleNum,requestList):
        tempArray=[]
        numOfRequest=requestList.__len__()
        for i in range(numOfTime):
            for j in range(numOfStation):
                tempArray.append(Node(i,j))
        nodeList=np.array(tempArray).reshape(numOfTime,numOfStation)
        #开始设定各个Node的列表,其中 头、尾需要和初始车场以及末尾车场相连,暂且留空
        for node in nodeList[0,:]:#设定头，头的inNode留空
            i=0
            for num in distanceMatrix[:,int(node.stationIndex)]:
                i+=1
                node.outNode.append(nodeList[int(node.timeIndex+num),i-1])
        for node in nodeList[numOfTime-1,:]:#设定尾，尾的outNode留空
            i=0
            for num in distanceMatrix[:,int(node.stationIndex)]:
                i+=1
                node.inNode.append(nodeList[int(node.timeIndex-num),i-1])
        for row in nodeList[1:numOfTime-1,:]:
            for node in row:
                i=0
                for num in distanceMatrix[:,int(node.stationIndex)]:
                    i+=1
                    if(node.timeIndex+num<=numOfTime-1):
                        node.outNode.append(nodeList[int(node.timeIndex+num),i-1])
                    if(node.timeIndex-num>=0):
                        node.inNode.append(nodeList[int(node.timeIndex-num),i-1])
        #建立初始edge表，所有连接先设为空连接
        edgeArray=[]
        for i in range(numOfTime):
            for j in range(numOfStation):
                for k in range(numOfTime):
                    for l in range(numOfStation):
                        edgeArray.append(Edge(nodeList[i,j],nodeList[k,l],empty=True))
        edgeList=np.array(edgeArray).reshape(numOfTime,numOfStation,numOfTime,numOfStation)
        #将有用的连接进行连接，并且生成变量列表
        for row in nodeList:
            for node in row:
                for oNode in node.outNode:
                    tempEdge=edgeList[node.timeIndex,node.stationIndex,oNode.timeIndex,oNode.stationIndex]
                    tempEdge.cv=costMatrix[node.stationIndex,oNode.stationIndex]
                    tempEdge.cp=customerCostMatrix[node.stationIndex,oNode.stationIndex]
                    tempEdge.empty=False
                    #开始填充每个edge所包含的变量
                    #x^k_ij
                    xList=[]
                    for k in range(shuttleNum):
                        tempStr= '^'+str(k)+'_('+str(node.timeIndex)+','+str(node.stationIndex)+')_('+str(oNode.timeIndex)+','+str(oNode.stationIndex)+')'
                        xList.append(tempStr)
                    tempEdge.xVar=LpVariable.dicts('x',xList,0)
                    tempEdge.yVar=[]
                    #y^l_ijk 这里建立一个长度为l的列表，每个元素是一个变量的列表
                    for l in range(numOfRequest):
                        yList=[]
                        for k in range(shuttleNum):
                            tempStr='^'+str(l)+str(node.timeIndex)+','+str(node.stationIndex)+')_('+str(oNode.timeIndex)+','+str(oNode.stationIndex)+')_'+str(k)
                            yList.append(tempStr)
                        tempEdge.yVar.append(LpVariable.dicts('y',yList,0))
        self.edgeList=edgeList
        self.nodeList=nodeList


def generateRequest(distanceMatrix,numOfTime,numOfRequest,speed=1):
    numOfStation=int(distanceMatrix.shape[0])
    longestTime=int(max(distanceMatrix.ravel())/speed)
    requestList=[]
    for i in range(numOfRequest):
        oStation=random.randint(0,numOfStation-1)
        dStation=random.randint(0,numOfStation-2)
        if dStation>=oStation:
            dStation+=1
        oTime=random.randint(0,numOfTime-longestTime-1)#在最后5个时间单位不再产生需求
        slackTime=random.randint(1,3)
        dTime=min(oTime+slackTime+int(distanceMatrix[oStation,dStation]/speed),numOfTime-1)
        requestList.append([oTime,oStation,dTime,dStation])
    return requestList



distanceMatrix=np.array([[1,3,2,3,4],[3,1,2,3,3],[2,2,1,2,3],[3,3,2,1,3],[4,3,3,3,1]],dtype=int)
costMatrix=distanceMatrix
customerCostMatrix=np.eye(5,dtype=float)
station=5
mytime=15
shuttleNum=4
#requestArray=np.array([[0,0,],[],])
requestList=generateRequest(distanceMatrix,mytime,15,1)
myGraph=Graph(station,mytime,distanceMatrix,costMatrix,customerCostMatrix,shuttleNum,requestList)
mytime=14



#disMatrix 现有站点间的距离矩阵,使用ndarray传递
#request 发出的请求，包括起始点和终点
#shuttleNum 使用几辆shuttle服务
#planHorizon 求解的horizon，建议采用最晚的预计服务完成时间+5个prec
#prec 时空网络中的时间精度，使用分钟min
def solveShuttle(disMatrix,request,shuttleNum,planHorizon,prec):
    test=0
    #还没有添加参数校验
    prob=LpProblem('Shuttle Bus',LpMinimize);
    #numOfStation=int(disMatrix.shape[0]);#有多少个站点
    #numOfTime=int(planHorizon/prec);#有多少个时间
   #