# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 13:31:01 2016

@author: Octavio Villarreal
"""

# -*- coding: utf-8 -*-
"""
Zebro algorithm
"""

import math

def MPTimes(a,b) :
    x = [[0 for i in range(len(b[0]))] for j in range(len(a))]
    y = [0 for i in range(len(b))]
   # for i in range(len(a)):
  #  for j in range(len(b[0])): 			
  #      x[i][j] = -float("inf")
    for i in range(len(a)):					
        for j in range(len(b[0])):		 	
            for k in range(len(b)):			
                y[k] = a[i][k]+b[k][j]
            x[i][j] = max(y)   
    return x
#Example of using MPTimes    
#u = [[2,0],[0,1]]
#v = [[0,-float("inf")],[-float("inf"),0]]
#x = MPTimes(u,v)
#print (x)


def MPPlus(a,b)	:
    x = [[0 for i in range(len(b[0]))] for j in range(len(a))]
    for i in range(len(a)):
        for j in range(len(b[0])):
            x[i][j]=max(a[i][j],b[i][j])
    return x
#Example of using MPPlus
#u = [[2,0],[0,1]]
#v = [[0,-float("inf")],[-float("inf"),0]]
#x = MPPlus(u,v)
#print (x)


def MPNullMatrix(x):
    E = [[0 for i in range(x)] for j in range(x)]
    for i in range(x):
        for j in range(0, x):
            E[i][j] = -float("inf")
    return E
#Example of creating MPNullMatrix with size of x
#E = MPNullMatrix(3)

def MPIdentityMatrix(x):
    I = MPNullMatrix(x)
    for i in range(x):
        I[i][i] = 0
    return I
 #Example of creating MPIdentityMatrix with size of x
#I = MPIdentityMatrix(2)

def MPGeneratePQMatrices(gait,number_legs,Td):
    P = MPNullMatrix(number_legs)
    Q = MPNullMatrix(number_legs)
    leng = len(gait)
    for i in range(0,leng-1):
        for k in range(0,len(gait[i+1])):
            for l in range(0,len(gait[i])):
                P[gait[i+1][k]-1][gait[i][l]-1] = Td[i]	
    
    for m in range(0,len(gait[0])):
        for n in range(0,len(gait[leng-1])):
            p = (gait[0][m])-1
            q = (gait[leng-1][n])-1
            Q[p][q] = Td[leng - 1]
    return (P,Q)


def MPGenerateGHMatrices(P,Q,Tf,Tg):
    PP = P				
#QQ = Q
    siz = len(P[0])
#null = MPNullMatrix(siz)
    I = MPIdentityMatrix(siz)
    K = MPIdentityMatrix(siz)
    for i in range(0,siz):
        for j in range(0,siz): 
            x = I[i][j] + Tf
            I[i][j] = x
    for i in range(0,siz):
        for j in range(0,siz):
            K[i][j] = K[i][j] + Tg	
    G = MPNullMatrix(2*siz)
    for i in range(0,siz):
        for j in range(0,siz):
            G[i][siz+j] = I[i][j]
    for i in range(0,siz):
        for j in range(0,siz):
            G[i+siz][j] = PP[i][j]
    H = MPNullMatrix(2*siz)
    T = MPPlus(K,Q)
    for i in range(0,siz):
        for j in range(0,siz):
            H[i+siz][j] = T[i][j]
    return (G, H)

def MPComputeAStar(A):	
    AA = A
    siz = len(A[0])
    As = MPIdentityMatrix(siz)
    As_ = MPIdentityMatrix(siz)
    for n in range(0,siz-1): 
        As = MPPlus(MPTimes(As,AA),AA)
    As=MPPlus(As,As_)
    return As

def MPGenerateAllMatrices(gait,number_legs,Tf,Tg,Td):
    P,Q = MPGeneratePQMatrices(gait,number_legs,Td)
    G,H = MPGenerateGHMatrices(P,Q,Tf,Tg)
    A = MPTimes(MPComputeAStar(G),H)
    return (A,G,H,P,Q)
	
def increaseEventList(EList, x, A):
    xnew = MPTimes(A,x)
    new_index = len(EList)+1
    EList_new = [[0 for i in range(len(EList[0]))] for j in range(new_index)]
    for j in range(0,len(EList)):
        for k in range(0,len(EList[0])):
            EList_new[j][k]=EList[j][k]
    #EList[new_index] = [0 for i in range(0,len(xnew))]
    for j in range(0,len(xnew)):
        EList_new[new_index-1][j] = xnew[j][0]
    return EList_new, xnew

def ComputeEventsHyQ(EventsList0,gait,numberOfLegs,Tf,Tg,Td,x_0):
    #	-- Generate Eventslist
    A,G,H,P,Q = MPGenerateAllMatrices(gait,numberOfLegs,Tf,Tg,Td)
    #EventsList = [[0 for i in range(0,numberOfLegs*2)]]

    for i in range(0,1):
        EventsList0, x_next = increaseEventList(EventsList0, x_0, A)    
        x_0 = x_next
    
    return EventsList0, x_0