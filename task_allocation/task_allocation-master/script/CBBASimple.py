agents = {1:(0,0), 2:(100,0), 3:(0,100)} #Should be ROS input
targets = {1:[1,(1,1)], 2:[0.5,(150,2)], 3:[0.5,(150,10)]} #, 3:[1,(6,6)], 4:[1,(4,6)]}   #Should be ROS input. [weight, (coords)]
reward = 1000
A={} #global bidspace, one win bid is i:[(j,reward_value),() ... ]

for agent in agents:
    A[agent]=[]

def distance(p1,p2):  #takes in coords, want to do better than Euclidian....
    result = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    return result

def reward_value(agent, target): #Reward-accumulative_distance takes into account importance value for targets
    #should be replaced by D*
    i=agent
    j=target
    agent_coord=agents[i]
    target_coord=targets[j][1]

    #empty A, then is the current position to target j
    if not A[i]:
        accum_dist = distance(agent_coord, target_coord)

    else:
        accum_dist=0
        current_allocation=A[i] #list of touples (j,reward_value)
        if len(current_allocation)>1:

            for index in range(len(current_allocation)-1):
                j=current_allocation[index][0]  #index of target
                next=current_allocation[index+1][0]
                j_coord=targets[j][1]
                next_coord=targets[next][1]
                weight=targets[index+1][0] #how important is it to go to this next_target
                accum_dist+=distance(j_coord, next_coord)*(1-weight) #adding distance between all allocated targets to the accumulative distance
            accum_dist+=distance(targets[current_allocation[-1][0]][1], target_coord)*(1-targets[j][0]) #adding the final term from last allocated target, to the target of interest, j
        else:
            weight=targets[current_allocation[0][0]][0]
            accum_dist+=distance(agent_coord,targets[current_allocation[0][0]][1] )*(1-weight)
            weight=targets[j][0]
            accum_dist+=distance(agent_coord,targets[j][1])*(1-weight)
    return reward-accum_dist

count=0

while count<10:  #bidding process, wind bids on the fly
    count+=1
    for i in agents:
        for j in targets:
            win=True
            for agent in A:
                if A[agent]:
                    for target in A[agent]: #target is a tuple (j, reward_value)
                        if target[0]==j:

                            if reward_value(i,j)<=target[1]:
                                win=False

            if win:

                for agent in A:
                    if A[agent]:
                        for target in A[agent]: #target is a tuple (j, reward_value)
                            if target[0]==j:
                                A[agent].remove(target)
                A[i].append((j, reward_value(i,j)))


print (A)
#To Navid: A is the final task allocation, and is what that should be passed on in the system
