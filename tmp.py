def constrainedUCS(src, goal):
    weights = {}  # weights store all the distance and energy states of each node
    pq = PriorityQ()
    pq.push(0, (None, 0, 0, src))
    # distance, (previous-node-state, distance, energy-cost, node)
    weights[src] = set()
    weights[src].add((0, 0))  # (distance, energy-cost)
    
    while not pq.isEmpty():
        curstate = pq.pop()  # curstate: current state
        cur = curstate[3]
        if cur == goal:
            distance = curstate[1]
            cost = curstate[2]
            path = [goal]
            while curstate[0]:
                path.insert(0, curstate[0][3])
                curstate = curstate[0]
            return path, distance, cost

        for v in G[cur]:
            to_add = True
            if curstate[2]+Cost[cur+','+v] <= energyConstraint:
                d = curstate[1]+Dist[cur+','+v]  # distance
                e = curstate[2]+Cost[cur+','+v]  # energy cost
                if v in weights:
                    for weight in weights[v]:
                        if d >= weight[0] and e >= weight[1]:
                        # if v has been visited in the current path, its current distance
                        # and energy cost must both be higher than the existing values.That's why
                        # this condition is enough
                            to_add = False
                            break
                    if to_add:
                        pq.push(d, (curstate, d, e, v))
                        weights[v].add((d, e))
                else:
                    pq.push(d, (curstate, d, e, v))
                    weights[v] = set()
                    weights[v].add((d, e))
    return None
