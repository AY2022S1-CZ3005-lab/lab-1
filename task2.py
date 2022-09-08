from queue import PriorityQueue
from time import time

from ReadGraph import readGraph

G, Coord, Dist, Cost = readGraph()

# n: num of nodes
# st: start vertex
# en: end vertex
# budget: cost constraint
n = len(G)
st = 1
en = 50
budget = 287932

# guarantee the cost is minimum
def UCS(G, Dist, Cost, st, en):
  # init
  global iteration_cnt
  pa = dict()
  vis = set()
  pq = PriorityQueue()
  cost = [float('inf')] * (n + 1)
  dis = [float('inf')] * (n + 1)

  pq.put((0, 0, st))
  pa[(0, 0, st)] = None
  while not pq.empty():
    iteration_cnt += 1
    cur = pq.get()
    # current state
    # (dis, cost, vertex)
    (d, co, u) = cur
    if u == en: 
      path = []
      p = cur
      while p:
        path.append(p[2])
        p = pa[p]
      path.reverse()
      return d, co, path
    for v_str in G[str(u)]:
      # for each edge(u, v) with lenght = w and cost = c
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      next = (d + w, co + c, v)
      # if next state is searched or worse than searched or break the constraint
      # abondon the state
      if next in vis or (c + co >= cost[v] and d + w >= dis[v]) or c + co > budget:
        continue
      # the state is either not searched or better than previous state of vertex u
      cost[v] = c + co # update the better state
      dis[v] = d + w  # update the better state
      pa[next] = cur # record next state's parent
      pq.put(next)
  # no path
  return float('inf'), float('inf'), []

# Run task 2
print()
print('-------- Running Task 2 --------')
print()

begin_time = time()
iteration_cnt = 0
dis, cost, path = UCS(G, Dist, Cost, st, en)
time_used = time() - begin_time

print('Shortest path: ', end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print(f'Shortest distance: {dis}')
print(f'Total energy cost: {cost}')
print(f'Iteration round: {iteration_cnt}')
print(f'Time used: {time_used}')
