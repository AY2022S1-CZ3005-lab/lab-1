from math import sqrt
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

h = [0] * (n + 1)
(en_x, en_y) = Coord[str(en)]

# pre-calculate h(x) functoin
# calculate in place or memorization are also okay
for i in range(1, n + 1):
  (x, y) = Coord[str(i)]
  h[i] = sqrt((x - en_x) ** 2 + (y - en_y) ** 2)

# define state
# instead comparing using f(x), a* uses f(x) + h(x)
# here f(x) == d, h(x) == h[x]
# h[x] should be pre-calculated
class state:
  def __init__(self, d, co, u):
    self.d = d
    self.co = co
    self.u = u

  # for PriorityQueue(), use f(x) + h(x) as evaluation
  def __lt__(self, other):
    global h
    if self.d + h[self.u] == other.d + h[other.u]:
      return self.co < other.co
    return self.d + h[self.u] < other.d + h[other.u]

  # for set() and dict()
  def __hash__(self):
      return hash((self.d, self.co, self.u))
  
  def __eq__(self, other):
      return (self.d, self.co, self.u) == (other.d, other.co, other.u)

# guarantee the cost is minimum
def astar(G, Dist, Cost, st, en):
  # init
  global iteration_cnt
  pa = dict()
  vis = set()
  pq = PriorityQueue()
  cost = [float('inf')] * (n + 1)
  dis = [float('inf')] * (n + 1)

  cost[st] = 0
  dis[st] = 0
  vis.add((0, 0, st))
  pq.put(state(0, 0, st))
  pa[state(0, 0, st)] = None
  while not pq.empty():
    iteration_cnt += 1
    # current state
    # (dis, cost, vertex)
    cur = pq.get()
    d = cur.d
    co = cur.co
    u = cur.u
    if u == en: 
      path = []
      p = cur
      while p:
        path.append(p.u)
        p = pa[p]
      path.reverse()
      return d, co, path
    for v_str in G[str(u)]:
      # for each edge(u, v) with lenght = w and cost = c
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      next = state(d + w, co + c, v)
      # if next state is searched or worse than searched or break the constraint
      # abondon the state
      if next in vis or (c + co >= cost[v] and d + w >= dis[v]) or c + co > budget:
        continue
      # the state is either not searched or better than previous state of vertex u
      vis.add(next) # the state is going to be visited
      cost[v] = c + co # update the better state
      dis[v] = d + w  # update the better state
      pa[next] = cur # record next state's parent
      pq.put(next)
  # no path
  return float('inf'), float('inf'), []

# Run task 3
print()
print('-------- Running Task 3 --------')
print()

begin_time = time()
iteration_cnt = 0
dis, cost, path = astar(G, Dist, Cost, st, en)
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
