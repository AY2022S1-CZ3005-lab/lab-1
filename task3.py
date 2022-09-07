from math import sqrt
import json
from queue import PriorityQueue

with open('G.json') as f:
  G = json.load(f)

with open('Coord.json') as f:
  Coord = json.load(f)

with open('Dist.json') as f:
  Dist = json.load(f)

with open('Cost.json') as f:
  Cost = json.load(f)

n = len(G)
m = len(Dist)
st = 1
en = 50
budget = 287932

h = [0] * (n + 1)

(en_x, en_y) = Coord[str(en)]

for i in range(1, n + 1):
  (x, y) = Coord[str(i)]
  h[i] = sqrt((x - en_x) ** 2 + (y - en_y) ** 2)

class state:
  def __init__(self, d, co, u):
    self.d = d
    self.co = co
    self.u = u

  def __lt__(self, other):
    return self.d + h[self.u] < other.d + h[other.u]

  def __hash__(self):
      return hash((self.d, self.co, self.u))

  def __eq__(self, other):
      return (self.d, self.co, self.u) == (other.d, other.co, other.u)


# guarantee the cost is minimum
def astar(st, en):
  pa = dict()
  vis = set()
  pq = PriorityQueue()
  cost = [float('inf')] * (n + 1)
  dis = [float('inf')] * (n + 1)
  pq.put(state(0, 0, st))
  pa[state(0, 0, st)] = None
  while not pq.empty():
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
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      next = state(d + w, co + c, v)
      if next in vis or (c + co >= cost[v] and d + w >= dis[v]) or c + co >= budget:
        continue
      cost[v] = c + co
      dis[v] = d + w
      pa[next] = cur
      pq.put(next)
  # no path
  return float('inf'), float('inf'), []


dis, cost, path = astar(st, en)

print('Shortest path: ', end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print('Shortest distance: ', dis)
print('Total energy cost: ', cost)