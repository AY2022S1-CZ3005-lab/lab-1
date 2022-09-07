import json
from operator import ne
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

# guarantee the cost is minimum
def UCS(st, en):
  pa = dict()
  vis = set()
  pq = PriorityQueue()
  cost = [float('inf')] * (n + 1)
  dis = [float('inf')] * (n + 1)
  pq.put((0, 0, st))
  pa[(0, 0, st)] = None
  while not pq.empty():
    cur = pq.get()
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
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      next = (d + w, co + c, v)
      if next in vis or (c + co >= cost[v] and d + w >= dis[v]) or c + co >= budget:
        continue
      cost[v] = c + co
      dis[v] = d + w
      pa[next] = cur
      pq.put(next)
  # no path
  return float('inf'), float('inf'), []


dis, cost, path = UCS(st, en)

print('Shortest path: ', end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print('Shortest distance: ', dis)
print('Total energy cost: ', cost)
