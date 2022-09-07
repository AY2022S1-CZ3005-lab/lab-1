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

# guarantee the cost is minimum
def dijkstra(st, en):
  vis = [False] * (n + 1)
  pa = [0] * (n + 1)

  pq = PriorityQueue()
  pq.put((0, 0, st, 0))

  while not pq.empty():
    (d, co, u, fa) = pq.get()
    if u == en: # UCS optimization
      p = fa
      path = [en]
      while p != 0:
        path.append(p)
        p = pa[p]
      path.reverse()
      return d, co, path
    if vis[u]:
      continue
    vis[u] = True
    pa[u] = fa
    for v_str in G[str(u)]:
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      if not vis[v] and co + c <= budget:
        pq.put((d + w, co + c, v, u))
  # no path
  return float('inf'), float('inf'), []


dis, cost, path = dijkstra(st, en)

print('Shortest path: ', end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print('Shortest distance: ', dis)
print('Total energy cost: ', cost)
