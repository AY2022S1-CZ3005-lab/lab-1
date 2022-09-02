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

class estimate:
  def __init__(self, d, co, u):
    self.d = d
    self.co = co
    self.u = u

  def __lt__(self, other):
    return self.d + h[self.u] < other.d + h[self.u]


def astar(st, en):
  vis = [False] * (n + 1)
  dis = [float('inf')] * (n + 1)
  pa = [0] * (n + 1)
  cost = float('inf')

  pq = PriorityQueue()
  dis[st] = 0
  pq.put(estimate(0, 0, st))

  while not pq.empty():
    cur = pq.get()
    d = cur.d
    co = cur.co
    u = cur.u
    if u == en:
      cost = co
      break
    if vis[u]:
      continue
    vis[u] = True
    for v_str in G[str(u)]:
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      if dis[v] > dis[u] + w and co + c <= budget:
        dis[v] = dis[u] + w
        pa[v] = u
        pq.put(estimate(dis[v], co + c, v))
  # no path
  if dis[en] == float('inf'):
    return dis[en], cost, []
  p = en
  path = []
  while p != 0:
    path.append(p)
    p = pa[p]
  path.reverse()
  return dis[en], cost, path

dis, cost, path = astar(st, en)

print('Shortest path: ', end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print('Shortest distance: ', dis)
print('Total energy cost: ', cost)