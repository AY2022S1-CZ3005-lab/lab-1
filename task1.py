import json
from queue import PriorityQueue
from time import time

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
iteration_cnt = 0

def dijkstra(st, en):
  #init
  global iteration_cnt
  vis = [False] * (n + 1)
  pa = [0] * (n + 1)
  pq = PriorityQueue()

  pq.put((0, 0, st, 0))
  while not pq.empty():
    iteration_cnt += 1
    (d, co, u, fa) = pq.get()
    if u == en:  # UCS optimiaztion
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
      if not vis[v]:
        pq.put((d + w, co +c, v, u))
  # no path
  return float('inf'), float('inf'), []
  
# run task1
print()
print('-------- Running Task 1 --------')
print()

begin_time = time()
dis, cost, path = dijkstra(st, en)
time_used = time() - begin_time

print("Shortest path: ", end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print('Shortest distance: ', dis)
print('Total energy cost: ', cost)
print(f'Iteration round: {iteration_cnt}')
print(f'Time used: {time_used}')
print()