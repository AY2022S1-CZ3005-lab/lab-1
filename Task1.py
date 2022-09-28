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

def dijkstra(G, Dist, Cost, st, en):
  #init
  global iteration_cnt
  vis = [False] * (n + 1)
  pa = [0] * (n + 1)
  pq = PriorityQueue()

  pq.put((0, 0, st, 0))
  while not pq.empty():
    iteration_cnt += 1
    # current state
    # (dis, cost, vertex, parent in searching tree)
    (d, co, u, fa) = pq.get()
    if u == en:
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
    # only until when the optimal path is decided
    # we record we parent of u (different from using a dis array)
    pa[u] = fa
    for v_str in G[str(u)]:
      # for each edge(u, v) with lenght = w and cost = c
      v = int(v_str)
      w = Dist[str(u) + ',' + str(v)]
      c = Cost[str(u) + ',' + str(v)]
      if not vis[v]:
        pq.put((d + w, co + c, v, u))
  # no path
  return float('inf'), float('inf'), []
  
# run task1
print()
print('-------- Running Task 1 --------')
print()

begin_time = time()
iteration_cnt = 0
dis, cost, path = dijkstra(G, Dist, Cost, st, en)
time_used = time() - begin_time

print("Shortest path: ", end = '')
sep = ''
for v in path:
  print(sep + str(v), end = '')
  sep = '->'
print()
print(f'Shortest distance: {dis}')
print(f'Total energy cost: {cost}')
print(f'Iteration round: {iteration_cnt}')
print(f'Time used: {time_used}')
