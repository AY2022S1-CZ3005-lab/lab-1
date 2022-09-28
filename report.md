[program output](https://github.com/AY2022S1-CZ3005-lab/lab-1/actions)

[markdown report](https://github.com/AY2022S1-CZ3005-lab/lab-1/blob/main/report.md)
# Pre-task

Before running the algorithm, `check.py` did a constraint checking on given input data:
- All edge lengths are non-negative.
- All edge costs are non-negative.
- The euclidean distance between two vertice is not greater than the lengths of the edge (if exists).

Those constrains are essential to the correctness of our algorithm and will be explained later.

## Output:

```
******** Files are complete
******** Dist is okay for dijkstra
******** Coord and Dist are legal
🎉🎉🎉All good🎉🎉🎉
```

# Task 1

## Output:

```
Shortest path: 1->1363->1358->1357->1356->1276->1273->1277->1269->1267->1268->1284->1283->1282->1255->1253->1260->1259->1249->1246->963->964->962->1002->952->1000->998->994->995->996->987->986->979->980->969->977->989->990->991->2369->2366->2340->2338->2339->2333->2334->2329->2029->2027->2019->2022->2000->1996->1997->1993->1992->1989->1984->2001->1900->1875->1874->1965->1963->1964->1923->1944->1945->1938->1937->1939->1935->1931->1934->1673->1675->1674->1837->1671->1828->1825->1817->1815->1634->1814->1813->1632->1631->1742->1741->1740->1739->1591->1689->1585->1584->1688->1579->1679->1677->104->5680->5418->5431->5425->5424->5422->5413->5412->5411->66->5392->5391->5388->5291->5278->5289->5290->5283->5284->5280->50
Shortest distance: 148648.63722140007
Total energy cost: 294853
Iteration round: 6439
Time used: 0.06976914405822754
```

## Approch explanation

In task 1, we implemented a dijkstra algorithm.
  - `st` begin vertex
  - `en` end vertex
  - `pq` PriorityQueue contains current seaching path and distance
  - `pa` parent vertex
  - `vis[x]`: `True` if shortestPath(st->x) is calculated



Initial state:

- `vis[st]` = True, `dis[st]` = 0
 
- `vis[other]` = False, `dis[other]` = infinity

For each iteration:

1. We take out the node `u` with shortest value `d` from `pq`, and set `vis[u]` = True
2. Expand the neighbor of `u`, which is `v`, and calculate the path st..>u->v: dis[u] + e(u, v), insert into `pq`

Stop condition:

- `pq` is empty: no solution
- `vis[en]` is true: dis[en] is the answer
  

Core code:
```python
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
```

## Complexity and proof (炒个证明啥的)

# Task 2
Output:
```
Shortest path: 1->1363->1358->1357->1356->1276->1273->1277->1269->1267->1268->1284->1283->1282->1255->1253->1260->1259->1249->1246->963->964->962->1002->952->1000->998->994->995->996->987->988->979->980->969->977->989->990->991->2465->2466->2384->2382->2385->2379->2380->2445->2444->2405->2406->2398->2395->2397->2142->2141->2125->2126->2082->2080->2071->1979->1975->1967->1966->1974->1973->1971->1970->1948->1937->1939->1935->1931->1934->1673->1675->1674->1837->1671->1828->1825->1817->1815->1634->1814->1813->1632->1631->1742->1741->1740->1739->1591->1689->1585->1584->1688->1579->1679->1677->104->5680->5418->5431->5425->5424->5422->5413->5412->5411->66->5392->5391->5388->5291->5278->5289->5290->5283->5284->5280->50
Shortest distance: 150335.55441905273
Total energy cost: 259087
Iteration round: 39041
Time used: 0.5206058025360107
```

## Approch explanation
In task 2, we implemented a UCS algorithm.
  - `state` tuple of (distance, cost, vertex)
  - `st` begin vertex
  - `en` end vertex
  - `pq` PriorityQueue
  - `vis` conains the states which are searched and expanded
  - `dis[x]`: The shortest distance of current node states.
  - `cost[x]`: The least cost of current node states.

Initial state: init = (0, 0, st)
  - vis[init] = True
  - dis[st] = 0
  - cost[st] = 0

For each iteration:
  - We take out the state `cur` with shortest value `d` from `pq`.
  - Expand all possible state from `cur`, expect:
    - State exceeding the budget since all cost are positive
    - State which is not better than searched states. i.e. with more cost but longer distance.
  - Update `vis`, `pa`, `dis` and `cost`.

Stop condition:

- `pq` is empty: no solution
- `vis[en]` is true: dis[en] is the answer
  
Core code:
```python
# guarantee the cost is minimum
def UCS(G, Dist, Cost, st, en):
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
      vis.add(next) # the state is going to be visited
      cost[v] = c + co # update the better state
      dis[v] = d + w  # update the better state
      pa[next] = cur # record next state's parent
      pq.put(next)
  # no path
  return float('inf'), float('inf'), []
```
## Complexity and proof (炒个证明啥的)

# Task3

## Output
```
Shortest path: 1->1363->1358->1357->1356->1276->1273->1277->1269->1267->1268->1284->1283->1282->1255->1253->1260->1259->1249->1246->963->964->962->1002->952->1000->998->994->995->996->987->988->979->980->969->977->989->990->991->2465->2466->2384->2382->2385->2379->2380->2445->2444->2405->2406->2398->2395->2397->2142->2141->2125->2126->2082->2080->2071->1979->1975->1967->1966->1974->1973->1971->1970->1948->1937->1939->1935->1931->1934->1673->1675->1674->1837->1671->1828->1825->1817->1815->1634->1814->1813->1632->1631->1742->1741->1740->1739->1591->1689->1585->1584->1688->1579->1679->1677->104->5680->5418->5431->5425->5424->5422->5413->5412->5411->66->5392->5391->5388->5291->5278->5289->5290->5283->5284->5280->50
Shortest distance: 150335.55441905273
Total energy cost: 259087
Iteration round: 3577
Time used: 0.09289240837097168
```

## Approch explanation

In task 3, we implemented a A* algorithm, which is adapted from UCS in task 2.

Firstly we precalculated `h[x]`, which is the euclidean distance between `x` and `en`.


Since the euclidean distance between any (`u`, `v`) is not greater than e(`u`, `v`), `h[x]` is suffcient as out heuristic function.

Instead of using a tuple as `state`, we define a `state` class with `__less__` operator overrided:
```python
  # for PriorityQueue(), use f(x) + h(x) as evaluation
  def __lt__(self, other):
    global h
    if self.d + h[self.u] == other.d + h[other.u]:
      return self.co < other.co
    return self.d + h[self.u] < other.d + h[other.u]

```

Core code:
```python
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
```

## Complexity and proof (炒个证明啥的)


# Conclusion 和 对比？