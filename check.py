from math import sqrt
import json
import os

print('-------- Checking input data --------')
print()

def indent():
  print("******** ", end = '')

filenames = ['Coord', 'Cost', 'Dist', 'G']
filesz = {
  'Coord': 264346,
  'Cost': 730100,
  'G': 264346,
  'Dist': 730100
}

data = {}

# data is complete
for name in filenames:
  with open(name + '.json') as f:
    data[name] = json.load(f)

assert(len(data) == 4)

for key, value in data.items():
  assert(len(value) == filesz[key])

Coord = data['Coord']
Cost = data['Cost']
G = data['G']
Dist = data['Dist']

indent()
print('Files are complete')

# no negative cost / edges
for key, value in Cost.items():
  assert(value >= 0)

for key, value in Dist.items():
  assert(value >= 0)

indent()
print('Dist is okay for dijkstra')
# edge(u, v) >= physical dis(u, v)
n = len(G)
eps = 1e-5

for key, value in Dist.items():
  pair = key.split(',')
  assert(len(pair) == 2)
  (u, v) = pair
  (x1, y1) = Coord[u]
  (x2, y2) = Coord[v]
  phy_dis = sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
  assert(phy_dis <= value)

indent()
print('Coord and Dist are legal')
if os.name == 'nt':
  print('>>>All good<<<')
else:
  print('πππAll goodπππ')
