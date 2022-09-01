import json

filenames = ['Coord', 'Cost', 'Dist', 'G']
filesz = {
  'Coord': 264346,
  'Cost': 730100,
  'G': 264346,
  'Dist': 730100
}
data = {}

for name in filenames:
  with open(name + '.json') as f:
    data[name] = json.load(f)

assert(len(data) == 4)

for key, value in data.items():
  print(key, set((type(a), type(b)) for a, b in value.items()))
  assert(len(value) == filesz[key])

print('Good')
