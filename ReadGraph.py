import json

def readGraph():
  with open('G.json') as f:
    G = json.load(f)

  with open('Coord.json') as f:
    Coord = json.load(f)

  with open('Dist.json') as f:
    Dist = json.load(f)

  with open('Cost.json') as f:
    Cost = json.load(f)
    
  return G, Coord, Dist, Cost
