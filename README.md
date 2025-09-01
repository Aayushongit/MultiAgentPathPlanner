	
Problem:

![Screenshot from 2021-03-31 21-26-54](https://user-images.githubusercontent.com/42487965/113177563-4a431b80-926b-11eb-971a-dc69f61aa3b5.png)


	

Robot sources are:
	
	source	Destination
	(0,0)	(0,8)
	(0,0)	(5,5)
	(5,9)	(5,5)

Tasks are:
	
	Task	Pickup	Delivery
	1	(5,0)	(3,12)
	2	(4,6)	(0,5)
	3	(5,0)	(0,5)	

Blockage:
	
	(2,1), (5,3) (4,10) (0,13) (2,15)

 
For Optimal Distance between two points Iterretive Deepening is used for storage requirement.
The schedule is made using hamiltonian cycles.



## Available Algorithms

### Single Agent Pathfinding
- **A* Algorithm**: Optimal pathfinding with heuristic search
- **Dijkstra Algorithm**: Guaranteed shortest path algorithm
- **D* Algorithm**: Dynamic pathfinding for changing environments
- **Jump Point Search (JPS)**: Optimized A* for grid-based pathfinding
- **Bidirectional BFS**: Breadth-first search from both start and goal
- **RRT Algorithm**: Rapidly-exploring Random Tree for complex spaces

### Multi-Agent Pathfinding
- **CBS Algorithm**: Conflict-Based Search for multi-agent coordination

## Usage

### Original Multi-Agent Task Solver
```bash
python search_heuristic.py task
```

### Individual Algorithm Usage
```bash
# A* Algorithm
python astar_algorithm.py

# Dijkstra Algorithm  
python dijkstra_algorithm.py

# D* Algorithm
python dstar_algorithm.py

# Jump Point Search
python jps_algorithm.py

# Bidirectional BFS
python bidirectional_bfs.py

# RRT Algorithm
python rrt_algorithm.py

# Conflict-Based Search (Multi-Agent)
python cbs_algorithm.py
```
