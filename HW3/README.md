Pathfinding demo (Dijkstra + A*)
Files:
  include/Graph.h
  src/Graph.cpp
  include/Pathfinder.h
  src/Pathfinder.cpp
  src/main.cpp
  Makefile

Build (GNU/Linux / macOS):
  make

Run:
  ./pathfinder_demo
This runs:
 - small sample graph (about 30 nodes): Dijkstra, A* (Euclidean), A* (inadmissible)
 - large random graph (default N=20000, k=4): Dijkstra and A* (Euclidean)

You can change large graph sizes (N,k) by passing arguments:
  ./pathfinder_demo 50000 4

Notes & Design decisions:
 - Graph stores optional (x,y) per node to enable spatial heuristics; edges are weighted and directed.
 - A* accepts any heuristic function of type std::function<double(int,int)>.
 - Dijkstra implemented as A* with zero heuristic.
 - Instrumentation: runtime (ms), number explored (closed), and maximum fringe size (peak open set size) are reported.
 - The large-graph generator uses a naive O(N^2) method to find k nearest neighbors. This is fine for N up to tens of thousands but will become slow for much larger N; swap-in a KD-tree or spatial hashing for speed when scaling to >100k nodes.
 - All weights are positive. For the sample graph they mirror Euclidean distances; for the random graph edge weight = Euclidean distance.
 - The code is modular and ready for SFML:
     * Graph includes coordinates; to visualize, simply include SFML and draw circles for nodes and lines for edges then step through the `res.path` to animate the agent.
     * Later path-following modules can consume `res.path` (sequence of node ids).
 - To add new heuristics: pass another lambda to `astar`. For clustering heuristics (when graph isn't geometric), compute cluster IDs and supply a heuristic based on cluster distances.

Caveats & Next steps (for later parts of HW):
 - Integrate with SFML for visualization and click-to-target quantization.
 - Implement path following and obstacle avoidance using results from HW2.
 - Add more heuristics, measure admissibility/consistency, and produce experimental tables/plots for the writeup.

