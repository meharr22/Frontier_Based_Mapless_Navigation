# WRITEUP

## Approach

The robot operates in an initially unknown environment and incrementally constructs an internal occupancy map using LiDAR observations.At every step,the latest scan is converted into free and occupied cells,allowing the planner to continuously update its knowledge of the environment without any prior map information.

Navigation follows a frontier-based exploration strategy.A frontier is defined as a discovered free cell that is adjacent to at least one unexplored cell.Rather than evaluating individual frontier cells independently,neighbouring frontiers are grouped into connected frontier clusters using Breadth-First Search (BFS).Treating frontiers as clusters reduces frequent target switching and encourages the robot to explore larger unexplored regions in a more systematic manner.

Each frontier cluster is evaluated using a lightweight utility function that considers three factors:cluster size,travel cost and distance to the goal.Larger clusters are prioritized because they are more likely to reveal additional unexplored space,while shorter paths and smaller goal distances improve navigation efficiency.The representative frontier for each cluster is chosen as the closest frontier cell to the robot and BFS is used to compute the shortest feasible path through the currently discovered free space.

The planner always checks whether the goal has become reachable within the explored map.If a valid path exists,the robot immediately switches to goal-directed navigation using BFS.Otherwise,it continues exploring frontier clusters until a route to the goal is discovered.

To remain responsive as the map evolves,the planner periodically replans after a fixed number of movements and also replans whenever the current exploration target becomes invalid.This allows the navigation policy to adapt to newly discovered information without performing unnecessary planning after every action.


## Balancing Exploration and Goal Completion

The primary design challenge was balancing exploration with efficient goal completion.Aggressively pursuing the goal resulted in lower map coverage,whereas excessive exploration produced longer paths and reduced path efficiency.

The implemented strategy separates these objectives into two phases.While the goal is unreachable,the robot prioritizes frontier cluster exploration to maximize map discovery.Once the goal becomes reachable through the explored environment,exploration stops and the planner computes the shortest available route directly to the goal.This approach maintains reliable goal completion while achieving strong environment coverage.
 
## Limitations and Future Work

Although the planner consistently reaches the goal and achieves high map coverage,frontier selection is still based on a heuristic scoring function.The current implementation evaluates clusters using size,travel cost and goal distance,but it does not explicitly estimate the expected information that future sensor observations may provide.

Given additional development time,I would replace the heuristic frontier selection with a more principled utility model based on expected information gain and implement Wavefront Frontier Detection (WFD) for more systematic frontier extraction.Additional improvements such as A* search for long-distance planning,persistent frontier tracking and look ahead planning across multiple frontier clusters could further improve both exploration efficiency and path quality while preserving reliable goal completion.

Overall,my focus was on designing a navigation strategy that is modular,computationally efficient,and robust in unknown environments.The final solution combines occupancy-grid mapping,frontier clustering,BFS path planning, and adaptive replanning into a simple yet effective autonomous navigation pipeline.