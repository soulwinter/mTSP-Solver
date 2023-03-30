# Java-based Simple mTSP Solver
This is a Java-based simple mTSP (multiple Traveling Salesman Problem) solver that utilizes a greedy algorithm. It is intended for generating flight paths for multiple drones to traverse all given points. The user can specify the number of drones and their maximum range.

## Files
The main functionality of the program is implemented in `MTSPWithLimits.java`. To visualize the algorithm's output, you can use `MTSPVisualizer.java`. The program can be tested using `main.java`.

## How to Use
To use the program, simply download the repository and import it into your Java environment. Then, you can run the sample program provided in `main.java` to see the solver in action.

You can adjust the parameters, such as the number of drones and their maximum range, to customize the output.

## Output
The solver outputs the path(maybe not the shortest) for each drone to visit all given points, with the total distance traveled by all drones.

The visualization provided by `MTSPVisualizer.java` can be used to visualize the output of the algorithm.

