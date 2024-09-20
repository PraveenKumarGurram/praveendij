#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cctype> // For toupper()

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    int destination, cost; // 'destination' is the target node, 'cost' is the weight of the edge
};

// Graph represented as an adjacency list
vector<vector<Edge>> network;

// Dijkstra's algorithm to find the shortest path
pair<int, vector<int>> findShortestPath(int startNode, int endNode) {
    int totalNodes = network.size(); // Total number of nodes in the graph
    vector<int> distances(totalNodes, numeric_limits<int>::max()); // Distance to each node, initialized to "infinity"
    vector<int> previousNode(totalNodes, -1); // To store the previous node in the optimal path
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> priorityQueue; // Min-heap priority queue

    // Initialize the start node
    distances[startNode] = 0;
    priorityQueue.push({0, startNode});

    while (!priorityQueue.empty()) {
        int currentDistance = priorityQueue.top().first;
        int currentNode = priorityQueue.top().second;
        priorityQueue.pop();

        // If the current node is the target node, exit the loop
        if (currentNode == endNode) break;

        // Skip if a shorter path to currentNode has already been found
        if (currentDistance > distances[currentNode]) continue;

        // Visit each neighbor of the current node
        for (const Edge& edge : network[currentNode]) {
            int neighbor = edge.destination;
            int edgeCost = edge.cost;

            int newDistance = currentDistance + edgeCost;
            if (newDistance < distances[neighbor]) {
                distances[neighbor] = newDistance;
                previousNode[neighbor] = currentNode; // Track the path
                priorityQueue.push({newDistance, neighbor});
            }
        }
    }

    // Reconstruct the shortest path
    vector<int> path;
    for (int node = endNode; node != -1; node = previousNode[node]) {
        path.push_back(node);
    }
    reverse(path.begin(), path.end());

    return {distances[endNode], path};
}

// Helper function to convert node names (characters) to indices
int getNodeIndex(char node) {
    return toupper(node) - 'A'; // Convert character to uppercase and map to an index
}

int main() {
    // Initialize the graph with 5 nodes
    network.resize(5);

    // Add edges to the graph
    // A -> B (10), A -> E (3)
    network[getNodeIndex('A')].push_back({getNodeIndex('B'), 10});
    network[getNodeIndex('A')].push_back({getNodeIndex('E'), 3});
    
    // B -> C (2), B -> E (4)
    network[getNodeIndex('B')].push_back({getNodeIndex('C'), 2});
    network[getNodeIndex('B')].push_back({getNodeIndex('E'), 4});
    
    // C -> D (9), C -> E (8)
    network[getNodeIndex('C')].push_back({getNodeIndex('D'), 9});
    network[getNodeIndex('C')].push_back({getNodeIndex('E'), 8});
    
    // D -> C (7), D -> E (2)
    network[getNodeIndex('D')].push_back({getNodeIndex('C'), 7});
    network[getNodeIndex('D')].push_back({getNodeIndex('E'), 2});
    
    // E -> B (1), E -> C (8)
    network[getNodeIndex('E')].push_back({getNodeIndex('B'), 1});
    network[getNodeIndex('E')].push_back({getNodeIndex('C'), 8});

    // Get user input for the start and end nodes
    char startNodeChar, endNodeChar;
    cout << "Please enter the starting node (A-E): ";
    cin >> startNodeChar;
    cout << "Please enter the ending node (A-E): ";
    cin >> endNodeChar;

    // Convert to uppercase to handle case insensitivity
    startNodeChar = toupper(startNodeChar);
    endNodeChar = toupper(endNodeChar);

    // Get the indices of the start and end nodes
    int startNode = getNodeIndex(startNodeChar);
    int endNode = getNodeIndex(endNodeChar);

    // Run Dijkstra's algorithm to find the shortest path
    pair<int, vector<int>> result = findShortestPath(startNode, endNode);
    int minimumCost = result.first;
    vector<int> shortestPath = result.second;

    // Output the results
    if (minimumCost == numeric_limits<int>::max()) {
        cout << "No path exists from " << startNodeChar << " to " << endNodeChar << endl;
    } else {
        cout << "The shortest path cost from " << startNodeChar << " to " << endNodeChar << " is: " << minimumCost << endl;
        cout << "The path is: ";
        for (int node : shortestPath) {
            cout << char(node + 'A') << " ";
        }
        cout << endl;
    }

    return 0;
}
