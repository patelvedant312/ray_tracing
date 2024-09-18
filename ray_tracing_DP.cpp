#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <queue>

using namespace std;

const double INF = numeric_limits<double>::infinity();
const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};  // 8 possible directions (up, down, left, right, diagonals)
const int dy[] = {0, 0, 1, -1, 1, 1, -1, -1};  // 8 possible directions

struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
};

// Structure for the room with obstacles and goal
struct Room {
    vector<vector<int>> grid;  // 0 = free space, 1 = obstacle
    Point start, goal;
    int rows, cols;

    Room(int r, int c, Point s, Point g) : rows(r), cols(c), start(s), goal(g) {
        grid = vector<vector<int>>(r, vector<int>(c, 0));  // Initialize with empty space
    }

    bool isValid(int x, int y) const {
        return x >= 0 && x < rows && y >= 0 && y < cols && grid[x][y] == 0;  // Valid if within bounds and not an obstacle
    }
};

// Function to calculate cost between two points (basic cost, can be extended)
double calculateCost(const Point& p1, const Point& p2, double distanceWeight, double obstacleWeight, const Room& room) {
    double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));  // Euclidean distance
    double cost = distance * distanceWeight;  // Add distance cost

    // Add additional cost if moving close to obstacles
    for (int i = max(0, p2.x - 1); i <= min(room.rows - 1, p2.x + 1); i++) {
        for (int j = max(0, p2.y - 1); j <= min(room.cols - 1, j + 1); j++) {
            if (room.grid[i][j] == 1) {  // Penalty for being near obstacles
                cost += obstacleWeight;
            }
        }
    }

    return cost;
}

// Dynamic Programming function to find the best path
double findBestPath(Room& room, double distanceWeight, double obstacleWeight) {
    vector<vector<double>> dp(room.rows, vector<double>(room.cols, INF));  // DP table to store minimum cost
    vector<vector<Point>> parent(room.rows, vector<Point>(room.cols, Point(-1, -1)));  // To store parent for path reconstruction

    dp[room.start.x][room.start.y] = 0;  // Start position has 0 cost

    // Min-heap priority queue to process cells based on cost
    priority_queue<pair<double, Point>, vector<pair<double, Point>>, greater<pair<double, Point>>> pq;
    pq.push({0, room.start});

    while (!pq.empty()) {
        auto [currentCost, currentPoint] = pq.top();
        pq.pop();

        if (currentPoint.x == room.goal.x && currentPoint.y == room.goal.y) {
            break;  // Reached goal
        }

        for (int i = 0; i < 8; i++) {  // Check 8 possible directions
            int newX = currentPoint.x + dx[i];
            int newY = currentPoint.y + dy[i];

            if (room.isValid(newX, newY)) {
                double newCost = currentCost + calculateCost(currentPoint, Point(newX, newY), distanceWeight, obstacleWeight, room);

                if (newCost < dp[newX][newY]) {  // Update if a better path is found
                    dp[newX][newY] = newCost;
                    pq.push({newCost, Point(newX, newY)});
                    parent[newX][newY] = currentPoint;  // Track parent for path reconstruction
                }
            }
        }
    }

    return dp[room.goal.x][room.goal.y];  // Return the cost to reach the goal
}

// Function to print the best path from the start to the goal
void printBestPath(const vector<vector<Point>>& parent, const Point& start, const Point& goal) {
    vector<Point> path;
    Point current = goal;
    while (current.x != -1 && current.y != -1) {
        path.push_back(current);
        current = parent[current.x][current.y];
    }
    reverse(path.begin(), path.end());

    cout << "Best Path:\n";
    for (const auto& p : path) {
        cout << "(" << p.x << ", " << p.y << ") ";
    }
    cout << endl;
}

int main() {
    // Define the room dimensions and start/goal points
    Room room(10, 10, Point(0, 0), Point(9, 9));

    // Add obstacles (represented by 1 in the grid)
    room.grid[3][3] = room.grid[4][3] = room.grid[5][3] = 1;  // Vertical obstacle
    room.grid[6][6] = room.grid[6][7] = room.grid[6][8] = 1;  // Horizontal obstacle

    // Parameters for the cost function
    double distanceWeight = 1.0;
    double obstacleWeight = 2.0;

    // Find the best path using dynamic programming
    double minCost = findBestPath(room, distanceWeight, obstacleWeight);
    cout << "Minimum cost to reach goal: " << minCost << endl;

    return 0;
}
