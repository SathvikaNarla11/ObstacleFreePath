#include <opencv2/opencv.hpp>
#include <vector>
#include <set>
#include <stack>
#include <iostream>
#include <algorithm>
#include <random>
#include <cmath>

// Node structure for RRT* tree
struct Node {
    cv::Point2f point;  
    int parent;        
    float cost;         
};

// Global variables
int gridSize = 5;                                       // Size of the grid (gridSize x gridSize)
const int canvasSize = 500;                             // Size of the drawing canvas in pixels
int cellSize;                                           // Size of one cell in pixels (computed from gridSize)
cv::Point start(-1, -1), goal(-1, -1);                  // Start and goal positions in grid coordinates
std::set<std::pair<int, int>> obstacles;                // Set of obstacle cell coordinates
std::stack<std::pair<int, int>> undoStack, redoStack;   // Undo/redo stacks for obstacle placement
cv::Mat gridImg;                                        // Image for grid display
bool selectingStart = true, configured = false;         // GUI interaction flags

// Clamp point within canvas bounds
cv::Point2f clampToGrid(const cv::Point2f& pt) {
    float x = std::clamp(pt.x, 0.0f, (float)(canvasSize - 1));
    float y = std::clamp(pt.y, 0.0f, (float)(canvasSize - 1));
    return cv::Point2f(x, y);
}

// Draws the grid with obstacles, start and goal
void drawGrid() {
    gridImg = cv::Mat(canvasSize, canvasSize, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw grid cells
    for (int r = 0; r < gridSize; ++r) {
        for (int c = 0; c < gridSize; ++c) {
            cv::Rect cell(c * cellSize, r * cellSize, cellSize, cellSize);
            cv::rectangle(gridImg, cell, cv::Scalar(200, 200, 200), 1);
        }
    }

    // Draw obstacles as filled black squares
    for (auto& obs : obstacles)
        cv::rectangle(gridImg, cv::Rect(obs.second * cellSize, obs.first * cellSize, cellSize, cellSize), cv::Scalar(0, 0, 0), cv::FILLED);

    // Draw start and goal points
    if (start.x != -1)
        cv::circle(gridImg, cv::Point(start.x * cellSize + cellSize / 2, start.y * cellSize + cellSize / 2), 6, cv::Scalar(0, 255, 0), -1);
    if (goal.x != -1)
        cv::circle(gridImg, cv::Point(goal.x * cellSize + cellSize / 2, goal.y * cellSize + cellSize / 2), 6, cv::Scalar(0, 0, 255), -1);

    cv::imshow("Grid Setup", gridImg);
}

// Handles mouse interaction for placing obstacles, setting start and goal
void mouseCallback(int event, int x, int y, int, void*) {
    int col = x / cellSize;
    int row = y / cellSize;
    if (col >= gridSize || row >= gridSize) return;

    auto cell = std::make_pair(row, col);
    if (event == cv::EVENT_LBUTTONDOWN) {
        // Left-click toggles obstacle
        if (start == cv::Point(col, row) || goal == cv::Point(col, row)) return;
        if (obstacles.count(cell)) {
            obstacles.erase(cell);
        } else {
            obstacles.insert(cell);
        }
        undoStack.push(cell);
        while (!redoStack.empty()) redoStack.pop();
    } else if (event == cv::EVENT_RBUTTONDOWN) {
        // Right-click sets start or goal
        if (selectingStart) {
            start = cv::Point(col, row);
            selectingStart = false;
        } else {
            goal = cv::Point(col, row);
        }
    }
    drawGrid();
}

// Checks if a point is inside the grid boundaries
bool isInsideGrid(const cv::Point2f& pt) {
    int r = pt.y / cellSize, c = pt.x / cellSize;
    return (r >= 0 && r < gridSize && c >= 0 && c < gridSize);
}

// Checks if a point lies in an obstacle
bool isObstacle(const cv::Point2f& pt) {
    if (!isInsideGrid(pt)) return true;
    int r = pt.y / cellSize, c = pt.x / cellSize;
    return obstacles.count({r, c});
}

// Checks if the path between two points is collision-free
bool collisionFree(const cv::Point2f& a, const cv::Point2f& b) {
    for (int i = 1; i <= 10; ++i) {
        cv::Point2f pt = a + (b - a) * (i / 10.0f);
        if (!isInsideGrid(pt) || isObstacle(pt)) return false;
    }
    return true;
}

// Euclidean distance between two points
float dist(const cv::Point2f& a, const cv::Point2f& b) {
    return cv::norm(a - b);
}

// Smooth the found path using collision checks
std::vector<cv::Point2f> smoothPath(const std::vector<Node>& tree, int goalIdx) {
    std::vector<cv::Point2f> path;
    for (int cur = goalIdx; cur != -1; cur = tree[cur].parent)
        path.push_back(tree[cur].point);
    std::reverse(path.begin(), path.end());

    std::vector<cv::Point2f> smoothed = { path.front() };
    for (int i = 0, j; i < path.size() - 1; i = j) {
        for (j = path.size() - 1; j > i; --j)
            if (collisionFree(path[i], path[j])) break;
        smoothed.push_back(path[j]);
    }
    return smoothed;
}

int main() {
    std::cout << "Enter grid size: ";
    std::cin >> gridSize;
    cellSize = canvasSize / gridSize;

    cv::namedWindow("Grid Setup");
    cv::setMouseCallback("Grid Setup", mouseCallback);
    drawGrid();

    std::cout << "Left-click to toggle obstacles.\nRight-click to set start(green) and goal(red)\n";
    std::cout << "Press 's' to start RRT*.\nPress 'u' to undo and 'r' to redo.";

    // Wait for user to set up grid
    while (!configured) {
        int key = cv::waitKey(10);
        if (key == 'u' && !undoStack.empty()) {
            // Undo last obstacle toggle
            auto cell = undoStack.top(); undoStack.pop();
            if (obstacles.count(cell)) obstacles.erase(cell);
            else obstacles.insert(cell);
            redoStack.push(cell);
            drawGrid();
        } else if (key == 'r' && !redoStack.empty()) {
            // Redo last undo
            auto cell = redoStack.top(); redoStack.pop();
            if (obstacles.count(cell)) obstacles.erase(cell);
            else obstacles.insert(cell);
            undoStack.push(cell);
            drawGrid();
        } else if (key == 's' && start.x != -1 && goal.x != -1) {
            // Start RRT* when setup is complete
            configured = true;
        }
    }

    cv::destroyWindow("Grid Setup");
    cv::Mat img = gridImg.clone();

    // Convert start and goal to pixel positions
    cv::Point2f startPt(start.x * cellSize + cellSize / 2, start.y * cellSize + cellSize / 2);
    cv::Point2f goalPt(goal.x * cellSize + cellSize / 2, goal.y * cellSize + cellSize / 2);

    // RRT* tree initialization
    std::vector<Node> tree = {{startPt, -1, 0}};
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dis(0, canvasSize);
    const int MAX_ITER = 10000;
    int goalIdx = -1;

    // Main RRT* loop
    for (int i = 0; i < MAX_ITER; ++i) {
        // Sample a random point (goal-biased every 5th iteration)
        cv::Point2f randPt = (i % 5 == 0) ? goalPt : clampToGrid(cv::Point2f(dis(rng), dis(rng)));
        if (!isInsideGrid(randPt) || isObstacle(randPt)) continue;

        // Find nearest tree node to sampled point
        int nearest = -1;
        float bestDist = 1e9;
        for (int j = 0; j < tree.size(); ++j) {
            float d = dist(tree[j].point, randPt);
            if (d < bestDist) bestDist = d, nearest = j;
        }

        // Move in the direction of the random point with a step limit
        float stepSize = std::min(50.0f, bestDist);
        cv::Point2f dir = randPt - tree[nearest].point;
        if (cv::norm(dir) == 0) continue;
        dir *= stepSize / cv::norm(dir);
        cv::Point2f newPt = clampToGrid(tree[nearest].point + dir);

        if (!isInsideGrid(newPt) || !collisionFree(tree[nearest].point, newPt)) continue;

        // Choose best parent based on cost within neighborhood radius
        int bestParent = nearest;
        float bestCost = tree[nearest].cost + dist(tree[nearest].point, newPt);
        float radius = 50.0f * std::sqrt(std::log(tree.size() + 1) / (tree.size() + 1));

        for (int j = 0; j < tree.size(); ++j) {
            if (dist(tree[j].point, newPt) < radius && collisionFree(tree[j].point, newPt)) {
                float cost = tree[j].cost + dist(tree[j].point, newPt);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestParent = j;
                }
            }
        }

        // Add new node to the tree
        int newIdx = tree.size();
        tree.push_back({newPt, bestParent, bestCost});
        cv::line(img, tree[bestParent].point, newPt, cv::Scalar(0, 200, 255), 1);

        // Rewire nearby nodes if new path is better
        for (int j = 0; j < tree.size(); ++j) {
            if (j == newIdx) continue;
            if (dist(tree[j].point, newPt) < radius && collisionFree(newPt, tree[j].point)) {
                float newCost = bestCost + dist(newPt, tree[j].point);
                if (newCost < tree[j].cost) {
                    tree[j].parent = newIdx;
                    tree[j].cost = newCost;
                }
            }
        }

        // Check if goal is reached
        if (dist(newPt, goalPt) < cellSize * 0.6f) {
            goalIdx = newIdx;
            break;
        }

        cv::imshow("RRT*", img);
        cv::waitKey(1);
    }

    // Draw smoothed path if found
    if (goalIdx != -1) {
        auto smoothed = smoothPath(tree, goalIdx);
        for (size_t i = 1; i < smoothed.size(); ++i)
            cv::line(img, smoothed[i - 1], smoothed[i], cv::Scalar(255, 0, 0), 2);
    } else {
        std::cout << "No path found.\n";
    }

    cv::imshow("RRT*", img);
    cv::waitKey(0);
    return 0;
}