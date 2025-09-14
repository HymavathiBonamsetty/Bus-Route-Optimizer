#include "BusRoutePlanner.h"
#include <queue>
#include <algorithm>
#include <set>
#include <limits>

// Constructor
BusRoutePlanner::BusRoutePlanner() : numStops(0) {}

// Helper to get or create id
int BusRoutePlanner::getStopId(const std::string& stopName) {
    auto it = stopNameToId.find(stopName);
    if (it == stopNameToId.end()) {
        stopNameToId[stopName] = numStops;
        stopIdToName.push_back(stopName);
        adjList.resize(numStops + 1); // ensure room for a new index
        numStops++;
        return numStops - 1;
    }
    return it->second;
}

void BusRoutePlanner::addStop(const std::string& stopName) {
    getStopId(stopName);
    std::cout << "Added stop: " << stopName << std::endl;
}

void BusRoutePlanner::addRoute(const std::string& stopA_Name, const std::string& stopB_Name,
                               double weight, const std::string& busLine, bool bidirectional) {
    int u = getStopId(stopA_Name);
    int v = getStopId(stopB_Name);

    // Ensure adjList size matches numStops
    if ((int)adjList.size() < numStops) adjList.resize(numStops);

    adjList[u].push_back(Edge(v, weight, busLine));
    if (bidirectional) {
        adjList[v].push_back(Edge(u, weight, busLine));
    }
    // Optional: silent add (comment out if too verbose)
    // std::cout << "Added route: " << stopA_Name << " --(" << weight << ", " << busLine << ")--> " << stopB_Name << std::endl;
}

void BusRoutePlanner::removeRoute(const std::string& stopA_Name, const std::string& stopB_Name,
                                  const std::string& busLine, bool bidirectional) {
    int u = -1, v = -1;
    if (stopNameToId.count(stopA_Name)) u = stopNameToId[stopA_Name];
    if (stopNameToId.count(stopB_Name)) v = stopNameToId[stopB_Name];
    if (u == -1 || v == -1) {
        std::cout << "Error: One or both stops not found for route removal." << std::endl;
        return;
    }
    auto& u_edges = adjList[u];
    for (size_t i = 0; i < u_edges.size(); ++i) {
        if (u_edges[i].destination == v && (busLine.empty() || u_edges[i].busLine == busLine)) {
            u_edges.erase(u_edges.begin() + i);
            break;
        }
    }
    if (bidirectional) {
        auto& v_edges = adjList[v];
        for (size_t i = 0; i < v_edges.size(); ++i) {
            if (v_edges[i].destination == u && (busLine.empty() || v_edges[i].busLine == busLine)) {
                v_edges.erase(v_edges.begin() + i);
                break;
            }
        }
    }
}

// ---------------- Dijkstra's ----------------
using DijkstraNode = std::pair<double, int>; // {dist, node}
void BusRoutePlanner::findShortestPathDijkstra(const std::string& startStopName, const std::string& endStopName) {
    if (stopNameToId.find(startStopName) == stopNameToId.end() ||
        stopNameToId.find(endStopName) == stopNameToId.end()) {
        std::cout << "Error: Start or end stop not found." << std::endl;
        return;
    }
    int startNode = stopNameToId[startStopName];
    int endNode = stopNameToId[endStopName];

    std::vector<double> dist(numStops, std::numeric_limits<double>::infinity());
    std::vector<int> parent(numStops, -1);
    std::vector<std::string> parentBusLine(numStops, "");

    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    dist[startNode] = 0;
    pq.push({0, startNode});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == endNode) break;
        for (const auto& edge : adjList[u]) {
            int v = edge.destination;
            double w = edge.weight;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                parent[v] = u;
                parentBusLine[v] = edge.busLine;
                pq.push({dist[v], v});
            }
        }
    }

    if (dist[endNode] == std::numeric_limits<double>::infinity()) {
        std::cout << "No path found from " << startStopName << " to " << endStopName << "." << std::endl;
        return;
    }

    std::vector<std::string> path;
    std::vector<std::string> busLinesUsed;
    int cur = endNode;
    while (cur != -1) {
        path.push_back(stopIdToName[cur]);
        if (parent[cur] != -1) busLinesUsed.push_back(parentBusLine[cur]);
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    std::reverse(busLinesUsed.begin(), busLinesUsed.end());

    std::cout << "\n--- Shortest Path (Dijkstra) from " << startStopName << " to " << endStopName << " ---\n";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i + 1 < path.size()) {
            std::cout << " --(" << (i < busLinesUsed.size() ? busLinesUsed[i] : "") << ", " << "weight)--> ";
        }
    }
    std::cout << "\nTotal Distance: " << dist[endNode] << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
}

// ---------------- BFS (min transfers) ----------------
void BusRoutePlanner::findMinTransfersBFS(const std::string& startStopName, const std::string& endStopName) {
    if (stopNameToId.find(startStopName) == stopNameToId.end() ||
        stopNameToId.find(endStopName) == stopNameToId.end()) {
        std::cout << "Error: Start or end stop not found." << std::endl;
        return;
    }
    int startNode = stopNameToId[startStopName];
    int endNode = stopNameToId[endStopName];

    std::vector<int> dist(numStops, -1);
    std::vector<int> parent(numStops, -1);
    std::vector<std::string> parentBusLine(numStops, "");

    std::queue<int> q;
    dist[startNode] = 0;
    q.push(startNode);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        if (u == endNode) break;
        for (const auto& edge : adjList[u]) {
            int v = edge.destination;
            if (dist[v] == -1) {
                dist[v] = dist[u] + 1;
                parent[v] = u;
                parentBusLine[v] = edge.busLine;
                q.push(v);
            }
        }
    }

    if (dist[endNode] == -1) {
        std::cout << "No path found from " << startStopName << " to " << endStopName << "." << std::endl;
        return;
    }

    std::vector<std::string> path;
    std::vector<std::string> busLinesUsed;
    int cur = endNode;
    while (cur != -1) {
        path.push_back(stopIdToName[cur]);
        if (parent[cur] != -1) busLinesUsed.push_back(parentBusLine[cur]);
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    std::reverse(busLinesUsed.begin(), busLinesUsed.end());

    std::cout << "\n--- Path with Minimum Transfers (BFS) from " << startStopName << " to " << endStopName << " ---\n";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i + 1 < path.size()) {
            std::cout << " --(" << (i < busLinesUsed.size() ? busLinesUsed[i] : "") << ")--> ";
        }
    }
    std::cout << "\nTotal Transfers: " << dist[endNode] << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
}

// ---------------- Prim's MST ----------------
using PrimNode = std::pair<double, int>;
void BusRoutePlanner::calculateMSTPrim() {
    if (numStops == 0) {
        std::cout << "No stops available to calculate MST." << std::endl;
        return;
    }
    std::vector<double> key(numStops, std::numeric_limits<double>::infinity());
    std::vector<int> parent(numStops, -1);
    std::vector<bool> inMST(numStops, false);

    std::priority_queue<PrimNode, std::vector<PrimNode>, std::greater<PrimNode>> pq;
    key[0] = 0;
    pq.push({0, 0});

    double totalMSTWeight = 0;
    int edgesInMST = 0;

    while (!pq.empty() && edgesInMST < numStops - 1) {
        int u = pq.top().second; pq.pop();
        if (inMST[u]) continue;
        inMST[u] = true;
        if (parent[u] != -1) {
            totalMSTWeight += key[u];
            edgesInMST++;
            std::cout << "Added edge: " << stopIdToName[parent[u]] << " - " << stopIdToName[u]
                      << " (Weight: " << key[u] << ")\n";
        }
        for (const auto& edge : adjList[u]) {
            int v = edge.destination;
            double w = edge.weight;
            if (!inMST[v] && w < key[v]) {
                key[v] = w;
                parent[v] = u;
                pq.push({key[v], v});
            }
        }
    }

    if (edgesInMST == numStops - 1 || numStops == 1) {
        std::cout << "Total MST Weight: " << totalMSTWeight << std::endl;
    } else {
        std::cout << "Warning: Graph might not be fully connected. Partial MST weight: " << totalMSTWeight << std::endl;
    }
    std::cout << "---------------------------------------------------------" << std::endl;
}

void BusRoutePlanner::printGraph() const {
    std::cout << "\n--- Current Bus Network ---\n";
    for (int i = 0; i < numStops; ++i) {
        std::cout << stopIdToName[i] << " (ID: " << i << "): ";
        for (const auto& e : adjList[i]) {
            std::cout << "-> " << stopIdToName[e.destination] << " (W:" << e.weight << ", L:" << e.busLine << ") ";
        }
        std::cout << "\n";
    }
    std::cout << "---------------------------\n";
}
