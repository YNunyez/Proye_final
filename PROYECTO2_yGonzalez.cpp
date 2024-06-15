#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <algorithm>
#include <unordered_set>

using namespace std;
typedef pair<int, int> iPair;

struct Edge {
    int to;
    int weight;
    int time;
};

struct Path {
    vector<int> nodes;
    int cost; // Puede ser peso o tiempo dependiendo del criterio
    int totalWeight; // Peso total
    int totalTime; // Tiempo total

    bool operator<(const Path& other) const {
        return cost > other.cost; // Orden inverso para la cola de prioridad
    }
};

vector<int> dijkstra(const vector<vector<Edge>>& graph, int src, int dest, vector<int>& dist, bool useWeight) {
    int V = graph.size();
    dist.assign(V, INT_MAX);
    vector<int> pred(V, -1);
    dist[src] = 0;

    priority_queue<iPair, vector<iPair>, greater<iPair>> pq;
    pq.push(make_pair(0, src));

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const auto& neighbor : graph[u]) {
            int v = neighbor.to;
            int parameter = useWeight ? neighbor.weight : neighbor.time;

            if (dist[u] + parameter < dist[v]) {
                dist[v] = dist[u] + parameter;
                pq.push(make_pair(dist[v], v));
                pred[v] = u;
            }
        }
    }

    vector<int> path;
    for (int v = dest; v != -1; v = pred[v])
        path.push_back(v);
    reverse(path.begin(), path.end());

    if (path.front() != src)
        path.clear(); // No hay camino

    return path;
}

vector<Path> yenKShortestPaths(const vector<vector<Edge>>& graph, int src, int dest, int K, bool useWeight) {
    vector<Path> shortestPaths;
    vector<int> dist;
    vector<int> firstPath = dijkstra(graph, src, dest, dist, useWeight);
    if (firstPath.empty())
        return shortestPaths; // No hay camino

    int firstCost = dist[dest];
    int firstTotalWeight = 0;
    int firstTotalTime = 0;
    for (int j = 0; j < firstPath.size() - 1; ++j) {
        for (const auto& edge : graph[firstPath[j]]) {
            if (edge.to == firstPath[j + 1]) {
                firstTotalWeight += edge.weight;
                firstTotalTime += edge.time;
                break;
            }
        }
    }
    shortestPaths.push_back({firstPath, firstCost, firstTotalWeight, firstTotalTime});
    priority_queue<Path> candidates;

    for (int k = 1; k < K; ++k) {
        const Path& lastPath = shortestPaths[k - 1];
        for (int i = 0; i < lastPath.nodes.size() - 1; ++i) {
            int spurNode = lastPath.nodes[i];
            vector<int> rootPath(lastPath.nodes.begin(), lastPath.nodes.begin() + i + 1);

            vector<vector<Edge>> tempGraph = graph;
            unordered_set<int> removedNodes(rootPath.begin(), rootPath.end() - 1);

            for (int u : rootPath) {
                for (int v : removedNodes) {
                    tempGraph[u].erase(
                        remove_if(tempGraph[u].begin(), tempGraph[u].end(),
                                  [v](const Edge& edge) { return edge.to == v; }),
                        tempGraph[u].end());
                }
            }

            for (const auto& path : shortestPaths) {
                if (path.nodes.size() > i && equal(path.nodes.begin(), path.nodes.begin() + i + 1, rootPath.begin())) {
                    int nextNode = path.nodes[i + 1];
                    tempGraph[spurNode].erase(
                        remove_if(tempGraph[spurNode].begin(), tempGraph[spurNode].end(),
                                  [nextNode](const Edge& edge) { return edge.to == nextNode; }),
                        tempGraph[spurNode].end());
                }
            }

            vector<int> spurPath = dijkstra(tempGraph, spurNode, dest, dist, useWeight);

            if (!spurPath.empty()) {
                vector<int> totalPath = rootPath;
                totalPath.insert(totalPath.end(), spurPath.begin() + 1, spurPath.end());

                int totalWeight = 0;
                int totalTime = 0;
                for (int j = 0; j < totalPath.size() - 1; ++j) {
                    for (const auto& edge : graph[totalPath[j]]) {
                        if (edge.to == totalPath[j + 1]) {
                            totalWeight += edge.weight;
                            totalTime += edge.time;
                            break;
                        }
                    }
                }

                int totalCost = useWeight ? totalWeight : totalTime;
                candidates.push({totalPath, totalCost, totalWeight, totalTime});
            }
        }

        if (candidates.empty())
            break;

        shortestPaths.push_back(candidates.top());
        candidates.pop();
    }

    return shortestPaths;
}

int main() {
    int V = 15;
    vector<vector<Edge>> graph(V);

    graph[0].push_back({3, 50, 26});
    graph[0].push_back({10, 60, 15});
    graph[1].push_back({2, 100, 10});
    graph[1].push_back({6, 60, 15});
    graph[1].push_back({11, 70, 12});
    graph[1].push_back({13, 120, 10});
    graph[2].push_back({4, 70, 15});
    graph[2].push_back({6, 50, 26});
    graph[2].push_back({7, 70, 17});
    graph[2].push_back({9, 50,23});
    graph[2].push_back({10,90, 11});
    graph[3].push_back({10, 60, 18});
    graph[3].push_back({13, 50, 26});
    graph[4].push_back({7, 60, 20});
    graph[4].push_back({9, 40, 27});
    graph[5].push_back({6, 85, 15});
    graph[5].push_back({12, 60, 10});
    graph[5].push_back({14, 60, 14});
    graph[6].push_back({6, 10, 30});
    graph[6].push_back({9, 50, 20});
    graph[7].push_back({7, 10, 27});
    graph[7].push_back({10, 80, 20});
    graph[8].push_back({11, 60, 23});
    graph[8].push_back({12, 50, 24});
    graph[9].push_back({14, 60, 20});
	graph[10].push_back({13, 80, 20});
	graph[11].push_back({12, 50, 25});
	
/*
	graph[0].push_back({3, 50, 10});
    graph[0].push_back({10, 60, 15});
    graph[1].push_back({2, 100, 35});
    graph[1].push_back({6, 60, 15});
    graph[1].push_back({11, 70, 15});
    graph[1].push_back({13, 120, 45});
    graph[2].push_back({4, 70, 15});
    graph[2].push_back({6, 50, 10});
    graph[2].push_back({7, 70, 20});
    graph[2].push_back({9, 50,15});
    graph[2].push_back({10,90, 35});
    graph[3].push_back({10, 60, 20});
    graph[3].push_back({13, 50, 10});
    graph[4].push_back({7, 60, 20});
    graph[4].push_back({9, 40, 15});
    graph[5].push_back({6, 85, 35});
    graph[5].push_back({12, 60, 40});
    graph[5].push_back({14, 60, 20});
    graph[6].push_back({6, 10, 5});
    graph[6].push_back({9, 50, 20});
    graph[7].push_back({7, 10, 10});
    graph[7].push_back({10, 80, 40});
    graph[8].push_back({11, 60, 30});
    graph[8].push_back({12, 50, 35});
    graph[9].push_back({14, 60, 20});
	graph[10].push_back({13, 80, 30});
	graph[11].push_back({12, 50, 35});*/

    int src = 1;
    int dest = 14;
    int K = 5;
    bool useWeight = false; // Cambiar a false para usar tiempo en lugar de peso

    vector<Path> kShortestPaths = yenKShortestPaths(graph, src, dest, K, useWeight);

    cout << "Las mejores " << K << " rutas del nodo " << src << " al nodo " << dest << " son:\n";
    for (int i = 0; i < kShortestPaths.size(); ++i) {
        cout << "Ruta " << i + 1 << " (coste mana " << kShortestPaths[i].totalWeight << ", tiempo " << kShortestPaths[i].totalTime << "): ";
        for (int j = 0; j < kShortestPaths[i].nodes.size(); ++j) {
            if (j != 0) cout << " -> ";
            cout << kShortestPaths[i].nodes[j];
        }
        cout << "\n";
    }

    return 0;
}

