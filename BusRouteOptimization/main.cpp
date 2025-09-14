// g++ -std=c++17 BusRoutePlanner.cpp main.cpp -o bus_planner
//./bus_planner

#include "BusRoutePlanner.h"
#include <fstream>
#include <sstream>
#include <limits>
#include <iostream>

// trim helper
static inline std::string trim(const std::string& s) {
    const char* ws = " \t\r\n";
    size_t start = s.find_first_not_of(ws);
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(ws);
    return s.substr(start, end - start + 1);
}

void clearInputBuffer() {
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void loadRoutesFromCSV(BusRoutePlanner& planner, const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Error: Could not open file " << filename << std::endl;
        return;
    }

    std::string line;
    // try to read header first (if exists)
    if (!std::getline(file, line)) {
        std::cout << "CSV file is empty." << std::endl;
        return;
    }

    int lineCount = 1;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string source, destination, weightStr;
        if (!std::getline(ss, source, ',')) continue;
        if (!std::getline(ss, destination, ',')) continue;
        if (!std::getline(ss, weightStr)) continue;

        source = trim(source);
        destination = trim(destination);
        weightStr = trim(weightStr);

        if (source.empty() || destination.empty() || weightStr.empty()) continue;

        try {
            double weight = std::stod(weightStr);
            std::string busLine = "Line" + std::to_string(lineCount++);
            planner.addRoute(source, destination, weight, busLine, true);
        } catch (const std::exception& e) {
            // If parsing fails, skip the line (log it)
            std::cerr << "Warning: skipping invalid line (weight parse fail): " << line << std::endl;
            continue;
        }
    }

    file.close();
    std::cout << "Routes successfully loaded from " << filename << std::endl;
}

int main() {
    BusRoutePlanner planner;
    std::string filename;
    std::cout << "Enter CSV filename (e.g., realistic_bus_routes_bidirectional.csv): ";
    std::getline(std::cin, filename);

    loadRoutesFromCSV(planner, filename);

    int choice = -1;
    std::string s,e;

    do {
        std::cout << "\n1. Print Network\n2. Shortest Path (Dijkstra)\n3. Min Transfers (BFS)\n4. MST (Prim)\n0. Exit\nChoice: ";
        std::cin >> choice;
        clearInputBuffer();

        switch (choice) {
            case 1:
                planner.printGraph();
                break;
            case 2:
                std::cout << "Start: "; std::getline(std::cin, s);
                std::cout << "End: ";   std::getline(std::cin, e);
                planner.findShortestPathDijkstra(s, e);
                break;
            case 3:
                std::cout << "Start: "; std::getline(std::cin, s);
                std::cout << "End: ";   std::getline(std::cin, e);
                planner.findMinTransfersBFS(s, e);
                break;
            case 4:
                planner.calculateMSTPrim();
                break;
            case 0:
                std::cout << "Goodbye.\n";
                break;
            default:
                std::cout << "Invalid choice.\n";
                break;
        }
    } while (choice != 0);

    return 0;
}
