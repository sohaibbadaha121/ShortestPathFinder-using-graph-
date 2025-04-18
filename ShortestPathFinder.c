#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define MAX_CITIES 50
#define MAX_CITY_LENGTH 50
#define MAX_VERTICES 50
#define INFINITY 99999

// Graph srtuct
typedef struct graph {
    int vertices;   // Number of vertices
    int adjacencyMatrix[MAX_VERTICES][MAX_VERTICES]; // Adjacency matrix to represent the graph
    char cities[MAX_CITIES][MAX_CITY_LENGTH]; // Array to store cities
} Graph;

// Table struct used later in Dijkestra algorithm
typedef struct table {
    int dist;
    int path;
    bool visited;
} TABLE;

// Function to initialize a graph
void initializeGraph(Graph* graph, int vertices) {
    graph->vertices = vertices;  // Assigning number of vertices

    // Setting initial value for all vertices to be an infinitive value
    int i, j;
    for(i = 0; i < vertices; i++) {
        for(j = 0; j < vertices; j++) {
            graph->adjacencyMatrix[i][j] = INFINITY;
        }
    }

    // Setting initial value for all cites to be NULL
    for (int k = 0; k < MAX_CITIES; k++) {
        graph->cities[k][0] = '\0';
    }
}

// Function to find the index of a city in the array of cities
int findCityIndex(char cities[MAX_CITIES][MAX_CITY_LENGTH], char city[]) {
    for (int i = 0; i < MAX_CITIES; i++) {
        if (strcmp(cities[i], city) == 0)
            return i;
    }
    return -1; // City not found
}

// Function to add an edge between two cities with a given weight
void addEdge(Graph* graph, char source[] , char destination[], int weight) {
    // Initializing source & destination indices to be -1
    int srcIndex = -1;
    int destIndex = -1;

    // Find the indices of source and destination cities
    srcIndex = findCityIndex(graph->cities, source);
    destIndex = findCityIndex(graph->cities, destination);

    // Add the edge to the adjacency matrix
    if(srcIndex != -1 && destIndex != -1) {
        graph->adjacencyMatrix[srcIndex][destIndex] = weight;
    } else if(srcIndex == -1) {
        printf("\nSource city is not found!\n");
    } else if(destIndex == -1) {
        printf("\nDestination city is not found!\n");
    }
}

// Function to find the shortest path using BreadthFirst Search (BFS) algorithm
void BFS(Graph* graph, char source[], char destination[], FILE* output) {
    int vertices = graph->vertices;
    bool isVisited[MAX_VERTICES]; // Array to store booleans of visiting vertices (cities)
    int previous[MAX_VERTICES]; // Array to store the previous vertices (cities) to find the shortest path later
    int distance[MAX_VERTICES]; // Array to store the distance from source city to each vertex (city)

    // Create a queue for BFS traversal
    int queue[MAX_VERTICES];
    int front = -1;
    int rear = -1;

    int i;

    // Initialize isVisited, previous, and distance arrays with default values
    for (i = 0; i < vertices; i++) {
        isVisited[i] = false;
        previous[i] = -1;
        distance[i] = INFINITY;
    }

    // Find the indices of source and destination cities
    int srcIndex = findCityIndex(graph->cities, source);
    int destIndex = findCityIndex(graph->cities, destination);

    // Check if source and destination cities are found
    if(srcIndex == -1 || destIndex == -1) {
        if(srcIndex == -1 && destIndex != -1) {
            printf("\nSource city is not found!\n");
        }

        if(destIndex == -1 && srcIndex != -1) {
            printf("\nDestination city is not found!\n");
        }

        if(destIndex == -1 && srcIndex == -1) {
            printf("Neither source nor destination cities are not found.\n");
        }

        return;
    }

    isVisited[srcIndex] = true; // Mark source city(if found) as visited
    distance[srcIndex] = 0; // Assign distance from source city to itself to be zero
    queue[++rear] = srcIndex; // Enqueue isVisited city

    // BFS algorithm
    while(front != rear) { // While the queue is not empty
        int currentVertex = queue[++front];

        for(i = 0; i < vertices; i++) {
            if(!isVisited[i] && graph->adjacencyMatrix[currentVertex][i] != INFINITY) { // Track adjacent unvisited vertices
                isVisited[i] = true;  // Mark adjacent vertex as visited
                previous[i] = currentVertex; // Update previous of adjacent vertex
                distance[i] = distance[currentVertex] + 1; // Update distance from source to adjacent vertex
                queue[++rear] = i; // Enqueue adjacent vertex

                if(i == destIndex) {
                    // Destination city found, break out of the loop
                    front = rear;
                    break;
                }
            }
        }
    }

    // Open output file in append mode
    output = fopen("shortest_distance.txt", "a");
    if (output == NULL) {
        printf("\nOutput file cannot open!\n");
    }

    // Print the shortest path and its distance
    printf("\nShortest path from %s to %s -Using BFS algorithm-:\n", source, destination);
    fprintf(output, "\nShortest path from %s to %s -Using BFS algorithm-:\n", source, destination);
    if (distance[destIndex] == INFINITY) {
        printf("No path found.\n");
        fprintf(output, "No path found.\n");
    } else {
        // Backtrack from the destination vertex to construct the shortest path
        int path[MAX_VERTICES];
        int pathLength = 0;
        int currentIndex = destIndex;

        while (currentIndex != srcIndex) {
            path[pathLength++] = currentIndex;
            currentIndex = previous[currentIndex];
        }

        // Print the path in reverse order (from source to destination)
        printf("%s", source);
        fprintf(output, "%s", source);
        for (i = pathLength - 1; i >= 0; i--) {
            printf(" -> %s", graph->cities[path[i]]);
            fprintf(output, " -> %s", graph->cities[path[i]]);
        }
        printf("\nDistance: %d\n", distance[destIndex]);
        fprintf(output, "\nDistance: %d\n", distance[destIndex]);
    }

}

// Function to find the shortest path using Dijkestra's algorithm
void dijkstra(Graph* graph, char source[], char destination[], FILE* output) {
    int numVertices = graph->vertices; // Number of vertices in the graph
    TABLE T[numVertices]; // Array of tables to store the distance, path, and visited information for each vertex

    // Find the indices of source and destination cities
    int srcIndex = findCityIndex(graph->cities, source);
    int destIndex = findCityIndex(graph->cities, destination);

    // Check if source and destination cities are found
    if(srcIndex == -1 || destIndex == -1) {
        if(srcIndex == -1 && destIndex != -1) {
            printf("\nSource city is not found!\n");
        }

        if(destIndex == -1 && srcIndex != -1) {
            printf("\nDestination city is not found!\n");
        }

        if(destIndex == -1 && srcIndex == -1) {
            printf("Neither source nor destination cities are not found.\n");
        }

        return;
    }

    // Initialize the table with default values
    for (int i = 0; i < numVertices; i++) {
        T[i].dist = INFINITY;
        T[i].path = -1;
        T[i].visited = false;
    }

    T[srcIndex].dist = 0; // set the distance of the source vertex is to 0

    // Create a queue
    int queue[MAX_VERTICES];
    int front = -1;
    int rear = -1;

    // Enqueue the source vertex
    queue[++rear] = srcIndex;

    // Dijkstra's algorithm
    while (front != rear) { // While the queue is not empty
        int v = queue[++front]; // Dequeue vertex v
        T[v].visited = true; // Mark dequeued vertex v as visited

        for (int w = 0; w < numVertices; w++) {
            if (graph->adjacencyMatrix[v][w] != INFINITY && T[w].visited == false) { // Check if the adjacent vertex is not visited
                if (T[w].dist > T[v].dist + graph->adjacencyMatrix[v][w]) { // Check if the new distance for adjacent vertex is smaller than the distance of current path
                    // Update distance and path in the table
                    T[w].dist = T[v].dist + graph->adjacencyMatrix[v][w];
                    T[w].path = v;
                }
                queue[++rear] = w; // Enqueue the adjacent vertex
            }
        }
    }

    // Open output file
    output = fopen("shortest_distance.txt", "a");
    if(output == NULL) {
        printf("\nOutput file cannot open!\n");
    }

    // Print the full route of the shortest path and its total cost
    printf("\nShortest path from %s to %s -Using Dijkstra's algorithm-:\n", source, destination);
    fprintf(output, "\nShortest path from %s to %s -Using Dijkstra's algorithm-:\n", source, destination);

    if (T[destIndex].dist == INFINITY) {
        printf("No path found.\n");
    } else {
        // Backtrack from the destination vertex to construct the shortest path
        int path[MAX_VERTICES]; // Array used to backtrack from the destination vertex to the source vertex
        int pathLength = 0;
        int currentIndex = destIndex;

        while (currentIndex != srcIndex) {
            path[pathLength++] = currentIndex;
            currentIndex = T[currentIndex].path;
        }

        // Print the path in reverse order (from source to destination)
        printf("%s", source);
        fprintf(output, "%s", source);
        int totalCost = 0;
        for (int i = pathLength - 1; i >= 0; i--) {
            int city1Index = path[i];
            int city2Index = T[city1Index].path;
            int cost = graph->adjacencyMatrix[city2Index][city1Index];
            totalCost += cost;
            printf(" -> (%d) %s", cost, graph->cities[city1Index]);
            fprintf(output, " -> (%d) %s", cost, graph->cities[city1Index]);
        }
        printf("\nTotal Shortest Cost: %d\n", totalCost);
        fprintf(output, "\nTotal Shortest Cost: %d\n", totalCost);

    }
}

// Function to print the adjacency matrix of the graph
void printGraph(Graph* graph) {
    int numVertices = graph->vertices;
    int i, j;

    printf("\nAdjacency Matrix:\n");
    for (i = 0; i < numVertices; i++) {
        for (j = 0; j < numVertices; j++) {
            if (graph->adjacencyMatrix[i][j] == INFINITY) {
                printf("INF\t");
            } else {
                printf("%d\t", graph->adjacencyMatrix[i][j]);
            }
        }
        printf("\n");
    }
}

// Function to check if a specified city exists in an array
bool isExist(char cities[MAX_CITIES][MAX_CITY_LENGTH], char city[]) {
    for(int i = 0; i < MAX_CITIES; i++) {
        if(strcmp(cities[i], city) == 0)
            return true;
    }
    return false;
}

// Function to insert a city to array of cities
void insertToArray(char cities[MAX_CITIES][MAX_CITY_LENGTH], char city[MAX_CITY_LENGTH]) {
    for (int i = 0; i < MAX_CITIES; i++) {
        if (cities[i][0] == '\0') {
            // Copy the string into the empty slot
            strcpy(cities[i], city);
            cities[i][MAX_CITY_LENGTH - 1] = '\0';  // Ensure null-termination
            return;  // Exit the function after successful insertion
        }
    }

    // If no empty slot is found, print an error message
    printf("\nArray is full. Unable to insert the string.\n");
}

// Function to print menu
void printMenu() {
    printf("\n\nPlease choose one from these options:\n");
    printf("\n1- Load cities.\n2- Enter source.\n3- Enter destination.\n4- Save and exit.\n");
}

// Main function
int main()
{
    Graph map; // Main graph

    char src[MAX_CITY_LENGTH];  // String for source city
    char dest[MAX_CITY_LENGTH]; // String for destination city

    FILE* output; // Output file

    int option;
    do {
        // Read menu option from user
        printMenu();
        scanf("%d", &option);

        if(option ==1) { // Load cities
            // Open the input file in read mode
            FILE* input = fopen("cities.txt", "r");
            if(input == NULL) {
                printf("\nInput file cannot open!\n");
            } else {
                initializeGraph(&map, 15); // Initializing main graph

                // Temporary variables for helping in creation of the graph
                char srcCity[MAX_CITY_LENGTH];
                char destCity[MAX_CITY_LENGTH];
                char weight[20];
                int intWeight;

                while((fscanf(input, "%s%s%s", &srcCity, &destCity, &weight) != EOF)) {
                    // Add cities to the graph (if don't exist)
                    if(!isExist(map.cities, srcCity)) {
                        insertToArray(map.cities, srcCity);
                    }
                    if(!isExist(map.cities, destCity)) {
                        insertToArray(map.cities, destCity);
                    }
                    sscanf(weight, "%d", &intWeight); // Taking integer value from distance
                    addEdge(&map, srcCity, destCity, intWeight); // Add an edge to the graph
                }
                printf("\nInput file is loaded successfully!\n");
            }

        } else if(option == 2) { // Enter source
            // Reading source city from user
            printf("\nEnter source city name: ");
            scanf("%s", src);

        } else if(option == 3) { // Enter destination
            // Reading destination city from user
            printf("\nEnter destination city name: ");
            scanf("%s", dest);

            printf("\n");
            BFS(&map, src, dest, output); // Shortest path using BFS algorithm
            printf("\n");
            dijkstra(&map, src, dest, output); // Shortest path using Dijkestra algorithm

        } else if(option == 4) {
            printf("\nFile saved..\n");
        }

    } while(option != 4);

    return 0;
}