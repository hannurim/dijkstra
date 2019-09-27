#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define INF 99999

using namespace std;

int **init_map;                                         // map information
float resolution;                                       // map resolution
int center = 0;                                         // half of map width

class Node{
    friend class Graph;
private:
    float x;
    float y;
    float z;
    float w;
public:
    Node(float xx=0, float yy=0, float zz=0, float ww=1)
    { x = xx; y = yy; z = zz; w = ww; }
};

class Graph {
public:
  Graph(const int size);
  int* PrintPath(const int start, const int finish);
  float returnval(const int index, const int num) {
      float temp;
      if (num == 0) { temp = node[index].x; }
      else if (num == 1) { temp = node[index].y; }
      else if (num == 2) { temp = node[index].z; }
      else if (num == 3) { temp = node[index].w; }
      return temp;
  }
private:
  class Node *node;
  float **cost;                                         // 이차원 배열
  int **dist;                                           // 가중치
  int **path;                                           // 경로
  bool *visited;                                        // 방문 여부를 표시
  int n;						// 정점의 수
  int Choose(int v);
  void Pathplan();
  float CalculateWeight(const int x1, const int y1, const int x2, const int y2);
  void CostUpdate();
  void ShortestPath();                                  // 최단 경로 함수
};

Graph::Graph(const int size)
{
  cost = new float*[size];				// 이차원 배열의 동적 생성
  path = new int*[size];				// n개의 path 필드 생성
  visited = new bool[size];				// n개의 visited 필드 생성
  dist = new int*[size];				// n개의 dist 필드 생성
  node = new class Node[size];
  n = size;
  for (int i = 0; i < n; i++) {
    cost[i] = new float[n];				// 이차원 배열의 초기화
    dist[i] = new int[n];
    path[i] = new int[n];				// 경로를 이차원 배열로 초기화
    visited[i] = false;                                 // visited 필드의 초기화
  }
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++)
      if (j == i) cost[i][j] = 0;
      else cost[i][j] = INF;
  Pathplan();
}

void Graph::Pathplan()
// read the data of goals and save to node class
{
    string filePath = "dijkstra_data.txt";
    ifstream openFile(filePath.data());
    if (openFile.is_open()) {
        for (int i = 0; i < n; i++) {
          openFile >> node[i].x >> node[i].y >> node[i].z >> node[i].w;
        }
        openFile.close();
    }
    else {
        ROS_ERROR("NO FILE");
    }
    CostUpdate();
}

void Graph::CostUpdate()
// save the weight to cost matrix
{
    for (int i = 0; i < n-1; i++) {
        for (int j = i+1; j < n; j++) {
            int first_x = (int) (node[i].x / resolution + center);
            int first_y = (int) (node[i].y / resolution + center);
            int second_x = (int) (node[j].x / resolution + center);
            int second_y = (int) (node[j].y / resolution + center);
            float weight;
            if (first_x < second_x)
                weight = CalculateWeight(first_x, first_y, second_x, second_y);
            else
                weight = CalculateWeight(second_x, second_y, first_x, first_y);
            cost[i][j] = weight;
            cost[j][i] = weight;
        }
    }
    ShortestPath();
}

float Graph::CalculateWeight(const int x1, const int y1, const int x2, const int y2)
// calculate euclidean distance. if there is an obstacle between two nodes, return INF
{
    float slope = (float)(y2 - y1) / (float)(x2 - x1);
    float bias = -1 * slope * x1 + y1;
    for (int i = x1; i < x2; i++) {
        int j = (int) (i * slope + bias);
        if (init_map[i][j] == 100) return INF;
    }
    float distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

void Graph::ShortestPath()
// 최단거리 구하기
{
    for (int v = 0; v < n; v++){
      for (int i = 0; i < n; i++) { visited[i] = false; dist[v][i] = cost[v][i]; path[v][i] = v; }
      visited[v] = true;
      dist[v][v] = 0;
      for (int i = 0; i < n - 2; i++) {
        int u = Choose(v);
        visited[u] = true;
        for (int w = 0; w < n; w++)
          if (!visited[w])
            if (dist[v][u] + cost[u][w] < dist[v][w]) {
              dist[v][w] = dist[v][u] + cost[u][w];
              path[v][w] = u;
            }
      }
    }
    cout << "\n\t<PATH>\n";
    for (int i = 0; i < n; i++) {
        for (int j=0; j<n; j++){
            cout << path[i][j] << "\t";
        }
        cout << endl;
    }
}

int Graph::Choose(int v)
// 방문되지 않은 정점 중 제일 비용이 적은 정점 구하기
{
    int min = INF;
    int w;
    for (int i=0;i<n;i++)
        if (dist[v][i] < min && !visited[i]) { min = dist[v][i]; w = i; }
    return w;
}

int* Graph::PrintPath(const int start, const int finish)
// 경로추적하여 출력
{
        int *temp = new int[n];
        int *Output = new int[n];
        int cnt = n - 2;

        for (int j = 0; j < n; j++) temp[j] = Output[j] = INF;

        temp[0] = start;
        temp[n - 1] = finish;
        cnt = n - 2;

        for (int j = path[start][finish];; j = path[start][j]) {
              if (start == j) break;
              temp[cnt--] = j;
        }

        if (start != finish && dist[start][finish] != INF) {
            cnt = 0;
            for (int j = 0; j < n; j++){
                  if (temp[j] != INF)
                    Output[cnt++] = temp[j];
             }
        }
        else
            ROS_ERROR("It have no path.");

        return Output;
}

void GoalPublisher(int *path, class Graph g);

#endif // DIJKSTRA_H
