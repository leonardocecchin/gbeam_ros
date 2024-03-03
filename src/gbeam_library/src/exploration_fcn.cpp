#include "exploration_fcn.h"

#define INF 100000

// find minimum element in array, return its index
int iMin(float arr[], int size)
{
  float min = INF;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]<min)
    {
      min = arr[i];
      id = i;
    }
  }
  return id;
}

// find maximum element in array, return its index
int iMax(float arr[], int size)
{
  float max = 0;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]>max)
    {
      max = arr[i];
      id = i;
    }
  }
  return id;
}

// return minimum element, from ones with con[i]==true
int iMinCon(float arr[], bool con[], int size)
{
  float min = INF;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]<min && con[i])
    {
      min = arr[i];
      id = i;
    }
  }
  return id;
}

// return maximum element, from ones with con[i]==true
int iMaxCon(float arr[], bool con[], int size)
{
  float max = 0;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]>max && con[i])
    {
      max = arr[i];
      id = i;
    }
  }
  return id;
}

//compute distances matrices of graph
void shortestDistances(gbeam_library::ReachabilityGraph graph, float dist[], int start)
{
  int E = graph.edges.size();
  int N = graph.nodes.size();
  bool Q_set[N];
  int prev[N];

  for(int i=0; i<N; i++)
    dist[i] = INF, prev[i] = -1, Q_set[i] = graph.nodes[i].is_reachable ? true : false;
    // note: add to Q_set only nodes that are reachable

  dist[start] = 0;

  for (int count=0; count<N; count++)//all vertices
  {
    int u = iMinCon(dist, Q_set, N); // select the node with shortest distance from Q
    Q_set[u] = false; // remove selected node from Q

    if(dist[u] >= INF)  // if the shortest distance is INF, then the graph is not connected.
      break;

    for(int e=0; e<E; e++)
    {
      // if an edge is connected to u, then check if the node it is connected to
      // belongs to set Q. In that case check if the total distance passing through u
      // is shorter than the current best one, in that case update it.
      if(graph.edges[e].is_walkable)
      {
        if(graph.edges[e].v1 == u && Q_set[graph.edges[e].v2] && dist[graph.edges[e].v2] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v2] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v2] = u;
        }
        if(graph.edges[e].v2 == u && Q_set[graph.edges[e].v1] && dist[graph.edges[e].v1] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v1] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v1] = u;
        }
      }
    }
  }

  return;
}

// compute shortest path in graph from start to end
// using Dijkstra algorithm
std::vector<int> dijkstra(gbeam_library::ReachabilityGraph graph, int s, int t)
{
  int N = graph.nodes.size();
  int E = graph.edges.size();

  // initialize variables
  // dist[i] will contain the minimum distance from s to i
  // prev[i] will contain the previous node in the shortest path
  float dist[N];
  int prev[N];
  bool Q_set[N];
  for (int i=0; i<N; i++)
    dist[i] = INF, prev[i] = -1, Q_set[i] = graph.nodes[i].is_reachable ? true : false;
  // note: add to Q_set only nodes that are reachable

  dist[s] = 0;

  for (int count=0; count<N; count++)//all vertices
  {
    int u = iMinCon(dist, Q_set, N); // select the node with shortest distance from Q
    Q_set[u] = false; // remove selected node from Q

    if(dist[u] >= INF || u == t)  // if the shortest distance is INF, then the graph is not connected.
    // if u == t, then the shortest path from s to t is already available.
    break;

    for(int e=0; e<E; e++)
    {
      // if an edge is connected to u, then check if the node it is connected to
      // belongs to set Q. In that case check if the total distance passing through u
      // is shorter than the current best one, in that case update it.
      if(graph.edges[e].v1 == u && Q_set[graph.edges[e].v2] && dist[graph.edges[e].v2] > (dist[u]+graph.edges[e].length))
      {
        dist[graph.edges[e].v2] = dist[u] + graph.edges[e].length;
        prev[graph.edges[e].v2] = u;
      }
      if(graph.edges[e].v2 == u && Q_set[graph.edges[e].v1] && dist[graph.edges[e].v1] > (dist[u]+graph.edges[e].length))
      {
        dist[graph.edges[e].v1] = dist[u] + graph.edges[e].length;
        prev[graph.edges[e].v1] = u;
      }
    }
  }

  std::vector<int> path;
  path.push_back(t);
  while(path[0] != s)
  path.insert(path.begin(), prev[path[0]]);

  return path;
}

// compute best path from s to t (if t<0 it is ignored)
std::vector<int> bestPath(gbeam_library::ReachabilityGraph graph, int s, int t)
{
  int N = graph.nodes.size();
  int E = graph.edges.size();

  // initialize variables
  // dist[i] will contain the minimum distance from s to i
  // prev[i] will contain the previous node in the shortest path
  float dist[N];
  int prev[N];
  bool Q_set[N];
  for (int i=0; i<N; i++)
  dist[i] = INF, prev[i] = -1, Q_set[i] = graph.nodes[i].is_reachable ? true : false;
  // note: add to Q_set only nodes that are reachable

  dist[s] = 0;

  for (int count=0; count<N; count++)//all vertices
  {
    int u = iMinCon(dist, Q_set, N); // select the node with shortest distance from Q
    Q_set[u] = false; // remove selected node from Q

    if(dist[u] >= INF || u == t)  // if the shortest distance is INF, then the graph is not connected.
    // if u == t, then the shortest path from s to t is already available.
    break;

    for(int e=0; e<E; e++)
    {
      // if an edge is connected to u, then check if the node it is connected to
      // belongs to set Q. In that case check if the total distance passing through u
      // is shorter than the current best one, in that case update it.
      if(graph.edges[e].is_walkable)
      {
        if(graph.edges[e].v1 == u && Q_set[graph.edges[e].v2] && dist[graph.edges[e].v2] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v2] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v2] = u;
        }
        if(graph.edges[e].v2 == u && Q_set[graph.edges[e].v1] && dist[graph.edges[e].v1] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v1] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v1] = u;
        }
      }
    }
  }

  std::vector<int> path;
  path.push_back(t);
  while(path[0] != s)
  path.insert(path.begin(), prev[path[0]]);

  return path;
}
