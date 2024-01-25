//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
// Name: PRATHAM SITOULA
// SID : 1660488
// CCID : sitoula
// CMPUT 275, Winter 2022
//
// Assignment 1: Trivial Navigation Systems (Part II)
//-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -
#include <iostream>
#include <cassert>
#include <fstream>
#include <string>
#include <list>
#include <sstream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "wdigraph.h"
#include "dijkstra.h"

struct Point {
    long long lat, lon;
};

// returns the manhattan distance between two points
long long manhattan(const Point& pt1, const Point& pt2) {
  long long dLat = pt1.lat - pt2.lat, dLon = pt1.lon - pt2.lon;
  return abs(dLat) + abs(dLon);
}

// finds the id of the point that is closest to the given point "pt"
int findClosest(const Point& pt, const unordered_map<int, Point>& points) {
  pair<int, Point> best = *points.begin();

  for (const auto& check : points) {
    if (manhattan(pt, check.second) < manhattan(pt, best.second)) {
      best = check;
    }
  }
  return best.first;
}

// read the graph from the file that has the same format as the "Edmonton graph" file
void readGraph(const string& filename, WDigraph& g, unordered_map<int, Point>& points) {
  ifstream fin(filename);
  string line;

  while (getline(fin, line)) {
    // split the string around the commas, there will be 4 substrings either way
    string p[4];
    int at = 0;
    for (auto c : line) {
      if (c == ',') {
        // start new string
        ++at;
      }
      else {
        // append character to the string we are building
        p[at] += c;
      }
    }

    if (at != 3) {
      // empty line
      break;
    }

    if (p[0] == "V") {
      // new Point
      int id = stoi(p[1]);
      assert(id == stoll(p[1])); // sanity check: asserts if some id is not 32-bit
      points[id].lat = static_cast<long long>(stod(p[2])*100000);
      points[id].lon = static_cast<long long>(stod(p[3])*100000);
      g.addVertex(id);
    }
    else {
      // new directed edge
      int u = stoi(p[1]), v = stoi(p[2]);
      g.addEdge(u, v, manhattan(points[u], points[v]));
    }
  }
}

int create_and_open_fifo(const char * pname, int mode) {
  // creating a fifo special file in the current working directory
  // with read-write permissions for communication with the plotter
  // both proecsses must open the fifo before they can perform
  // read and write operations on it
  if (mkfifo(pname, 0666) == -1) {
    cout << "Unable to make a fifo. Ensure that this pipe does not exist already!" << endl;
    exit(-1);
  }

  // opening the fifo for read-only or write-only access
  // a file descriptor that refers to the open file description is
  // returned
  int fd = open(pname, mode);

  if (fd == -1) {
    cout << "Error: failed on opening named pipe." << endl;
    exit(-1);
  }

  return fd;
}

// keep in mind that in part 1, the program should only handle 1 request
// in part 2, you need to listen for a new request the moment you are done
// handling one request
int main()
{

  WDigraph graph;
  unordered_map<int, Point> points;

  const char *inpipe = "inpipe";
  const char *outpipe = "outpipe";

  // Open the two pipes
  int in = create_and_open_fifo(inpipe, O_RDONLY);
  cout << "inpipe opened..." << endl;
  int out = create_and_open_fifo(outpipe, O_WRONLY);
  cout << "outpipe opened..." << endl;

  // buffer char array
  char buffer[1024] = {0};
  readGraph("server/edmonton-roads-2.0.1.txt", graph, points);

  while (true)
  {
    // read a request
    read(in, buffer, 1024);
    if (buffer[0] == 'Q')
    {
      break;
    }
    stringstream info(buffer);

    // vars to be casted later
    double lon1;
    double lat1;
    double lon2;
    double lat2;
    Point Point1, Point2;
    // gets the information
    info >> lat1 >> lon1 >> lat2 >> lon2;

    // cast to type long long
    Point1.lon = static_cast<long long>(lon1 * 100000.0);
    Point1.lat = static_cast<long long>(lat1 * 100000.0);
    Point2.lon = static_cast<long long>(lon2 * 100000.0);
    Point2.lat = static_cast<long long>(lat2 * 100000.0);
    // c is guaranteed to be 'R', no need to error check
    // get the points closest to the two points we read
    int start = findClosest(Point1, points), end = findClosest(Point2, points);

    // run dijkstra's algorithm, this is the unoptimized version that
    // does not stop when the end is reached but it is still fast enough
    unordered_map<int, PIL> tree;
    dijkstra(graph, start, tree);
    // no path
    if (tree.find(end) == tree.end())
    {
      continue;
    }

    else
    {
      // read off the path by stepping back through the search tree
      list<int> path;
      while (end != start)
      {
        path.push_front(end);
        end = tree[end].first;
      }

      path.push_front(start);
      for (int v : path)
      {
        // cast to type double
        lon1 = static_cast<double>(points[v].lon / 100000.0);
        lat1 = static_cast<double>(points[v].lat / 100000.0);
        string double_lon = to_string(lon1);
        string double_lat = to_string(lat1);
        // removes trailing 0's
        double_lon.pop_back();
        double_lat.pop_back();
        // format the waypoint
        string formatted_waypoint = double_lat + ' ' + double_lon + '\n';
        // send the formatted waypoint
        write(out, formatted_waypoint.c_str(), formatted_waypoint.length());
      }
    }
    // send this message to indicate that we've reached the destination
    write(out, "E\n", 2);
  }
  // closing and unlinking the pipes
  close(in);
  close(out);
  unlink(inpipe);
  unlink(outpipe);
  return 0;
}
