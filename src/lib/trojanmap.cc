#include "trojanmap.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <utility>

#include <cctype>
#include <unordered_set>
#include <unordered_map>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <chrono> 

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu()
{

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0)
    {
      for (auto x : results)
        std::cout << x << std::endl;
    }
    else
    {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = GetPosition(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1)
    {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    }
    else
    {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    std::cout << "If you select bellman-ford please expect around 4 minute runtime\n" << std::endl;
    menu = "Please select function: 1 = dijkstra, 2 = bellman-ford: ";
    std::cout << menu;
    getline(std::cin, input);
    int selFtn = std::stoi(input);
    if(selFtn < 1 || selFtn > 2) {
      std::cout << "Invalid input\n";
      break;
    }
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    //used to calculate the time of the function
    // auto start = std::chrono::high_resolution_clock::now(); 
    std::vector<std::string> results;
    if(selFtn == 1) {
      results = CalculateShortestPath(input1, input2);
    }
    else {
      results = CalculateShortestPath_bellmanFord(input1, input2);
    }
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); 
    // std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0)
    {
      for (auto x : results)
        std::cout << x << std::endl;
      PlotPath(results);
    }
    else
    {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl
              << std::endl;
    menu = "Please select function: 1 = brute force, 2 = 2_opt: ";
    std::cout << menu;
    getline(std::cin, input);
    int selFtn = std::stoi(input);
    if(selFtn < 1 || selFtn > 2) {
      std::cout << "Invalid input\n";
      break;
    }
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data)
    {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    std::pair<double, std::vector<std::vector<std::string>>> results;
    /////////////////////////////////////
    //used for execution time calculation 

    // auto start = std::chrono::high_resolution_clock::now(); 

    if(selFtn == 1) {
      results = TravellingTrojan(locations);
    }
    else {
      results = TravellingTrojan_2opt(locations);
    }
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); 
    // std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

    // auto results = TravellingTrojan(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0)
    {
      for (auto x : results.second[results.second.size() - 1])
        std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size() - 1]);
    }
    else
    {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '5':
    break;
  default:
    std::cout << "Please select 1 - 5." << std::endl;
    PrintMenu();
    break;
  }
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile()
{
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line))
  {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ','))
    {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else
      {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++)
  {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids)
{
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids)
  {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress)
{
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(1248, 992));
  for (auto location_ids : path_progress)
  {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++)
    {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                 cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
               cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
               LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
  video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon)
{
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) { return data[id].lat; }

/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { return data[id].lon; }

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { return data[id].name; }

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id)
{
  // std::vector<std::string> result;
  // std::cout << id << std::endl;
  return data[id].neighbors;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b)
{
  if (a.id == b.id)
  { //if the same place then distance is 0
    return 0;
  }

  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double lat1 = a.lat * M_PI / 180.0;
  double lat2 = b.lat * M_PI / 180.0;

  double a_dub = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * asin(std::min(1.0, sqrt(a_dub)));
  return 3961 * c;
  //keep for now but delete later
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path)
{
  int pathSize = path.size();
  if (pathSize < 2)
  {
    return 0;
  }
  double sum = 0;
  //assume that the path is not cyclic unless specified
  for (int i = 0; i < pathSize - 1; i++)
  {
    sum += CalculateDistance(data[path[i]], data[path[i + 1]]);
  }
  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */

// void dfsComp(int &currIndex, std::string name, Node &data, std::vector<bool> &marked, std::vector<std::string> &results) {
//   marked[currIndex] = true;
//   //check if name is in the current node
//   if(name.size() >= data.)
// }

std::vector<std::string> TrojanMap::Autocomplete(std::string name)
{
  std::vector<std::string> results;
  if(name == "") { //if input is empty then return empty 
    return results;
  }
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    std::string currStr = it->second.name;
    if (name.size() <= currStr.size())
    {                                                             //check if the ID input is less than or equal to the ID
      std::string tempBeginning = currStr.substr(0, name.size()); //copy the beginning # characters same as name

      std::transform(name.begin(), name.end(), name.begin(), ::tolower); //set both input and temp to lowercase
      std::transform(tempBeginning.begin(), tempBeginning.end(), tempBeginning.begin(), ::tolower);

      // std::cout << tempBeginning << std::endl;
      if (name.compare(tempBeginning) == 0)
      { //they are equal
        results.push_back(currStr);
      }
    }
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name)
{
  std::pair<double, double> results(-1, -1);
  if(name == "") { //if empty name then return empty 
    return results;
  }
  bool found = false;
  for (auto it = data.begin(); it != data.end() && !found; ++it)
  {
    std::string currStr = it->second.name;

    //set both input and currStr to lowercase
    // std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    // std::transform(currStr.begin(), currStr.end(), currStr.begin(), ::tolower);

    if (name.compare(currStr) == 0)
    { //check if input name matches currentMap name
      results.first = it->second.lat;
      results.second = it->second.lon;
      found = true;
    }
  }
  return results;
} //CHECK IF IT MATTERS FOR CASE!

/**
 * CalculateShortestPath: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */

int FindMinInDButNotInVisited(std::vector<double> &d, std::unordered_set<int> &visited, bool &unreach)
{
  // Find min
  int u;
  double min = INT_MAX;

  for (int i = 0; i < d.size(); i++)
  {
    if (visited.count(i) > 0)
    {
      continue;
    }
    double e = d[i];

    if (min > e)
    {
      min = e;
      u = i;
      unreach = false;
    }
  }

  return u;
}

std::string TrojanMap::getID(std::string name)
{
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    std::string currStr = it->second.name;
    if (name.compare(currStr) == 0)
    {
      return it->first;
    }
  }
  return "";
}

void TrojanMap::createWeights()
{
  weight_ = std::vector<std::vector<double>>(data.size(), std::vector<double>(data.size(), INT_MAX));
  for (size_t i = 0; i < data.size(); i++)
  {
    weight_[i][i] = 0;
    // std::cout << "node num " << i << " num neighbors " << data[nodeID[i]].neighbors.size() << std::endl;
    for (auto n : data[nodeID[i]].neighbors)
    {
      weight_[i][IDnode[n]] = CalculateDistance(data[nodeID[i]], data[n]);
      weight_[IDnode[n]][i] = weight_[i][IDnode[n]];
      // std::cout << "node " << n << " weight " << weight_[i][IDnode[n]] << std::endl;
    }
  }
}

void pathVec(std::vector<int> &path, std::vector<int> &parent, int currNode)
{
  if (parent[currNode] == -1)
  {
    return;
  }
  path.push_back(parent[currNode]);
  pathVec(path, parent, parent[currNode]);
}

std::vector<std::string> TrojanMap::CalculateShortestPath(
    std::string location1_name, std::string location2_name)
{
  std::string ID1 = getID(location1_name); //returns "" if not found
  std::string ID2 = getID(location2_name); //returns "" if not found

  if(ID1 == "" || ID2 == "") { //invalid location names
    std::vector<std::string> emptyReturn;
    return emptyReturn;
  }

  //create mapping from ID to element # and visa versa
  auto it = data.begin();
  for (int i = 0; i < data.size(); i++)
  {
    IDnode[it->first] = i;
    nodeID[i] = it->first;
    it++;
  }

  createWeights();

  std::unordered_set<int> visited;
  std::vector<double> d(data.size());
  std::vector<int> parent(data.size(), -1);
  std::vector<int> path;
  bool unreach = false;

  //create weights matrix

  for (int i = 0; i < weight_.size(); i++)
  {
    d[i] = weight_[IDnode[ID1]][i];
  }

  visited.insert(IDnode[ID1]);

  //terminate if all nodes visited, or no more reachable, OR if destination has been visited
  while (visited.size() < weight_.size() && !unreach && visited.count(IDnode[ID2]) == 0)
  {
    unreach = true;
    int u = FindMinInDButNotInVisited(d, visited, unreach);
    visited.insert(u);

    for (int i = 0; i < weight_.size() && !unreach; i++)
    {
      if (d[i] > d[u] + weight_[u][i])
      { //didnt use std min because want to set parent
        d[i] = d[u] + weight_[u][i];
        parent[i] = u;
      }
    }
  }
  std::vector<std::string> x;

  if(visited.count(IDnode[ID2]) == 0) { //not reachable so return empty vector
    return x;
  }

  path.push_back(IDnode[ID2]);

  pathVec(path, parent, IDnode[ID2]);

  x.push_back(ID1);

  for (int i = path.size() - 1; i >= 0; i--)
  {
    x.push_back(nodeID[path[i]]);
  }

  return x;
}

std::vector<std::string> TrojanMap::CalculateShortestPath_bellmanFord(
    std::string location1_name, std::string location2_name)
{
  std::string ID1 = getID(location1_name); //returns "" if not found
  std::string ID2 = getID(location2_name); //returns "" if not found

  if(ID1 == "" || ID2 == "") { //invalid location names
    std::vector<std::string> emptyReturn;
    return emptyReturn;
  }

  //create mapping from ID to element # and visa versa
  auto it = data.begin();
  for (int i = 0; i < data.size(); i++)
  {
    IDnode[it->first] = i;
    nodeID[i] = it->first;
    it++;
  }

  createWeights();

  std::vector<double> d(data.size());
  std::vector<int> parent(data.size(), -1);
  std::vector<int> path;

  //create weights matrix

  for (int i = 0; i < weight_.size(); i++)
  {
    d[i] = weight_[IDnode[ID1]][i];
  }
  d[IDnode[ID1]] = 0;
  for(int i = 0; i < weight_.size()-1; i++) {
    for(int u = 0; u < weight_.size(); u++) {
      for(int v = 0; v < weight_.size(); v++) {
        if(d[v] > d[u] + weight_[u][v]) {
          d[v] = d[u] + weight_[u][v];
          parent[v] = u;
        }
      }
    }
  }
 
  std::vector<std::string> x;

  if(d[IDnode[ID2]] == INT_MAX) { //not reachable so return empty vector
    return x;
  }

  path.push_back(IDnode[ID2]);

  pathVec(path, parent, IDnode[ID2]);

  x.push_back(ID1);

  for (int i = path.size() - 1; i >= 0; i--)
  {
    x.push_back(nodeID[path[i]]);
  }

  return x;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */

void TrojanMap::createWeightsPathNodes(std::vector<std::string> &location_ids)
{
  weight_TS = std::vector<std::vector<double>>(location_ids.size(), std::vector<double>(location_ids.size(), INT_MAX));

  auto it = data.begin();
  for (int i = 0; i < location_ids.size(); i++)
  {
    IDnode[location_ids[i]] = i;
    nodeID[i] = location_ids[i];
    it++;
  }

  for (size_t i = 0; i < location_ids.size(); i++)
  { //calc distance between all of the nodes
    weight_TS[i][i] = 0;
    // std::cout << "node num " << i << " num neighbors " << data[nodeID[i]].neighbors.size() << std::endl;
    for (size_t j = 0; j < location_ids.size(); j++)
    {
      weight_TS[i][j] = CalculateDistance(data[nodeID[i]], data[nodeID[j]]);
    }
  }
}

void TrojanMap::PermuteCombs(std::vector<int> &nums,
                  std::vector<std::vector<std::string>> &result,
                  std::vector<std::string> curResult)
{
  // If we get to a leaf, we save the path in result and return return.
  if (curResult.size() == nums.size())
  {
    result.push_back(curResult);
    return;
  }
  // Generate the branches
  for (int i = 1; i < nums.size(); i++)
  {
    // Avoid duplicates and backtrack
    if (find(curResult.begin(), curResult.end(), nodeID[i]) != curResult.end())
    {
      continue;
    }
    // Recursive DFS-like call
    std::vector<std::string> nextCurResult = curResult;
    if(nextCurResult.size() == 0) {
      nextCurResult.push_back(nodeID[0]);
    }
    nextCurResult.push_back(nodeID[i]);
    PermuteCombs(nums, result, nextCurResult);
  }
}

//////////////////////////////////////////////////////////////////

//brute-force

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
    std::vector<std::string> &location_ids)
{
  if(location_ids.size() == 0) {
    return {0, {}};
  }
  else if(location_ids.size() == 1) {
    return {0, {location_ids}};
  }
  //create a weights matrix where it doesn't matter about being neighbors
  createWeightsPathNodes(location_ids);

  std::vector<int> nodesLoc;
  std::vector<std::vector<std::string>> permutes;

  for (int i = 0; i < location_ids.size(); i++)
  {
    nodesLoc.push_back(i);
  }

  //creates each permutation of the nodes
  PermuteCombs(nodesLoc, permutes, {});
  // for(auto n: permutes) {
  //   for(auto j: n) {
  //     std::cout << j << " ";
  //   }
    // std::cout << " " << std::endl;
  // }
  double pathMin = INT_MAX;
  std::vector<std::vector<std::string>> paths;

  //go over every permutation and determine which one has the shortest pathlength
  //start node has to be 0
  int indexMin = 0;
  int index = 0;
  for (auto n : permutes)
  {
      // //create list of ids from nodes
      std::vector<std::string> idPermutes = n;
      idPermutes.push_back(n[0]); //add the last ID again to complete the cycle
      double pathLen = CalculatePathLength(idPermutes);
      if (pathLen < pathMin)
      {
        pathMin = pathLen;
        indexMin = index;
      }
      paths.push_back(idPermutes);
      // std::cout << "path cost " << pathLen << std::endl;
      index++;
  }
  std::vector<std::string> pathsTemp = paths.at(indexMin);
  paths.erase(paths.begin() + indexMin);
  paths.push_back(pathsTemp); //make the answer the last element in the list

  std::pair<double, std::vector<std::vector<std::string>>> results;

  results = {pathMin, paths};
  return results;
}

void twoOptSwap(const int &i, const int &k, const std::vector<int> &nodesVec, std::vector<int> &newNodesVec)
{
  for (int c = 0; c < i; ++c)
  { //add 0 to i-1 to newnodes
    newNodesVec.push_back(nodesVec[c]);
  }
  int dec = 0;
  for (int c = i; c <= k; ++c)
  { //add i to k in reverse
    newNodesVec.push_back(nodesVec[k - dec]);
    dec++;
  }
  for (int c = k + 1; c < nodesVec.size(); ++c)
  {
    newNodesVec.push_back(nodesVec[c]); //add the k+1 to the end
  }
}

//2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(std::vector<std::string> &location_ids)
{
  //create a weights matrix where it doesn't matter about being neighbors
  createWeightsPathNodes(location_ids);

  if(location_ids.size() == 0) {
    return {0, {}};
  }
  else if(location_ids.size() == 1) {
    return {0, {location_ids}};
  }
  else if(location_ids.size() == 2) { //last base case since min for 2opt swap is 3
    std::vector<std::string> path = {location_ids[0], location_ids[1], location_ids[0]};
    return {CalculatePathLength(path), {path}};
  }

  std::vector<int> nodesLoc;
  std::vector<std::vector<int>> permutes;

  for (int i = 0; i < location_ids.size(); i++)
  {
    nodesLoc.push_back(i);
  }
  nodesLoc.push_back(0); //return to the beginning

  bool improve = true;

  std::vector<std::string> nodesIDsPath;
  std::vector<std::vector<std::string>> paths;

  for (auto n : nodesLoc) {
      nodesIDsPath.push_back(nodeID[n]);
  }
  nodesIDsPath.push_back(nodeID[nodesLoc[0]]); //make complete cycle

  double bestLen = CalculatePathLength(nodesIDsPath);

  int index = 0;
  int minIndex = 0;

  //repeat until no improvement
  while (improve)
  {
    improve = false;
    std::vector<int> newNodesVec;
    std::vector<int> bestLen2OP;
    std::vector<std::string> newIDsVec;

    for (int i = 1; i < location_ids.size() - 1; i++)
    {
      for (int k = i+1; k < location_ids.size(); k++)
      {
        if(k == i) continue;
        //twoOptSwap
        twoOptSwap(i, k, nodesLoc, newNodesVec);
        for (auto n : newNodesVec) {
          newIDsVec.push_back(nodeID[n]);
        }
        paths.push_back(newIDsVec);
        
        double currLen = CalculatePathLength(newIDsVec);
        if(currLen < bestLen) {
          // std::cout << "new " << currLen << " prev " << bestLen << std::endl;
          improve = true;
          bestLen2OP = newNodesVec;
          bestLen = currLen;
          minIndex = index;
        }
        newIDsVec.clear();
        newNodesVec.clear();
        index++;
      }
    }
    nodesLoc = bestLen2OP;
  }

  //delete the minimum and then add to the end
  std::vector<std::string> minTemp;
  minTemp = paths.at(minIndex); 
  paths.erase(paths.begin() + minIndex);
  paths.push_back(minTemp);

  std::pair<double, std::vector<std::vector<std::string>>> results;
  results = {bestLen, paths};
  return results;
}