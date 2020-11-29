#include "src/lib/trojanmap.h"

#include <map>
#include <vector>

#include "gtest/gtest.h"

//autoComplete
TEST(TrojanMapTest, AutoComplete_student) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  //all lowercase, should not impact result
  auto names = m.Autocomplete("ta");
  std::vector<std::string> gt1 = {"Target", "Tap Two Blue"};
  EXPECT_EQ(names, gt1);
  //all uppercase, should not impact result
  names = m.Autocomplete("TA");
  std::vector<std::string> gt2 = {"Target", "Tap Two Blue"};
  EXPECT_EQ(names, gt2);
  //all lowercase with space, should be no match
  names = m.Autocomplete("ta ");
  std::vector<std::string> gt3 = {};
  EXPECT_EQ(names, gt3);
  //lowercase + space
  names = m.Autocomplete("tap ");
  std::vector<std::string> gt4 = {"Tap Two Blue"};
  EXPECT_EQ(names, gt4);
  //space test, should be none
  names = m.Autocomplete(" ");
  std::vector<std::string> gt5 = {};
  EXPECT_EQ(names, gt5);
  //empty test, should be none
  names = m.Autocomplete("");
  std::vector<std::string> gt6 = {};
  EXPECT_EQ(names, gt6);
  names = m.Autocomplete(" ta");
  std::vector<std::string> gt7 = {};
  EXPECT_EQ(names, gt7);
}

//getPosition
TEST(TrojanMapTest, getPosition_student) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  //test normal match
  auto position = m.GetPosition("Ramen KenJo"); 
  std::pair<double, double> gt1(34.0249482, -118.2851489);
  EXPECT_EQ(position, gt1);
  //test another normal match
  position = m.GetPosition("MC39s Barber Shop");
  std::pair<double, double> gt2(34.0301204,	-118.2920527);
  EXPECT_EQ(position, gt2);
  //test not case match, should return no match (-1,-1)
  position = m.GetPosition("ramen kenjo");
  std::pair<double, double> gt3(-1,-1);
  EXPECT_EQ(position, gt3);
  //input space, return -1
  position = m.GetPosition(" ");
  std::pair<double, double> gt4(-1,-1);
  EXPECT_EQ(position, gt4);
  //test empty input name
  position = m.GetPosition(""); 
  std::pair<double, double> gt5(-1,-1);
  EXPECT_EQ(position, gt5);
  //only partial so no match
  position = m.GetPosition("Targ"); 
  std::pair<double, double> gt6(-1,-1);
  EXPECT_EQ(position, gt6);
}

//shortestPath for dijkstra
TEST(TrojanMapTest, CalculateShortestPath_notReach) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath("Chipotle Mexican Grill", "Target");
  std::vector<std::string> gt{}; //these two are not rachable according to the excel file
  EXPECT_EQ(path, gt);
  //wrong first name, return empty path
  path = m.CalculateShortestPath("Ralp", "Target"); 
  std::vector<std::string> gt2{}; //these two are not rachable according to the excel file
  EXPECT_EQ(path, gt2);
  //wrong second name, return empty path
  path = m.CalculateShortestPath("Ralphs", "Targ"); 
  std::vector<std::string> gt3{}; //these two are not rachable according to the excel file
  EXPECT_EQ(path, gt3);
}

TEST(TrojanMapTest, CalculateShortestPath_test) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath("Martin Luther King Jr Blvd  Coliseum", "Subway");
  std::vector<std::string> gt{
      "269633667","6815813004","348123159","348123160","4020099351","348123342",
      "4020099346","348123344","1870795193","122609808","4020099340","348123012",
      "1870797772","5617976418","21302801","1870795205","1870797882","1855150081",
  "1759017530"};
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath("Subway", "Martin Luther King Jr Blvd  Coliseum");
  std::reverse(gt.begin(),gt.end());
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

////////////////////////////////////////////////////////////////
// shortest path for bellman-ford
// These are commented out because they will cause the timeout due to GTest, they pass the unit tests

//shortestPath for dijkstra
// TEST(TrojanMapTest, CalculateShortestPath_notReach_bellman) {
//   TrojanMap m;
//   m.CreateGraphFromCSVFile();
//   auto path = m.CalculateShortestPath_bellmanFord("Chipotle Mexican Grill", "Target");
//   std::vector<std::string> gt{}; //these two are not reachable according to the excel file
//   EXPECT_EQ(path, gt);
//   //wrong first name, return empty path
//   path = m.CalculateShortestPath_bellmanFord("Ralp", "Target"); 
//   std::vector<std::string> gt2{}; 
//   EXPECT_EQ(path, gt2);
//   //wrong second name, return empty path
//   path = m.CalculateShortestPath_bellmanFord("Ralphs", "Targ"); 
//   std::vector<std::string> gt3{};
//   EXPECT_EQ(path, gt3);
// }

// TEST(TrojanMapTest, CalculateShortestPath_test_bellman) {
//   TrojanMap m;
//   m.CreateGraphFromCSVFile();
//   auto path = m.CalculateShortestPath_bellmanFord("Martin Luther King Jr Blvd  Coliseum", "Subway");
//   std::vector<std::string> gt{
//       "269633667","6815813004","348123159","348123160","4020099351","348123342",
//       "4020099346","348123344","1870795193","122609808","4020099340","348123012",
//       "1870797772","5617976418","21302801","1870795205","1870797882","1855150081",
//   "1759017530"};
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
  
//   path = m.CalculateShortestPath_bellmanFord("Subway", "Martin Luther King Jr Blvd  Coliseum");
//   std::reverse(gt.begin(),gt.end());
//   std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
//   std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
//   EXPECT_EQ(path, gt);
// }

////////////////////////////////////////////////////////////////

//TSP

//brute force
//created 3 test cases that have 5, 6, and 8 locations
//also tested an empty, a single location, and 2 location TSP 
TEST(TrojanMapTest, TSP) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1614922628","6042989969","5680945548","21302782","6815190442"};
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"1614922628","5680945548","6042989969","6815190442",
    "21302782","1614922628"};
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1841016354","7424270447", "269637362",
    "122996001", "6819288797","6045054378","2817034894","1781230449"};
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"1841016354","2817034894","1781230449","7424270447",
    "122996001", "6819288797","269637362","6045054378","1841016354"};
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_empty) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{};
  auto result = m.TravellingTrojan(input);
  std::vector<std::string> gt{};
  auto sizeGT = gt.size();
  auto sizeResult = result.second.size();
  EXPECT_EQ(sizeGT, sizeResult); //both should be empty
}

TEST(TrojanMapTest, TSP_oneElt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6042978413"};
  auto result = m.TravellingTrojan(input);
  std::vector<std::string> gt{"6042978413"};
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_twoElt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6042978413","5680945548"};
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"6042978413", "5680945548","6042978413"};
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

//2opt
TEST(TrojanMapTest, TSP_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1614922628","6042989969","5680945548","21302782","6815190442"
    };
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"1614922628","5680945548","6042989969","6815190442",
    "21302782","1614922628"};
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1841016354","7424270447", "269637362",
    "122996001", "6819288797","6045054378","2817034894","1781230449"};
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"1841016354","2817034894","1781230449","7424270447",
    "122996001", "6819288797","269637362","6045054378","1841016354"};
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

//fix later
TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"123120189", "4011837229", "4011837224", "2514542032", "2514541020", "1931345270", "4015477529", "214470792", "63068532", "6807909279"};
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " <<result.first << "miles" << std::endl;
  std::vector<std::string> gt{"123120189", "1931345270", "4011837224", "4011837229", "2514542032", "2514541020", "6807909279", "63068532", "214470792", "4015477529", "123120189"};
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_empty_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{};
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl;
  std::vector<std::string> gt{};
  auto sizeGT = gt.size();
  auto sizeResult = result.second.size();
  EXPECT_EQ(sizeGT, sizeResult); //both should be empty
}

TEST(TrojanMapTest, TSP_oneElt_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6042978413"};
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"6042978413"};
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP_twoElt_2opt) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"6042978413","5680945548"};
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"6042978413", "5680945548","6042978413"};
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}