
/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 */

#pragma once

#include <vector>

#include "mothership.h"

class SeafloorSurvey
{
public:
  void reset(int robotCount);
  void recordDebris(Location location);

  // Number of debris segments within SIGNAL_RANGE of this location,
  // on its own path and the paths immediately to its left and right.
  int getSignalStrength(Location location) const;

private:
  int countDebris(int robot, int low, int high) const;

  // debris[r] holds the segments on robot r's path where debris was found,
  // kept in increasing order so a range can be counted with binary search.
  std::vector<std::vector<int>> debris;
};
