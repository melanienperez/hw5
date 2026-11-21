
/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 */

#include "seafloor_survey.h"

#include <algorithm>

void SeafloorSurvey::reset(int robotCount)
{
  debris.assign(robotCount, std::vector<int>());
}

void SeafloorSurvey::recordDebris(Location location)
{
  std::vector<int> &path = debris[location.robot];

  // Robots only drive forward, so new debris is normally the furthest yet
  // and can go on the end. Fall back to a sorted insert just in case.
  if (path.empty() || path.back() < location.segment)
  {
    path.push_back(location.segment);
    return;
  }

  auto spot = std::lower_bound(path.begin(), path.end(), location.segment);
  if (spot == path.end() || *spot != location.segment)
  {
    path.insert(spot, location.segment);
  }
}

int SeafloorSurvey::countDebris(int robot, int low, int high) const
{
  if (robot < 0 || robot >= static_cast<int>(debris.size()))
  {
    return 0;
  }
  const std::vector<int> &path = debris[robot];
  auto first = std::lower_bound(path.begin(), path.end(), low);
  auto last = std::upper_bound(path.begin(), path.end(), high);
  return static_cast<int>(last - first);
}

int SeafloorSurvey::getSignalStrength(Location location) const
{
  const int low = location.segment - SIGNAL_RANGE;
  const int high = location.segment + SIGNAL_RANGE;
  return countDebris(location.robot - 1, low, high) +
         countDebris(location.robot, low, high) +
         countDebris(location.robot + 1, low, high);
}
