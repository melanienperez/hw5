
/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>
#include <queue>
#include <vector>

namespace
{
  // Tuning knobs: change these and re-run the performance tests.
  // A switch costs SWITCH_COST (200) ticks, so each turn on a robot should
  // cover many segments. With BURST = 5000, switching adds only ~4% overhead.
  const int BURST = 5000;             // segments to drive per turn on a robot
  const double DEBRIS_BOOST = 1.0;    // score added per debris found on a path
  const double NEIGHBOR_BOOST = 0.5;  // score added to the paths on either side
  const double DECAY = 0.5;           // score multiplier after a burst with no debris

  struct Entry
  {
    double score;
    int position;
    int robot;

    // std::priority_queue is a max-heap: the "largest" entry comes out first.
    // Highest score wins; ties go to the robot that has explored the least.
    bool operator<(const Entry &other) const
    {
      if (score != other.score)
        return score < other.score;
      if (position != other.position)
        return position > other.position;
      return robot > other.robot;
    }
  };
}

Search::Search(Mothership &mothership)
    : mothership(mothership)
{
}

SearchResult Search::findShipwreck()
{
  const int robotCount = mothership.getRobotCount();

  // 1. Each robot's last safe segment, computed once (robots + abysses).
  std::vector<int> limit(robotCount, TETHER_LENGTH);
  for (const Location &abyss : mothership.getAbysses())
  {
    limit[abyss.robot] = std::min(limit[abyss.robot], abyss.segment - 1);
  }

  std::vector<double> score(robotCount, 0.0);
  std::priority_queue<Entry> queue;

  for (int robot = 0; robot < robotCount; ++robot)
  {
    int position = mothership.getRobotPosition(robot);
    if (position < limit[robot])
    {
      queue.push({0.0, position, robot});
    }
  }

  // When a path's score changes, push a fresh entry; the old one goes stale.
  auto boost = [&](int robot, double amount)
  {
    if (robot < 0 || robot >= robotCount)
      return;
    int position = mothership.getRobotPosition(robot);
    if (position >= limit[robot])
      return;
    score[robot] += amount;
    queue.push({score[robot], position, robot});
  };

  int activeRobot = mothership.getActiveRobot();

  // 2. Best-first search: give the most promising path a long burst.
  while (!queue.empty())
  {
    Entry top = queue.top();
    queue.pop();

    int robot = top.robot;
    int position = mothership.getRobotPosition(robot);

    // Skip stale entries.
    if (top.score != score[robot] || top.position != position || position >= limit[robot])
    {
      continue;
    }

    if (robot != activeRobot)
    {
      mothership.switchToRobot(robot);
      activeRobot = robot;
    }

    int burstEnd = std::min(limit[robot], position + BURST);
    int debrisFound = 0;

    while (position < burstEnd)
    {
      DriveResult result = mothership.driveActiveRobot(
          std::min(MAX_SEGMENTS_PER_DRIVE, burstEnd - position));
      position = mothership.getRobotPosition(robot);

      if (result.reason == DriveStopReason::ShipwreckFound)
      {
        return SearchResult{true, Location{robot, position}};
      }

      if (result.reason == DriveStopReason::DebrisFound)
      {
        seafloorSurvey.recordDebris({robot, position});
        ++debrisFound;
        // The wreck should be within SIGNAL_RANGE of its debris, so keep
        // going at least that far past the latest debris before switching.
        burstEnd = std::min(limit[robot], std::max(burstEnd, position + SIGNAL_RANGE));
      }
    }

    if (debrisFound > 0)
    {
      score[robot] += DEBRIS_BOOST * debrisFound;
      boost(robot - 1, NEIGHBOR_BOOST * debrisFound);
      boost(robot + 1, NEIGHBOR_BOOST * debrisFound);
    }
    else
    {
      // A whole burst with nothing: this lead is cooling off.
      score[robot] *= DECAY;
    }

    if (position < limit[robot])
    {
      queue.push({score[robot], position, robot});
    }
  }

  return SearchResult{false, Location{}};
}
