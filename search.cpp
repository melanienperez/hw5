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
  const double DEBRIS_BOOST = 1.0;    // added to a path's score when it finds debris
  const double NEIGHBOR_BOOST = 0.5;  // added to the paths on either side
  const double DECAY = 0.75;          // score multiplier after a drive with no debris

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

  // 1. Work out each robot's last safe segment ONCE.
  //    The old code looped over every abyss for every robot (robots x abysses),
  //    which is what made test 13 take 3 seconds.
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

  // When a path's score changes, push a fresh entry. The old one goes stale
  // and gets skipped when it reaches the top.
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

  int activeRobot = -1;

  // 2. Best-first search: always drive the most promising path one chunk.
  //    With no debris anywhere, this acts like round robin (least-explored first).
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

    // Only switch when needed, so a hot path keeps its robot.
    if (robot != activeRobot)
    {
      mothership.switchToRobot(robot);
      activeRobot = robot;
    }

    DriveResult result = mothership.driveActiveRobot(
        std::min(MAX_SEGMENTS_PER_DRIVE, limit[robot] - position));
    position = mothership.getRobotPosition(robot);

    if (result.reason == DriveStopReason::ShipwreckFound)
    {
      return SearchResult{true, Location{robot, position}};
    }

    if (result.reason == DriveStopReason::DebrisFound)
    {
      seafloorSurvey.recordDebris({robot, position});
      score[robot] += DEBRIS_BOOST;
      boost(robot - 1, NEIGHBOR_BOOST);
      boost(robot + 1, NEIGHBOR_BOOST);
    }
    else
    {
      // Nothing here: let this lead cool off (handles false leads, test 08).
      score[robot] *= DECAY;
    }

    if (position < limit[robot])
    {
      queue.push({score[robot], position, robot});
    }
  }

  return SearchResult{false, Location{}};
}