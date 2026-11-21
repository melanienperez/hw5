
/*
 * STUDENT CODE - EDIT THIS FILE
 */

#include "search.h"

#include <algorithm>
#include <queue>
#include <vector>

namespace
{
  // When no robot has any signal, drive the least-explored robot this far
  // before switching. A switch costs SWITCH_COST (200) ticks, so longer bursts
  // waste less on switching; shorter bursts keep neighboring robots at similar
  // depths, which matters because signal strength only counts neighbor debris
  // at nearby segment numbers.
  const int EXPLORE_BURST = 2000;

  struct Entry
  {
    int signal;
    int position;
    int robot;

    // Max-heap order: strongest signal first; ties go to the robot that has
    // explored the least (so with no signal anywhere, we sweep evenly).
    bool operator<(const Entry &other) const
    {
      if (signal != other.signal)
        return signal < other.signal;
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
  seafloorSurvey.reset(robotCount);

  // Each robot's last safe segment, computed once (robots + abysses).
  std::vector<int> limit(robotCount, TETHER_LENGTH);
  for (const Location &abyss : mothership.getAbysses())
  {
    limit[abyss.robot] = std::min(limit[abyss.robot], abyss.segment - 1);
  }

  auto positionOf = [&](int robot)
  { return mothership.getRobotPosition(robot); };

  auto isOpen = [&](int robot)
  { return robot >= 0 && robot < robotCount && positionOf(robot) < limit[robot]; };

  // signal[r] = current signal strength at robot r's position.
  std::vector<int> signal(robotCount, 0);
  std::priority_queue<Entry> queue;

  // Recompute a robot's signal and push a fresh heap entry.
  // Older entries for that robot become stale and are skipped later.
  auto refresh = [&](int robot)
  {
    if (!isOpen(robot))
      return;
    signal[robot] = seafloorSurvey.getSignalStrength({robot, positionOf(robot)});
    queue.push({signal[robot], positionOf(robot), robot});
  };

  // Throw out stale entries so the heap stays O(robots) in size.
  auto rebuild = [&]()
  {
    std::priority_queue<Entry> fresh;
    for (int robot = 0; robot < robotCount; ++robot)
    {
      if (isOpen(robot))
        fresh.push({signal[robot], positionOf(robot), robot});
    }
    queue.swap(fresh);
  };

  auto isStale = [&](const Entry &entry)
  {
    return !isOpen(entry.robot) ||
           entry.signal != signal[entry.robot] ||
           entry.position != positionOf(entry.robot);
  };

  rebuild();
  int active = mothership.getActiveRobot();

  while (true)
  {
    while (!queue.empty() && isStale(queue.top()))
    {
      queue.pop();
    }
    if (queue.empty())
    {
      break;
    }

    // Follow the strongest signal. If the active robot is tied for the
    // strongest and has a lead, stay on it and save the switch cost.
    int robot = queue.top().robot;
    if (isOpen(active) && signal[active] > 0 && signal[active] >= queue.top().signal)
    {
      robot = active;
    }

    if (robot != active)
    {
      mothership.switchToRobot(robot);
      active = robot;
    }

    // With a lead, drive one step at a time and re-check the signal.
    // With no lead anywhere, explore in a long burst.
    int position = positionOf(robot);
    int stepSize = signal[robot] > 0 ? MAX_SEGMENTS_PER_DRIVE : EXPLORE_BURST;
    int stop = std::min(limit[robot], position + stepSize);

    while (position < stop)
    {
      DriveResult result = mothership.driveActiveRobot(
          std::min(MAX_SEGMENTS_PER_DRIVE, stop - position));
      position = positionOf(robot);

      if (result.reason == DriveStopReason::ShipwreckFound)
      {
        return SearchResult{true, Location{robot, position}};
      }

      if (result.reason == DriveStopReason::DebrisFound)
      {
        seafloorSurvey.recordDebris({robot, position});
        // This debris also counts toward the neighbors' signals.
        refresh(robot - 1);
        refresh(robot + 1);
        break; // re-evaluate now that there's new evidence
      }
    }

    refresh(robot);

    if (queue.size() > 4 * static_cast<std::size_t>(robotCount) + 64)
    {
      rebuild();
    }
  }

  return SearchResult{false, Location{}};
}
