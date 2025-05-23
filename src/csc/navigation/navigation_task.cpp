/**
 * @file navigation_task.cpp
 * @brief Implements the navigation task
 *
 */
#include "csc/navigation/navigation_task.h"
#include <cmath>
#include <random>

static std::mt19937& rng()
{
  static std::mt19937 eng{std::random_device{}()};
  return eng;
}

void NavigationTask::process_message(const msg::Msg& msg)
{
  switch (msg.get_type())
  {
    case msg::Type::StateMsg:
    {
      transition_to_state(static_cast<task::TaskState>(msg.get_data_as<msg::StateMsg>()->state));
      break;
    }
    case msg::Type::HeartbeatMsg:
    {
      handle_heartbeat(msg.get_data_as<msg::HeartbeatMsg>());
      break;
    }
    case msg::Type::LocalizationEstimateMsg:
    {
      if (current_state == task::TaskState::RUNNING)
      {
        handle_localization_estimate(msg.get_data_as<msg::LocalizationEstimateMsg>());
      }
      break;
    }

    default:
    {
      std::cout << get_name()
                << " received unhandled message type: " << msg::msg_type_to_string(msg.get_type())
                << std::endl;
      break;
    }
  }
}

void NavigationTask::transition_to_state(task::TaskState new_state)
{
  if (new_state == current_state)
    return;
  std::cout << get_name() << " transitioning to " << task_state_to_string(new_state) << std::endl;
  current_state = new_state;

  // Always clear goal on a new state transistion
  clear_goal();

  switch (new_state)
  {
    case task::TaskState::NOT_STARTED:
    {
      break;
    }
    case task::TaskState::IDLE:
    {
      break;
    }
    case task::TaskState::RUNNING:
    {
      evaluate_behavior();
      break;
    }

    case task::TaskState::STOPPED:
    {
      break;
    }
    case task::TaskState::ERROR:
    {
      break;
    }
    default:
    {
      std::cerr << "Error: Unknown state transition requested: " << task_state_to_string(new_state)
                << std::endl;
      break;
    }
  }
  safe_publish(msg::Msg(this, msg::StateAckMsg{static_cast<uint8_t>(current_state)}));
}

void NavigationTask::handle_localization_estimate(const msg::LocalizationEstimateMsg* loc_est)
{
  if (!loc_est)
    return;

  current_loc_est = *loc_est;

  // capture home on first ever update
  if (!home_pose_initialized)
  {
    home_pose             = current_loc_est.est_pose.pose;
    home_pose_initialized = true;
  }

  // if we have no active goal, pick one
  if (!active_goal.has_value())
  {
    evaluate_behavior();
    return;
  }

  // otherwise check if we've reached our current waypoint
  const Pose& goal = active_goal->target_pose;
  double      dx   = current_loc_est.est_pose.pose.point(0) - goal.point(0);
  double      dy   = current_loc_est.est_pose.pose.point(1) - goal.point(1);

  if (std::sqrt(dx * dx + dy * dy) < POSITION_TOLERANCE)
  {
    send_next_waypoint();
  }
}

bool NavigationTask::plan_path_to_goal()
{
  // unwrap our goal
  const Pose& start = current_loc_est.est_pose.pose;
  const Pose& end   = active_goal->target_pose;

  // ask the map service
  auto path = MapService::instance().plan_path(start, end);
  if (path.get_path_size() == 0)
  {
    path_valid = false;
    return false;
  }

  // store and initialize iterator
  planned_path  = std::move(path);
  path_iterator = planned_path->begin();
  path_valid    = true;

  return true;
}

void NavigationTask::evaluate_behavior()
{
  // must have home initialized
  if (!home_pose_initialized)
    return;

  // first time in RUNNING: GO_HOME if far, else EXPLORE
  if (current_behavior == BehaviorMode::NONE)
  {
    auto&  p    = current_loc_est.est_pose.pose.point;
    double dx   = p(0) - home_pose.point(0);
    double dy   = p(1) - home_pose.point(1);
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist > 0.5)
    {
      set_behavior(BehaviorMode::GO_HOME);
    }
    else
    {
      set_behavior(BehaviorMode::EXPLORE);
    }
    return;
  }

  // if we finish GO_HOME, switch to EXPLORE
  if (current_behavior == BehaviorMode::GO_HOME &&
      (!path_valid || path_iterator == planned_path->end()))
  {
    set_behavior(BehaviorMode::EXPLORE);
  }
}

void NavigationTask::send_next_waypoint()
{
  if (!path_valid || !planned_path || !path_iterator)
  {
    // nothing left
    clear_goal();
    return;
  }

  Pose next = **path_iterator;
  // overwrite the active_goal’s pose to be *this* waypoint
  active_goal->target_pose = next;

  // publish current waypoint
  msg::WaypointMsg next_waypoint;
  next_waypoint.goal_pose = next;

  safe_publish(msg::Msg(this, next_waypoint));

  // advance iterator
  ++*path_iterator;
  if (path_iterator == planned_path->end())
  {
    // path done
    clear_goal();
  }
}

void NavigationTask::set_behavior(BehaviorMode mode)
{
  current_behavior = mode;
  clear_goal();

  switch (mode)
  {
    case BehaviorMode::GO_HOME:
    {
      go_home_behavior();
      break;
    }
    case BehaviorMode::EXPLORE:
    {
      explore_world_behavior();
      break;
    }
    default:
    {
      break;
    }
  }
}

void NavigationTask::clear_goal()
{
  active_goal.reset();
  planned_path.reset();
  path_iterator.reset();
  path_valid = false;
}

void NavigationTask::go_home_behavior()
{
  MissionGoal goal;

  goal.target_pose = home_pose;
  goal.mode        = current_behavior;
  active_goal      = goal;

  if (!plan_path_to_goal())
  {
    std::cerr << "NavigationTask: cannot plan GO_HOME path\n";
    clear_goal();
    return;
  }

  send_next_waypoint();
}

void NavigationTask::explore_world_behavior()
{
  // get frontier candidates
  auto frontiers = MapService::instance().find_frontiers(current_loc_est.est_pose.pose);

  if (frontiers.get_path_size() > 0)
  {
    // 2) Scan to find the nearest one
    const auto& me     = current_loc_est.est_pose.pose.point;
    double      best_d = std::numeric_limits<double>::infinity();
    Pose        best_p;

    for (auto& p : frontiers)
    {
      double dx = p.point(0) - me(0);
      double dy = p.point(1) - me(1);
      double d  = std::hypot(dx, dy);
      if (d < best_d)
      {
        best_d = d;
        best_p = p;
      }
    }

    // 3) Set that as our active goal
    MissionGoal goal;
    goal.mode        = BehaviorMode::EXPLORE;
    goal.target_pose = best_p;
    active_goal      = goal;

    // 4) Plan & send
    if (!plan_path_to_goal())
    {
      std::cerr << get_name() << ": could not plan to frontier\n";
      clear_goal();
    }
    else
    {
      send_next_waypoint();
    }
    return;
  }

  // --- fallback: random sampling of free space ---
  std::uniform_real_distribution<double> dx(-10.0, 10.0);
  std::uniform_real_distribution<double> dy(-10.0, 10.0);

  constexpr int MAX_TRIES = 20;
  bool          found     = false;

  for (int i = 0; i < MAX_TRIES; ++i)
  {
    Pose cand     = current_loc_est.est_pose.pose;
    cand.point(0) = dx(rng());
    cand.point(1) = dy(rng());

    MissionGoal goal{cand, BehaviorMode::EXPLORE};
    active_goal = goal;

    if (plan_path_to_goal())
    {
      found = true;
      send_next_waypoint();
      break;
    }
  }

  if (!found)
  {
    std::cerr << get_name() << ": explore_world_behavior could not find any reachable free spot\n";
    clear_goal();
  }
}
