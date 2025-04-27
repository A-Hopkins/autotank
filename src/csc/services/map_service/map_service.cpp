/**
 * @file map_service.cpp
 * @brief Implements the map service.
 */
#include <thread>

#include "csc/services/map_service/map_service.h"

MapService& MapService::instance()
{
  static MapService singleton;
  return singleton;
}

MapService::MapService()
{
  occupancy_grid.fill(CellStatus::UNKNOWN);
}


void MapService::update_map(const msg::LidarDataMsg& scan, const Pose& pose)
{
  // Mark in progress
  seq.fetch_add(1, std::memory_order_acq_rel);

  // Update occupancy grid based on new scan
  insert_scan(scan, pose);
  
  // Mark done
  seq.fetch_add(1, std::memory_order_acq_rel);
}

MapService::Path MapService::plan_path(const Pose& start, const Pose& goal)
{
  uint32_t before;
  uint32_t after;
  Path result{path_pool};

  do
  {
    // Wait for the map to be updated
    while ((before = seq.load(std::memory_order_acquire)) & 1)
    {
      std::this_thread::yield();
    }

    int start_idx = pose_to_index(start);
    int goal_idx  = pose_to_index(goal);

    // if either start or goal cell is occupied, bail out:
    if (occupancy_grid[start_idx] == CellStatus::OCCUPIED ||
        occupancy_grid[goal_idx]  == CellStatus::OCCUPIED)
    {
      return Path{path_pool};  // empty
    }

    // Run pathing algorithm
    result = run_a_star(start, goal);

    // Check that the map didn't change
    after = seq.load(std::memory_order_acquire);
  } while (before != after);
  
  return result;
}

void MapService::insert_scan(const msg::LidarDataMsg& scan, const Pose& pose)
{
  // Convert robot world pose to grid indices
  double rx_world = pose.point(0);
  double ry_world = pose.point(1);
  int rx = world_coord_to_index(rx_world, GRID_WIDTH);
  int ry = world_coord_to_index(ry_world, GRID_HEIGHT);

  // get yaw from quaternion
  auto& q = pose.orientation;
  double siny = 2 * (q(3) * q(2) + q(0) * q(1));
  double cosy = q(3) * q(3) + q(0) * q(0) - q(1) * q(1) - q(2) * q(2);
  double yaw = std::atan2(siny, cosy);
  double ox = pose.point(0);
  double oy = pose.point(1);

  // For each beam in the scan
  for (uint32_t i = 0; i < scan.ranges_count; ++i)
  {
    double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
    {
      continue;
    }

    // Compute endpoint in world coordinates
    double angle = scan.angle_min + i * scan.angle_increment;
    double wx = rx_world + r * std::cos(yaw + angle);
    double wy = ry_world + r * std::sin(yaw + angle);

    // Endpoint grid indices
    int gx = world_coord_to_index(wx, GRID_WIDTH);
    int gy = world_coord_to_index(wy, GRID_HEIGHT);

    // --- Bresenham line algorithm variables ---
    // (x0, y0): current grid cell, initialized to robot's cell
    int x0 = rx, y0 = ry;
    // (x1, y1): target grid cell corresponding to beam endpoint
    int x1 = gx, y1 = gy;
    // Differences in each axis
    int dx = std::abs(x1 - x0);         ///< Absolute horizontal distance
    int dy = -std::abs(y1 - y0);        ///< Negative of absolute vertical distance (for algorithm)
    // Step direction for each axis
    int sx = (x0 < x1) ? 1 : -1;        ///< Horizontal step direction (+1 right, -1 left)
    int sy = (y0 < y1) ? 1 : -1;        ///< Vertical step direction (+1 up, -1 down)
    // Error term: balance between horizontal and vertical steps
    int err = dx + dy;                  ///< Initial decision variable

    // Trace a line from robot cell to endpoint cell
    while (true)
    {
      // mark free cell
      if (x0 >= 0 && x0 < GRID_WIDTH && y0 >= 0 && y0 < GRID_HEIGHT)
      {
        occupancy_grid[y0 * GRID_WIDTH + x0] = CellStatus::FREE;
      }
      // reached endpoint
      if (x0 == x1 && y0 == y1)
      {
        break;
      }
      int e2 = 2 * err;
      if (e2 >= dy)
      {
        err += dy;
        // move horizontally
        x0 += sx;
      }
      if (e2 <= dx)
      {
        err += dx;
        // move vertically
        y0 += sy;
      }
    }

    // Mark the endpoint as occupied
    if (gx >= 0 && gx < GRID_WIDTH && gy >= 0 && gy < GRID_HEIGHT)
    {
      occupancy_grid[gy * GRID_WIDTH + gx] = CellStatus::OCCUPIED;
    }
  }
}

MapService::Path MapService::run_a_star(const Pose& start, const Pose& goal)
{
  Path path{path_pool};

  int start_idx = pose_to_index(start);
  int goal_idx  = pose_to_index(goal);

  // Boundary check and occupancy
  if (start_idx < 0 || start_idx >= GRID_WIDTH * GRID_HEIGHT ||
      goal_idx < 0 || goal_idx >= GRID_WIDTH * GRID_HEIGHT ||
      occupancy_grid[goal_idx] == CellStatus::OCCUPIED ||
      occupancy_grid[start_idx] == CellStatus::OCCUPIED)
  {
    // return empty path
    return path;
  }

  // === A* search implementation (4-connected grid) ===
  constexpr int TOTAL = GRID_WIDTH * GRID_HEIGHT;
  // Static arrays to avoid heap allocations
  static std::array<bool, TOTAL> closed;  // closed set
  static std::array<float, TOTAL> g_score; // cost from start
  static std::array<float, TOTAL> f_score; // g_score + heuristic
  static std::array<int, TOTAL> came_from; // backpointers
  static std::array<int, TOTAL> open_heap; // binary heap of nodes
  static std::array<bool, TOTAL> in_open;  // membership flag

  // Initalize per-cell scratch data
  for (int i = 0; i < TOTAL; ++i)
  {
    closed[i] = false;
    in_open[i] = false;
    g_score[i] = std::numeric_limits<float>::infinity();
    f_score[i] = std::numeric_limits<float>::infinity();
    came_from[i] = -1;
  }

  // Lambda for heuristic (Euclidean distance)
  auto heuristic = [&](int idx)
  {
    int x = idx % GRID_WIDTH;
    int y = idx / GRID_WIDTH;
    int gx = goal_idx % GRID_WIDTH;
    int gy = goal_idx / GRID_WIDTH;
    float dx = float(x - gx);
    float dy = float(y - gy);

    return std::sqrt(dx * dx + dy * dy);
  };

  // Heap helper lambdas
  int heap_size = 0;
  auto heap_push = [&](int idx)
  {
    open_heap[heap_size] = idx;
    in_open[idx] = true;
    int child = heap_size++;

    // sift up
    while (child > 0)
    {
      int parent = (child - 1) >> 1;
      if (f_score[open_heap[child]] < f_score[open_heap[parent]])
      {
        std::swap(open_heap[child], open_heap[parent]);
        child = parent;
      }
      else
      {
        break;
      }
    }
  };

  auto heap_pop = [&]()
  {
    int best = open_heap[0];
    in_open[best] = false;
    open_heap[0] = open_heap[--heap_size];
    int parent = 0;

    // sift down
    while (true)
    {
      int left = (parent << 1) + 1;
      int right = left + 1;
      int smallest = parent;

      if (left < heap_size && f_score[open_heap[left]] < f_score[open_heap[smallest]])
      {
        smallest = left;
      }

      if (right < heap_size && f_score[open_heap[right]] < f_score[open_heap[smallest]])
      {
        smallest = right;
      }

      if (smallest != parent)
      {
        std::swap(open_heap[parent], open_heap[smallest]);
        parent = smallest;
      }
      else
      {
        break;
      }
    }
    return best;
  };

  // Initialize start node
  g_score[start_idx] = 0.0f;
  f_score[start_idx] = heuristic(start_idx);
  heap_push(start_idx);

  int current = -1;
  // Neighbor coordinate offsets (4-way)
  constexpr int dirX[4] = { +1, -1, 0,  0 };
  constexpr int dirY[4] = {  0,  0, +1, -1 };
  
  // A* main loop
  while (heap_size > 0)
  {
    current = heap_pop();
    if (current == goal_idx)
    {
      break;
    }

    closed[current] = true;

    int cx = current % GRID_WIDTH;
    int cy = current / GRID_WIDTH;

    for (int k = 0; k < 4; ++k)
    {
      int nx = cx + dirX[k];
      int ny = cy + dirY[k];

      if (nx < 0 || nx >= GRID_WIDTH || ny < 0 || ny >= GRID_HEIGHT)
      {
        continue;
      }

      int neighbor = ny * GRID_WIDTH + nx;
      
      if (closed[neighbor] || occupancy_grid[neighbor] == CellStatus::OCCUPIED)
      {
        continue;
      }

      float tentative_G = g_score[current] + 1.0f;
      if (tentative_G < g_score[neighbor])
      {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_G;
        f_score[neighbor] = tentative_G + heuristic(neighbor);

        if (!in_open[neighbor]) 
        {
          heap_push(neighbor);
        }
      }
    }
  }

  if (current != goal_idx)
  {
    return path; // no path
  }

  // Walk the cameFrom chain from goal -> start,
  // pushing each pose to the front of the path
  for (int i = goal_idx; i >= 0; i = came_from[i])
  {
    // convert cell index -> world pose
    int x = i % GRID_WIDTH;
    int y = i / GRID_WIDTH;
    Pose waypoint;
    waypoint.point(0) = x * GRID_RESOLUTION - GRID_WIDTH * 0.5f * GRID_RESOLUTION;
    waypoint.point(1) = y * GRID_RESOLUTION - GRID_HEIGHT * 0.5f * GRID_RESOLUTION;
    waypoint.point(2) = 0.0f;

    path.push_front(waypoint);
  }

  return path;
}

MapService::Path MapService::find_frontiers(const Pose& origin, size_t max_frontiers)
{
  Path result{path_pool};

  // helper to test if any 4-neighbor is UNKNOWN
  auto has_unknown_neighbor = [&](int ix, int iy)
  {
    constexpr int d[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    for (auto &o : d)
    {
      int nx = ix + o[0], ny = iy + o[1];

      if (nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT)
      {
        size_t idx = ny * GRID_WIDTH + nx;
        if (occupancy_grid[idx] == CellStatus::UNKNOWN)
        {
          return true;
        }
      }
    }
    return false;
  };

  // scan the entire grid
  for (int y=0; y<GRID_HEIGHT && result.get_path_size()<max_frontiers; ++y)
  {
    for (int x=0; x<GRID_WIDTH && result.get_path_size()<max_frontiers; ++x)
    {
      size_t idx = y * GRID_WIDTH + x;
      if (occupancy_grid[idx] == CellStatus::FREE && has_unknown_neighbor(x,y))
      {
        // convert cell to world pose
        Pose p;
        p.point(0) = x * GRID_RESOLUTION  -  GRID_WIDTH * 0.5f * GRID_RESOLUTION;
        p.point(1) = y * GRID_RESOLUTION  -  GRID_HEIGHT * 0.5f * GRID_RESOLUTION;
        result.push_back(p);
      }
    }
  }
  return result;
}