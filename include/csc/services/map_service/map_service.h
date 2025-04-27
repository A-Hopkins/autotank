/**
 * @file map_service.h
 * @brief Defines the MapService class for heap-free occupancy-grid mapping and path planning.
 *
 * MapService maintains a fixed-size occupancy grid and provides:
 *  - update_map(): fuse LiDAR scans into the grid under a seqlock for writers
 *  - plan_path(): run A* on the current grid with lock-free readers via the same seqlock
 * 
 * Also offers a nested Path type, a pool-backed singly-linked list of waypoints,
 * which allocates from a MemoryPool and frees automatically on destruction.
 */
#pragma once
#include <atomic>
#include <array>
#include <cstdint>

#include "protocore/include/memory_pool.h"

#include "msg/lidar_msg.h"
#include "msg/common_types/pose.h"

#ifdef UNIT_TESTING
#include <gtest/gtest_prod.h>
#endif

/**
 * @class MapService
 * @brief Provides a shared, heap-free occupancy grid and path-planning service.
 *
 * MapService maintains a fixed-size occupancy grid and allows planning a path
 * through traversable space. It is safe for one writer (update_map) and
 * multiple readers (plan_path) via a sequence-lock pattern:
 *
 *  - Writers: bump seq (odd), update grid, bump seq (even).
 *  - Readers: load seq (even), run planner, reload seq, retry if changed.
 *
 * This enables lock-free reads and minimal overhead writes—ideal for real-time.
 */
class MapService
{
public:
  static constexpr size_t MAX_FRONTIERS = 50; ///< Max frontiers we’ll ever collect
  /**
   * @struct Path
   * @brief A heap-free, pool-allocated singly-linked list of waypoints.
   *
   * Uses MemoryPool to allocate up to MAX_NODE_LEN nodes; push_back()
   * appends at the tail.  Supports forward iteration via Range-for.
   */
  struct Path
  {
    /// @brief A single waypoint node in the path.
    struct Node
    {
      Pose pose;  ///< The robot pose at this waypoint.
      Node *next; ///< Pointer to the next node.
    };

    /**
     * @brief Construct an empty Path using the given memory pool.
     * @param pool Reference to the MemoryPool for node allocations.
     */
    explicit Path(MemoryPool& pool) : pool(&pool), head(nullptr), tail(nullptr), path_size(0) { }

    /// @brief Destroy the Path and return all nodes to the pool.
    ~Path() { clear(); }

    /// @name Move Semantics (non-copyable)
    Path(Path&& o) noexcept : pool(o.pool), head(o.head), path_size(o.path_size)
    {
      o.head = nullptr;
      o.path_size = 0;
    }

    Path& operator=(Path&& o) noexcept
    {
      if (this != &o)
      {
        clear();
        pool = o.pool;
        head = o.head;
        path_size = o.path_size;
        o.head = nullptr;
        o.tail = nullptr;
        o.path_size = 0;
      }
      return *this;
    }

    Path(const Path&) = delete;            ///< copying is disallowed
    Path& operator=(const Path&) = delete; ///< assignment is disallowed
    
    /**
     * @brief Remove and deallocate all waypoints.
     *
     * Returns each Node to the MemoryPool. Leaves the Path empty.
     */
    void clear()
    {
      Node *current = head;

      while(current)
      {
        Node *next = current->next;
        pool->deallocate(current);
        current = next;
      }

      head = nullptr;
      tail = nullptr;
      path_size = 0;
    }

    /**
     * @brief Append a new waypoint to the end of the path.
     * @param waypoint The pose to append.
     * @return true if the node was allocated and appended; false on pool exhaustion.
     */
    bool push_back(const Pose& waypoint)
    {
      void *mem = pool->allocate();

      if (!mem)
      {
        return false;
      }

      Node *node = new(mem) Node{waypoint, nullptr};
      
      if (!head)
      {
        head = tail = node;
      }
      else
      {
        tail->next = node;
        tail = node;
      }

      path_size++;
      return true;
    }

    /**
     * @brief Prepend a waypoint to the front of the list.
     *
     * This lets you build a path in “start→goal” order simply by
     * walking the back‐pointer chain from goal back to start and
     * calling push_front() for each node.
     *
     * @param waypoint  The pose to insert at the head.
     * @return          True on success, false if the pool is exhausted.
     */
    bool push_front(const Pose &waypoint)
    {
      void* mem = pool->allocate();
      if (!mem)
      {
        return false;
      }

      Node* node = new(mem) Node{ waypoint, head };
      head = node;
      if (!tail)
      {
        tail = node;
      }

      path_size++;
      return true;
    }

    /**
     * @brief Get the current number of waypoints.
     * @return Number of nodes in this Path.
     */
    size_t get_path_size() const { return path_size; }

    /**
     * @class Iterator
     * @brief Forward iterator over Path waypoints.
     *
     * Supports C++11 range-for loops. Iterates from head→tail.
     */
    struct Iterator
    {
      using value_type = Pose;
      using reference = const Pose&;
      using pointer = const Pose*;
      using iterator_category = std::forward_iterator_tag;
      using difference_type = std::ptrdiff_t;
  
      explicit Iterator(Node* c) : cur(c) {}
  
      reference operator*() const { return cur->pose; }
      pointer operator->() const { return &cur->pose; }
  
      Iterator& operator++()
      {
        cur = cur->next;
        return *this;
      }

      Iterator operator++(int)
      {
        Iterator tmp = *this;
        ++*this;
        return tmp;
      }
  
      bool operator==(const Iterator& o) const { return cur == o.cur; }
      bool operator!=(const Iterator& o) const { return cur != o.cur; }

    private:
      Node* cur; ///< Current node in iteration.
    };
  
    /// @brief Begin iterator (first waypoint).
    Iterator begin()       { return Iterator(head); }
    /// @brief End iterator (one past last).
    Iterator end()         { return Iterator(nullptr); }
    Iterator begin() const { return Iterator(head); }
    Iterator end()   const { return Iterator(nullptr); }

  private:
    MemoryPool *pool; ///< Pool from which nodes are allocated.
    Node *head;       ///< First node in the linked list.
    Node *tail;       ///< Last node, for O(1) push_back().
    size_t path_size; ///< Number of waypoints currently stored.
  };

  /**
   * @enum CellStatus
   * @brief State of each occupancy grid cell.
   */
  enum class CellStatus
  {
    FREE = 0,     ///< Known free cell
    OCCUPIED = 1, ///< Known occupied cell
    UNKNOWN = 2   ///< Not yet observed
  };

  static MapService& instance();

  /**
   * @brief Incorporate a new LiDAR scan into the occupancy grid.
   *
   * Writers must wrap updates in a sequence lock:
   *  - increment seq (to odd)
   *  - modify occupancy_grid
   *  - increment seq (to even)
   *
   * @param scan  The latest LiDAR ranges and angles.
   * @param pose  The robot pose at the time of the scan.
   */
  void update_map(const msg::LidarDataMsg& scan, const Pose& pose);

  /**
   * @brief Plan a route from start to goal using A* on the current grid/graph.
   *
   * Readers use the sequence lock to ensure consistency:
   *  do {
   *    before = seq.load(); // must be even
   *    perform planning on occupancy_grid
   *    after  = seq.load();
   *  } while (before != after);
   *
   * @param start  The starting pose.
   * @param goal   The goal pose.
   * @return       A Path of waypoints (heap-free, uses MemoryPool).
   */
  Path plan_path(const Pose& start, const Pose& goal);

  /**
   * @brief Find up to `max_frontiers` free‐unknown boundary cells (frontiers).
   * @param origin  The robot’s current pose (for later filtering or sorting).
   * @param max_frontiers  Maximum number of frontiers to push (default = MAX_FRONTIERS).
   * @return APath of frontier poses (uses internal MemoryPool).
   */
  Path find_frontiers(const Pose& origin, size_t max_frontiers = MAX_FRONTIERS);

  /**
   * @brief get sequence from sequence lock
   * @return sequence number
   */
  uint32_t get_sequence() const { return seq.load(std::memory_order_acquire); }

#ifdef UNIT_TESTING
  /// Test‐only: reset the grid & sequence counter
  void reset_for_test()
  {
    occupancy_grid.fill(CellStatus::UNKNOWN);
    seq.store(0, std::memory_order_relaxed);
  }

  /// Test‐only: Inspect a single cell
  CellStatus get_cell(int gx, int gy) const
  {
    return occupancy_grid[gy * GRID_WIDTH + gx];
  }

  /// Test‐only: Force‐set a cell
  void set_cell(int gx, int gy, CellStatus s)
  {
    occupancy_grid[gy * GRID_WIDTH + gx] = s;
  }
#endif

private:

  static constexpr size_t MAX_NODE_LEN = 200;    ///< Max waypoints per path
  static constexpr int GRID_WIDTH = 200;         ///< Grid columns
  static constexpr int GRID_HEIGHT = 200;        ///< Grid rows
  static constexpr float GRID_RESOLUTION = 0.1f; ///< Meters per cell
  
  std::array<CellStatus, GRID_WIDTH * GRID_HEIGHT> occupancy_grid; ///< The map

  /** 
   * @brief Pool from which Path nodes are allocated.
   * Block size = sizeof(Path::Node), block count = MAX_NODE_LEN.
   */
  MemoryPool path_pool{ sizeof(Path::Node), MAX_NODE_LEN };

  /// Sequence counter for seqlock: even = stable, odd = writing.
  std::atomic<uint32_t> seq{0};

  /**
   * @brief Construct the MapService, initializing grid to UNKNOWN.
   */
  MapService();

  /**
   * @brief Destruct the MapService
   */
  ~MapService() = default;

  MapService(const MapService&)=delete;
  MapService& operator=(const MapService&)=delete;

  /**
   * @brief Integrate a LiDAR scan into the occupancy grid using ray-tracing.
   *
   * Performs Bresenham ray-tracing from the robot's cell to each valid beam
   * endpoint: marks all traversed cells as FREE, then marks the final cell as OCCUPIED.
   *
   * @param scan  The incoming LiDAR data (ranges, angles).
   * @param pose  The robot's world pose at the time of the scan.
   */
  void insert_scan(const msg::LidarDataMsg& scan, const Pose& pose);

  /**
   * @brief Execute the A* search algorithm on the current occupancy grid.
   *
   * Readers must use the sequence-lock protocol to ensure they read a
   * consistent snapshot of the grid before and after calling this method.
   *
   * @param start  The start pose for the path planning (world coordinates).
   * @param goal   The goal pose for the path planning (world coordinates).
   * @return       A Path object containing the sequence of waypoints from
   *               start to goal, or an empty Path on failure.
   */
  Path run_a_star(const Pose& start, const Pose& goal);

  /**
   * @brief Convert a single world‐coordinate to a [0..grid_size) index,
   *        using floor() followed by clamping.
   */
  static int world_coord_to_index(double world_coord, int grid_size)
  {
    double raw = (world_coord + grid_size * 0.5 * GRID_RESOLUTION) / GRID_RESOLUTION;
    int idx = static_cast<int>(std::floor(raw + 0.5));

    if (idx < 0)
    {
      return 0;
    }
    if (idx >= grid_size)
    {
      return grid_size - 1;
    }
    return idx;
  }

  /**
   * @brief Convert a Pose (x,y) into a single linear index into occupancy_grid.
   */
  static int pose_to_index(const Pose& p)
  {
    int ix = world_coord_to_index(p.point(0), GRID_WIDTH);
    int iy = world_coord_to_index(p.point(1), GRID_HEIGHT);
    return iy * GRID_WIDTH + ix;
  }
};
