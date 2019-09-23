
#pragma once

#include <Eigen/Core>
#include <memory>
#include <unordered_set>

namespace hedge
{

struct edge_t;
struct edge_index_t;
class edge_fn_t;

struct face_t;
struct face_index_t;
class face_fn_t;

struct vertex_t;
struct vertex_index_t;
class vertex_fn_t;

struct point_t;
struct point_index_t;
class point_fn_t;

class kernel_t;
class mesh_t;
class mesh_builder_t;
class edge_loop_builder_t;

using position_t = Eigen::Vector3f;
using normal_t   = Eigen::Vector3f;
using uv_t       = Eigen::Vector2f;
using color_t    = Eigen::Vector4f;

using offset_t     = uint32_t;
using generation_t = uint32_t;

/**
   Using a strong index type instead of a bare pointer or generic integer index
   allows you to potentially re-use cells (if the kernel implements support for
   it) and can be more easily or directly validated.
 */
enum class index_type_t : unsigned char
{
  vertex,
  edge,
  face,
  point,
  unsupported
};
template <index_type_t TIndexType = index_type_t::unsupported>
struct index_t
{
  offset_t     offset;
  generation_t generation;

  explicit index_t() noexcept
    : offset(0)
    , generation(0)
  {
  }

  explicit index_t(const offset_t o) noexcept
    : offset(o)
    , generation(0)
  {
  }

  explicit index_t(const offset_t o, const generation_t g) noexcept
    : offset(o)
    , generation(g)
  {
  }

  void reset()
  {
    offset     = 0;
    generation = 0;
  }

  bool operator!=(const index_t& other) const
  {
    return !(*this == other);
  }
  bool operator==(const index_t& other) const
  {
    return offset == other.offset && generation == other.generation;
  }

  friend bool operator<(const index_t& lhs, const index_t& rhs)
  {
    return lhs.offset < rhs.offset;
  }
  friend bool operator>(const index_t& lhs, const index_t& rhs)
  {
    return rhs < lhs;
  }

  explicit operator bool() const noexcept
  {
    return offset > 0;
  }
};

template <typename TIndex>
struct index_hasher_t
{
  std::size_t operator()(TIndex const& index) const
  {
    std::size_t h1 = std::hash<offset_t>{}(index.offset);
    std::size_t h2 = std::hash<generation_t>{}(index.generation);
    return h1 ^ (h2 << 1);
  }
};

// Discriminated types to assist in API design and reduce the potential for
// errors that can arise from using generic index types like plain integers and
// so on.
struct edge_index_t : index_t<index_type_t::edge>
{
  using index_t::index_t;
  static edge_index_t const invalid;
  using hasher_t = index_hasher_t<edge_index_t>;
  using set_t    = std::unordered_set<edge_index_t, hasher_t>;
};

struct face_index_t : index_t<index_type_t::face>
{
  using index_t::index_t;
  static face_index_t const invalid;
};

struct vertex_index_t : index_t<index_type_t::vertex>
{
  using index_t::index_t;
  static vertex_index_t const invalid;
  using hasher_t = index_hasher_t<vertex_index_t>;
  using set_t    = std::unordered_set<vertex_index_t, hasher_t>;
};

struct point_index_t : index_t<index_type_t::point>
{
  using index_t::index_t;
  static point_index_t const invalid;
};

/**
   The mesh kernel implements/provides the fundamental storage and access
   operations.
 */
class kernel_t
{
public:
  using ptr_t = std::unique_ptr<kernel_t, void (*)(kernel_t*)>;

  virtual ~kernel_t() = default;

  virtual edge_t*   get(edge_index_t index)   = 0;
  virtual face_t*   get(face_index_t index)   = 0;
  virtual vertex_t* get(vertex_index_t index) = 0;
  virtual point_t*  get(point_index_t index)  = 0;

  virtual edge_index_t   insert(edge_t edge)     = 0;
  virtual face_index_t   insert(face_t face)     = 0;
  virtual vertex_index_t insert(vertex_t vertex) = 0;
  virtual point_index_t  insert(point_t point)   = 0;

  virtual edge_index_t   emplace(edge_t&& edge)     = 0;
  virtual face_index_t   emplace(face_t&& face)     = 0;
  virtual vertex_index_t emplace(vertex_t&& vertex) = 0;
  virtual point_index_t  emplace(point_t&& point)   = 0;

  virtual void remove(edge_index_t index)   = 0;
  virtual void remove(face_index_t index)   = 0;
  virtual void remove(vertex_index_t index) = 0;
  virtual void remove(point_index_t index)  = 0;

  virtual size_t point_count() const  = 0;
  virtual size_t vertex_count() const = 0;
  virtual size_t face_count() const   = 0;
  virtual size_t edge_count() const   = 0;

  virtual void defrag() = 0;

  /**
   * Create an empty edge and it's adjacent edge.
   */
  edge_index_t make_edge_pair();

  /**
   * Given the root edge index of a connected edge loop, create a new associated
   * face
   */
  face_index_t make_face(edge_index_t root_eindex);

  /**
   * Connects two edges with a new vertex associated with the specified point
   */
  vertex_index_t connect_edges(edge_index_t  eindex,
                               point_index_t pindex,
                               edge_index_t  next_eindex);
};

////////////////////////////////////////////////////////////////////////////////
// Our principle element structures.

enum class element_status_t : uint16_t
{
  active   = 0x0000,
  inactive = 0x8000
};

/**
 * TODO: docs
 */
struct element_t
{
  element_status_t status = element_status_t::active;
  uint16_t         tag    = 0;
  generation_t generation = 1; // Using 1 as default to allow indexes with gen 0
                               // to have another meaning.
};

/**
 * TODO: docs
 */
struct edge_t : element_t
{
  vertex_index_t vertex_index;
  face_index_t   face_index;
  edge_index_t   next_index;
  edge_index_t   prev_index;
  edge_index_t   adjacent_index;
};

/**
 * TODO: docs
 */
struct face_t : element_t
{
  edge_index_t        root_edge_index;
  edge_index_t::set_t edges;
  normal_t            normal;
};

/**
 * TODO: docs
 */
struct vertex_t : element_t
{
  point_index_t point_index;
  edge_index_t  edge_index;
  normal_t      normal;
};

// TODO: make point attributes configurable
/**
 * TODO: docs
 */
struct point_t : element_t
{
  position_t position;
  normal_t   normal;

  vertex_index_t::set_t vertices;

  point_t();
  point_t(float x, float y, float z);
  explicit point_t(position_t p);
};

////////////////////////////////////////////////////////////////////////////////
// Mesh element proxies to allow easy traversal.

/**
   We have this simple templated base class which allows functions that request
   an alement to return an "empty" proxy. An alternative to this would be to
   use std::optional but it seems like this would be sufficient for now.
 */
template <typename TIndex, typename TElement>
class element_fn_t
{
protected:
  kernel_t* _kernel;
  TIndex    _index;

public:
  explicit element_fn_t(kernel_t* kernel, TIndex index) noexcept
    : _kernel(kernel)
    , _index(index)
  {
  }

  explicit operator bool() const noexcept
  {
    return _kernel != nullptr && (bool)_index && element() != nullptr;
  }

  bool operator==(element_fn_t const& other) const
  {
    return _index == other._index;
  }

  bool operator!=(element_fn_t const& other) const
  {
    return _index != other._index;
  }

  TElement* element() const
  {
    if (_kernel != nullptr)
    {
      return _kernel->get(_index);
    }
    else
    {
      return nullptr;
    }
  }

  TIndex index()
  {
    return _index;
  }
};

using edge_points_t   = std::array<point_fn_t, 2>;
using edge_vertices_t = std::array<vertex_fn_t, 2>;

/**
 * TODO: docs
 */
class edge_fn_t : public element_fn_t<edge_index_t, edge_t>
{
public:
  using element_fn_t::element_fn_t;

  vertex_fn_t vertex() const;
  face_fn_t   face() const;
  edge_fn_t   next() const;
  edge_fn_t   prev() const;
  edge_fn_t   adjacent() const;

  bool is_boundary() const;

  edge_points_t   points() const;
  edge_vertices_t vertices() const;

  static const edge_fn_t invalid;
};

/**
 * TODO: docs
 */
class face_fn_t : public element_fn_t<face_index_t, face_t>
{
public:
  using element_fn_t::element_fn_t;

  edge_fn_t root_edge() const;
  // TODO: Returning a `const &` is actually bad. If this proxy is invalid there
  //       is no way to fullfill the contract.
  edge_index_t::set_t const& edges() const;

  /**
   * (Re)calculates vertex normal and returns the updated value.
   */
  normal_t calculate_normal();
  float    calculate_area() const;
  normal_t normal() const;

  static const face_fn_t invalid;
};

/**
 * TODO: docs
 */
class vertex_fn_t : public element_fn_t<vertex_index_t, vertex_t>
{
public:
  using element_fn_t::element_fn_t;

  edge_fn_t  edge() const;
  point_fn_t point() const;

  normal_t normal() const;

  /**
   * (Re)calculates vertex normal and returns the updated value.
   */
  normal_t calculate_normal();

  static const vertex_fn_t invalid;
};

/**
 * TODO: docs
 */
class point_fn_t : public element_fn_t<point_index_t, point_t>
{
public:
  using element_fn_t::element_fn_t;

  position_t position() const;
  normal_t   normal() const;
  // TODO: Returning a `const &` is actually bad. If this proxy is invalid there
  //       is no way to fullfill the contract.
  vertex_index_t::set_t const& vertices() const;

  /**
   * (Re)calculates vertex normal and returns the updated value.
   */
  normal_t calculate_normal();

  static const point_fn_t invalid;
};

////////////////////////////////////////////////////////////////////////////////

/**
 * TODO: Add documentation.
 */
class mesh_builder_t
{
protected:
  mesh_t& _mesh;

public:
  explicit mesh_builder_t(mesh_t& mesh);

  face_index_t add_triangle(point_t p0, point_t p1, point_t p2);
  face_index_t add_triangle(point_index_t pindex0,
                            point_index_t pindex1,
                            point_index_t pindex2);
  face_index_t add_triangle(edge_index_t eindex, point_t p0);
  face_index_t add_triangle(edge_index_t eindex, point_index_t pindex);

  edge_loop_builder_t start_edge_loop(point_index_t pindex);
  edge_loop_builder_t start_edge_loop(edge_index_t eindex0);
};

/**
 * A simple interface for constructing edge loops originating at the
 * specified point.
 */
class edge_loop_builder_t
{
  mesh_t&       _mesh;
  edge_index_t  _root_eindex;
  edge_index_t  _last_eindex;
  point_index_t _root_pindex;
  point_index_t _last_pindex;

public:
  /**
   * Starts a new edge loop.
   * @param mesh
   * @param pindex0
   * @param pindex1
   */
  edge_loop_builder_t(mesh_t& mesh, point_index_t pindex);

  /**
   * Starts an edge loop from an existing boundary edge.
   * @param mesh
   * @param root_eindex
   */
  edge_loop_builder_t(mesh_t& mesh, edge_index_t root_eindex);

  edge_loop_builder_t& add_point(point_index_t next_pindex);

  edge_index_t close();
};

/**
 * TODO: Add documentation.
 */
class mesh_t
{
  kernel_t::ptr_t _kernel;

public:
  mesh_t();
  explicit mesh_t(kernel_t::ptr_t&&);

  kernel_t* kernel()
  {
    return _kernel.get();
  }

  size_t point_count() const;
  size_t vertex_count() const;
  size_t edge_count() const;
  size_t face_count() const;

  edge_fn_t   edge(edge_index_t index) const;
  face_fn_t   face(face_index_t index) const;
  vertex_fn_t vertex(vertex_index_t index) const;
  point_fn_t  point(point_index_t pindex) const;
};

} // namespace hedge
