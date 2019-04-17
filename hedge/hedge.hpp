
#pragma once

#include <memory>
#include <Eigen/Core>

namespace hedge {

struct edge_t; struct edge_index_t; class edge_fn_t;
struct face_t; struct face_index_t; class face_fn_t;
struct vertex_t; struct vertex_index_t; class vertex_fn_t;
struct point_t; struct point_index_t;

class kernel_t;
class mesh_t;
class mesh_builder_t;
class edge_loop_builder_t;

using position_t = Eigen::Vector3f;
using color_t = Eigen::Vector4f;
using offset_t = size_t;
using generation_t = size_t;

/**
   Using a strong index type instead of a bare pointer or generic integer index
   allows you to potentially re-use cells (if the kernel implements support for
   it) and can be more easily or directly validated.
 */
enum class index_type_t : unsigned char {
  vertex, edge, face, point, unsupported
};
template<index_type_t TIndexType = index_type_t::unsupported>
struct index_t {
  offset_t offset;
  generation_t generation;

  explicit index_t() noexcept
    : offset(0)
    , generation(0)
  {}

  explicit index_t(const offset_t o) noexcept
    : offset(o)
    , generation(0)
  {}

  explicit index_t(const offset_t o, const generation_t g) noexcept
    : offset(o)
    , generation(g)
  {}

  void reset() {
    offset = 0;
    generation = 0;
  }

  bool operator !=(const index_t& other) const {
    return !(*this == other);
  }
  bool operator ==(const index_t& other) const {
    return offset == other.offset && generation == other.generation;
  }

  friend bool operator< (const index_t& lhs, const index_t& rhs) {
    return lhs.offset < rhs.offset;
  }
  friend bool operator> (const index_t& lhs, const index_t& rhs) {
    return rhs < lhs;
  }

  explicit operator bool() const noexcept {
    return offset > 0;
  }
};

// Discriminated types to assist in API design and reduce the potential for errors
// that can arise from using generic index types like plain integers and so on.
struct edge_index_t : index_t<index_type_t::edge> { using index_t::index_t; };
struct face_index_t : index_t<index_type_t::face> { using index_t::index_t; };
struct vertex_index_t : index_t<index_type_t::vertex> { using index_t::index_t; };
struct point_index_t : index_t<index_type_t::point> { using index_t::index_t; };

/**
   The mesh kernel implements/provides the fundamental storage and access operations.
 */
class kernel_t {
public:
  using ptr_t = std::unique_ptr<kernel_t, void(*)(kernel_t*)>;

  virtual ~kernel_t() = default;

  virtual edge_t* get(edge_index_t index) = 0;
  virtual face_t* get(face_index_t index) = 0;
  virtual vertex_t* get(vertex_index_t index) = 0;
  virtual point_t* get(point_index_t index) = 0;

  virtual edge_index_t insert(edge_t edge) = 0;
  virtual face_index_t insert(face_t face) = 0;
  virtual vertex_index_t insert(vertex_t vertex) = 0;
  virtual point_index_t insert(point_t point) = 0;

  virtual edge_index_t emplace(edge_t&& edge) = 0;
  virtual face_index_t emplace(face_t&& face) = 0;
  virtual vertex_index_t emplace(vertex_t&& vertex) = 0;
  virtual point_index_t emplace(point_t&& point) = 0;

  virtual void remove(edge_index_t index) = 0;
  virtual void remove(face_index_t index) = 0;
  virtual void remove(vertex_index_t index) = 0;
  virtual void remove(point_index_t index) = 0;

  virtual size_t point_count() const = 0;
  virtual size_t vertex_count() const = 0;
  virtual size_t face_count() const = 0;
  virtual size_t edge_count() const = 0;
};

////////////////////////////////////////////////////////////////////////////////
// Our principle element structures.

enum class element_status_t : uint16_t {
  active = 0x0000,
  inactive = 0x8000
};

struct element_t {
  element_status_t status = element_status_t::active;
  uint16_t tag = 0;
  generation_t generation = 1; // Using 1 as default to allow indexes with gen 0 to have another meaning.
};

struct edge_t : element_t {
  vertex_index_t vertex_index;
  face_index_t face_index;
  edge_index_t next_index;
  edge_index_t prev_index;
  edge_index_t adjacent_index;
};

struct face_t : element_t {
  edge_index_t edge_index;
};

struct vertex_t : element_t {
  point_index_t point_index;
  edge_index_t edge_index;
};

// TODO: how can we make point attributes configurable
struct point_t : element_t {
  position_t position;

  point_t();
  point_t(float x, float y, float z);
};

////////////////////////////////////////////////////////////////////////////////
// Mesh element proxies to allow easy traversal.

/**
   We have this simple templated base class which allows functions that request
   an alement to return an "empty" proxy. An alternative to this would be to
   use std::optional but it seems like this would be sufficient for now.
 */
template<typename TIndex, typename TElement>
class element_fn_t {
protected:
  kernel_t* _kernel;
  TIndex _index;
public:
  explicit element_fn_t(kernel_t* kernel, TIndex index)
    : _kernel(kernel)
    , _index(index)
  {}

  explicit operator bool() const noexcept {
    return _kernel != nullptr && (bool)_index && element() != nullptr;
  }

  bool operator==(element_fn_t const& other) const {
    return _index == other._index;
  }

  bool operator!=(element_fn_t const& other) const {
    return _index != other._index;
  }

  TElement* element() const {
    if (_kernel != nullptr) {
      return _kernel->get(_index);
    }
    else {
      return nullptr;
    }
  }

  TIndex index() {
    return _index;
  }
};

class edge_fn_t : public element_fn_t<edge_index_t, edge_t> {
public:
  using element_fn_t::element_fn_t;

  vertex_fn_t vertex() const;
  face_fn_t face() const;
  edge_fn_t next() const;
  edge_fn_t prev() const;
  edge_fn_t adjacent() const;

  bool is_boundary() const;
};

class face_fn_t : public element_fn_t<face_index_t, face_t> {
public:
  using element_fn_t::element_fn_t;

  edge_fn_t edge() const;
  float area() const;
};

class vertex_fn_t : public element_fn_t<vertex_index_t, vertex_t> {
public:
  using element_fn_t::element_fn_t;

  edge_fn_t edge() const;
  point_t* point() const;
};

////////////////////////////////////////////////////////////////////////////////

namespace utils {
edge_index_t make_edge(kernel_t* kernel);
face_index_t make_face(kernel_t* kernel, edge_index_t root_eindex);
vertex_index_t make_vertex(kernel_t* kernel, edge_index_t eindex, point_index_t pindex);
void connect_edges(
  kernel_t* kernel,
  edge_index_t prev_eindex,
  point_index_t pindex,
  edge_index_t next_eindex
  );
} // namespace utils

/**
 * TODO
 */
class mesh_builder_t {
protected:
  mesh_t& _mesh;
public:
  explicit mesh_builder_t(mesh_t& mesh);

  face_index_t add_triangle(point_t p0, point_t p1, point_t p2);
  face_index_t add_triangle(point_index_t pindex0, point_index_t pindex1, point_index_t pindex2);
  face_index_t add_triangle(edge_index_t eindex, point_t p0);
  face_index_t add_triangle(edge_index_t eindex, point_index_t pindex);

  edge_loop_builder_t start_edge_loop(point_index_t pindex);
  edge_loop_builder_t start_edge_loop(edge_index_t eindex0);
};

/**
 * A simple interface for constructing edge loops originating at the
 * specified point.
 */
class edge_loop_builder_t {
  mesh_t& _mesh;
  edge_index_t _root_eindex;
  edge_index_t _last_eindex;
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
 * TODO
 */
class mesh_t {
  uint16_t _tag;
  kernel_t::ptr_t _kernel;
public:
  mesh_t();
  explicit mesh_t(kernel_t::ptr_t&&);

  uint16_t next_tag() {
    return ++_tag;
  }

  kernel_t* kernel() {
    return _kernel.get();
  }

  size_t point_count() const;
  size_t vertex_count() const;
  size_t edge_count() const;
  size_t face_count() const;

  edge_fn_t edge(edge_index_t index) const;
  face_fn_t face(face_index_t index) const;
  vertex_fn_t vertex(vertex_index_t index) const;

  point_t* point(point_index_t pindex) const;
  point_t* point(vertex_index_t vindex) const;

  std::tuple<point_index_t, point_index_t> points(edge_index_t eindex) const;
};

} // namespace hedge
