
#pragma once

#include <memory>
#include <mathfu/glsl_mappings.h>

namespace hedge {

struct edge_t; struct edge_index_t; class edge_fn_t;
struct face_t; struct face_index_t; class face_fn_t;
struct vertex_t; struct vertex_index_t; class vertex_fn_t;
struct point_t; struct point_index_t;

class kernel_t;
class mesh_t;

using position_t = mathfu::vec3;
using color_t = mathfu::vec4;
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
    return offset != 0;
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

  virtual void resolve(edge_index_t* index, edge_t** edge) const = 0;
  virtual void resolve(face_index_t* index, face_t** face) const = 0;
  virtual void resolve(point_index_t* index, point_t** point) const = 0;
  virtual void resolve(vertex_index_t* index, vertex_t** vertex) const = 0;
};

////////////////////////////////////////////////////////////////////////////////
// Our principle element structures.

enum class element_status_t : uint16_t {
  active = 0x0000,
  inactive = 0x8000
};

struct element_t {
  element_status_t status;
  uint16_t tag;
  uint32_t generation;
  element_t();
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

struct point_t : element_t {
  mathfu::vec3 position;

  point_t();
  point_t(float x, float y, float z);
};

////////////////////////////////////////////////////////////////////////////////
// "Function sets" proxy the mesh and elements and provide an easy access api

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
    : _kernel(kernel), _index(index)
  {}

  explicit operator bool() const noexcept {
    return _kernel != nullptr && (bool)_index && element() != nullptr;
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

/**
 * Provide whatever functions needed to perform basic mesh operations at
 * a higher level.
 */
class mesh_modifier_t {
protected:
  mesh_t& _mesh;
  mesh_modifier_t(mesh_t& mesh);

  vertex_index_t make_vertex(point_index_t pindex);
  void update_vertex(vertex_index_t vindex, edge_index_t eindex);

  edge_index_t make_edge(vertex_index_t vindex);
  edge_index_t make_edge(vertex_index_t vindex, edge_index_t prev_eindex);
  void set_next_edge(edge_index_t prev_eindex, edge_index_t next_eindex);
  void set_prev_edge(edge_index_t prev_eindex, edge_index_t next_eindex);
  void connect_edges(edge_index_t prev_eindex, edge_index_t next_eindex);
};

/**
 * A simple interface for constructing edge loops originating at the
 * specified point.
 */
class edge_loop_builder_t : public mesh_modifier_t {
  edge_index_t _root_eindex;
  edge_index_t _last_eindex;
public:
  edge_loop_builder_t(mesh_t& mesh, point_index_t root_pindex);
  edge_loop_builder_t(mesh_t& mesh, edge_index_t root_eindex);
  bool add_point(point_index_t next_pindex);
  edge_index_t close();
};

/**
   Mesh can do a great deal of work on it's own as long as the kernel implements
   a couple of principle functions related to data storage.
 */
class mesh_t {
  uint16_t tag;
public:
  mesh_t();
  mesh_t(kernel_t::ptr_t&&);

  size_t point_count() const;
  size_t vertex_count() const;
  size_t edge_count() const;
  size_t face_count() const;

  edge_fn_t edge(edge_index_t index) const;
  face_fn_t face(face_index_t index) const;
  vertex_fn_t vertex(vertex_index_t index) const;

  point_t* point(offset_t offset) const;
  point_t* point(point_index_t pindex) const;
  point_t* point(vertex_index_t vindex) const;

  std::pair<point_t*, point_t*> points(edge_index_t eindex) const;

  point_index_t add_point(float x, float y, float z);
  edge_index_t add_edge(point_index_t p0, point_index_t p1);

  face_index_t add_triangle(point_t p0, point_t p1, point_t p2);
  face_index_t add_triangle(point_index_t pindex0, point_index_t pindex1, point_index_t pindex3);
  face_index_t add_triangle(edge_index_t eindex, point_index_t pindex);

  face_index_t add_face(edge_index_t root_eindex);

  kernel_t::ptr_t kernel;
};

} // namespace hedge
