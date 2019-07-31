
#include "hedge.hpp"

#include <array>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>
#include <easylogging++.h>

namespace hedge {

face_index_t const face_index_t::invalid;
vertex_index_t const vertex_index_t::invalid;
edge_index_t const edge_index_t::invalid;
point_index_t const point_index_t::invalid;

face_fn_t const face_fn_t::invalid(nullptr, face_index_t::invalid);
vertex_fn_t const vertex_fn_t::invalid(nullptr, vertex_index_t::invalid);
edge_fn_t const edge_fn_t::invalid(nullptr, edge_index_t::invalid);
point_fn_t const point_fn_t::invalid(nullptr, point_index_t::invalid);

/**
   Rather than create a bunch of preprocessor macros to prevent copypasta I
   decided to create a simple templated wrapper over std::vector which
   implements the requirements for element storage.

   This is used by `basic_kernel_t`.
 */
template <typename TElement, typename TElementIndex>
class element_vector_t
{
public:
  using collection_t = std::vector<TElement>;
  using free_cells_t =
    std::priority_queue<TElementIndex, std::vector<TElementIndex>,
                        std::greater<>>;

  element_vector_t() {
    collection.emplace_back(TElement{});
  }

  void reserve(size_t elements) {
    collection.reserve(elements);
  }

  size_t count() const {
    return collection.size() - free_cells.size();
  }

  TElement* get(TElementIndex index) const {
    TElement* element = get(index.offset);
    if (element != nullptr && index.generation != 0) {
      if (element->generation != index.generation) {
        LOG(WARNING) << "Generation mismatch for element: " << index.offset
                     << ", " << index.generation;
        LOG(DEBUG) << "Offset: " << index.offset << ", Generation "
                   << index.generation << " != " << element->generation;
        element = nullptr;
      }
    }
    return element;
  }

  TElement* get(offset_t offset) const {
    TElement* element = nullptr;
    if (offset < collection.size()) {
      element = (TElement*)collection.data() + offset;
    }
    else {
      LOG(ERROR) << "Offset requested exceeded element current storage size: "
                 << offset << " > " << collection.size();
    }
    return element;
  }

  TElementIndex emplace(TElement&& element) {
    TElementIndex index;
    if (!free_cells.empty()) {
      index = free_cells.top();
      free_cells.pop();
      element.generation     = index.generation;
      auto* element_at_index = get(index.offset);
      (*element_at_index)    = element;
    }
    else {
      auto offset = collection.size();
      assert(offset < std::numeric_limits<offset_t>::max());
      index.offset     = static_cast<offset_t>(offset);
      index.generation = element.generation;
      collection.emplace_back(std::forward<TElement>(element));
    }
    return index;
  }

  void remove(TElementIndex index) {
    auto* element_at_index = get(index);
    if (element_at_index != nullptr) {
      element_at_index->status = element_status_t::inactive;
      element_at_index->generation++;
      auto max_generation = std::numeric_limits<generation_t>::max();
      if (element_at_index->generation == max_generation) {
        element_at_index->generation = 1;
      }
      index.generation = element_at_index->generation;
      free_cells.push(index);
    }
  }

  // TODO: Add tests for this
  void swap(TElementIndex aindex, TElementIndex bindex) {
    auto* element_a = get(aindex);
    auto* element_b = get(bindex);
    if (element_a && element_b) {
      element_a->generation++;
      element_b->generation++;

      TElement temp = *element_a;
      *element_a    = *element_b;
      *element_b    = temp;
    }
  }

private:
  collection_t collection;
  free_cells_t free_cells;
};

///////////////////////////////////////////////////////////////////////////////////////

edge_index_t kernel_t::make_edge_pair() {
  auto eindex0 = insert(edge_t());
  auto eindex1 = insert(edge_t());

  auto* e0           = get(eindex0);
  auto* e1           = get(eindex1);
  e0->adjacent_index = eindex1;
  e1->adjacent_index = eindex0;

  return eindex0;
}

// FIXME: If given certain inputs we'd end up with invalid face data left in
// memory
face_index_t kernel_t::make_face(edge_index_t root_eindex) {
  auto* elem = get(root_eindex);
  if (elem == nullptr) {
    LOG(ERROR) << "Invalid root edge specified.";
    return face_index_t::invalid;
  }

  auto const findex = emplace(face_t());
  auto* face        = get(findex);
  face->root_edge_index = root_eindex;

  auto current_index = root_eindex;
  while (current_index) {
    elem = get(current_index);
    if (elem == nullptr) {
      LOG(ERROR) << "Disconnected edge loop. Unable to build face.";
      return face_index_t::invalid;
    }
    face->edges.insert(current_index);
    elem->face_index = findex;

    auto next_eindex = elem->next_index;

    // prevent an infinite loop when some edge is connected to itself
    if (next_eindex == current_index) {
      LOG(ERROR)
        << "A face requires a connected edge loop of at least 3 edges. "
           "Self-connected edges are not supported.";
      return face_index_t::invalid;
    }
    // terminate our loop once we reach the root edge
    if (next_eindex == root_eindex) {
      LOG(DEBUG) << "Completed edge loop.";
      break;
    }

    current_index = next_eindex;
  }

  return findex;
}

vertex_index_t kernel_t::connect_edges(
  edge_index_t eindex,
  point_index_t pindex,
  edge_index_t next_eindex
) {
  if (!eindex) {
    LOG(ERROR) << "Invalid in edge index specified. Unable to create "
                  "associated vertex.";
    return vertex_index_t::invalid;
  }
  if (!pindex) {
    LOG(ERROR) << "Invalid point index specified. Unable to create vertex.";
    return vertex_index_t::invalid;
  }
  if (!next_eindex) {
    LOG(ERROR) << "Invalid out edge index specified. Unable to create "
                  "associated vertex.";
    return vertex_index_t::invalid;
  }

  auto* edge      = get(eindex);
  auto* point     = get(pindex);
  auto* next_edge = get(next_eindex);

  if (edge == nullptr) {
    LOG(ERROR) << "Invalid edge specified for incoming edge.";
    return vertex_index_t::invalid;
  }

  if (point == nullptr) {
    LOG(ERROR) << "Invalid point specified.";
    return vertex_index_t::invalid;
  }

  if (next_edge == nullptr) {
    LOG(ERROR) << "Invalid edge specified for outgoing edge.";
    return vertex_index_t::invalid;
  }

  vertex_t vertex;
  vertex.point_index = pindex;
  vertex.edge_index  = next_eindex;
  auto vindex        = emplace(std::move(vertex));
  if (!vindex) {
    LOG(ERROR)
      << "Failed to create associated vertex. Edges can not be connected.";
    return vertex_index_t::invalid;
  }

  edge->next_index = next_eindex;
  next_edge->vertex_index = vindex;
  next_edge->prev_index = eindex;
  point->vertices.insert(vindex);

  return vindex;
}

///////////////////////////////////////////////////////////////////////////////////////

class basic_kernel_t : public kernel_t
{
  element_vector_t<vertex_t, vertex_index_t> vertices;
  element_vector_t<face_t, face_index_t> faces;
  element_vector_t<edge_t, edge_index_t> edges;
  element_vector_t<point_t, point_index_t> points;

public:
  edge_t* get(edge_index_t index) override {
    return edges.get(index);
  }
  face_t* get(face_index_t index) override {
    return faces.get(index);
  }
  vertex_t* get(vertex_index_t index) override {
    return vertices.get(index);
  }
  point_t* get(point_index_t index) override {
    return points.get(index);
  }

  edge_index_t insert(edge_t edge) override {
    return edges.emplace(std::move(edge));
  }

  face_index_t insert(face_t face) override {
    return faces.emplace(std::move(face));
  }

  vertex_index_t insert(vertex_t vertex) override {
    return vertices.emplace(std::move(vertex));
  }

  point_index_t insert(point_t point) override {
    return points.emplace(std::move(point));
  }

  edge_index_t emplace(edge_t&& edge) override {
    return edges.emplace(std::forward<edge_t>(edge));
  }
  face_index_t emplace(face_t&& face) override {
    return faces.emplace(std::forward<face_t>(face));
  }
  vertex_index_t emplace(vertex_t&& vertex) override {
    return vertices.emplace(std::forward<vertex_t>(vertex));
  }
  point_index_t emplace(point_t&& point) override {
    return points.emplace(std::forward<point_t>(point));
  }

  void remove(edge_index_t const index) override {
    return edges.remove(index);
  }
  void remove(face_index_t const index) override {
    return faces.remove(index);
  }
  void remove(vertex_index_t const index) override {
    return vertices.remove(index);
  }
  void remove(point_index_t const index) override {
    return points.remove(index);
  }

  size_t point_count() const override {
    return points.count();
  }

  size_t vertex_count() const override {
    return vertices.count();
  }

  size_t face_count() const override {
    return faces.count();
  }

  size_t edge_count() const override {
    return edges.count();
  }

  void defrag() override {
    LOG(WARNING) << "defrag not yet implemented.";
  }
};

// basic_kernel_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

mesh_builder_t::mesh_builder_t(mesh_t& mesh)
  : _mesh(mesh) {}

face_index_t mesh_builder_t::add_triangle(point_t p0, point_t p1, point_t p2) {
  auto* kernel = _mesh.kernel();
  auto pindex0 = kernel->emplace(std::move(p0));
  auto pindex1 = kernel->emplace(std::move(p1));
  auto pindex2 = kernel->emplace(std::move(p2));
  return add_triangle(pindex0, pindex1, pindex2);
}

face_index_t mesh_builder_t::add_triangle(
  point_index_t pindex0,
  point_index_t pindex1,
  point_index_t pindex2
) {
  auto root_eindex = start_edge_loop(pindex0)
    .add_point(pindex1)
    .add_point(pindex2)
    .close();
  return _mesh.kernel()->make_face(root_eindex);
}

face_index_t mesh_builder_t::add_triangle(edge_index_t eindex, point_t p0) {
  if (!eindex) {
    LOG(ERROR) << "Invalid edge index specified. Unable to add triangle from "
                  "adjacent edge.";
    return face_index_t::invalid;
  }
  auto pindex0 = _mesh.kernel()->emplace(std::move(p0));
  return add_triangle(eindex, pindex0);
}

face_index_t mesh_builder_t::add_triangle(
  edge_index_t eindex,
  point_index_t pindex
) {
  if (!eindex) {
    LOG(ERROR) << "Invalid edge index specified. Unable to add triangle from "
                  "adjacent edge.";
    return face_index_t::invalid;
  }
  auto adjacent_index = _mesh.edge(eindex).adjacent().index();
  auto root_eindex    = start_edge_loop(adjacent_index)
                       .add_point(_mesh.edge(eindex).vertex().point().index())
                       .add_point(pindex)
                       .close();
  return _mesh.kernel()->make_face(root_eindex);
}

edge_loop_builder_t mesh_builder_t::start_edge_loop(point_index_t pindex) {
  return {_mesh, pindex};
}

edge_loop_builder_t mesh_builder_t::start_edge_loop(edge_index_t eindex0) {
  return {_mesh, eindex0};
}

// mesh_builder_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

edge_loop_builder_t::edge_loop_builder_t(mesh_t& mesh, point_index_t pindex)
  : _mesh(mesh) {
  if (!pindex) {
    LOG(ERROR) << "Loop builder initialized with invalid root point index.";
  }
  else {
    auto* kernel = _mesh.kernel();
    _root_eindex = kernel->make_edge_pair();
    _last_eindex = _root_eindex;
    _root_pindex = pindex;
    _last_pindex = _root_pindex;
  }
}

edge_loop_builder_t::edge_loop_builder_t(mesh_t& mesh, edge_index_t root_eindex)
  : _mesh(mesh) {
  if (!root_eindex) {
    LOG(ERROR) << "Loop builder initialized with invalid root edge.";
  }
  else {
    auto edge = mesh.edge(root_eindex);
    if (edge.face()) {
      LOG(ERROR)
        << "Starting a new edge loop from an edge already assigned to a face.";
    }
    else {
      auto adjacent_edge = edge.adjacent();
      if (!adjacent_edge) {
        LOG(ERROR) << "Specified edge is missing its adjacent edge. Unable to "
                      "initialize loop builder.";
      }
      else {
        auto edge_points = adjacent_edge.points();
        _root_pindex     = edge_points[0].index();
        _last_pindex     = edge_points[1].index();
        if (!_root_pindex || !_last_pindex) {
          LOG(ERROR)
            << "Loop builder initialized with edge that has no valid points.";
          _root_pindex.reset();
          _last_pindex.reset();
        }
        else {
          _root_eindex = root_eindex;
          _last_eindex = _root_eindex;
        }
      }
    }
  }
}

edge_loop_builder_t& edge_loop_builder_t::add_point(point_index_t next_pindex) {
  if (!_last_pindex) {
    LOG(WARNING) << "Unable to add point to uninitialized loop builder.";
  }
  else {
    auto* kernel        = _mesh.kernel();
    auto current_eindex = kernel->make_edge_pair();
    auto vindex = kernel->connect_edges(_last_eindex, _last_pindex, current_eindex);
    _last_pindex = next_pindex;
    _last_eindex = current_eindex;
  }
  return *this;
}

edge_index_t edge_loop_builder_t::close() {
  if (!_last_pindex) {
    LOG(ERROR) << "Unable to close an uninitialized loop builder.";
  }
  else if (_last_pindex == _root_pindex) {
    LOG(ERROR) << "Unable to close loop when the previous point and the root "
                  "point are the same.";
  }
  else {
    auto* kernel = _mesh.kernel();
    auto vindex = kernel->connect_edges(_last_eindex, _last_pindex, _root_eindex);
    // Make sure we have no references to existing elements.
    _last_pindex.reset();
  }
  return _root_eindex;
}

// edge_loop_builder_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

mesh_t::mesh_t()
  : _tag(0)
  , _kernel(new basic_kernel_t, [](kernel_t* k) { delete k; }) {}

mesh_t::mesh_t(kernel_t::ptr_t&& _kernel)
  : _tag(0)
  , _kernel(std::move(_kernel)) {}

size_t mesh_t::point_count() const {
  return _kernel->point_count() - 1;
}
size_t mesh_t::vertex_count() const {
  return _kernel->vertex_count() - 1;
}
size_t mesh_t::edge_count() const {
  return _kernel->edge_count() - 1;
}
size_t mesh_t::face_count() const {
  return _kernel->face_count() - 1;
}

edge_fn_t mesh_t::edge(edge_index_t index) const {
  return edge_fn_t(_kernel.get(), index);
}

face_fn_t mesh_t::face(face_index_t index) const {
  return face_fn_t(_kernel.get(), index);
}

vertex_fn_t mesh_t::vertex(vertex_index_t index) const {
  return vertex_fn_t(_kernel.get(), index);
}

point_fn_t mesh_t::point(point_index_t pindex) const {
  return point_fn_t(_kernel.get(), pindex);
}

// mesh_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

point_t::point_t(float x, float y, float z)
  : element_t()
  , position(x, y, z) {}

point_t::point_t()
  : element_t()
  , position(0.0f, 0.0f, 0.0f) {}

point_t::point_t(position_t p)
  : element_t()
  , position(p) {}

///////////////////////////////////////////////////////////////////////////////

#define FN_GETTER(A, B)                                                        \
  auto* elem = element();                                                      \
  if (elem != nullptr) {                                                       \
    return A;                                                                  \
  }                                                                            \
  else {                                                                       \
    LOG(DEBUG) << "Returning empty function set";                              \
    return B;                                                                  \
  }

#define MAKE_VERT_FN(INDEX)                                                    \
  FN_GETTER(vertex_fn_t(_kernel, INDEX), vertex_fn_t::invalid)

#define MAKE_FACE_FN(INDEX)                                                    \
  FN_GETTER(face_fn_t(_kernel, INDEX), face_fn_t::invalid)

#define MAKE_EDGE_FN(INDEX)                                                    \
  FN_GETTER(edge_fn_t(_kernel, INDEX), edge_fn_t::invalid)

#define MAKE_POINT_FN(INDEX)                                                   \
  FN_GETTER(point_fn_t(_kernel, INDEX), point_fn_t::invalid)

///////////////////////////////////////////////////////////////////////////////

vertex_fn_t edge_fn_t::vertex() const {MAKE_VERT_FN(elem->vertex_index)}

face_fn_t edge_fn_t::face() const {MAKE_FACE_FN(elem->face_index)}

edge_fn_t edge_fn_t::next() const {MAKE_EDGE_FN(elem->next_index)}

edge_fn_t edge_fn_t::prev() const {MAKE_EDGE_FN(elem->prev_index)}

edge_fn_t edge_fn_t::adjacent() const {
  MAKE_EDGE_FN(elem->adjacent_index)
}

bool edge_fn_t::is_boundary() const {
  auto* edge = element();
  if (edge != nullptr) {
    if (!edge->face_index) {
      return true;
    }

    auto* adjacent_edge = adjacent().element();
    if (!adjacent_edge->face_index) {
      return true;
    }

    return false;
  }
  return true;
}

edge_points_t edge_fn_t::points() const {
  auto p0 = vertex().point();
  auto p1 = next().vertex().point();
  return {p0, p1};
}

edge_vertices_t edge_fn_t::vertices() const {
  auto v0 = vertex();
  auto v1 = next().vertex();
  return {v0, v1};
}

// edge_fn_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

edge_fn_t vertex_fn_t::edge() const {MAKE_EDGE_FN(elem->edge_index)}

point_fn_t vertex_fn_t::point() const {MAKE_POINT_FN(elem->point_index)}

normal_t vertex_fn_t::normal() const {
  assert(false); // unimplemented
  return normal_t();
}

void vertex_fn_t::calculate_normal() {
  assert(false); // unimplemented
}

// vertex_fn_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

edge_fn_t face_fn_t::root_edge() const {MAKE_EDGE_FN(elem->root_edge_index)}

edge_index_t::set_t const& face_fn_t::edges() const {
  auto* face = element();
  assert(face);
  return face->edges;
}

float calc_area(point_t const* p0, point_t const* p1, point_t const* p2) {
  auto A         = p1->position - p0->position;
  auto B         = p2->position - p0->position;
  auto magnitude = A.cross(B).norm();
  return magnitude / 2.0f;
}

float face_fn_t::area() const {
  auto area = 0.0f;
  auto v0   = root_edge().vertex();
  auto v1   = v0.edge().next().vertex();
  auto v2   = v1.edge().next().vertex();
  while (v2 != v0) {
    area += calc_area(v0.point().element(), v1.point().element(),
                      v2.point().element());
    v1 = v2;
    v2 = v1.edge().next().vertex();
  }
  return area;
}

// face_fn_t
///////////////////////////////////////////////////////////////////////////////

position_t point_fn_t::position() const {
  auto* elem = _kernel->get(_index);
  assert(elem);
  return elem->position;
}

normal_t point_fn_t::normal() const {
  assert(false); // unimplemented
  return normal_t();
}

void point_fn_t::calculate_normal() {
  assert(false); // unimplemented
}
} // namespace hedge
