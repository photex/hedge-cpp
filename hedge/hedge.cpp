
#include "hedge.hpp"

#include <array>
#include <vector>
#include <queue>

#include <Eigen/Geometry>
#include <easylogging++.h>


namespace hedge {

/**
   Rather than create a bunch of preprocessor macros to prevent copypasta I decided
   to create a simple templated wrapper over std::vector which implements the
   requirements for element storage.

   This is used by `basic_kernel_t`. 
 */
template<typename TElement, typename TElementIndex>
class element_vector_t {
public:
  using collection_t = std::vector<TElement>;
  using free_cells_t =
    std::priority_queue<
      TElementIndex,
      std::vector<TElementIndex>,
      std::greater<>
    >;

  element_vector_t() {
    collection.emplace_back( TElement {} );
  }

  void reserve(size_t elements) {
    collection.reserve(elements);
  }

  size_t count() const {
    return collection.size() - free_cells.size();
  }

  TElement* get(TElementIndex index) const {
    TElement* element = get(index.offset);
    if (element != nullptr) {
      if (element->generation != index.generation) {
        LOG(WARNING) << "Generation mismatch for element: " << index.offset << ", " << index.generation;
        LOG(DEBUG) << "Offset: " << index.offset
                   << ", Generation " << index.generation << " != " << element->generation;
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
    if (free_cells.size()) {
      index = free_cells.top();
      free_cells.pop();
      element.generation = index.generation;
      auto* element_at_index = get(index.offset);
      (*element_at_index) = element;
    }
    else {
      index.offset = collection.size();
      index.generation = element.generation;
      collection.emplace_back(std::move(element));
    }
    return index;
  }

  void remove(TElementIndex index) {
    auto* element_at_index = get(index);
    if (element_at_index != nullptr) {
      element_at_index->generation++;
      element_at_index->status = element_status_t::inactive;
      index.generation++;
      free_cells.push(index);
    }
  }

  void swap(TElementIndex aindex, TElementIndex bindex) {
    auto* element_a = get(aindex);
    auto* element_b = get(bindex);
    if (element_a && element_b) {
      element_a->generation++;
      element_b->generation++;

      TElement temp = *element_a;
      *element_a = *element_b;
      *element_b = temp;
    }
  }

private:
  collection_t collection;
  free_cells_t free_cells;
};

///////////////////////////////////////////////////////////////////////////////////////

class basic_kernel_t : public kernel_t {
  element_vector_t<vertex_t, vertex_index_t> vertices;
  element_vector_t<face_t, face_index_t>     faces;
  element_vector_t<edge_t, edge_index_t>     edges;
  element_vector_t<point_t, point_index_t>   points;

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

  edge_index_t emplace(edge_t&& edge) override {
    return edges.emplace(std::move(edge));
  }
  face_index_t emplace(face_t&& face) override {
    return faces.emplace(std::move(face));
  }
  vertex_index_t emplace(vertex_t&& vertex) override {
    return vertices.emplace(std::move(vertex));
  }
  point_index_t emplace(point_t&& point) override {
    return points.emplace(std::move(point));
  }

  void remove(edge_index_t index) override {
    return edges.remove(index);
  }
  void remove(face_index_t index) override {
    return faces.remove(index);
  }
  void remove(vertex_index_t index) override {
    return vertices.remove(index);
  }
  void remove(point_index_t index) override {
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

  void resolve(edge_index_t* index, edge_t** edge) const override {
    *edge = edges.get(index->offset);
    index->generation = (*edge)->generation;
  }
  void resolve(face_index_t* index, face_t** face) const override {
    *face = faces.get(index->offset);
    index->generation = (*face)->generation;
  }
  void resolve(point_index_t* index, point_t** point) const override {
    *point = points.get(index->offset);
    index->generation = (*point)->generation;
  }
  void resolve(vertex_index_t* index, vertex_t** vert) const override {
    *vert = vertices.get(index->offset);
    index->generation = (*vert)->generation;
  }
};

// basic_kernel_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

mesh_modifier_t::mesh_modifier_t(mesh_t& mesh)
  : _mesh(mesh)
{}

vertex_index_t mesh_modifier_t::make_vertex(point_index_t pindex) {
  vertex_t vert;
  vert.point_index = pindex;
  return _mesh.kernel->emplace(std::move(vert));
}

void mesh_modifier_t::update_vertex(vertex_index_t vindex, edge_index_t eindex) {
  auto* vert = _mesh.kernel->get(vindex);
  if (vert) {
    vert->edge_index = eindex;
  }
}

edge_index_t mesh_modifier_t::make_edge(vertex_index_t vindex) {
  edge_t edge;
  edge.vertex_index = vindex;
  auto eindex = _mesh.kernel->emplace(std::move(edge));
  update_vertex(vindex, eindex);
  return eindex;
}

edge_index_t mesh_modifier_t::make_edge(vertex_index_t vindex, edge_index_t prev_index) {
  edge_t edge;
  edge.vertex_index = vindex;
  edge.prev_index = prev_index;
  auto eindex = _mesh.kernel->emplace(std::move(edge));
  set_next_edge(prev_index, eindex);
  return eindex;
}

void mesh_modifier_t::connect_edges(edge_index_t prev_eindex, edge_index_t next_eindex) {
  set_prev_edge(prev_eindex, next_eindex);
  set_next_edge(prev_eindex, next_eindex);
}

void mesh_modifier_t::set_next_edge(edge_index_t prev_index, edge_index_t next_index) {
  auto* prev = _mesh.kernel->get(prev_index);
  prev->next_index = next_index;
}

void mesh_modifier_t::set_prev_edge(edge_index_t prev_index, edge_index_t next_index) {
  auto* next = _mesh.kernel->get(next_index);
  next->prev_index = prev_index;
}

// mesh_modifier_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

edge_loop_builder_t::edge_loop_builder_t(mesh_t& mesh, point_index_t root_pindex)
  : mesh_modifier_t(mesh)
  , _root_eindex()
  , _last_eindex()
{
  auto vindex = make_vertex(root_pindex);
  _root_eindex = make_edge(vindex);
  _last_eindex = _root_eindex;
}

edge_loop_builder_t::edge_loop_builder_t(mesh_t& mesh, edge_index_t root_eindex)
  : mesh_modifier_t(mesh)
  , _root_eindex(root_eindex)
  , _last_eindex(root_eindex)
{}

bool edge_loop_builder_t::add_point(point_index_t next_pindex) {
  if (!_last_eindex) return false;
  auto vindex = make_vertex(next_pindex);
  _last_eindex = make_edge(vindex, _last_eindex);
  return true;
}

edge_index_t edge_loop_builder_t::close()  {
  connect_edges(_last_eindex, _root_eindex);
  _last_eindex.reset();
  return _root_eindex;
}

// edge_loop_builder_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

mesh_t::mesh_t()
  : tag(0)
  , kernel(new basic_kernel_t, [](kernel_t* k) { delete k; })
{}

mesh_t::mesh_t(kernel_t::ptr_t&& _kernel)
  : tag(0)
  , kernel(std::move(_kernel))
{}

size_t mesh_t::point_count() const {
  return kernel->point_count() - 1;
}
size_t mesh_t::vertex_count() const {
  return kernel->vertex_count() - 1;
}
size_t mesh_t::edge_count() const {
  return kernel->edge_count() - 1;
}
size_t mesh_t::face_count() const {
  return kernel->face_count() - 1;
}

edge_fn_t mesh_t::edge(edge_index_t index) const {
  return edge_fn_t(kernel.get(), index);
}

face_fn_t mesh_t::face(face_index_t index) const {
  return face_fn_t(kernel.get(), index);
}

vertex_fn_t mesh_t::vertex(vertex_index_t index) const {
  return vertex_fn_t(kernel.get(), index);
}

point_t* mesh_t::point(offset_t offset) const {
  point_index_t index(offset);
  point_t* p;
  kernel->resolve(&index, &p);
  return p;
}
point_t* mesh_t::point(point_index_t pindex) const {
  return kernel->get(pindex);
}
point_t* mesh_t::point(vertex_index_t vindex) const {
  auto* vert = kernel->get(vindex);
  return point(vert->point_index);
}

std::pair<point_t*, point_t*> mesh_t::points(edge_index_t eindex) const {
  auto* p0 = edge(eindex).vertex().point();
  auto* p1 = edge(eindex).next().vertex().point();
  return std::make_pair(p0, p1);
}

point_index_t mesh_t::add_point(float x, float y, float z) {
  return kernel->emplace(point_t(x, y, z));
}

edge_index_t mesh_t::add_edge(point_index_t pindex0, point_index_t pindex1) {
  vertex_t v0, v1;
  v0.point_index = pindex0;
  v1.point_index = pindex1;

  auto vindex0 = kernel->emplace(std::move(v0));
  auto vindex1 = kernel->emplace(std::move(v1));

  edge_t e0, e1;
  e0.vertex_index = vindex0;
  e1.vertex_index = vindex1;

  auto eindex0 = kernel->emplace(std::move(e0));
  auto eindex1 = kernel->emplace(std::move(e1));

  kernel->get(vindex0)->edge_index = eindex0;
  kernel->get(vindex1)->edge_index = eindex1;

  return eindex0;
}

face_index_t mesh_t::add_triangle(point_t p0, point_t p1, point_t p2) {
  auto pindex0 = kernel->emplace(std::move(p0));
  auto pindex1 = kernel->emplace(std::move(p1));
  auto pindex2 = kernel->emplace(std::move(p2));
  return add_triangle(pindex0, pindex1, pindex2);
}

face_index_t mesh_t::add_triangle(point_index_t pindex0, point_index_t pindex1, point_index_t pindex3) {
  edge_loop_builder_t loop(*this, pindex0);
  loop.add_point(pindex1);
  loop.add_point(pindex3);
  auto root_eindex = loop.close();
  return add_face(root_eindex);
}

face_index_t mesh_t::add_triangle(edge_index_t eindex, point_index_t pindex) {
  auto adjacent_index = edge(eindex).adjacent().index();
  edge_loop_builder_t loop(*this, adjacent_index);
  loop.add_point(edge(eindex).vertex().element()->point_index);
  loop.add_point(pindex);
  auto root_eindex = loop.close();
  return add_face(root_eindex);
}

face_index_t mesh_t::add_face(edge_index_t root_eindex) {
  tag++;

  face_t face;
  face.edge_index = root_eindex;
  auto findex = kernel->emplace(std::move(face));

  auto* elem = kernel->get(root_eindex);
  while( elem && elem->tag != tag ) {
    elem->tag = tag;
    elem->face_index = findex;
    elem = kernel->get(elem->next_index);
  }
  return findex;
}

// mesh_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

element_t::element_t()
  : status(element_status_t::active)
  , tag(0)
  , generation(0)
{}

point_t::point_t(float x, float y, float z)
  : element_t()
  , position(x, y, z)
{}

point_t::point_t()
  : element_t()
  , position(0.0f, 0.0f, 0.0f)
{}

///////////////////////////////////////////////////////////////////////////////

#define FN_GETTER(A, B)                           \
  auto *elem = element();                         \
  if (elem != nullptr)                            \
  {                                               \
    return A;                                     \
  }                                               \
  else                                            \
  {                                               \
    LOG(DEBUG) << "Returning empty function set"; \
    return B;                                     \
  }

#define MAKE_VERT_FN(INDEX) \
  FN_GETTER(vertex_fn_t(_kernel, INDEX), vertex_fn_t(_kernel, vertex_index_t()))

#define MAKE_FACE_FN(INDEX) \
  FN_GETTER(face_fn_t(_kernel, INDEX), face_fn_t(_kernel, face_index_t()))

#define MAKE_EDGE_FN(INDEX) \
  FN_GETTER(edge_fn_t(_kernel, INDEX), edge_fn_t(_kernel, edge_index_t()))

///////////////////////////////////////////////////////////////////////////////

vertex_fn_t edge_fn_t::vertex() const {
  MAKE_VERT_FN(elem->vertex_index)
}

face_fn_t edge_fn_t::face() const {
  MAKE_FACE_FN(elem->face_index)
}

edge_fn_t edge_fn_t::next() const {
  MAKE_EDGE_FN(elem->next_index)
}

edge_fn_t edge_fn_t::prev() const {
  MAKE_EDGE_FN(elem->prev_index)
}

edge_fn_t edge_fn_t::adjacent() const {
  MAKE_EDGE_FN(elem->adjacent_index)
}

bool edge_fn_t::is_boundary() const {
  auto* edge = element();
  if (edge != nullptr) {
    auto adjacent_face = adjacent().face();
    if (adjacent_face) {
      return false;
    }
  }
  return true;
}

// edge_fn_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

edge_fn_t vertex_fn_t::edge() const {
  MAKE_EDGE_FN(elem->edge_index)
}

point_t* vertex_fn_t::point() const {
  auto* vert = element();
  return _kernel->get(vert->point_index);
}

// vertex_fn_t
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

edge_fn_t face_fn_t::edge() const {
  MAKE_EDGE_FN(elem->edge_index)
}

float calc_area(point_t const* p0, point_t const* p1, point_t const* p2) {
  auto A = p1->position - p0->position;
  auto B = p2->position - p0->position;
  return A.cross(B).norm() / 2.0f;
}

vertex_fn_t next_vert(vertex_fn_t const& vert) {
  return vert.edge().next().vertex();
}

// TODO: tests
float face_fn_t::area() const {
  auto area = 0.0f;
  auto v0 = edge().vertex();
  auto v1 = next_vert(v0);
  auto v2 = next_vert(v1);
  while(v2 != v0) {
    area += calc_area(v0.point(), v1.point(), v2.point());
    v1 = v2;
    v2 = next_vert(v1);
  }
  return area;
}

// face_fn_t
///////////////////////////////////////////////////////////////////////////////

} // namespace hedge
