
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <easylogging++.h>

#include "hedge.hpp"

INITIALIZE_EASYLOGGINGPP;

TEST_CASE( "An index can be created and assigned a value from indexes of the same type.", "[index_types]") {
  hedge::edge_index_t def;
  hedge::edge_index_t e1(1, 1);
  hedge::edge_index_t e2 = e1;

  REQUIRE(def.offset == 0);
  REQUIRE(def.generation == 0);

  REQUIRE(e1.offset == e2.offset);
  REQUIRE(e1.generation == e2.generation);
}

TEST_CASE( "An index can be compared with other indexes of the same type.", "[index_types]") {
  hedge::edge_index_t def;
  hedge::edge_index_t e1(1, 1);
  hedge::edge_index_t e2 = e1;
  hedge::edge_index_t e3(20, 0);

  REQUIRE(def != e1);
  REQUIRE(e1 == e2);

  REQUIRE(e3 > e2);
  REQUIRE(e2 < e3);
}

TEST_CASE( "Edges can be created and updated.", "[edges]") {
  hedge::edge_t edge;
  hedge::vertex_index_t vert(3, 0);
  edge.vertex_index = vert;
  REQUIRE(edge.vertex_index == vert);
}

TEST_CASE("Edge function sets can be given null input when needed and can be checked.", "[function_sets]") {
  hedge::edge_fn_t bad(nullptr, hedge::edge_index_t());
  REQUIRE_FALSE(bad);
}

TEST_CASE("Vertex function sets can be given null input when needed and can be checked.", "[function_sets]") {
  hedge::vertex_fn_t bad(nullptr, hedge::vertex_index_t());
  REQUIRE_FALSE(bad);
}

TEST_CASE("Face function sets can be given null input when needed and can be checked.", "[function_sets]") {
  hedge::face_fn_t bad(nullptr, hedge::face_index_t());
  REQUIRE_FALSE(bad);
}

TEST_CASE( "The default basic_mesh_t is of the expected number of elements", "[mesh]" ) {
  hedge::mesh_t mesh;

  REQUIRE(mesh.kernel->point_count() == 1);
  REQUIRE(mesh.kernel->vertex_count() == 1);
  REQUIRE(mesh.kernel->face_count() == 1);
  REQUIRE(mesh.kernel->edge_count() == 1);

  // The mesh should report the count as the user of the api would expect
  REQUIRE(mesh.point_count() == 0);
  REQUIRE(mesh.vertex_count() == 0);
  REQUIRE(mesh.face_count() == 0);
  REQUIRE(mesh.edge_count() == 0);
}

TEST_CASE( "Removing an element doesn't invalidate other cells", "[kernel_operations]" ) {
  hedge::mesh_t mesh;

  auto pindex0 = mesh.kernel->emplace(hedge::point_t(0.f, 0.f, 0.f));
  auto pindex1 = mesh.kernel->emplace(hedge::point_t(1.f, 0.f, 0.f));
  auto pindex2 = mesh.kernel->emplace(hedge::point_t(0.f, 1.f, 0.f));
  auto pindex3 = mesh.kernel->emplace(hedge::point_t(0.f, 0.f, 1.f));

  REQUIRE(mesh.point_count() == 4);
  REQUIRE(pindex1.offset == 2);
  REQUIRE(pindex2.offset == 3);

  hedge::point_t* p = mesh.kernel->get(pindex1);
  REQUIRE(p->generation == 0);
  REQUIRE(p->position.x == 1.f);
  REQUIRE(p->position.y == 0.f);
  REQUIRE(p->position.z == 0.f);
  REQUIRE(p);

  mesh.kernel->remove(pindex1);
  REQUIRE(mesh.point_count() == 3);
  REQUIRE(p->generation == 1);
  p = mesh.kernel->get(pindex1);
  REQUIRE_FALSE(p);

  mesh.kernel->remove(pindex2);
  REQUIRE(mesh.point_count() == 2);

  pindex2 = mesh.kernel->emplace(hedge::point_t(1.f, 1.f, 1.f));;
  REQUIRE(mesh.point_count() == 3);

  pindex1 = mesh.kernel->emplace(hedge::point_t(0.f, 1.0f, 1.0f));
  REQUIRE(mesh.point_count() == 4);
  p = mesh.kernel->get(pindex1);
  REQUIRE(p);
  REQUIRE(p->generation == 1);
  REQUIRE(p->position.x == 0.f);
  REQUIRE(p->position.y == 1.f);
  REQUIRE(p->position.z == 1.f);

  REQUIRE(pindex1.offset == 3);
  REQUIRE(pindex2.offset == 2);
}

SCENARIO( "Essential kernel operations allow you to create a triangle.", "[kernel_operations]" ) {

  GIVEN("An empty mesh.") {
    hedge::mesh_t mesh;
    REQUIRE(mesh.point_count() == 0);

    WHEN("We add three points to the mesh") {
      mesh.kernel->emplace(hedge::point_t(0.f, 0.f, 0.f));
      auto pi1 = mesh.kernel->emplace(hedge::point_t(1.f, 0.f, 0.f));
      mesh.kernel->emplace(hedge::point_t(1.f, 1.f, 0.f));

      REQUIRE(mesh.kernel->point_count() == 4);

      THEN( "We can retrieve one of the previously created points." ) {
        REQUIRE(pi1);
        hedge::point_t* p1 = mesh.kernel->get(pi1);
        REQUIRE(p1 != nullptr);
        REQUIRE(p1->position.x == 1.f);
        REQUIRE(p1->position.y == 0.f);
        REQUIRE(p1->position.z == 0.f);
      }
    }

    WHEN("We add three points and three vertices") {
      auto pindex0 = mesh.kernel->emplace(hedge::point_t(0.f, 0.f, 0.f));
      auto pindex1 = mesh.kernel->emplace(hedge::point_t(1.f, 0.f, 0.f));
      auto pindex2 = mesh.kernel->emplace(hedge::point_t(0.f, 1.f, 0.f));

      auto add_vertex =
        [&mesh](hedge::point_index_t pindex) -> hedge::vertex_index_t {
          hedge::vertex_t vert;
          vert.point_index = pindex;
          auto vindex = mesh.kernel->emplace(std::move(vert));
          auto* pv = mesh.kernel->get(vindex);
          REQUIRE(pv);
          return vindex;
        };
      auto vindex0 = add_vertex(pindex0);
      auto vindex1 = add_vertex(pindex1);
      auto vindex2 = add_vertex(pindex2);

      REQUIRE(mesh.kernel->vertex_count() == 4);

      THEN("We can create three edges and a face.") {
        auto add_edge =
          [&mesh](hedge::vertex_index_t vindex) -> hedge::edge_index_t {
            hedge::edge_t edge;
            edge.vertex_index = vindex;
            auto eindex = mesh.kernel->emplace(std::move(edge));
            auto* vert = mesh.kernel->get(vindex);
            REQUIRE(vert);
            vert->edge_index = eindex;
            return eindex;
          };

        auto update_edge =
          [&mesh](hedge::edge_index_t eindex, hedge::edge_index_t prev, hedge::edge_index_t next,
                  hedge::face_index_t findex)
          {
            auto* edge = mesh.kernel->get(eindex);
            REQUIRE(edge);
            edge->prev_index = prev;
            edge->next_index = next;
            edge->face_index = findex;
          };

        auto eindex0 = add_edge(vindex0);
        auto eindex1 = add_edge(vindex1);
        auto eindex2 = add_edge(vindex2);

        hedge::face_index_t findex0;
        {
          hedge::face_t face;
          face.edge_index = eindex0;
          findex0 = mesh.kernel->emplace(std::move(face));
        }

        REQUIRE(mesh.kernel->face_count() == 2);
        REQUIRE(mesh.kernel->edge_count() == 4);

        auto* face = mesh.kernel->get(findex0);
        REQUIRE(face);
        face->edge_index = eindex0;

        update_edge(eindex0, eindex2, eindex1, findex0);
        update_edge(eindex1, eindex0, eindex2, findex0);
        update_edge(eindex2, eindex1, eindex0, findex0);

        auto check_edge =
          [&mesh](hedge::edge_index_t eindex, hedge::edge_index_t prev,
                  hedge::edge_index_t next,
                  hedge::vertex_index_t vindex, hedge::face_index_t findex)
          {
            auto* edge = mesh.kernel->get(eindex);
            REQUIRE(edge);
            REQUIRE(edge->face_index == findex);
            REQUIRE(edge->prev_index == prev);
            REQUIRE(edge->next_index == next);
            REQUIRE(edge->vertex_index == vindex);
          };

        check_edge(eindex0, eindex2, eindex1, vindex0, findex0);
        check_edge(eindex1, eindex0, eindex2, vindex1, findex0);
        check_edge(eindex2, eindex1, eindex0, vindex2, findex0);
      }
    }
  }
}

TEST_CASE( "High level mesh api can easily add geometry", "[mesh_operations]" ) {
  hedge::mesh_t mesh;

  auto findex0 = mesh.add_triangle(
    hedge::point_t(0.f, 0.f, 0.f),
    hedge::point_t(1.f, 0.f, 0.f),
    hedge::point_t(0.f, 1.f, 0.f)
    );

  REQUIRE(mesh.edge_count() == 3);
  REQUIRE(mesh.point_count() == 3);
  REQUIRE(mesh.vertex_count() == 3);
  REQUIRE(mesh.face_count() == 1);

  auto edge = mesh.face(findex0).edge();
  REQUIRE(edge.is_boundary());
  REQUIRE(edge.next().is_boundary());
  REQUIRE(edge.prev().is_boundary());
  REQUIRE(edge.index().offset == 1);
  REQUIRE(edge.next().index().offset == 2);
  REQUIRE(edge.next().next().index().offset == 3);
  REQUIRE(edge.prev().index().offset == 3);
}
