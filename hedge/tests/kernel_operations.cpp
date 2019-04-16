
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <easylogging++.h>

#include "hedge.hpp"

INITIALIZE_EASYLOGGINGPP;

SCENARIO( "Adding and removing elements behaves as expected.", "[kernel_operations]" ) {

  GIVEN("A mesh with four initial points") {
    hedge::mesh_t mesh;
    auto pindex0 = mesh.kernel()->emplace(hedge::point_t(0.f, 0.f, 0.f));
    auto pindex1 = mesh.kernel()->emplace(hedge::point_t(1.f, 0.f, 0.f));
    auto pindex2 = mesh.kernel()->emplace(hedge::point_t(0.f, 1.f, 0.f));
    auto pindex3 = mesh.kernel()->emplace(hedge::point_t(0.f, 0.f, 1.f));

    THEN("We get the anticipated point count and cell offsets") {
      REQUIRE(mesh.point_count() == 4);
      REQUIRE(pindex0.offset == 1);
      REQUIRE(pindex1.offset == 2);
      REQUIRE(pindex2.offset == 3);
      REQUIRE(pindex3.offset == 4);
    }

    WHEN("We query for the second point") {
      auto* p = mesh.kernel()->get(pindex1);
      REQUIRE(p);

      THEN("We have the expected element generation and position values.") {
        REQUIRE(p->generation == 1);
        REQUIRE(p->position.x() == 1.f);
        REQUIRE(p->position.y() == 0.f);
        REQUIRE(p->position.z() == 0.f);
      }
    }

    WHEN("We remove the second point") {
      auto* p = mesh.kernel()->get(pindex1);
      mesh.kernel()->remove(pindex1);

      THEN("We have the expected point count.") {
        REQUIRE(mesh.point_count() == 3);
      }

      THEN("We see that the previous cell generation was incremented correctly.") {
        REQUIRE(p->generation == 2);
      }

      THEN("Querying for that point returns nullptr") {
        p = mesh.kernel()->get(pindex1);
        REQUIRE_FALSE(p);
      }
    }

    WHEN("We remove the second and third point") {
      mesh.kernel()->remove(pindex1);
      mesh.kernel()->remove(pindex2);

      THEN("Mesh point count reflects the change") {
        REQUIRE(mesh.point_count() == 2);
      }

      WHEN("We add a new third point") {
        pindex2 = mesh.kernel()->emplace(hedge::point_t(1.f, 1.f, 1.f));;

        THEN("Mesh point count reflects the change") {
          REQUIRE(mesh.point_count() == 3);
        }
        THEN("We get the original cell for the second point.") {
          REQUIRE(pindex2.offset == 2);
        }

        WHEN("We add a new fourth point") {
          pindex1 = mesh.kernel()->emplace(hedge::point_t(0.f, 1.0f, 1.0f));
          THEN("Mesh point count reflects the change") {
            REQUIRE(mesh.point_count() == 4);
          }
          THEN("We get the original cell for the third point.") {
            REQUIRE(pindex1.offset == 3);
          }
          THEN("We can query for this point and verify it is correct.") {
            auto* p = mesh.kernel()->get(pindex1);
            REQUIRE(p);
            REQUIRE(p->generation == 2);
            REQUIRE(p->position.x() == 0.f);
            REQUIRE(p->position.y() == 1.f);
            REQUIRE(p->position.z() == 1.f);
          }
        }
      }
    }
  }
}

SCENARIO( "Essential kernel operations allow you to create a triangle.", "[kernel_operations]" ) {

  GIVEN("An empty mesh.") {
    hedge::mesh_t mesh;
    REQUIRE(mesh.point_count() == 0);

    WHEN("We add three points to the mesh") {
      mesh.kernel()->emplace(hedge::point_t(0.f, 0.f, 0.f));
      auto pi1 = mesh.kernel()->emplace(hedge::point_t(1.f, 0.f, 0.f));
      mesh.kernel()->emplace(hedge::point_t(1.f, 1.f, 0.f));

      THEN( "mesh point count reports one less than the kernel point count." ) {
        REQUIRE(mesh.point_count() == 3);
        REQUIRE(mesh.kernel()->point_count() == 4);
      }

      THEN( "We can retrieve one of the previously created points." ) {
        REQUIRE(pi1);
        hedge::point_t* p1 = mesh.kernel()->get(pi1);
        REQUIRE(p1 != nullptr);
        REQUIRE(p1->position.x() == 1.f);
        REQUIRE(p1->position.y() == 0.f);
        REQUIRE(p1->position.z() == 0.f);
      }
    }

    WHEN("We add three points and three vertices") {
      auto pindex0 = mesh.kernel()->emplace(hedge::point_t(0.f, 0.f, 0.f));
      auto pindex1 = mesh.kernel()->emplace(hedge::point_t(1.f, 0.f, 0.f));
      auto pindex2 = mesh.kernel()->emplace(hedge::point_t(0.f, 1.f, 0.f));

      auto add_vertex =
        [&mesh](hedge::point_index_t pindex) -> hedge::vertex_index_t {
          hedge::vertex_t vert;
          vert.point_index = pindex;
          auto vindex = mesh.kernel()->emplace(std::move(vert));
          auto* pv = mesh.kernel()->get(vindex);
          REQUIRE(pv);
          return vindex;
        };
      auto vindex0 = add_vertex(pindex0);
      auto vindex1 = add_vertex(pindex1);
      auto vindex2 = add_vertex(pindex2);

      REQUIRE(mesh.kernel()->vertex_count() == 4);

      THEN("We can create three edges and a face.") {
        auto add_edge =
          [&mesh](hedge::vertex_index_t vindex) -> hedge::edge_index_t {
            hedge::edge_t edge;
            edge.vertex_index = vindex;
            auto eindex = mesh.kernel()->emplace(std::move(edge));
            auto* vert = mesh.kernel()->get(vindex);
            REQUIRE(vert);
            vert->edge_index = eindex;
            return eindex;
          };

        auto update_edge =
          [&mesh](
            hedge::edge_index_t eindex,
            hedge::edge_index_t prev,
            hedge::edge_index_t next,
            hedge::face_index_t findex)
          {
            auto* edge = mesh.kernel()->get(eindex);
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
          findex0 = mesh.kernel()->emplace(std::move(face));
        }

        REQUIRE(mesh.kernel()->face_count() == 2);
        REQUIRE(mesh.kernel()->edge_count() == 4);

        auto* face = mesh.kernel()->get(findex0);
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
            auto* edge = mesh.kernel()->get(eindex);
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
