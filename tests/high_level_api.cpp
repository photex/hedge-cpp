
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <easylogging++.h>

#include "hedge.hpp"

INITIALIZE_EASYLOGGINGPP;

SCENARIO("High level mesh api can \"easily\" build a quad", "[mesh_operations]")
{
  GIVEN("An empty mesh and builder")
  {
    hedge::mesh_t         mesh;
    hedge::mesh_builder_t builder(mesh);

    WHEN("We add 4 points")
    {
      auto* kernel  = mesh.kernel();
      auto  pindex0 = kernel->insert(hedge::point_t(0.f, 0.f, 0.f));
      auto  pindex1 = kernel->insert(hedge::point_t(2.f, 0.f, 0.f));
      auto  pindex2 = kernel->insert(hedge::point_t(0.f, 2.f, 0.f));
      auto  pindex3 = kernel->insert(hedge::point_t(2.f, 2.f, 0.f));

      THEN("We can add the first triangle")
      {
        auto findex0 = builder.add_triangle(pindex0, pindex1, pindex2);
        REQUIRE(mesh.edge_count() == 6);
        REQUIRE(mesh.point_count() == 4);
        REQUIRE(mesh.vertex_count() == 3);
        REQUIRE(mesh.face_count() == 1);

        auto face = mesh.face(findex0);
        // REQUIRE(face.area() == 1.73205);
        auto edge = face.root_edge();
        REQUIRE(edge.is_boundary());
        REQUIRE(edge.next().is_boundary());
        REQUIRE(edge.prev().is_boundary());
        REQUIRE(edge.index().offset == 1);
        REQUIRE(edge.next().index().offset == 3);
        REQUIRE(edge.next().next().index().offset == 5);
        REQUIRE(edge.prev().index().offset == 5);

        THEN("We can add a second triangle, extending the second edge")
        {
          auto root_eindex2 = edge.next().index();
          auto findex1      = builder.add_triangle(root_eindex2, pindex3);

          REQUIRE(mesh.edge_count() == 10);
          REQUIRE(mesh.vertex_count() == 6);
          REQUIRE(mesh.face_count() == 2);

          edge = mesh.face(findex1).root_edge();
          REQUIRE_FALSE(edge.is_boundary());
          REQUIRE(edge.next().is_boundary());
          REQUIRE(edge.prev().is_boundary());
          REQUIRE(edge.index().offset == 4);
          REQUIRE(edge.next().index().offset == 7);
          REQUIRE(edge.next().next().index().offset == 9);
          REQUIRE(edge.prev().index().offset == 9);
        }
      }
    }
  }
}
