
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <easylogging++.h>

#include "hedge.hpp"

INITIALIZE_EASYLOGGINGPP;

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
