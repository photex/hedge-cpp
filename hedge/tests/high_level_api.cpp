
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <easylogging++.h>

#include "hedge.hpp"

INITIALIZE_EASYLOGGINGPP;

TEST_CASE( "High level mesh api can easily add geometry", "[mesh_operations]" ) {
  hedge::mesh_t mesh;

  hedge::mesh_builder_t builder(mesh);

  auto findex0 = builder.add_triangle(
    hedge::point_t(0.f, 0.f, 0.f),
    hedge::point_t(2.f, 0.f, 0.f),
    hedge::point_t(0.f, 2.f, 0.f)
    );

  REQUIRE(mesh.edge_count() == 6);
  REQUIRE(mesh.point_count() == 3);
  REQUIRE(mesh.vertex_count() == 3);
  REQUIRE(mesh.face_count() == 1);

  auto face = mesh.face(findex0);
  //REQUIRE(face.area() == 1.73205);
  auto edge = face.edge();
  REQUIRE(edge.is_boundary());
  REQUIRE(edge.next().is_boundary());
  REQUIRE(edge.prev().is_boundary());
  REQUIRE(edge.index().offset == 1);
  REQUIRE(edge.next().index().offset == 3);
  REQUIRE(edge.next().next().index().offset == 5);
  REQUIRE(edge.prev().index().offset == 5);

  auto findex1 = builder.add_triangle(
    hedge::edge_index_t(1), hedge::point_t(1.f, 1.f, 2.0f)
    );

  REQUIRE(mesh.edge_count() == 10);
  REQUIRE(mesh.point_count() == 4);
  REQUIRE(mesh.vertex_count() == 6);
  REQUIRE(mesh.face_count() == 2);

  edge = mesh.face(findex1).edge();
  REQUIRE_FALSE(edge.is_boundary());
  REQUIRE(edge.next().is_boundary());
  REQUIRE(edge.prev().is_boundary());
  REQUIRE(edge.index().offset == 2);
  REQUIRE(edge.next().index().offset == 7);
  REQUIRE(edge.next().next().index().offset == 9);
  REQUIRE(edge.prev().index().offset == 9);
}
