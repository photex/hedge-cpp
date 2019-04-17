
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

  auto* kernel = mesh.kernel();
  REQUIRE(kernel->point_count() == 1);
  REQUIRE(kernel->vertex_count() == 1);
  REQUIRE(kernel->face_count() == 1);
  REQUIRE(kernel->edge_count() == 1);

  // The mesh should report the count as the user of the api would expect
  REQUIRE(mesh.point_count() == 0);
  REQUIRE(mesh.vertex_count() == 0);
  REQUIRE(mesh.face_count() == 0);
  REQUIRE(mesh.edge_count() == 0);
}

TEST_CASE("The edge loop builder will not modify a mesh when given bad input.", "[builders]") {
  using hedge::mesh_t;
  using hedge::mesh_builder_t;
  using hedge::point_index_t;

  mesh_t mesh;
  mesh_builder_t builder(mesh);

  auto eindex = builder.start_edge_loop(point_index_t(0))
    .add_point(point_index_t(1))
    .add_point(point_index_t(2))
    .close();

  REQUIRE(mesh.point_count() == 0);
  REQUIRE(mesh.vertex_count() == 0);
  REQUIRE(mesh.face_count() == 0);
  REQUIRE(mesh.edge_count() == 0);
  REQUIRE_FALSE(eindex);
}
