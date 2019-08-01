
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <easylogging++.h>

#include "hedge.hpp"

INITIALIZE_EASYLOGGINGPP;

SCENARIO("Adding and removing elements behaves as expected.",
         "[kernel_operations]")
{

  GIVEN("A mesh with four initial points")
  {
    hedge::mesh_t mesh;
    auto* kernel = mesh.kernel();
    auto pindex0 = kernel->emplace(hedge::point_t(0.f, 0.f, 0.f));
    auto pindex1 = kernel->emplace(hedge::point_t(1.f, 0.f, 0.f));
    auto pindex2 = kernel->emplace(hedge::point_t(0.f, 1.f, 0.f));
    auto pindex3 = kernel->emplace(hedge::point_t(0.f, 0.f, 1.f));

    THEN("We get the anticipated point count and cell offsets")
    {
      REQUIRE(mesh.point_count() == 4);
      REQUIRE(pindex0.offset == 1);
      REQUIRE(pindex1.offset == 2);
      REQUIRE(pindex2.offset == 3);
      REQUIRE(pindex3.offset == 4);
    }

    WHEN("We query for the second point")
    {
      auto* p = kernel->get(pindex1);
      REQUIRE(p);

      THEN("We have the expected element generation and position values.")
      {
        REQUIRE(p->generation == 1);
        REQUIRE(p->position.x() == 1.f);
        REQUIRE(p->position.y() == 0.f);
        REQUIRE(p->position.z() == 0.f);
      }
    }

    WHEN("We remove the second point")
    {
      auto* p = kernel->get(pindex1);
      mesh.kernel()->remove(pindex1);

      THEN("We have the expected point count.")
      {
        REQUIRE(mesh.point_count() == 3);
      }

      THEN(
        "We see that the previous cell generation was incremented correctly.")
      {
        REQUIRE(p->generation == 2);
      }

      THEN("Querying for that point returns nullptr")
      {
        p = kernel->get(pindex1);
        REQUIRE_FALSE(p);
      }
    }

    WHEN("We remove the second and third point")
    {
      kernel->remove(pindex1);
      kernel->remove(pindex2);

      THEN("Mesh point count reflects the change")
      {
        REQUIRE(mesh.point_count() == 2);
      }

      WHEN("We add a new third point")
      {
        pindex2 = kernel->emplace(hedge::point_t(1.f, 1.f, 1.f));
        ;

        THEN("Mesh point count reflects the change")
        {
          REQUIRE(mesh.point_count() == 3);
        }
        THEN("We get the original cell for the second point.")
        {
          REQUIRE(pindex2.offset == 2);
        }

        WHEN("We add a new fourth point")
        {
          pindex1 = kernel->emplace(hedge::point_t(0.f, 1.0f, 1.0f));
          THEN("Mesh point count reflects the change")
          {
            REQUIRE(mesh.point_count() == 4);
          }
          THEN("We get the original cell for the third point.")
          {
            REQUIRE(pindex1.offset == 3);
          }
          THEN("We can query for this point and verify it is correct.")
          {
            auto* p = kernel->get(pindex1);
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

SCENARIO("Essential kernel operations allow you to create a triangle.",
         "[kernel_operations]")
{

  GIVEN("An empty mesh.")
  {
    hedge::mesh_t mesh;
    auto* kernel = mesh.kernel();
    REQUIRE(mesh.point_count() == 0);

    WHEN("We add three points to the mesh")
    {
      kernel->emplace(hedge::point_t(0.f, 0.f, 0.f));
      auto pi1 = kernel->emplace(hedge::point_t(1.f, 0.f, 0.f));
      kernel->emplace(hedge::point_t(1.f, 1.f, 0.f));

      THEN("mesh point count reports one less than the kernel point count.")
      {
        REQUIRE(mesh.point_count() == 3);
        REQUIRE(kernel->point_count() == 4);
      }

      THEN("We can retrieve one of the previously created points.")
      {
        REQUIRE(pi1);
        hedge::point_t* p1 = kernel->get(pi1);
        REQUIRE(p1 != nullptr);
        REQUIRE(p1->position.x() == 1.f);
        REQUIRE(p1->position.y() == 0.f);
        REQUIRE(p1->position.z() == 0.f);
      }
    }

    WHEN("We add three points and three vertices")
    {
      auto pindex0 = kernel->emplace(hedge::point_t(0.f, 0.f, 0.f));
      auto pindex1 = kernel->emplace(hedge::point_t(1.f, 0.f, 0.f));
      auto pindex2 = kernel->emplace(hedge::point_t(0.f, 1.f, 0.f));
      auto* p0     = kernel->get(pindex0);
      auto* p1     = kernel->get(pindex1);
      auto* p2     = kernel->get(pindex2);
      REQUIRE(p0);
      REQUIRE(p1);
      REQUIRE(p2);
      REQUIRE(p0->vertices.size() == 0);
      REQUIRE(p1->vertices.size() == 0);
      REQUIRE(p2->vertices.size() == 0);

      THEN("We can create three edges and a face.")
      {
        auto eindex0 = kernel->make_edge_pair();
        auto eindex1 = kernel->make_edge_pair();
        auto eindex2 = kernel->make_edge_pair();

        auto vindex0 = kernel->connect_edges(eindex0, pindex1, eindex1);
        REQUIRE(p1->vertices.find(vindex0) != p1->vertices.end());

        auto vindex1 = kernel->connect_edges(eindex1, pindex2, eindex2);
        REQUIRE(p2->vertices.find(vindex1) != p2->vertices.end());

        auto vindex2 = kernel->connect_edges(eindex2, pindex0, eindex0);
        REQUIRE(p0->vertices.find(vindex2) != p0->vertices.end());

        REQUIRE(kernel->vertex_count() == 4);
        REQUIRE(p0->vertices.size() == 1);
        REQUIRE(p1->vertices.size() == 1);
        REQUIRE(p2->vertices.size() == 1);

        auto findex0 = kernel->make_face(eindex0);
        REQUIRE(findex0);

        REQUIRE(kernel->face_count() == 2);
        REQUIRE(kernel->edge_count() == 7);

        auto* face = kernel->get(findex0);
        REQUIRE(face);
        REQUIRE(face->edges.size() == 3);
        REQUIRE(face->edges.find(eindex0) != face->edges.end());
        REQUIRE(face->edges.find(eindex1) != face->edges.end());
        REQUIRE(face->edges.find(eindex2) != face->edges.end());

        auto check_edge =
          [&mesh](hedge::edge_index_t eindex, hedge::edge_index_t prev,
                  hedge::edge_index_t next, hedge::vertex_index_t vindex,
                  hedge::face_index_t findex) {
            auto* edge      = mesh.kernel()->get(eindex);
            auto* prev_edge = mesh.kernel()->get(prev);
            auto* next_edge = mesh.kernel()->get(next);
            REQUIRE(edge);
            REQUIRE(prev_edge);
            REQUIRE(next_edge);
            REQUIRE(edge->face_index == findex);
            REQUIRE(edge->prev_index == prev);
            REQUIRE(edge->next_index == next);
            REQUIRE(edge->vertex_index == vindex);
          };

        check_edge(eindex0, eindex2, eindex1, vindex2, findex0);
        check_edge(eindex1, eindex0, eindex2, vindex0, findex0);
        check_edge(eindex2, eindex1, eindex0, vindex1, findex0);
      }
    }
  }
}
