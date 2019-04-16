# Hedge++

This is a small library that I'm just using for personal development and exploration.

The half-edge mesh data structure is pretty well discussed and probably used and built 
in a million better and more interesting ways than I'm doing here. But it seemed like a 
very fun and interesting subject to explore not only as the basis for creative applications, 
but also as a frame for "How would I make a nice modern C++ library for that?".

This is all super duper WIP but if you have feedback or are interested in it I'd love to talk shop.

```cpp
hedge::mesh_t mesh;
hedge::mesh_builder_t builder;

auto findex0 = builder.add_triangle(
  hedge::point_t(0.f, 0.f, 0.f),
  hedge::point_t(1.f, 0.f, 0.f),
  hedge::point_t(0.f, 1.f, 0.f)
  );
  
// This example will hopefully be improved someday. :)
```

## Core Ideas

- Strongly typed indices into mesh elements. 
- 4 principle mesh elements: `edge`, `vertex`, `face`, `point`.
- A kernel provides the primary storage and retrieval of elements interfaces.
- A high-level interface is provided by the mesh class.
- Traversing the mesh is made possible with function sets.
- Common mesh operations will be provided by a set of simple operators.

*The library is not thread safe yet*.

### Goals

TODO

