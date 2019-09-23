
#include <iostream>
#include <limits>
#include <vector>
#include <tuple>
#include <optional>
#include <functional>

#undef NDEBUG
#include <cassert>
#define NDEBUG

namespace hedge {

using index_t = size_t;
using generation_t = uint32_t;

const index_t INVALID_INDEX = std::numeric_limits<index_t>::max();
const generation_t IGNORED_GENERATION = 0;

struct half_edge_t;
struct vertex_t;
struct face_t;


/**
 * At the end of the day, handles are just fancy indexes. Their main
 * purpose is to be a type safe version of an index so that the associated
 * element container will only work with a specific handle type.
 *
 * Their other job is to encode an optional 'generation' field which is used
 * to invalidate a handle when an element was modified.
 *
 * Generally the handle tries to look and act like a simple value type.
 */
template <typename TElement>
class handle_t {
  index_t index_ = INVALID_INDEX;
  generation_t generation_ = IGNORED_GENERATION;
public:
  using element_t = TElement;

  handle_t() = default;

  handle_t(index_t const index)
    : index_(index)
  {}

  explicit handle_t(index_t const index, generation_t const generation)
    : index_(index)
    , generation_(generation)
  {}

  [[nodiscard]] index_t index() const { return index_; }
  [[nodiscard]] generation_t generation() const { return generation_; }

  void reset() {
    index_ = INVALID_INDEX;
    generation_ = IGNORED_GENERATION;
  }

  bool operator!=(handle_t const &other) const {
    return !(*this == other);
  }

  bool operator==(handle_t const &other) const {
    auto index_eq = index_ == other.index_;
    if (generation_ != IGNORED_GENERATION &&
        other.generation_ != IGNORED_GENERATION) {
      return index_eq && generation_ == other.generation_;
    }
    return index_eq;
  }

  operator bool() const noexcept {
    return is_valid(*this);
  }

  friend bool is_valid(handle_t const& handle) {
    return handle.index_ != INVALID_INDEX;
  }
};


struct half_edge_handle_t : handle_t<half_edge_t> {
  using handle_t::handle_t;
  static half_edge_handle_t const invalid;
};
half_edge_handle_t const half_edge_handle_t::invalid;


/**
 */
template <typename THandle>
struct mesh_element_t {
  using ref_t = std::reference_wrapper<typename THandle::element_t>;
  using cref_t = std::reference_wrapper<const typename THandle::element_t>;
  using handle_t = THandle;

  uint16_t tag = 0;
  generation_t generation = 1;

  mesh_element_t() = default;

  mesh_element_t(mesh_element_t const&& other) noexcept
    : tag(other.tag)
    , generation(other.generation)
  {}

  mesh_element_t(mesh_element_t const& other) = delete;
  mesh_element_t& operator=(mesh_element_t const& other) = delete;
  mesh_element_t& operator=(mesh_element_t && other) noexcept = delete;
  ~mesh_element_t() = default;
};


/**
 */
struct half_edge_t : mesh_element_t<half_edge_handle_t> {
  using mesh_element_t::mesh_element_t;
  half_edge_handle_t next = handle_t::invalid;
  half_edge_handle_t prev = handle_t::invalid;
  half_edge_handle_t adjacent = handle_t::invalid;
};


/**
 */
template<typename TElement>
class element_buffer_t {
  std::vector<TElement> elements_;
public:
  [[nodiscard]] bool is_valid_handle(typename TElement::handle_t handle) const {
    return handle.index() < elements_.size();
  }

  typename TElement::handle_t push(TElement&& element) {
    auto index = elements_.size();
    auto& elem = elements_.emplace_back(element);
    return typename TElement::handle_t(index, elem.generation);
  }

  template <typename... TArgs>
  typename TElement::handle_t add(TArgs... args) {
    auto index = elements_.size();
    auto& elem = elements_.emplace_back(TElement(std::forward<TArgs>(args)...));
    return typename TElement::handle_t(index, elem.generation);
  }

  TElement& get(typename TElement::handle_t handle) {
    return elements_.at(handle.index());
  }
};

using half_edge_buffer_t = element_buffer_t<half_edge_t>;


/**
 */
template<typename TElement>
class proxy_t {
  using buffer_t = element_buffer_t<TElement>;
  buffer_t* buffer_;
  typename TElement::handle_t handle_;
protected:
  [[nodiscard]] buffer_t *buffer() const {
    return buffer_;
  }

public:
  proxy_t()
    : buffer_(nullptr)
    , handle_()
  {}

  proxy_t(buffer_t* buffer, typename TElement::handle_t handle)
    : buffer_(buffer)
    , handle_(handle)
  {}

  [[nodiscard]] typename TElement::handle_t handle() const {
    return handle_;
  }

  [[nodiscard]] std::optional<typename TElement::ref_t> element() const {
    return buffer_->get(handle_);
  }

  bool operator==(proxy_t<TElement> const &other) const {
    return buffer_ == other.buffer_ && handle_ == other.handle_;
  }

  bool operator!=(proxy_t<TElement> const &other) const {
    return !(*this == other);
  }

  inline operator bool() const noexcept {
    return is_valid(*this);
  }

  friend bool is_valid(proxy_t<TElement> const& proxy) {
    return proxy.buffer_ != nullptr &&
      is_valid(proxy.handle_) &&
      proxy.buffer_->is_valid_handle(proxy.handle_);
  }
};


/**
 */
class half_edge_proxy_t : public proxy_t<half_edge_t> {
public:
  using proxy_t::proxy_t;
  half_edge_proxy_t next() {
    if (!buffer()->is_valid_handle(handle())) {
      return invalid;
    }
    auto elem = element();
    if (elem) {
      return half_edge_proxy_t(buffer(), elem.value().get().next);
    }
    return invalid;
  }

  void connect_to(half_edge_proxy_t const &other) {
    auto elem = element();
    auto other_elem = other.element();
    if (elem && other_elem) {
      auto& prev = elem.value().get();
      auto& next = other_elem.value().get();
      prev.next = other.handle();
      next.prev = handle();
    }
  }

  half_edge_proxy_t prev() {
    if (!buffer()->is_valid_handle(handle())) {
      return invalid;
    }
    auto elem = element();
    if (elem) {
      return half_edge_proxy_t(buffer(), elem.value().get().prev);
    }
    return invalid;
  }

  half_edge_proxy_t adjacent() {
    if (!buffer()->is_valid_handle(handle())) {
      return invalid;
    }
    auto elem = element();
    if (elem) {
      return half_edge_proxy_t(buffer(), elem.value().get().adjacent);
    }
    return invalid;
  }

  static half_edge_proxy_t const invalid;
};

half_edge_proxy_t const half_edge_proxy_t::invalid;


} // namespace hedge

////////////////////////////////////////////////////////////////////////////////
// The rest is here just to make sure it compiles and generally works as
// a simple example.

hedge::half_edge_proxy_t make_edge_pair(hedge::half_edge_buffer_t &buffer) {
  auto eh0 = buffer.add();
  auto eh1 = buffer.add();
  auto& e0 = buffer.get(eh0);
  auto& e1 = buffer.get(eh1);
  e0.adjacent = eh1;
  e1.adjacent = eh0;
  return hedge::half_edge_proxy_t(&buffer, eh0);
}

int main(int argc, char* argv[]) {
  using hedge::half_edge_handle_t;
  using hedge::half_edge_t;
  using hedge::half_edge_buffer_t;
  using hedge::half_edge_proxy_t;

  half_edge_handle_t bad_hnd;
  half_edge_handle_t good_hnd = 1;

  assert(bad_hnd == half_edge_handle_t::invalid);
  assert(bad_hnd != good_hnd);

  assert(static_cast<bool>(good_hnd) == true);
  assert(is_valid(good_hnd) == true);
  assert(static_cast<bool>(bad_hnd) == false);
  assert(is_valid(bad_hnd) == false);

  if (good_hnd) {
    bad_hnd = good_hnd;
    assert(bad_hnd);
  }

  half_edge_buffer_t edge_buffer;

  auto e0 = make_edge_pair(edge_buffer);
  auto e1 = make_edge_pair(edge_buffer);
  auto e2 = make_edge_pair(edge_buffer);

  half_edge_proxy_t bad_proxy;
  assert(is_valid(bad_proxy) == false);
  assert(bad_proxy == half_edge_proxy_t::invalid);

  half_edge_proxy_t pe0(&edge_buffer, e0.handle());
  assert(is_valid(pe0) == true);

  auto ape0 = e0.adjacent();
  assert(ape0);

  e0.connect_to(e1);
  e1.connect_to(e2);
  e2.connect_to(e0);

  e0.adjacent().connect_to(e2.adjacent());
  e2.adjacent().connect_to(e1.adjacent());
  e1.adjacent().connect_to(e0.adjacent());

  assert(e0.next() == e1);
  assert(e0.prev() == e2);
  assert(e1.next() == e2);
  assert(e1.prev() == e0);
  assert(e2.next() == e0);
  assert(e2.prev() == e1);
  assert(e0.next().next() == e2);
  assert(e0.next().next().adjacent() == e2.adjacent());
  assert(e0.prev().adjacent() == e2.adjacent());
  assert(e0.prev().adjacent() != e2);

  return 0;
}
