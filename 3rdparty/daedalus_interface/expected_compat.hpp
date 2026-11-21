#pragma once

#if __has_include(<expected>) && defined(__cpp_lib_expected) && __cpp_lib_expected >= 202202L
#include <expected>
namespace talos::ipc {
template<typename T, typename E>
using Expected = std::expected<T, E>;
template<typename E>
constexpr auto make_unexpected(E&& e) {
    return std::unexpected(std::forward<E>(e));
}
} // namespace talos::ipc
#else
#include <utility>
#include <variant>
namespace talos::ipc {
template<typename E>
struct Unexpected {
    E err;
};
template<typename E>
constexpr Unexpected<std::decay_t<E>> make_unexpected(E&& e) {
    return Unexpected<std::decay_t<E>>{std::forward<E>(e)};
}
template<typename T, typename E>
class Expected {
public:
    Expected(T&& v) : data_(std::in_place_index<0>, std::move(v)) {}
    Expected(const T& v) : data_(std::in_place_index<0>, v) {}
    Expected(Unexpected<E> u) : data_(std::in_place_index<1>, std::move(u.err)) {}

    [[nodiscard]] bool has_value() const noexcept { return data_.index() == 0; }
    [[nodiscard]] explicit operator bool() const noexcept { return has_value(); }

    T& operator*() & noexcept { return std::get<0>(data_); }
    const T& operator*() const& noexcept { return std::get<0>(data_); }
    T&& operator*() && noexcept { return std::get<0>(std::move(data_)); }

    T* operator->() noexcept { return &std::get<0>(data_); }
    const T* operator->() const noexcept { return &std::get<0>(data_); }

    E& error() & noexcept { return std::get<1>(data_); }
    const E& error() const& noexcept { return std::get<1>(data_); }

private:
    std::variant<T, E> data_;
};
} // namespace talos::ipc
#endif
