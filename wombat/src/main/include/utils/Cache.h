#pragma once

#include <functional>

#include <units/time.h>

namespace wom {
namespace utils {

/* A class for a cached value that can be updated lazily */
template <typename T>
class Cached {
 public:
  /**
   * Creates a new Cached<T> with the given supplier. Sets the validity period to 20 milliseconds by default.
   *
   * @param supplier A function that sources the value.
   */
  Cached(std::function<T()> supplier);

  /**
   * Creates a new Cached<T> with the given supplier.
   *
   * @param supplier A function that sources the value.
   * @param dt The validity length of the stored value.
   */
  Cached(std::function<T()> supplier, units::second_t dt);

  Cached(Cached&&) = default;
  Cached& operator=(Cached&&) = default;

  /**
   * Updates the Cached class with the current timestamp and value.
   *
   * @return The updated class.
   */
  Cached<T>& Update();

  /**
   * Gets the cached value.
   */
  T& GetValue();

  /**
   * Checks whether or not the stored value is stale.
   */
  inline bool IsStale();

 private:
  T& m_value;
  std::function<T()> m_supplier;
  const units::second_t k_timestamp;
  const units::second_t k_dt;
};

}  // namespace utils
}  // namespace wom
