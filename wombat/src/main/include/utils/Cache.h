// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <units/time.h>

#include <functional>

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
  explicit Cached(std::function<T()> supplier);

  /**
   * Creates a new Cached<T> with the given supplier.
   *
   * @param supplier A function that sources the value.
   * @param cacheperiod The validity length of the stored value.
   */
  Cached(std::function<T()> supplier, units::second_t cacheperiod);

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
  const std::function<T()> m_supplier;
  const units::second_t k_timestamp;
  const units::second_t k_cacheperiod;
};

}  // namespace utils
}  // namespace wom
