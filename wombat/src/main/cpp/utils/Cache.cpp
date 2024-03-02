#include "utils/Cache.h"

#include "utils/Util.h"

template <typename T>
wom::utils::Cached<T>::Cached(std::function<T()> supplier)
    : m_supplier{supplier}, k_timestamp{wom::utils::now()}, k_cacheperiod{20_ms} {};

template <typename T>
wom::utils::Cached<T>::Cached(std::function<T()> supplier, units::second_t cacheperiod)
    : m_supplier{supplier}, k_timestamp{wom::utils::now()}, k_cacheperiod{cacheperiod} {};

template <typename T>
wom::utils::Cached<T>& wom::utils::Cached<T>::Update() {
  this->k_timestamp = wom::utils::now();
  this->m_value = m_supplier();
  return this;
}

template <typename T>
T& wom::utils::Cached<T>::GetValue() {
  if (IsStale()) {
    Update();
  }
  return m_value;
}

template <typename T>
bool wom::utils::Cached<T>::IsStale() {
  return wom::utils::now() - this->k_timestamp > k_cacheperiod;
}
