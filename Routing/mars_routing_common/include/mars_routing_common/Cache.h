#ifndef MARS_ROUTING_COMMON_CACHE_H
#define MARS_ROUTING_COMMON_CACHE_H

#include <cstddef>
#include <mutex>
#include <unordered_map>

namespace mars
{
namespace routing
{
namespace common
{
template <typename Key, typename Value> class Cache
{
public:
  Cache(){};

  /**
   * @brief Finds given key or inserts a new value with default constructor.
   * @param pKey Key to search.
   * @return Found value, if key exists in cache. New value otherwise.
   */
  Value& operator[](const Key& pKey)
  {
    std::lock_guard<std::mutex>{mMutex};
    const auto& lCachedValue = mCache.find(pKey);

    if (lCachedValue == mCache.end())
    {
      const auto& lEmplacedValue = mCache.emplace(pKey, Value(pKey, false));
      return (*lEmplacedValue.first).second;
    }

    return lCachedValue->second;
  };

  /**
   * @brief Clears all cached data.
   * Resets it to initialization state.
   */
  void clear()
  {
    std::lock_guard<std::mutex>{mMutex};
    mCache.clear();
  };

  /**
   * @brief Checks if there is something in the cache with the given key
   * @param key Key after which should be checked
   * @return True if key is cached
   */
  bool isCached(const Key& key) const
  {
    std::lock_guard<std::mutex>{mMutex};
    return mCache.find(key) != mCache.end();
  };

  /**
   * @brief Returns the size of the current cache.
   * @return Amount of key-value entries in the cache.
   */
  size_t size() const
  {
    std::lock_guard<std::mutex>{mMutex};
    return mCache.size();
  };

private:
  std::unordered_map<Key, Value> mCache;

  mutable std::mutex mMutex;
};
} // namespace common
} // namespace routing
} // namespace mars

#endif // MARS_ROUTING_COMMON_CACHE_H
