//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

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
