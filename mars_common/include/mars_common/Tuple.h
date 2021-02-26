#ifndef MARS_COMMON_TUPLE_H
#define MARS_COMMON_TUPLE_H

#include <mars_common/Value.h>

#include <string>

namespace mars
{
namespace common
{
class Tuple
{
public:
  /**
   * @brief Tuple Constructs an object of this class
   * @param key Key of the value
   * @param value Value behind the key
   */
  Tuple(std::string key, mars::common::Value value);

  /**
   * @brief key Gets the key
   * @return The key
   */
  std::string getKey() const;

  /**
   * @brief setKey Sets the Key
   * @param key Next key value
   */
  void setKey(const std::string& key);

  /**
   * @brief value Gets the value
   * @return The value
   */
  mars::common::Value getValue() const;

  /**
   * @brief setValue Set the value
   * @param value Next value
   */
  void setValue(const Value& value);

private:
  /**
   * @brief mKey The key of this value
   */
  std::string mKey;

  /**
   * @brief mValue The value of this class
   */
  mars::common::Value mValue;
};
}  // namespace common
}  // namespace mars

#endif  // MARS_COMMON_TUPLE_H
