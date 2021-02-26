#ifndef MARS_COMMON_VALUE_H
#define MARS_COMMON_VALUE_H

#include <string>

namespace mars
{
namespace common
{
class Value
{
public:
  /**
   * @brief Value Constructs an object of this class
   * @param valueType Type description of the value
   * @param value Value of type "valueType"
   */
  Value(std::string& valueType, std::string& value);

  /**
   * @brief getValueType Gets the valuetype
   * @return The valuetype of this object
   */
  std::string getValueType() const;

  /**
   * @brief setValueType Sets the valuetype
   * @param valueType Next valuetype
   */
  void setValueType(const std::string& valueType);

  /**
   * @brief getValue Gets the value of this object
   * @return The value of this object
   */
  std::string getValue() const;

  /**
   * @brief setValue Sets the value of this object
   * @param value Next value
   */
  void setValue(const std::string& value);

private:
  /**
   * @brief mValueType Type description of the value
   */
  std::string mValueType;

  /**
   * @brief mValue Value of type "mValueType"
   */
  std::string mValue;
};
}  // namespace common
}  // namespace mars

#endif  // MARS_COMMON_VALUE_H
