#include "mars_common/Value.h"

mars::common::Value::Value(std::string& valueType, std::string& value)
{
  this->mValueType = valueType;
  this->mValue = value;
}

std::string mars::common::Value::getValueType() const
{
  return mValueType;
}

void mars::common::Value::setValueType(const std::string& valueType)
{
  mValueType = valueType;
}

std::string mars::common::Value::getValue() const
{
  return mValue;
}

void mars::common::Value::setValue(const std::string& value)
{
  mValue = value;
}
