#include "mars_common/Tuple.h"

mars::common::Tuple::Tuple(std::string key, mars::common::Value value) : mKey(key), mValue(value)
{
}

std::string mars::common::Tuple::getKey() const
{
  return mKey;
}

void mars::common::Tuple::setKey(const std::string& key)
{
  mKey = key;
}

mars::common::Value mars::common::Tuple::getValue() const
{
  return mValue;
}

void mars::common::Tuple::setValue(const Value& value)
{
  mValue = value;
}
