#include "mars_common/Result.h"

mars::common::Result::Result(int8_t resultStatus, std::string description)
  : mResultStatus(resultStatus), mDescription(description)
{
}

int8_t mars::common::Result::getResultStatus() const
{
  return mResultStatus;
}

void mars::common::Result::setResultStatus(const int8_t& resultStatus)
{
  mResultStatus = resultStatus;
}

std::string mars::common::Result::getDescription() const
{
  return mDescription;
}

void mars::common::Result::setDescription(const std::string& description)
{
  mDescription = description;
}
