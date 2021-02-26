#ifndef MARS_COMMON_RESULT_H
#define MARS_COMMON_RESULT_H

#include <string>
#include <cstdint>

namespace mars
{
namespace common
{
class Result
{
public:
  Result(int8_t resultStatus, std::string description);

  int8_t getResultStatus() const;
  void setResultStatus(const int8_t& resultStatus);

  std::string getDescription() const;
  void setDescription(const std::string& description);

private:
  int8_t mResultStatus;
  std::string mDescription;
};
}  // namespace common
}  // namespace mars

#endif  // MARS_COMMON_RESULT_H
