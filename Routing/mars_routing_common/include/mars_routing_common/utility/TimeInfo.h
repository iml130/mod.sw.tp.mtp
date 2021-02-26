#ifndef TIMEINFO_H
#define TIMEINFO_H

#include "mars_routing_common/utility/AffineProfile.h"

namespace mars
{
namespace routing
{
namespace common
{
namespace utility
{
template <class Profile>
class TimeInfo
{
public:
  TimeInfo();

  TimeInfo(double pEntryMotion, double pAngularMotion, double pExitMotion, Profile pEntryProfile, Profile pAngularProfile, Profile pExitProfile);

  Profile getRotationProfile() const;
  void setRotationProfile(const Profile &pRotationProfile);

  Profile getEntryProfile() const;
  void setEntryProfile(const Profile &pEntryProfile);

  Profile getExitProfile() const;
  void setExitProfile(const Profile &pExitProfile);

  double getAngularMotion() const;
  void setAngularMotion(double pAngularMotion);

  double getEntryMotion() const;
  void setEntryMotion(double pEntryMotion);

  double getExitMotion() const;
  void setExitMotion(double pExitMotion);

private:
  Profile mRotationProfile;
  Profile mEntryProfile;
  Profile mExitProfile;
  double mAngularMotion;
  double mEntryMotion;
  double mExitMotion;
};
}  // namespace utility
}  // namespace common
}  // namespace routing
}  // namespace mars

#endif // TIMEINFO_H
