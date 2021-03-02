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
