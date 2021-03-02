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

#include "mars_routing_common/utility/TimeInfo.h"


template<class Profile>
mars::routing::common::utility::TimeInfo<Profile>::TimeInfo()
{

}

template<class Profile>
mars::routing::common::utility::TimeInfo<Profile>::TimeInfo(double pEntryMotion, double pAngularMotion,
                                                   double pExitMotion, Profile pEntryProfile,
                                                   Profile pAngularProfile, Profile pExitProfile)
    : mEntryMotion(pEntryMotion), mAngularMotion(pAngularMotion), mExitMotion(pExitMotion),
      mEntryProfile(pEntryProfile), mRotationProfile(pAngularProfile), mExitProfile(pExitProfile)
{
}

template <class Profile>
double mars::routing::common::utility::TimeInfo<Profile>::getExitMotion() const { return mExitMotion; }

template <class Profile>
void mars::routing::common::utility::TimeInfo<Profile>::setExitMotion(double pExitMotion)
{
  this->mExitMotion = pExitMotion;
}

template <class Profile>
double mars::routing::common::utility::TimeInfo<Profile>::getEntryMotion() const { return mEntryMotion; }

template <class Profile>
void mars::routing::common::utility::TimeInfo<Profile>::setEntryMotion(double pEntryMotion)
{
  this->mEntryMotion = pEntryMotion;
}

template <class Profile>
double mars::routing::common::utility::TimeInfo<Profile>::getAngularMotion() const { return mAngularMotion; }

template <class Profile>
void mars::routing::common::utility::TimeInfo<Profile>::setAngularMotion(double pAngularMotion)
{
  this->mAngularMotion = pAngularMotion;
}


template <class Profile> Profile mars::routing::common::utility::TimeInfo<Profile>::getExitProfile() const
{
  return this->mExitProfile;
}

template <class Profile>
void mars::routing::common::utility::TimeInfo<Profile>::setExitProfile(const Profile& pExitProfile)
{
  this->mExitProfile = pExitProfile;
}

template <class Profile> Profile mars::routing::common::utility::TimeInfo<Profile>::getEntryProfile() const
{
  return mEntryProfile;
}

template <class Profile>
void mars::routing::common::utility::TimeInfo<Profile>::setEntryProfile(const Profile& pEntryProfile)
{
  this->mEntryProfile = pEntryProfile;
}

template <class Profile>
Profile mars::routing::common::utility::TimeInfo<Profile>::getRotationProfile() const
{
  return this->mRotationProfile;
}

template <class Profile>
void mars::routing::common::utility::TimeInfo<Profile>::setRotationProfile(const Profile& pRotationProfile)
{
  this->mRotationProfile = pRotationProfile;
}


template class mars::routing::common::utility::TimeInfo<mars::routing::common::utility::AffineProfile>;
