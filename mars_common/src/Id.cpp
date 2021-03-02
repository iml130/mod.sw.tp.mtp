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

#include "mars_common/Id.h"

#include <algorithm>
#include <boost/uuid/uuid_io.hpp>

// Default 32 character string for invalid ids
static const boost::array<uint8_t, 16UL> INVALID_ID = {0, 0, 0, 0, 0, 0, 0, 0,
                                                       0, 0, 0, 0, 0, 0, 0, 0};
static const std::string INVALID_ID_STRING = "INVALID+++++++++++++++++++++++++";

static const std::wstring DNS_NAMESPACE_UUID = L"6ba7b810-9dad-11d1-80b4-00c04fd430c8";

boost::uuids::random_generator mars::common::Id::mUUIDRandomGen = boost::uuids::random_generator();
boost::uuids::string_generator mars::common::Id::mUUIDStringGen = boost::uuids::string_generator();
boost::uuids::name_generator mars::common::Id::mUUIDNameGen =
    boost::uuids::name_generator(mUUIDStringGen(DNS_NAMESPACE_UUID));

static const std::string EXCEPTION_MSG_ALREADY_INITIALIZED = "Id is already initialized!";
static const std::string EXCEPTION_MSG_NOT_INITIALIZED = "Id was not initialized!";
static const std::string EXCEPTION_MSG_WRONG_ID_STRING_FORMAT =
    "Id has too many or too few signs! At least 16 characters are necessary; 20, if uuid blocks "
    "are seperated by '-'!";
static const std::string EXCEPTION_MSG_INVALID_ID =
    "UUID cannot be initialized as [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]!";
static const std::string EXCEPTION_MSG_UNSUPPORTED_ID_CREATION_TYPE =
    "Unsupported id creation type (mars::common::Id::UUIDCreationType)";

static const int UUID_STRING_LENGTH = 16;
static const int UUID_HEXDEC_NOT_SPLITTED_LENGTH = 32;
static const int UUID_HEXDEC_SPLITTED_LENGTH = 36;

mars::common::Id::Id()
{
  this->mIsInitialized = false;
  this->mDescription = "";
}

mars::common::Id::Id(const mars_common_msgs::Id pMsg)
{
  if (pMsg.uuid == INVALID_ID)
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_INVALID_ID);
  }

  this->mUUID = pMsg.uuid;
  this->mDescription = pMsg.description;
  this->mIsInitialized = true;
}

mars::common::Id::Id(const UUID& pUUID, const std::string& pDescription)
{
  this->mUUID = pUUID;
  this->mDescription = pDescription;
  this->mIsInitialized = true;
}

mars::common::Id::Id(const boost::array<uint8_t, 16>& pUUID, const std::string& pDescription)
{
  this->mUUID = pUUID;
  this->mDescription = pDescription;
  this->mIsInitialized = true;
}

mars::common::Id::Id(const std::string& pUUID, const std::string& pDescription,
                     UUIDCreationType pUUIDCreationType)
{
  this->mIsInitialized = false;
  this->initialize(pUUID, pDescription, pUUIDCreationType);
}

void mars::common::Id::initialize() noexcept(false)
{
  if (this->mIsInitialized)
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_ALREADY_INITIALIZED);
  }

  boost::uuids::uuid lUUID = this->mUUIDRandomGen();
  std::copy(lUUID.begin(), lUUID.end(), this->mUUID.begin());

  this->mIsInitialized = true;
}

void mars::common::Id::initialize(const UUID& pUUID, const std::string& pDescription) noexcept(false)
{
  if (this->mIsInitialized)
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_ALREADY_INITIALIZED);
  }

  this->mUUID = pUUID;
  this->mDescription = pDescription;
  this->mIsInitialized = true;
}

void mars::common::Id::initialize(
    const std::string& pUUID, const std::string& pDescription,
    UUIDCreationType pUUIDCreationType) noexcept(false)
{
  boost::uuids::uuid lTmpBoostUUID;

  if (mIsInitialized)
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_ALREADY_INITIALIZED);
  }

  if ((pUUIDCreationType == Id::CREATE_FROM_UUID) && (pUUID != INVALID_ID_STRING))
  {
    if ((pUUID.length() == UUID_HEXDEC_SPLITTED_LENGTH) ||
        (pUUID.length() == UUID_HEXDEC_NOT_SPLITTED_LENGTH))
    {
      lTmpBoostUUID = this->mUUIDStringGen(std::string(pUUID));
      std::copy(lTmpBoostUUID.begin(), lTmpBoostUUID.end(), this->mUUID.begin());

      this->mDescription = pDescription;
      this->mIsInitialized = true;
    }
    else if (pUUID.length() == UUID_STRING_LENGTH)
    {
      std::copy(pUUID.begin(), pUUID.end(), this->mUUID.begin());

      this->mDescription = pDescription;
      this->mIsInitialized = true;
    }
    else
    {
      throw mars::common::exception::SetParamException(EXCEPTION_MSG_WRONG_ID_STRING_FORMAT);
    }
  }
  else if ((pUUIDCreationType == Id::CREATE_FROM_STRING) || (pUUID == INVALID_ID_STRING))
  {
    lTmpBoostUUID = this->mUUIDNameGen(pUUID);

    std::copy(lTmpBoostUUID.begin(), lTmpBoostUUID.end(), this->mUUID.begin());

    this->mDescription = pDescription;
    this->mIsInitialized = true;
  }
  else
  {
    throw mars::common::exception::SetParamException(EXCEPTION_MSG_UNSUPPORTED_ID_CREATION_TYPE);
  }
}

void mars::common::Id::setDescription(const std::string& pDescription)
{
  this->mDescription = pDescription;
}

const mars::common::UUID& mars::common::Id::getUUID() const
    noexcept(false)
{
  if (!mIsInitialized)
  {
    throw exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  return this->mUUID;
}

std::string mars::common::Id::getUUIDAsString(mars::common::Id::UUIDFormat pFormat) const
    noexcept(false)
{
  std::string lString;
  boost::uuids::uuid lUUID;

  if (!mIsInitialized)
  {
    throw exception::NotInitializedException(EXCEPTION_MSG_NOT_INITIALIZED);
  }

  switch (pFormat)
  {
  case mars::common::Id::UUIDFormat::BYTEWISE:
    lString.resize(16);
    std::copy(this->mUUID.begin(), this->mUUID.end(), lString.begin());
    break;
  case mars::common::Id::UUIDFormat::HEXDEC:
    std::copy(this->mUUID.begin(), this->mUUID.end(), lUUID.begin());
    lString = boost::uuids::to_string(lUUID);
    lString.erase(std::remove(lString.begin(), lString.end(), '-'), lString.end());
    break;
  case mars::common::Id::UUIDFormat::HEXDEC_SPLIT:
    std::copy(this->mUUID.begin(), this->mUUID.end(), lUUID.begin());
    lString = boost::uuids::to_string(lUUID);
    break;
  default:
    break;
  }

  return lString;
}

std::string mars::common::Id::getDescription() const { return this->mDescription; }

bool mars::common::Id::isInitialized() const { return this->mIsInitialized; }

bool mars::common::Id::isValid() const
{
  return this->mIsInitialized && (*this != createInvalidId());
}

bool mars::common::Id::operator==(const Id& pOtherId) const
{
  return (this->getUUID() == pOtherId.getUUID());
}

bool mars::common::Id::operator!=(const Id& pOtherId) const
{
  return (this->getUUID() != pOtherId.getUUID());
}

mars_common_msgs::Id mars::common::Id::toMsg() const
{
  mars_common_msgs::Id msgId;

  msgId.uuid = this->getUUID();
  msgId.description = this->getDescription();

  return msgId;
}

std::vector<mars_common_msgs::Id>
mars::common::Id::convertToMsgId(const std::vector<mars::common::Id>& ids)
{
  std::vector<mars_common_msgs::Id> msgIds;

  msgIds.reserve(ids.size());

  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    msgIds.push_back(convertToMsgId(ids[i]));
  }

  return msgIds;
}

mars_common_msgs::Id mars::common::Id::convertToMsgId(const mars::common::Id& id)
{
  mars_common_msgs::Id msgId;

  msgId.uuid = id.getUUID();
  msgId.description = id.getDescription();

  return msgId;
}

std::vector<mars::common::Id> mars::common::Id::createId(const std::vector<std::string>& ids)
{
  std::vector<mars::common::Id> resIds;

  resIds.reserve(ids.size());

  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    resIds.push_back(mars::common::Id(ids[i]));
  }

  return resIds;
}

mars::common::Id mars::common::Id::createInvalidId()
{
  mars::common::Id tmp(INVALID_ID_STRING, "", mars::common::Id::UUIDCreationType::CREATE_FROM_STRING);

  return tmp;
}

std::string std::to_string(const mars::common::Id& pId)
{
  std::stringstream ss;
  mars::common::UUID lUUID = pId.getUUID();
  std::string lUUIDString(lUUID.begin(), lUUID.end());

  ss << "UUID: " << lUUIDString << ", Description: " << pId.getDescription();

  return ss.str();
}

std::string std::to_string(const mars::common::UUID& pUUID)
{
  std::string lUUIDString(pUUID.begin(), pUUID.end());

  return lUUIDString;
}
