/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <future>
#include <boost/algorithm/string/replace.hpp>
#include <string>

#include "SideContactPlugin.hh"
#include <ignition/math/Vector3.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SideContactPlugin)

/////////////////////////////////////////////////
SideContactPlugin::SideContactPlugin() : ModelPlugin()
{
}

/////////////////////////////////////////////////
SideContactPlugin::~SideContactPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void SideContactPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");

  if (!_sdf->HasElement("contact_sensor_name"))
  {
    gzerr << "'contact_sensor_name' not specified in SDF\n";
  }
  this->model = _model;
  this->world = this->model->GetWorld();
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  this->contactSensorName = _sdf->Get<std::string>("contact_sensor_name");
  bool sensorFound = this->FindContactSensor();
  if (!sensorFound || !this->parentSensor)
  {
    gzerr << "Contact sensor not found: " << this->contactSensorName << "\n";
  }

  std::string parentLinkName = this->parentLink->GetScopedName();
  std::string defaultCollisionName = parentLinkName + "::__default__";
  if (this->parentSensor->GetCollisionCount() != 1)
  {
    gzerr << "SideContactPlugin requires a single collision to observe contacts for\n";
    return;
  }

  this->collisionName = this->parentSensor->GetCollisionName(0);
  if (this->collisionName == defaultCollisionName)
  {
    // Use the first collision of the parent link by default
    if (this->parentLink->GetCollisions().empty())
    {
      gzerr << "Couldn't find any collisions for the contact sensor.";
      return;
    }
    unsigned int index = 0;
    this->collisionName = this->parentLink->GetCollision(index)->GetScopedName();
  }
  gzdbg << "[" << this->model->GetName() << "] Watching collisions on: " << this->collisionName << "\n";

  if (_sdf->HasElement("contact_side_normal"))
  {
    this->sideNormal = _sdf->Get<ignition::math::Vector3d>("contact_side_normal");
  }
  else
  {
    this->sideNormal = ignition::math::Vector3d(0, 0, 1);
  }

  if (_sdf->HasElement("update_rate"))
  {
    std::string ur = _sdf->Get<std::string>("update_rate");
    try
    {
      double v = std::stod(ur);
      if (v <= 0)
      {
        gzerr << "Illegal update_rate value [" << v << "]" << std::endl;
      }
      this->updateRate = v;
    } catch (const std::exception& e)
    {
      gzerr << "Unable to parse update_rate [" << ur << "]" << std::endl;
    }
  }

  this->lastUpdateTime = this->world->GetSimTime();

  // FIXME: how to not hard-code this gazebo prefix?
  std::string contactTopic = "/gazebo/" + this->scopedContactSensorName;
  boost::replace_all(contactTopic, "::", "/");
  this->contactSub =
    this->node->Subscribe(contactTopic, &SideContactPlugin::OnContactsReceived, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&SideContactPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
bool SideContactPlugin::FindContactSensor()
{
  auto sensorManager = sensors::SensorManager::Instance();
  auto links = this->model->GetLinks();
  for (const auto &link : links)
  {
    std::string scopedContactSensorName =
      this->world->GetName() + "::" + link->GetScopedName() + "::" + this->contactSensorName;
    for (unsigned int i = 0; i < link->GetSensorCount(); ++i)
    {
      if (link->GetSensorName(i) == scopedContactSensorName)
      {
        this->parentLink = link;
        this->scopedContactSensorName = scopedContactSensorName;
        this->parentSensor =
          std::static_pointer_cast<sensors::ContactSensor>(
            sensorManager->GetSensor(scopedContactSensorName));
        return this->parentSensor != 0;
      }
    }
  }
  return false;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnContactsReceived(ConstContactsPtr& _msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->nextContactsMsg = *_msg;
  this->newMsg = true;
}

/////////////////////////////////////////////////
void SideContactPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  this->CalculateContactingModels();
}

/////////////////////////////////////////////////
void SideContactPlugin::PrepareNextCycle()
{
  this->partialUpdateReady = false;

  {
    boost::mutex::scoped_lock lock(this->mutex);

    if (!this->newMsg)
    {
      return;
    }

    this->newMsg = false;
    this->newestContactsMsg = this->nextContactsMsg;
  }

  // Update contacting links with the result from the last cycle of updates.
  this->contactingLinks = this->partialContactingLinks;

  this->partialUpdateReady = true;
  this->partialContactingLinks.clear();
  this->contactsNextIndex = 0;

  if (this->updateRate > 0)
  {
    this->contactsPerCycle = ceil(this->newestContactsMsg.contact_size() /
      ((1.0 / this->updateRate) /
       this->world->GetPhysicsEngine()->GetMaxStepSize()));
  }
  else
  {
    this->contactsPerCycle = ceil(this->newestContactsMsg.contact_size() /
      ((1.0 / 1000.0) /
       this->world->GetPhysicsEngine()->GetMaxStepSize()));
  }

  //std::cout << "Contacts: " << this->newestContactsMsg.contact_size() << std::endl;
  //std::cout << "Update period: " << this->world->GetPhysicsEngine()->GetUpdatePeriod() << std::endl;
  //std::cout << "Contacts per cycle:" << this->contactsPerCycle << std::endl;
}

/////////////////////////////////////////////////
void SideContactPlugin::UpdatePartialContactingLinks()
{
  auto t1 = std::chrono::steady_clock::now();

  if (!this->partialUpdateReady)
    return;

  unsigned int counter = 0;
  int index = this->contactsNextIndex + counter;
  while (counter < this->contactsPerCycle &&
         index < this->newestContactsMsg.contact_size())
  {
    // Get the collision that's not the parent link
    const auto &contact = this->newestContactsMsg.contact(index);
    const std::string *collision = &(contact.collision1());
    if (this->collisionName == *collision) {
      collision = &(contact.collision2());
    }

    physics::CollisionPtr collisionPtr =
      boost::static_pointer_cast<physics::Collision>(this->world->GetEntity(*collision));
    if (collisionPtr) { // ensure the collision hasn't been deleted
      this->partialContactingLinks.insert(collisionPtr->GetLink());
    }

    ++counter;
    ++index;
  }

  this->contactsNextIndex += counter;

  auto t2 = std::chrono::steady_clock::now();
  auto elapsed = t2 - t1;
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>
           (elapsed).count();
  if (ms > 1)
  {
    std::cout << "Size: " << counter << std::endl;
    std::cout << "Elapsed: " << ms << std::endl;
  }
}

/////////////////////////////////////////////////
std::set<physics::LinkPtr> SideContactPlugin::CalculatePartialContactingLinks(
  const int _from, const int _len) const
{
  std::set<physics::LinkPtr> res;

  for (int i = _from; i < _from + _len; ++i)
  {
    // Get the collision that's not the parent link
    const auto &contact = this->newestContactsMsg.contact(i);
    const std::string *collision = &(contact.collision1());
    if (this->collisionName == *collision) {
      collision = &(contact.collision2());
    }

    physics::CollisionPtr collisionPtr =
      boost::static_pointer_cast<physics::Collision>(this->world->GetEntity(*collision));
    if (collisionPtr) { // ensure the collision hasn't been deleted
      res.insert(collisionPtr->GetLink());
    }
  }

  return res;
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingLinksMultithread()
{
  auto t1 = std::chrono::steady_clock::now();
  {
    boost::mutex::scoped_lock lock(this->mutex);

    if (!this->newMsg)
    {
      return;
    }

    this->newMsg = false;
    this->newestContactsMsg = this->nextContactsMsg;
  }

  this->contactingLinks.clear();

  // Get all the contacts
  auto size = this->newestContactsMsg.contact_size();
  std::set<physics::LinkPtr> part1;
  std::set<physics::LinkPtr> part2;
  std::set<physics::LinkPtr> part3;
  std::set<physics::LinkPtr> part4;
  std::set<physics::LinkPtr> part5;
  std::set<physics::LinkPtr> part6;
  std::set<physics::LinkPtr> part7;
  std::set<physics::LinkPtr> part8;
  if (size <= 100)
  {
    part1 = this->CalculatePartialContactingLinks(0, size);
  }
  else
  {
    auto f1 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 0, size / 8);
    auto f2 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 1 * size / 8, size / 8);
    auto f3 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 2 * size / 8, size / 8);
    auto f4 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 3 * size / 8, size / 8);
    auto f5 = std::async(std::launch::async,
                           &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 4 * size / 8, size / 8);
    auto f6 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 5 * size / 8, size / 8);
    auto f7 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 6 * size / 8, size / 8);

    std::future<std::set<physics::LinkPtr>> f8;
    if (size % 2 == 0)
    {
      f8 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 7 * size / 8, size / 8);
    }
    else
    {
      f8 = std::async(std::launch::async,
                         &SideContactPlugin::CalculatePartialContactingLinks,
                         this, 7 * size / 8, (size / 8) + 1);
    }

    part1 = f1.get();
    part2 = f2.get();
    part3 = f3.get();
    part4 = f4.get();
    part5 = f5.get();
    part6 = f6.get();
    part7 = f7.get();
    part8 = f8.get();
  }

  // Merge result.
  part1.insert(part2.begin(), part2.end());
  part1.insert(part3.begin(), part3.end());
  part1.insert(part4.begin(), part4.end());
  part1.insert(part5.begin(), part5.end());
  part1.insert(part6.begin(), part6.end());
  part1.insert(part7.begin(), part7.end());
  part1.insert(part8.begin(), part8.end());

  this->contactingLinks = part1;

  auto t2 = std::chrono::steady_clock::now();
  auto elapsed = t2 - t1;
  std::cout << "Size: " << size << std::endl;
  std::cout << "Elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>
           (elapsed).count() << std::endl;
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingLinks()
{
  auto t1 = std::chrono::steady_clock::now();
  {
    boost::mutex::scoped_lock lock(this->mutex);

    if (!this->newMsg)
    {
      return;
    }

    this->newMsg = false;
    this->newestContactsMsg = this->nextContactsMsg;
  }

  this->contactingLinks.clear();

  // Get all the contacts
  for (int i = 0; i < this->newestContactsMsg.contact_size(); ++i)
  {
    // Get the collision that's not the parent link
    const auto &contact = this->newestContactsMsg.contact(i);
    const std::string *collision = &(contact.collision1());
    if (this->collisionName == *collision) {
      collision = &(contact.collision2());
    }

    physics::CollisionPtr collisionPtr =
      boost::static_pointer_cast<physics::Collision>(this->world->GetEntity(*collision));
    if (collisionPtr) { // ensure the collision hasn't been deleted
      this->contactingLinks.insert(collisionPtr->GetLink());
    }
  }
  auto t2 = std::chrono::steady_clock::now();
  auto elapsed = t2 - t1;
  std::cout << "Size: " << this->newestContactsMsg.contact_size() << std::endl;
  std::cout << "Elapsed: " << std::chrono::duration_cast<std::chrono::milliseconds>
           (elapsed).count() << std::endl;
}

/////////////////////////////////////////////////
void SideContactPlugin::CalculateContactingModels()
{
  this->CalculateContactingLinks();
  this->contactingModels.clear();
  for (auto link : this->contactingLinks)
  {
    physics::ModelPtr model = link->GetModel();
    this->contactingModels.insert(model);
  }
}

/////////////////////////////////////////////////
bool SideContactPlugin::TimeToExecute()
{
  // We're using a custom update rate.
  if (this->updateRate <= 0)
    return true;

  gazebo::common::Time curTime = this->world->GetSimTime();
  auto dt = (curTime - this->lastUpdateTime).Double();
  if (dt < 0)
  {
    // Probably we had a reset.
    this->lastUpdateTime = curTime;
    return false;
  }

  // Update based on sensorsUpdateRate.
  if (dt < (1.0 / this->updateRate))
    return false;

  this->lastUpdateTime = curTime;
  return true;
}
