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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/algorithm/string.hpp>

#include "gazebo/physics/World.hh"
#include "RaySensorPrivate.hh"
#include "ParallelRaySensor.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Collision.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/Noise.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("parallel_ray", ParallelRaySensor)

//////////////////////////////////////////////////
ParallelRaySensor::ParallelRaySensor()
: RaySensor(),
  dataPtr(new RaySensorPrivate)
{
}

//////////////////////////////////////////////////
ParallelRaySensor::~ParallelRaySensor()
{
}

//////////////////////////////////////////////////
void ParallelRaySensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);
  this->dataPtr->scanPub =
    this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

  GZ_ASSERT(this->world != nullptr,
      "RaySensor did not get a valid World pointer");

  physics::PhysicsEnginePtr physicsEngine =
    this->world->GetPhysicsEngine();

  GZ_ASSERT(physicsEngine != nullptr,
      "Unable to get a pointer to the physics engine");

  this->dataPtr->laserCollision = physicsEngine->CreateCollision("multiray",
      this->ParentName());

  GZ_ASSERT(this->dataPtr->laserCollision != nullptr,
      "Unable to create a multiray collision using the physics engine.");

  this->dataPtr->laserCollision->SetName("ray_sensor_collision");
  this->dataPtr->laserCollision->SetRelativePose(this->pose);
  this->dataPtr->laserCollision->SetInitialRelativePose(this->pose);

  this->dataPtr->laserShape =
    boost::dynamic_pointer_cast<physics::MultiRayShape>(
        this->dataPtr->laserCollision->GetShape());

  GZ_ASSERT(this->dataPtr->laserShape != nullptr,
      "Unable to get the laser shape from the multi-ray collision.");

  this->dataPtr->laserShape->Load(this->sdf);
  this->dataPtr->laserShape->Init();

  // Handle noise model settings.
  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  if (rayElem->HasElement("noise"))
  {
    this->noises[RAY_NOISE] =
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->Type());
  }

  this->dataPtr->parentEntity =
    this->world->GetEntity(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != nullptr,
      "Unable to get the parent entity.");
}
