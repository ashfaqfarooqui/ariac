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

#include "ParallelMultiRayShape.hh"

using namespace gazebo;

//////////////////////////////////////////////////
ParallelMultiRayShape::ParallelMultiRayShape(physics::CollisionPtr _parent)
  : physics::MultiRayShape(_parent)
{
  //this->AddType(PARALLEL_MULTIRAY_SHAPE);
  this->SetName("parallelmultiray");
}

//////////////////////////////////////////////////
void ParallelMultiRayShape::Init()
{
  math::Vector3 start, end, axis;
  math::Quaternion ray;
  double horzOffset, vertOffset;
  int horzSamples = 1;
  // double horzResolution = 1.0;

  int vertSamples = 1;
  // double vertResolution = 1.0;

  double minRange, maxRange;
  this->height = 0.0;

  this->rayElem = this->sdf->GetElement("ray");
  this->scanElem = this->rayElem->GetElement("scan");
  this->horzElem = this->scanElem->GetElement("horizontal");
  this->rangeElem = this->rayElem->GetElement("range");

  if (this->scanElem->HasElement("vertical"))
  {
    this->vertElem = this->scanElem->GetElement("vertical");
    this->height = this->vertElem->Get<double>("height");
    vertSamples = this->vertElem->Get<unsigned int>("samples");
  }

  this->width = this->horzElem->Get<double>("width");
  horzSamples = this->horzElem->Get<unsigned int>("samples");

  minRange = this->rangeElem->Get<double>("min");
  maxRange = this->rangeElem->Get<double>("max");

  this->offset = this->collisionParent->GetRelativePose();

  // Create an array of ray collisions
  for (unsigned int j = 0; j < (unsigned int)vertSamples; ++j)
  {
    for (unsigned int i = 0; i < (unsigned int)horzSamples; ++i)
    {
      horzOffset = (horzSamples == 1) ? 0 :
        i * this->width / (horzSamples - 1) - this->width / 2.0;

      vertOffset = (vertSamples == 1)? 0 :
        j * this->height / (vertSamples - 1) - this->height / 2.0;

      start = math::Vector3(minRange, horzOffset, vertOffset) + this->offset.pos;
      end = math::Vector3(maxRange, horzOffset, vertOffset) + this->offset.pos;

      this->AddRay(start, end);
    }
  }
}

//////////////////////////////////////////////////
math::Angle ParallelMultiRayShape::GetMinAngle() const
{
  return this->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
math::Angle ParallelMultiRayShape::GetMaxAngle() const
{
  return this->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
math::Angle ParallelMultiRayShape::GetVerticalMinAngle() const
{
  if (this->vertElem)
    return this->vertElem->Get<double>("min_angle");
  else
    return math::Angle(0);
}

//////////////////////////////////////////////////
math::Angle ParallelMultiRayShape::GetVerticalMaxAngle() const
{
  if (this->vertElem)
    return this->vertElem->Get<double>("max_angle");
  else
    return math::Angle(0);
}
