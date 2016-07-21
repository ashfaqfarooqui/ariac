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
#ifndef _PARALLELMULTIRAYSHAPE_HH_
#define _PARALLELMULTIRAYSHAPE_HH_

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MultiRayShape.hh"

namespace gazebo
{
  /// \class MultiRayShape MultiRayShape.hh physics/physics.hh
  /// \brief Laser collision contains a set of ray-collisions,
  /// structured to simulate a laser range scanner.
  class GAZEBO_VISIBLE ParallelMultiRayShape : public physics::MultiRayShape
  {
    /// \brief Constructor.
    /// \param[in] _parent Parent collision shape.
    public: ParallelMultiRayShape(physics::CollisionPtr _parent);

    /// \brief Init the shape.
    public: virtual void Init();

    /// \return Minimum angle of ray scan.
    public: math::Angle GetMinAngle() const;

    /// \brief Get the maximum angle.
    /// \return Maximum angle of ray scan.
    public: math::Angle GetMaxAngle() const;

    /// \brief Get the vertical min angle.
    /// \return Vertical min angle.
    public: math::Angle GetVerticalMinAngle() const;

    /// \brief Get the vertical max angle.
    /// \return Vertical max angle.
    public: math::Angle GetVerticalMaxAngle() const;

    protected: double width;
    protected: double height;
  };
}
#endif
