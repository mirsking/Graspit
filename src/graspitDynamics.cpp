//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2015  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie, Mengyu Wu
//
// $Id:$
//
//######################################################################

#include "graspitDynamics.h"

#include "body.h"
#include "dynJoint.h"
#include "robot.h"
#include "triangle.h"
#include "world.h"

GraspitDynamics::GraspitDynamics(World *world) {
  mWorld = world;
}

GraspitDynamics::~GraspitDynamics() {
}

void GraspitDynamics::addBody(Body *newBody) {
}

void GraspitDynamics::addRobot(Robot *robot) {
}


void GraspitDynamics::turnOnDynamics() {
}

void GraspitDynamics::turnOffDynamics() {
}

void GraspitDynamics::stepDynamics() {
  mWorld->resetDynamicWrenches();
  double actualTimeStep = mWorld->moveDynamicBodies(mWorld->getTimeStep());
  if (actualTimeStep < 0) {
    GraspitDynamics::turnOffDynamics();
    mWorld->emitdynamicsError("Timestep failsafe reached.");
    return;
  }

  for (int i = 0; i < mWorld->getNumRobots(); i++) {
    mWorld->getRobot(i)->DOFController(actualTimeStep);
    mWorld->getRobot(i)->applyJointPassiveInternalWrenches();
  }

  if (mWorld->computeNewVelocities(actualTimeStep)) {
    mWorld->emitdynamicsError("LCP could not be solved.");
    return;
  }
}
