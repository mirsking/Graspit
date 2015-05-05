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
// Author(s): Matei T. Ciocarlie
//
// $Id:$
//
//######################################################################

#include "bulletDynamics.h"

#include "btBulletDynamicsCommon.h"
#include "btBvhTriangleMeshShape.h"
#include "btTriangleMesh.h"
#include "btGImpactShape.h"
#include "btGImpactCollisionAlgorithm.h"
#include "btHingeConstraint.h"

#include "body.h"
#include "dynJoint.h"
#include "robot.h"
#include "triangle.h"
#include "world.h"

#include "debug.h"
#include "dynamics.h"
#include "humanHand.h"
BulletDynamics::BulletDynamics(World *world)
{
  mWorld = world;

  // collision configuration contains default setup for memory, collision setup. 
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  
  // use the default collision dispatcher. 
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
  
  // btDbvtBroadphase is a good general purpose broadphase. .
  btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

  //the default constraint solver. 
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
  mBtDynamicsWorld = 
    new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);
  mBtDynamicsWorld->setGravity(btVector3(0,0,-10));
}

BulletDynamics::~BulletDynamics()
{
  delete mBtDynamicsWorld;
}

void BulletDynamics::addBody(Body *newBody)
{
  // Creation of CollisionShape
  btTriangleMesh* triMesh = new btTriangleMesh(true,true);//true,true);
  
  // Get the geometry data form the Graspit object
  std::vector<Triangle> triangles;

  newBody->getGeometryTriangles(&triangles);
  int numTriangles = triangles.size();
  Triangle tritemp = triangles.at(0);

  for(int i = 0; i < numTriangles-1; i=i+1)
  {
    tritemp = triangles.at(i);
    btScalar v01(tritemp.v1[0]);
    btScalar v02(tritemp.v1[1]);
    btScalar v03(tritemp.v1[2]);
    btScalar v11(tritemp.v2[0]);
    btScalar v12(tritemp.v2[1]);
    btScalar v13(tritemp.v2[2]);
    btScalar v21(tritemp.v3[0]);
    btScalar v22(tritemp.v3[1]);
    btScalar v23(tritemp.v3[2]);

    btVector3 v0(v01,v02,v03);
    btVector3 v1(v11,v12,v13);
    btVector3 v2(v21,v22,v23);    
    triMesh->btTriangleMesh::addTriangle(v0,v1,v2,true);    
  }
  
  btCollisionShape* triMeshShape;
  btScalar mass(0.);
  btVector3 localInertia(0,0,0);
  
  if (newBody->isDynamic())
  {  
    mass = ((DynamicBody*)newBody)->getMass();
    triMeshShape = new btGImpactMeshShape(triMesh);
    ((btGImpactMeshShape*)triMeshShape)->updateBound();
    triMeshShape->calculateLocalInertia(mass,localInertia);  
  }
  else
  {
    triMeshShape = new btBvhTriangleMeshShape(triMesh, true,true);  
  }

  //using motionstate is recommended, it provides interpolation capabilities, 
  //and only synchronizes 'active' objects
  btDefaultMotionState* myMotionState = 
    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0 , 0)));
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,triMeshShape,localInertia);
  btRigidBody* body = new btRigidBody(rbInfo);
  
  //add the body to the dynamics world
  mBtDynamicsWorld->addRigidBody(body);

  mBtLinks.push_back(body);
  //add the newlink and body to the map
  btBodyMap.insert(btBodyPair(newBody,body));
}

void BulletDynamics::addRobot(Robot *robot)
{
  btRigidBody* btbase;

  if (robot->getBase()) {
    btScalar mass(0.);
    btVector3 localInertia(0, 0, 0);    
    if ((btbase=btBodyMap.find(robot->getBase())->second) == NULL) {
      printf("error, base is not in the btBodyMap\n");
    }
    btbase->setMassProps(mass , localInertia);
  }
  printf("### of m_links: %d \n ", mBtLinks.size());

  for (int f=0; f <robot->getNumChains(); f++) {
    // get number of links
    int numberLinks = robot->getChain(f)->getNumLinks();
    // get number of joints
    int numberjoints = robot->getChain(f)->getNumJoints();
    // get the transfrom from the origin of the palm to the base of this chain
    transf chaintransf = robot->getChain(f)->getTran();
    vec3 chaintranslation = chaintransf.translation();
    printf("chain %d TRANSLATION %f,%f,%f \n", f, 
	   chaintranslation.x(), chaintranslation.y(), chaintranslation.z());
    btVector3 chainpos(chaintranslation.x(), chaintranslation.y(), chaintranslation.z());
    // calculate chain rotation, and to get base z,x,y in the frame of world frame 
    //(palm origin frame)
    Quaternion rotq = chaintransf.rotation();
    vec3 zbaseinOrigin = rotq*vec3(0, 0, 1);
    vec3 xbaseinOrigin = rotq*vec3(1, 0, 0);
    vec3 ybaseinOrigin = rotq*vec3(0, 1, 0);
    btVector3 baseaxis(zbaseinOrigin.x(), zbaseinOrigin.y(), zbaseinOrigin.z());
    btVector3 xbaseaxis(xbaseinOrigin.x(), xbaseinOrigin.y(), xbaseinOrigin.z());
    btVector3 ybaseaxis(ybaseinOrigin.x(), ybaseinOrigin.y(), ybaseinOrigin.z());

    int jointind = 0;  // keep track current joint index
    for (int l=0; l <numberLinks; l++) {

      btRigidBody* btcurrentlink = btBodyMap.find(robot->getChain(f)->getLink(l))->second;
      btRigidBody* btprevlink;
      if (l > 0) {
        btprevlink = btBodyMap.find(robot->getChain(f)->getLink(l-1))->second;
      } else if (l == 0) {
        btprevlink = btbase;
      }
      bool constructor3 = false;
      printf("link#: %d \n ", l);
      // get the rotation axis in the frame of next joint
      vec3 proxjointaxis = robot->getChain(f)->getLink(l)->getProximalJointAxis();
      btVector3 linkpaxis(proxjointaxis.x() , proxjointaxis.y() , proxjointaxis.z());
      printf("The link PROXIMAL JOINT AXIS:%f,%f,%f \n", 
	     proxjointaxis.x() , proxjointaxis.y(), proxjointaxis.z());
      // get the proximal joint location in the frame of next joint
      position prolocation = robot->getChain(f)->getLink(l)->getProximalJointLocation();
      printf("proximalJoint localtion: %f, %f, %f \n", 
	     prolocation.x(), prolocation.y(), prolocation.z());
      btVector3 pivot2(prolocation.x(), prolocation.y(), prolocation.z());

      // get the property "dynamicjointtype"
      // DynamicJointT{FIXED, REVOLUTE, UNIVERSAL, BALL, PRISMATIC};
      DynJoint::DynamicJointT djtype = robot->getChain(f)->getLink(l)->getDynJoint()->getType();

      if (djtype == DynJoint::REVOLUTE) {
        printf("~~~~~~chain: %d link: %d  type: REVOLUTE \n", f, l);
        printf("jointind: %d \n ", jointind);
        jointind++;

        btHingeConstraint* newjoint;
        if (l == 0) {  // palm and link
          newjoint = new btHingeConstraint(*(btbase) , *(btcurrentlink) , 
					   chainpos, pivot2,  baseaxis, linkpaxis);
        } else {
          position dislocation = robot->getChain(f)->getLink(l-1)->getDistalJointLocation();
          printf("DistalJointLocation localtion: %f, %f, %f  \n", 
		 dislocation.x(), dislocation.y(), dislocation.z());
          btVector3 linkpivot1(dislocation.x(), dislocation.y(), dislocation.z());
          newjoint = new btHingeConstraint(*(btprevlink), *(btcurrentlink),
                                           btVector3(0 , 0 , 0), pivot2, 
					   btVector3(0 , 0 , 1), linkpaxis);
        }
        // set the second parameter to be true, disable collision between two constrraint body
        mBtDynamicsWorld->addConstraint(newjoint , true);
      } else if (djtype == DynJoint::UNIVERSAL) {
        printf("~~~~~~chain: %d link: %d  type: UNIVERSAL \n", f, l);
        Joint* joint0 = robot->getChain(f)->getJoint(jointind);
        Joint* joint1 = robot->getChain(f)->getJoint(jointind+1);
        jointind+=2;  //universal: use two joints

        transf T1 = joint0->getTran();
        transf T2 = joint1->getTran();
        Quaternion rotqj01 = T1.rotation();
        Quaternion rotqj12 = T2.rotation();
        Quaternion rot02 = rotqj01*rotqj12;
        Quaternion rot20 = rot02.inverse();
        Quaternion rot21 = rotqj12.inverse();
        //get the joint0 x,y,z in new frame(after two transformation T1,T2)
        vec3 zjoint0new = rot20*vec3(0, 0, 1);
        vec3 xjoint0new = rot20*vec3(1, 0, 0);
        vec3 yjoint0new = rot20*vec3(0, 1, 0);
        btVector3 btzjoint0new(zjoint0new.x(), zjoint0new.y(), zjoint0new.z());
        btVector3 btxjoint0new(xjoint0new.x(), xjoint0new.y(), xjoint0new.z());
        btVector3 btyjoint0new(yjoint0new.x(), yjoint0new.y(), yjoint0new.z());
        printf("chain %d link: %d joint0 z in frame 2 %f,%f,%f \n",
               f, l, zjoint0new.x(), zjoint0new.y(), zjoint0new.z());
        //get the joint1 x,y,z in new frame(after 1 transformation T2)
        vec3 zjoint1new = rot21*vec3(0, 0, 1);
        vec3 xjoint1new = rot21*vec3(1, 0, 0);
        vec3 yjoint1new = rot21*vec3(0, 1, 0);
        btVector3 btzjoint1new(zjoint1new.x(), zjoint1new.y(), zjoint1new.z());
        btVector3 btxjoint1new(xjoint1new.x(), xjoint1new.y(), xjoint1new.z());
        btVector3 btyjoint1new(yjoint1new.x(), yjoint1new.y(), yjoint1new.z());
        printf("chain %d link: %d joint1 z in frame2 %f,%f,%f \n",
                f, l, zjoint1new.x(), zjoint1new.y(), zjoint1new.z());
        // joint1 z axis in the frame of joint0
        vec3 zjoint1infram0 = rotqj01*vec3(0, 0, 1);
        

        // universal constraint
        btTransform frameInA;
        btTransform frameInB;
        frameInA.setIdentity();
        frameInB.setIdentity();
        //set the constraint frameB: which is the joint0 x,y,z coordinates in new frame
	//(after 2 transformation T1,T2)
        frameInB.getBasis().setValue(btxjoint0new.x(), btyjoint0new.x(), btzjoint0new.x(),
                                     btxjoint0new.y(), btyjoint0new.y(), btzjoint0new.y(),
                                     btxjoint0new.z(), btyjoint0new.z(), btzjoint0new.z());
        frameInB.setOrigin(pivot2);
        btGeneric6DofConstraint* newjoint;

        if (l == 0) {  // connect with palm ,frameinA: palm
          frameInA.getBasis().setValue(xbaseaxis.x(), ybaseaxis.x(), baseaxis.x(),
                                       xbaseaxis.y(), ybaseaxis.y(), baseaxis.y(),
                                       xbaseaxis.z(), ybaseaxis.z(), baseaxis.z());

          frameInA.setOrigin(chainpos);
          newjoint = new btGeneric6DofConstraint(*(btbase) , *(btcurrentlink), 
						 frameInA, frameInB, false);
        } else {  // privous one is not palm, frameinA: previous link 
          frameInA.setOrigin(btVector3(0, 0, 0));
          frameInA.getBasis().setValue(1, 0, 0,
                                       0, 1, 0,
                                       0, 0, 1);
          newjoint = new btGeneric6DofConstraint(*(btprevlink), *(btcurrentlink), 
						 frameInA, frameInB, false);
        }
        //set translation and rotation limits, they are based on joint0 frame
        //set x,y,z translation limit=0,0; 0,1,2-> x,y,z translation, 3,4,5-> x,y,z rotation.
        newjoint->setLimit(0, 0, 0);
        newjoint->setLimit(1, 0, 0);
        newjoint->setLimit(2, 0, 0);  
        printf("chain %d joint1 z in joint0  frame %f,%f,%f \n",
               f, zjoint1infram0.x(), zjoint1infram0.y(), zjoint1infram0.z());
        //universal constraints: two rotation axises, one is z axis of joint0,
        //the other one is the joint1 z axis in the frame of joint0 (x or y),
	//set the left one limit=0,0
        if (zjoint1infram0.x() != 0) {
          printf("zjoint1 in joint0 frame is x, limit the y rotation \n ");
          newjoint->setLimit(4, 0, 0);  // limit the y rotation;
        } else if (zjoint1infram0.y() != 0) {
          printf("zjoint1 in joint0 frame is y, limit the x rotation \n ");
          newjoint->setLimit(3, 0, 0);  // limit the x rotation;
        }

        mBtDynamicsWorld->addConstraint(newjoint, true);
      } else if (djtype == DynJoint::BALL) {
        printf("~~~~~~chain: %d link: %d  type: BALL \n", f, l);
        jointind+=3;
      } else if (djtype == DynJoint::PRISMATIC) {
        printf("~~~~~~chain: %d link: %d  type: PRISMATIC \n", f, l);
        jointind++;
      } else if (djtype == DynJoint::FIXED) {
        printf("~~~~~~chain: %d link: %d  type: FIXED \n", f, l);
        jointind++;
      }
    }  // for link
  }  // for chain
}

void BulletDynamics::turnOnDynamics()
{
  // Update BULLET TRANSFORM whenever Dynamics are turned on
  for (int j=mBtDynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--) {
    Body* tempbody = mWorld->getBody(j);
    transf temptrans = tempbody->getTran();    
    
    btScalar wrot(temptrans.rotation().w);
    btScalar xrot(temptrans.rotation().x);
    btScalar yrot(temptrans.rotation().y);
    btScalar zrot(temptrans.rotation().z);
    
    btScalar xtrans(temptrans.translation().x());
    btScalar ytrans(temptrans.translation().y());
    btScalar ztrans(temptrans.translation().z());
        
    btCollisionObject* obj = mBtDynamicsWorld->getCollisionObjectArray()[j];
    btRigidBody* body = btRigidBody::upcast(obj);
    body->setCenterOfMassTransform(btTransform(btQuaternion( xrot , yrot , zrot , wrot), 
					       btVector3(xtrans, ytrans, ztrans)));
    body->setLinearVelocity(btVector3(0 , 0 , 0));
    body->setAngularVelocity(btVector3(0 , 0 , 0));
  }  
}

void BulletDynamics::turnOffDynamics()
{
}


void bulletApplyInternalWrench (Joint * activeJoint, double magnitude, std::map<Body*, btRigidBody*> btBodyMap) {
     vec3 worldAxis=activeJoint->getWorldAxis();
     //printf(" axis: x: %lf, y: %lf, z: %lf  \n", worldAxis.x(),worldAxis.y(),worldAxis.z());
     btRigidBody* btPrevLink = btBodyMap.find(activeJoint->dynJoint->getPrevLink())->second;
     btRigidBody* btNextLink = btBodyMap.find(activeJoint->dynJoint->getNextLink())->second;
     
     //printf("magnitude : %lf \n", magnitude);
        
     btVector3 torquePrev(-magnitude*worldAxis.x(),-magnitude*worldAxis.y(), -magnitude*worldAxis.z()); 
     btVector3 torqueNext(magnitude*worldAxis.x(),magnitude*worldAxis.y(), magnitude*worldAxis.z()); 
     btPrevLink->applyTorque(torquePrev);
     btNextLink->applyTorque(torqueNext);
}

int BulletDynamics::stepDynamics()
{
  mBtDynamicsWorld->stepSimulation(1.f/60.f,10); 
  for (int j=mBtDynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
  {
    // FEEDBACK TO GRASPIT
    btCollisionObject* obj = mBtDynamicsWorld->getCollisionObjectArray()[j];
    btRigidBody* body = btRigidBody::upcast(obj);
    btTransform feedbacktransform = body->getCenterOfMassTransform();
    btQuaternion btrotation = feedbacktransform.getRotation();
    btVector3 bttranslation = feedbacktransform.getOrigin();
    Body* tempbody = mWorld->getBody(j);               
    Quaternion* rot = new Quaternion(btrotation.getAngle() , 
				     vec3(btrotation.getAxis()[0] , btrotation.getAxis()[1] , 
					  btrotation.getAxis()[2]) );    
    vec3* transl = new vec3(bttranslation.getX() , bttranslation.getY() , bttranslation.getZ());
    Quaternion rotfix = *rot;
    vec3 translfix = *transl;    
    transf* temptrans2 = new transf(rotfix , translfix) ;
    transf temptrans2fix = *temptrans2;          
    tempbody->setTran(temptrans2fix);
 
  
  }  
 // --------------------------add torque--------------------------------------------------
  double timeStep=1.0f/60.f;
  mWorld->resetDynamicWrenches();
 
  double dofUpdateTime=0.0;
  for (int i = 0; i < mWorld->getNumRobots(); i++) {
    Robot* robot=mWorld->getRobot(i);     
    robot->updateJointValuesFromDynamics(); // !!!!!!

    //printf("dofcontroller: getWorldTime() %lf , dofupdatetime: %lf \n", mWorld->getWorldTime(),dofUpdateTime); 
    int numDOF=robot->getNumDOF();
    //printf("numdof: %d \n", numDOF);

    //if(mWorld->getWorldTime() >= dofUpdateTime)
    if (1) {
       DBGP("Updating setpoints");
       for (int d = 0; d < numDOF;d++) {
         DOF * dof = robot->getDOF(d);   
         dof->updateSetPoint();       
         printf("DOF: %d, dof val: %lf, set position: %lf, desired position: %lf \n",d, dof->getVal(),dof->getSetPoint(),dof->getDesiredPos());
       }
	 dofUpdateTime += mWorld->getTimeStep();
       }
     for (int d=0;d<numDOF;d++) {
       
         //DOF * dof=dofVec[d];
         DOF * dof=robot->getDOF(d);          

         dof->callController(timeStep);

        //get the joint of that dof    
        Joint *activeJoint = *(dof->getJointList().begin());
        double magnitude=robot->getDOF(d)->getForce();
        magnitude=magnitude/1000;
        printf("DOF: %d  magnitude : %lf \n", d, magnitude);
        bulletApplyInternalWrench( activeJoint,  magnitude,  btBodyMap);
       
      }


     //mWorld->getRobot(i)->applyJointPassiveInternalWrenches();
     for (int c=0; c<robot->getNumChains(); c++) {
	for (int j=0; j<robot->getChain(c)->getNumJoints(); j++) {
	  //getChain(c)->getJoint(j)->applyPassiveInternalWrenches();
          Joint *tempjoint=robot->getChain(c)->getJoint(j);
          double f = tempjoint->getFriction();
          //printf("chain: %d joint: %d Friction f: %lf \n",c,j,f);
	  if (f != 0.0){
            //applyInternalWrench(f);
            bulletApplyInternalWrench(tempjoint, f, btBodyMap);
          }

	  f = tempjoint->getSpringForce();
          //printf("chain: %d joint: %d SpringForce f: %lf \n",c,j,f);
	  //applyInternalWrench(-f);
          bulletApplyInternalWrench(tempjoint, f,btBodyMap);
	}
      }

   }


}




/*! One of the two main functions of the dynamics time step. This function is 
called to move the bodies according to the velocities and accelerations found
in the previous step. The move is attempted for the duration of the time step 
given in \a timeStep.

After all the bodies are moved, then collisions are checked. If any collisions
are found, the move is traced back and the value of the times step is 
interpolated until the exact moment of contact is found. The actual value
of the time step until contact is made is returned. If interpolation fails,
a negative actual time step is returned. All the resulting contacts are added
to the bodies that are in contact to be used for subsequent computations.

The same procedure is carried out if, by executing a full time step, a joint
ends up outside of its legal range.
*/
double BulletDynamics::moveDynamicBodies(double timeStep) {
  int i, numDynBodies, numCols, moveErrCode;
  std::vector<DynamicBody *> dynBodies;
  static CollisionReport colReport;
  bool jointLimitHit;
  double contactTime, delta, tmpDist, minDist, dofLimitDist;

  int numBodies = mWorld->getNumBodies();
  int numRobots = mWorld->getNumRobots();
 
  // save the initial position
  for (i = 0; i < numBodies; i++) {
    if (mWorld->getBody(i)->isDynamic()) {
      dynBodies.push_back((DynamicBody *)mWorld->getBody(i));
      ((DynamicBody *)mWorld->getBody(i))->markState();
    }
  }
  numDynBodies = dynBodies.size();

  // call to the dynamics engine to perform the move by the full time step
  DBGP("moving bodies with timestep: " << timeStep);
  moveErrCode = moveBodies(numDynBodies, dynBodies, timeStep);
  if (moveErrCode == 1) {  // object out of bounds
    mWorld->popDynamicState();
    turnOffDynamics();
    return -1.0;
  }

  // this sets the joints internal values according to how bodies have moved
  for (i = 0; i < numRobots; i++) {
    mWorld->getRobot(i)->updateJointValuesFromDynamics();
  }

  // check if we have collisions
  if (numDynBodies > 0) numCols = mWorld->getCollisionReport(&colReport);
  else numCols = 0;

  // check if we have joint limits exceeded
  jointLimitHit = false;
  for (i = 0; i < numRobots; i++) {
    if (mWorld->getRobot(i)->jointLimitDist() < 0.0) jointLimitHit = true;
  }

  // if so, we must interpolate until the exact moment of contact or limit hit
  if (numCols || jointLimitHit) {
    // return to initial position
    for (i = 0; i < numDynBodies; i++) {
      dynBodies[i]->returnToMarkedState();
    }
    minDist = 1.0e+255;
    dofLimitDist = 1.0e+255;

#ifdef GRASPITDBG
    if (numCols) {
      std::cout << "COLLIDE!" << std::endl;
      for (i = 0; i < numCols; i++) {
        std::cout << colReport[i].first->getName() << " collided with " <<
          colReport[i].second->getName() << std::endl;
      }

      for (i = 0; i < numCols; i++) {
        tmpDist = getDist(colReport[i].first, colReport[i].second);
        if (tmpDist < minDist) minDist = tmpDist;
        std::cout << "minDist: " << tmpDist <<" between " << std::endl;
        std::cout << colReport[i].first->getName() << " and " <<
          colReport[i].second->getName() << std::endl;
      }
    }
#endif

    // this section refines the timestep until the objects are separated
    // by a distance less than CONTACT_THRESHOLD
    bool done = false;
    contactTime = timeStep;
    delta = contactTime/2;
    contactTime -= delta;

    while (!done) {
      delta /= 2.0;
      for (i = 0; i < numDynBodies; i++) {
        dynBodies[i]->returnToMarkedState();
      }
      DBGP("moving bodies with timestep: " << contactTime);
      moveErrCode = moveBodies(numDynBodies, dynBodies, contactTime);

      if (moveErrCode == 1) {  // object out of bounds
        mWorld->popDynamicState();
        turnOffDynamics();
        return -1.0;
      }

      const char *min_body_1, *min_body_2;

      // this computes joints values according to how dynamic bodies have moved
      for (i = 0; i < numRobots; i++) {
        mWorld->getRobot(i)->updateJointValuesFromDynamics();
      }

      if (numCols) {
        minDist = 1.0e+255;
        for (i = 0; i < numCols; i++) {
          tmpDist = mWorld->getDist(colReport[i].first, colReport[i].second);
          if (tmpDist < minDist) {
            minDist = tmpDist;
            min_body_1 = colReport[i].first->getName().latin1();
            min_body_2 = colReport[i].second->getName().latin1();
            DBGP("minDist: " << minDist << " between " << colReport[i].first->getName() <<
              " and " << colReport[i].second->getName());
          }
        }
      }

      if (jointLimitHit) {
        dofLimitDist = 1.0e10;
        for (i = 0; i < numRobots; i++) {
          dofLimitDist = MIN(dofLimitDist, mWorld->getRobot(i)->jointLimitDist());
        }
      }

      if (minDist <= 0.0 || dofLimitDist < -resabs)
        contactTime -= delta;
      else if (minDist > Contact::THRESHOLD * 0.5 && dofLimitDist > 0.01)  // why is this not resabs
        contactTime += delta;
      else break;

      if (fabs(delta) < 1.0E-15 || contactTime < 1.0e-7) {
        if (minDist <= 0) {
          fprintf(stderr, "Delta failsafe due to collision: %s and %s\n", min_body_1, min_body_2);
        } else {
          fprintf(stderr, "Delta failsafe due to joint\n");
        }
        done = true;  // failsafe
      }
    }

    // COULD NOT FIND COLLISION TIME
    if (done && contactTime < 1.0E-7) {
      DBGP("!! could not find contact time !!");
      for (i = 0; i < numDynBodies; i++)
        dynBodies[i]->returnToMarkedState();
    }
    mWorld->getWorldTimeRef() += contactTime;
  } else {  // if no collision
    mWorld->getWorldTimeRef() += timeStep;
    contactTime = timeStep;
  }

#ifdef GRASPITDBG
  std::cout << "CHECKING COLLISIONS AT MIDDLE OF STEP: ";
  numCols = getCollisionReport(colReport);

  if (!numCols) {
    std::cout << "None." << std::endl;
  } else {
    std::cout << numCols <<" found!!!" << std::endl;
    for (i = 0; i < numCols; i++) {
      std::cout << colReport[i].first->getName() << " collided with " <<
        colReport[i].second->getName() << std::endl;
    }
  }
#endif

  if (numDynBodies > 0)
    mWorld->findAllContacts();

  for (i = 0; i < numRobots; i++) {
        if ( mWorld->getRobot(i)->inherits("HumanHand") ) ((HumanHand*)mWorld->getRobot(i))->updateTendonGeometry();
          mWorld->getRobot(i)->emitConfigChange();
  }
  mWorld->tendonChange();

  if (contactTime < 1.0E-7) return -1.0;
  return contactTime;
}


/*! Asks the dynamics engine to compute the velocities of all bodies at
the current time step. These will be used in the next time step when
the bodies are moved by World::moveDynamicBodies.

The bodies are separated into "islands" of bodies connected by contacts 
or joints.  Two dynamic bodies are connected if they share a contact or
a joint.  Then for each island, this calls the iterate dynamics routine
to build and solve the LCP to find the velocities of all the bodies
in the next iteration.
*/  
int BulletDynamics::computeNewVelocities(double timeStep) {
  bool allDynamicsComputed;
  static std::list<Contact *> contactList;
  std::list<Contact *>::iterator cp;
  std::vector<DynamicBody *> robotLinks;
  std::vector<DynamicBody *> dynIsland;
  std::vector<Robot *> islandRobots;
  int i, j, numLinks, numDynBodies, numIslandRobots, lemkeErrCode;

  int numBodies = mWorld->getNumBodies();

#ifdef GRASPITDBG
  int islandCount = 0;
#endif

  do {
    // seed the island with one dynamic body
    for (i = 0; i < numBodies; i++)
      if (mWorld->getBody(i)->isDynamic() &&
        !((DynamicBody *)mWorld->getBody(i))->dynamicsComputed()) {
          // if this body is a link, add all robots connected to the link
          if (mWorld->getBody(i)->inherits("Link")) {
            Robot *robot = ((Robot *)((Link *)mWorld->getBody(i))->getOwner())->getBaseRobot();
            robot->getAllLinks(dynIsland);
            robot->getAllAttachedRobots(islandRobots);
          } else{
            dynIsland.push_back((DynamicBody *)mWorld->getBody(i));
          }
          break;
      }
      numDynBodies = dynIsland.size();
      for (i = 0; i < numDynBodies; i++)
        dynIsland[i]->setDynamicsFlag();

      // add any bodies that contact any body already in the dynamic island
      for (i = 0; i < numDynBodies; i++) {
        contactList = dynIsland[i]->getContacts();
        for (cp = contactList.begin(); cp != contactList.end(); cp++) {
          // if the contacting body is dynamic and not already in the list, add it
          if ((*cp)->getBody2()->isDynamic() &&
            !((DynamicBody *)(*cp)->getBody2())->dynamicsComputed()) {
              DynamicBody *contactedBody = (DynamicBody *)(*cp)->getBody2();

              // is this body is a link, add all robots connected to the link
              if (contactedBody->isA("Link")) {
                Robot *robot = ((Robot *)((Link *)contactedBody)->getOwner())->getBaseRobot();
                robot->getAllLinks(robotLinks);
                robot->getAllAttachedRobots(islandRobots);
                numLinks = robotLinks.size();
                for (j = 0; j < numLinks; j++)
                  if (!robotLinks[j]->dynamicsComputed()) {
                    dynIsland.push_back(robotLinks[j]);
                    robotLinks[j]->setDynamicsFlag();
                    numDynBodies++;
                  }
                  robotLinks.clear();
              } else {
                dynIsland.push_back(contactedBody);
                contactedBody->setDynamicsFlag();
                numDynBodies++;
              }
          }
        }
      }

      numIslandRobots = islandRobots.size();

#ifdef GRASPITDBG
      std::cout << "Island " << ++islandCount << " Bodies: ";
      for ( i = 0; i < numDynBodies; i++)
        std::cout << dynIsland[i]->getName() << " ";
      std::cout << std::endl;
      std::cout << "Island Robots"<< islandCount<<" Robots: ";
      for ( i = 0; i < numIslandRobots; i++)
        std::cout << islandRobots[i]->getName() <<" ";
      std::cout << std::endl << std::endl;
#endif

      for (i = 0; i < numDynBodies; i++)
        dynIsland[i]->markState();

      DynamicParameters dp;
      if (numDynBodies > 0) {
        dp.timeStep = timeStep;
        dp.useContactEps = true;
        dp.gravityMultiplier = 1.0;
        lemkeErrCode = iterateDynamics(islandRobots, dynIsland, &dp);

        if (lemkeErrCode == 1){  // dynamics could not be solved
          std::cerr << "LCP COULD NOT BE SOLVED!"<<std::endl<<std::endl;
          turnOffDynamics();
          return -1;
        }
      }

      dynIsland.clear(); 
      islandRobots.clear();
      allDynamicsComputed = true;
      for (i = 0; i < numBodies; i++)
        if (mWorld->getBody(i)->isDynamic() &&
          !((DynamicBody *)mWorld->getBody(i))->dynamicsComputed()) {
            allDynamicsComputed = false;
            break;
        }
  }  while (!allDynamicsComputed);

  // clear all the dynamicsComputed flags
  for (i = 0; i < numBodies; i++)
    if (mWorld->getBody(i)->isDynamic())
      ((DynamicBody *)mWorld->getBody(i))->resetDynamicsFlag();

  mWorld->emitDynamicStepTaken();
  return 0;
}
