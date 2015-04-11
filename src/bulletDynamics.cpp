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
  mBtDynamicsWorld->setGravity(btVector3(5,5,5));
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
      
      //is this needed?
      mWorld->addLink(robot->getChain(f)->getLink(l));
      
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

void BulletDynamics::stepDynamics()
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
}

