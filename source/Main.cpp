#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <iostream>

int main(int argc, char *argv[])
{
  // Build the broadphase
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  // The actual physics solver
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -10, 0));
  // END BOOTSTRAPPING
  btGhostPairCallback* ghostCall = new btGhostPairCallback();
  dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(ghostCall);
  btGhostObject* ghostObj = new btGhostObject();

  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);

  btCollisionShape* fallShape = new btSphereShape(1);

  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));

  btRigidBody::btRigidBodyConstructionInfo
    groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody);

  //btTransform(x, startposition)
  btDefaultMotionState* fallMotionState =
    new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,50,0)));
  btDefaultMotionState* fallMotionState2 =
    new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,70,0)));

  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI2(mass, fallMotionState2, fallShape, fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  btRigidBody* fallRigidBody2 = new btRigidBody(fallRigidBodyCI2);
  dynamicsWorld->addRigidBody(fallRigidBody);
  dynamicsWorld->addRigidBody(fallRigidBody2);

  for (int i=0; i<300; i++) {
    dynamicsWorld->stepSimulation( 1.f/60.f, 10);

    btTransform trans;
    btTransform trans2;
    fallRigidBody->getMotionState()->getWorldTransform(trans);
    fallRigidBody2->getMotionState()->getWorldTransform(trans2);

    std::cout << "Y position: " << trans.getOrigin().getY();
    std::cout << " other body Y position: " << trans2.getOrigin().getY() << std::endl;

    //collision
    const int manifoldCount(dispatcher->getNumManifolds());
    for(int loop = 0; loop < manifoldCount; loop++) {
      const btPersistentManifold *mf = dispatcher->getManifoldByIndexInternal(loop);
      const void *obja = mf->getBody0();
      const void *objb = mf->getBody1();
      //      if(obja == fallRigidBody && objb == groundRigidBody) {
      if(obja == fallRigidBody && objb == fallRigidBody2) {
	// This manifold deals with the btRigidBody we are looking for.
	// A manifold is a RB-RB pair containing a list of potential (predicted) contact points.
	const unsigned int numContacts(mf->getNumContacts());
	for(int check = 0; check < numContacts; check++) {
	  const btManifoldPoint &pt(mf->getContactPoint(check));
	  std::cout << "HIT" << std::endl;
	  // do something here, in case you're interested.
	}
      }
    }
    ///collision
  }

  dynamicsWorld->removeRigidBody(fallRigidBody);
  delete fallRigidBody->getMotionState();
  delete fallRigidBody;

  dynamicsWorld->removeRigidBody(groundRigidBody);
  delete groundRigidBody->getMotionState();
  delete groundRigidBody;

  delete fallShape;
  delete groundShape;

  // Clean up
  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;

  return 0;
}

