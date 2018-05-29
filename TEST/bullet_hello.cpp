/*
	Falling ball
	Miguel Leitao, ISEP, 2011
 */

#include <iostream> 
#include <btBulletDynamicsCommon.h>
 
int main (void)
{
 	// Create dynamic world
        btBroadphaseInterface* broadphase = new btDbvtBroadphase();
        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
        btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
 
	// Set gravity
        dynamicsWorld->setGravity(btVector3(0., 0., -9.8));
 
	// Create Object
        btCollisionShape* ballShape = new btSphereShape(1);
        btDefaultMotionState* ballMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,50)));
        btScalar mass = 1;
        btVector3 ballInertia(0,0,0);
        ballShape->calculateLocalInertia(mass,ballInertia);
        btRigidBody::btRigidBodyConstructionInfo ballRigidBodyCI(mass,ballMotionState,ballShape,ballInertia);
        btRigidBody* ballRigidBody = new btRigidBody(ballRigidBodyCI);
        dynamicsWorld->addRigidBody(ballRigidBody);

 	// Create ground
 	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,0,1),1);
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-1)));
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
        dynamicsWorld->addRigidBody(groundRigidBody);
 
 	// Simulate 
        for (int i=0 ; i<300 ; i++) {
                dynamicsWorld->stepSimulation(1/60.f,10);
 
                btTransform trans;
                ballRigidBody->getMotionState()->getWorldTransform(trans);
 
                std::cout << "sphere height: " << trans.getOrigin().getZ() << std::endl;
        }
 
	// Delete Object
        dynamicsWorld->removeRigidBody(ballRigidBody);
        delete ballRigidBody->getMotionState();
        delete ballRigidBody;
        delete ballShape;

	// Delete ground
        dynamicsWorld->removeRigidBody(groundRigidBody);
        delete groundRigidBody->getMotionState();
        delete groundRigidBody; 
 
	// Delet Dynamic world
        delete dynamicsWorld;
        delete solver;
        delete collisionConfiguration;
        delete dispatcher;
        delete broadphase;
 
        return 0;
}

