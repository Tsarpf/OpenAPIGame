#include "ofApp.h"
#include "btBulletDynamicsCommon.h"

//--------------------------------------------------------------
ofApp::ofApp()
{
	m_broadphase = new btDbvtBroadphase();

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));

	m_groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	m_fallShape = new btSphereShape(m_sphereSize);


	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo m_groundRigidBodyCI(0, groundMotionState, m_groundShape, btVector3(0, 0, 0));
	m_groundRigidBody = new btRigidBody(m_groundRigidBodyCI);
	m_dynamicsWorld->addRigidBody(m_groundRigidBody);


	btDefaultMotionState* fallMotionState =	new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, m_sphereYPos, 5)));
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	m_fallShape->calculateLocalInertia(mass, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo m_fallRigidBodyCI(mass, fallMotionState, m_fallShape, fallInertia);

	m_fallRigidBody = new btRigidBody(m_fallRigidBodyCI);
	m_dynamicsWorld->addRigidBody(m_fallRigidBody);


}
ofApp::~ofApp()
{

	m_dynamicsWorld->removeRigidBody(m_fallRigidBody);
	delete m_fallRigidBody->getMotionState();
	delete m_fallRigidBody;

	m_dynamicsWorld->removeRigidBody(m_groundRigidBody);
	delete m_groundRigidBody->getMotionState();
	delete m_groundRigidBody;


	delete m_fallShape;

	delete m_groundShape;


	delete m_dynamicsWorld;
	delete m_solver;
	delete m_collisionConfiguration;
	delete m_dispatcher;
	delete m_broadphase;

}
void ofApp::setup()
{
	ofSetOrientation(OF_ORIENTATION_DEFAULT, false);
	m_camera.setFov(60);
	ofBackground(0, 0, 0);
	ofSetFrameRate(90);

	m_sphere.setRadius(m_sphereSize);

	m_sphere.setPosition(0, m_sphereYPos, 5);
}

//--------------------------------------------------------------
void ofApp::update()
{
	m_dynamicsWorld->stepSimulation(1 / 60.f, 10);

	btTransform trans;
	m_fallRigidBody->getMotionState()->getWorldTransform(trans);

	//std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;

	float x = trans.getOrigin().getX();
	float y = trans.getOrigin().getY();
	float z = trans.getOrigin().getZ();

	//std::cout << "sphere pos: " << x << " " << y << " " << " " << z << std::endl;

	m_sphere.setPosition(x, y, z);
}

//--------------------------------------------------------------
void ofApp::draw()
{
	m_camera.begin();
	ofSetHexColor(0xCCCCCC);
	m_sphere.draw();
	m_camera.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
