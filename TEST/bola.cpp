/*
	bola.cpp 
	Miguel Leitao, ISEP, 2008
*/


#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/Planode>

int main()
{
	double z_bola = 0.;
	double v_bola = 0.;
	osg::Matrix myMatrix;

	// Creating the root node
        osg::Group* SceneRoot = new osg::Group;

	// Bola
        osg::Node* loadedModel = osgDB::readNodeFile("bola.obj");	
    	SceneRoot->addChild(loadedModel); 

	osg::PositionAttitudeTransform * bpos = new osg::PositionAttitudeTransform();
        bpos->setPosition(osg::Vec3d(0.,20.,0.));
	bpos->addChild(loadedModel);
        SceneRoot->addChild(bpos);

	osg::Node* myPlanode = new osg::Planode();
	SceneRoot->addChild(myPlanode); 

	// Creating the viewer
	osgViewer::Viewer viewer;
        viewer.setUpViewInWindow(400, 400, 640, 480);
	viewer.setSceneData( SceneRoot );

	// Setup camera
	osg::Matrix matrix;
 	matrix.makeLookAt( osg::Vec3(0.,-30.,5.), osg::Vec3(0.,0.,0.), osg::Vec3(0.,0.,1.) );
 	viewer.getCamera()->setViewMatrix(matrix);

        //viewer.setCameraManipulator(  new osgGA::TrackballManipulator() );
	// Manipulator
        osg::ref_ptr<osgGA::TrackballManipulator> manipulator = new osgGA::TrackballManipulator;
        viewer.setCameraManipulator( manipulator );
        // Set the desired home coordinates for the manipulator
        osg::Vec3d eye(osg::Vec3(-15., 0., 15.));
        osg::Vec3d center(0., 0., 1.);
        // Make sure that OSG is not overriding our home position
        manipulator->setAutoComputeHomePosition(false);
        // Set the desired home position of the Trackball Manipulator
        manipulator->setHomePosition(eye, center, osg::Vec3(0.,0.,1.));
        // Force the camera to move to the home position
        manipulator->home(0.0);

	// record the timer tick at the start of rendering.
	osg::Timer myTimer;
	double time_now = myTimer.time_s();
    	double last_time = time_now;
	double frame_time;
        while( !viewer.done() )
  	{

     	  viewer.frame();
	  time_now = myTimer.time_s();
	  frame_time = time_now - last_time;
	  last_time = time_now;
  	}
}
