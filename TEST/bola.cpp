/*
	bola.cpp 
	Miguel Leitao, ISEP, 2008
*/


#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
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

	osg::Node* myPlanode = new osg::Planode();
	SceneRoot->addChild(myPlanode); 

	// Creating the viewer
	osgViewer::Viewer viewer ;
	viewer.setSceneData( SceneRoot );

	// Setup camera
	osg::Matrix matrix;
 	matrix.makeLookAt( osg::Vec3(0.,-30.,5.), osg::Vec3(0.,0.,0.), osg::Vec3(0.,0.,1.) );
 	viewer.getCamera()->setViewMatrix(matrix);

        //viewer.setCameraManipulator(  new osgGA::TrackballManipulator() );

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
