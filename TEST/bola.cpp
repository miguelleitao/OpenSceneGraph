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


osg::Drawable* createShrub(const float & scale, osg::StateSet* bbState)
{
   float width = 1.5f;
   float height = 3.0f;

   width *= scale;
   height *= scale;

   osg::Geometry* shrubQuad = new osg::Geometry;

   osg::Vec3Array* shrubVerts = new osg::Vec3Array(4);
   (*shrubVerts)[0] = osg::Vec3(-width/2.0f, 0, 0);
   (*shrubVerts)[1] = osg::Vec3( width/2.0f, 0, 0);
   (*shrubVerts)[2] = osg::Vec3( width/2.0f, 0, height);
   (*shrubVerts)[3] = osg::Vec3(-width/2.0f, 0, height);

   shrubQuad->setVertexArray(shrubVerts);

   osg::Vec2Array* shrubTexCoords = new osg::Vec2Array(4);
   (*shrubTexCoords)[0].set(0.0f,0.0f);
   (*shrubTexCoords)[1].set(1.0f,0.0f);
   (*shrubTexCoords)[2].set(1.0f,1.0f);
   (*shrubTexCoords)[3].set(0.0f,1.0f);
   shrubQuad->setTexCoordArray(0,shrubTexCoords);

   shrubQuad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

   // Need to assign a color to the underlying geometry, otherwise we'll get
   // whatever color is current applied to our geometry.
   // Create a color array, add a single color to use for all the vertices

   osg::Vec4Array* colorArray = new osg::Vec4Array;
   colorArray->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); // white, fully opaque

   // An index array for assigning vertices to colors (based on index in the array)
   osg::TemplateIndexArray
      <unsigned int, osg::Array::UIntArrayType,4,1> *colorIndexArray;
   colorIndexArray = 
      new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType,4,1>;
   colorIndexArray->push_back(0);

   // Use the index array to associate the first entry in our index array with all 
   // of the vertices.
   shrubQuad->setColorArray( colorArray);
   //shrubQuad->setColorIndices(colorIndexArray);    jml 2018
   shrubQuad->setColorBinding(osg::Geometry::BIND_OVERALL);

   shrubQuad->setStateSet(bbState); 

   return shrubQuad;
}


int main()
{
	double z_bola = 0.;
	double v_bola = 0.;
	osg::Matrix myMatrix;

	// Creating the root node
        osg::Group* SceneRoot = new osg::Group;

	// Bola
        osg::Node* loadedModel = osgDB::readNodeFile("bola.obj");	
    	//SceneRoot->addChild(loadedModel); 

	osg::PositionAttitudeTransform * bpos = new osg::PositionAttitudeTransform();
        bpos->setPosition(osg::Vec3d(0.,20.,0.));
	bpos->addChild(loadedModel);
        SceneRoot->addChild(bpos);

	osg::Node* myPlanode = new osg::Planode();
	SceneRoot->addChild(myPlanode); 

	osg::Billboard* myBillboard = new osg::Billboard();
	SceneRoot->addChild(myBillboard); 

	myBillboard->addChild(loadedModel); 
///
   osg::Billboard* shrubBillBoard = new osg::Billboard();
   SceneRoot->addChild(shrubBillBoard);

   shrubBillBoard->setMode(osg::Billboard::AXIAL_ROT);
   shrubBillBoard->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
   shrubBillBoard->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));
///


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
