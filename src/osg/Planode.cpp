/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/
#include <stdio.h>
#include <math.h>

#include <osg/Planode>
#include <osg/ShapeDrawable>
#include <osgUtil/UpdateVisitor>
#include <osgUtil/CullVisitor>

using namespace osg;

#define square(x)   ((x)*(x))

/*
Planode(Plane pl) : Plane(pl)
{

}
*/
Planode::Planode() : Plane(0.,0.,1.,0)
{
fprintf(stderr,"init planode\n");

    osg::Shape *sp = new osg::Box( osg::Vec3(0.,0.,0.), 1., 1., 1. );
    if ( sp ) {
          osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
          if ( sd ) {
		bool res = addDrawable(sd);
		if (res) fprintf(stderr,"added planode\n");
	  }
          else fprintf(stderr,"Error creating osg::Shape\n");
    } else fprintf(stderr,"Error creating osg::Shape\n");


    // Single polygon
    osg::ref_ptr<osg::Geometry> planodeQuad = new osg::Geometry;

    // Vertexes
    //osg::ref_ptr<osg::Vec3Array> 
    planodeVerts = new osg::Vec3Array(4);
    (*planodeVerts)[0].set( osg::Vec3( 10.,  10., 0.) );
    (*planodeVerts)[1].set( osg::Vec3( 10., -10., 0.) );
    (*planodeVerts)[2].set( osg::Vec3(-10., -10., 0.) );
    (*planodeVerts)[3].set( osg::Vec3(-10.,  10., 0.) );
fprintf(stderr,"Vai alocar verts\n");
    planodeQuad->setVertexArray(planodeVerts);
fprintf(stderr,"alocou verts\n");

    // Color
    osg::ref_ptr<osg::Vec4Array> planodeColor = new osg::Vec4Array;
    planodeColor->push_back( osg::Vec4(1., 1., 1., 1.) );
    planodeQuad->setColorArray(planodeColor);
    planodeQuad->setColorBinding(osg::Geometry::BIND_OVERALL);
/*
    // Texture Verts
    osg::ref_ptr<osg::Vec2Array> planodeTexCoords = new osg::Vec2Array(4);
    (*planodeTexCoords)[0] = osg::Vec2( 1.,  1.);
    (*planodeTexCoords)[1] = osg::Vec2( 1., -1.);
    (*planodeTexCoords)[2] = osg::Vec2(-1., -1.);
    (*planodeTexCoords)[3] = osg::Vec2(-1.,  1.);
    //planodeQuad->setTexCoordArray(0, planodeTexCoords);
*/
    // Normal
    osg::ref_ptr<osg::Vec3Array> planodeNormal = new osg::Vec3Array(1);
    (*planodeNormal)[0].set( osg::Vec3(0., 0., 1.) );
    planodeQuad->setNormalArray ( planodeNormal );
    planodeQuad->setNormalBinding( osg::Geometry::BIND_OVERALL );

    planodeQuad->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4) );

    // Attach Texture to state of the geode
    osg::ref_ptr<osg::StateSet> planodeState( getOrCreateStateSet() );

    setName("Ground");
    bool res = addDrawable(planodeQuad);
		if (res) fprintf(stderr,"added quad planode\n");
}



/*
Planode::Planode(const Planode& planode,const CopyOp& copyop):
        Geode(planode,copyop), Plane(planode,copyop)
{

}
*/
Planode::~Planode()
{
}


bool Planode::addDrawable(Drawable *gset)
{
fprintf(stderr,"adicionando filho a planode %ld\n", _children.size());
    // Planode can only have ONE gset.
  //  if ( _children.size() == 0 ) 
        return Geode::addDrawable(gset);
    return false;
}

bool Planode::removeDrawable( )
{
    if ( _children.size() > 0 ) {
	    _children.erase(_children.begin());
   	    dirtyBound();
            return true;
    }
    return false;
}

void Planode::traverse(NodeVisitor& nv) {
    for( unsigned int i=0 ; i<_children.size() ; i++ ) 
	_children[i]->accept(nv);


   //float eye_dist = nv.getDistanceToViewPoint(getCenter(),true);
    osg::Vec3 eye = nv.getEyePoint();
    osg::Vec3 view = nv.getViewPoint();
    

   // _children[0]->accept(nv);

    fprintf(stderr, "traversing Planode with %ld children, eye %.2f %.2f %.2f view %.2f %.2f %.2f\n",
		    _children.size(), eye[0],eye[1],eye[2], view[0],view[1],view[2]);

//    for( int i=0 ; i<4 ; i++ ) 
//	_children[1]

fprintf(stderr,"Dists %.2f %.2f %.2f\n", nv.getDistanceToEyePoint(osg::Vec3(0,0,0),true), nv.getDistanceFromEyePoint(osg::Vec3(0,0,0),true), nv.getDistanceToViewPoint(osg::Vec3(0,0,0),true));

        float dist = nv.getDistanceToEyePoint(osg::Vec3(0,0,0),true);

	osgUtil::CullVisitor *cv = nv.asCullVisitor();
	if ( cv ) {
	    osg::Camera* cam = cv->getCurrentCamera();
	    if (cam) {
		fprintf(stderr," ######## obteve camaere\n");
		osg::Vec3d c_eye, c_center, c_up;
		cam->getViewMatrixAsLookAt (c_eye, c_center, c_up, 1.0);
    		fprintf(stderr, "         camera with eye %.2f %.2f %.2f view %.2f %.2f %.2f\n",
		    c_eye[0],c_eye[1],c_eye[2], c_center[0],c_center[1],c_center[2]);
		// ok. 
		// calc intersection point
		osg::Vec3d vdir = c_center - c_eye;
		fprintf(stderr, "          plano=%.2f %.2f %.2f, d=%f \n",_fv[0],_fv[1],_fv[2],_fv[3]);
		fprintf(stderr, "          vdir=%.2f %.2f %.2f, len=%f \n",vdir[0],vdir[1],vdir[2],vdir.length());
		double r = _fv[0]*vdir[0]+_fv[1]*vdir[1]+_fv[2]*vdir[2];
		if ( abs(r)>0.00000001 ) {
			double k = -(_fv[0]*c_eye[0]+_fv[1]*c_eye[1]+_fv[2]*c_eye[2]+_fv[3]) / r; 
			osg::Vec3d iPoint = eye + vdir*k;
			fprintf(stderr, "          k=%f, ipoint=%.2f %.2f %.2f \n",k, iPoint[0],iPoint[1],iPoint[2]);

		}
		osg::Matrixd c_proj = cam->getProjectionMatrix();
		
		//osg::Matrixd temp = proj * mv;
		//osg::Matrixd inv = inverse(temp); // compute inverse of matrix
	    }
	}
	//osg::Matrixd proj = nv.asCullVisitor()->getCurrentCamera()->getProjectionMatrix();
/*
	osg::Matrixd mv   = nv.getCurrentCamera()->getViewMatrix();
	osg::Matrixd temp = proj * mv;
	osg::Matrixd inv = inverse(temp); // compute inverse of matrix
	 
	VECTOR4 fr[8]= {
		// near
		{-1, -1, -1, 1}, { 1, -1, -1, 1}, { 1,  1, -1, 1},  {-1,  1, -1, 1},
		// far
		{-1, -1, 1, 1},	{ 1, -1, 1, 1},	{ 1,  1, 1, 1},  {-1,  1, 1, 1}
	};

	// Transform all vertices:
	// multiply vertex array (fr) by matrix. result is transformed vertex array (tfr)
	osg::Vec4 tfr[8];
	//transform_points(fr, 8, inv, tfr); 
	 
	int i;
	for (i=0; i<8; i++)
	{
	    tfr = inv * fr;
	    tfr[i].x /= tfr[i].w;
	    tfr[i].y /= tfr[i].w;
	    tfr[i].z /= tfr[i].w;
	    tfr[i].w = 1.0f;
	}
	(*planodeVerts)[0].set( tfr[4] );
        (*planodeVerts)[1].set( tfr[5] );
        (*planodeVerts)[2].set( tfr[6] );
        (*planodeVerts)[3].set( tfr[7] ); */
 
/*
glBegin(GL_LINES);
connect tfr points as follow:
0-1, 1-2, 2-3, 3-0, 
4-5, 5-6, 6-7, 7-4,
0-4, 1-5, 2-6, 3-7
glEnd();

*/

    (*planodeVerts)[0].set( osg::Vec3( dist,  dist, 0.) );
    (*planodeVerts)[1].set( osg::Vec3( dist, -dist, 0.) );
    (*planodeVerts)[2].set( osg::Vec3(-dist, -dist, 0.) );
    (*planodeVerts)[3].set( osg::Vec3(-dist,  dist, 0.) );

    planodeVerts->dirty();
    dirtyBound();
computeBound();
    
}

bool Planode::computeMatrix(Matrix& modelview, const Vec3& eye_local, const Vec3& pos_local) const
{
    //Vec3 up_local(matrix(0,1),matrix(1,1),matrix(2,1));
fprintf(stderr, "computing matrix Planode eye %f %f %f pos %f %f %f\n",eye_local[0],eye_local[1],eye_local[2], pos_local[0],pos_local[1],pos_local[2]);

    Matrix matrix;

    Vec3 ev(eye_local-pos_local);

    matrix.setTrans(pos_local);

    modelview.preMult(matrix);

    return true;

}

BoundingSphere Planode::computeBound() const
{
    int ngsets = _children.size();

    if( ngsets == 0 ) return BoundingSphere();

    osg::Vec3 total = osg::Vec3(0,0,0);
    for( int i=0 ; i<4 ; i++ )
	total += (*planodeVerts)[i];


    BoundingSphere bsphere;
    //bsphere._center.set(0.0f,0.0f,0.0f);
    bsphere._center.set(total / 4.);

    //bsphere._center /= (float)(ngsets);

    float maxDist = 0.f;
    
    for( int i=0   ; i<3 ; i++ )
    for( int j=i+1 ; j<4 ; j++ ) {
	osg::Vec3 ij = (*planodeVerts)[i]-(*planodeVerts)[j];
	float dij = ij.length2();
	if ( dij>maxDist ) maxDist = dij;
fprintf(stderr,"dist %d-%d = %f\n", i,j,dij);
    }
    bsphere._radius = sqrt(maxDist);

    //bsphere._radius = 4*getDrawable(1)->getGLObjectSizeHint();

    fprintf(stderr,"    sphere radius %f\n", bsphere._radius);
    bsphere._radius = 20;

    return bsphere;
}
