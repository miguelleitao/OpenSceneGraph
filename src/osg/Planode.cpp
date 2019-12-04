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
fprintf(stderr,"Vai alocar verts\n");
    planodeVerts = new osg::Vec3Array(4);
    (*planodeVerts)[0].set( osg::Vec3( 10.,  10., 0.) );
    (*planodeVerts)[1].set( osg::Vec3( 10., -10., 0.) );
    (*planodeVerts)[2].set( osg::Vec3(-10., -10., 0.) );
    (*planodeVerts)[3].set( osg::Vec3(-10.,  10., 0.) );
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
    planodeQuad->setDataVariance(osg::Object::DYNAMIC);

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
  // _children[0]->accept(nv);

   //float eye_dist = nv.getDistanceToViewPoint(getCenter(),true);
    osg::Vec3 eye = nv.getEyePoint();
    osg::Vec3 view = nv.getViewPoint();

    fprintf(stderr, "traversing Planode with %ld children, eye %.2f %.2f %.2f view %.2f %.2f %.2f\n",
		    _children.size(), eye[0],eye[1],eye[2], view[0],view[1],view[2]);

/*
fprintf(stderr,"Dists %.2f %.2f %.2f\n", nv.getDistanceToEyePoint(osg::Vec3(0,0,0),true), nv.getDistanceFromEyePoint(osg::Vec3(0,0,0),true), nv.getDistanceToViewPoint(osg::Vec3(0,0,0),true));
*/

    float dist = nv.getDistanceToEyePoint(osg::Vec3(0,0,0),true);

	osgUtil::CullVisitor *cv = nv.asCullVisitor();

	osg::Matrixd proj = nv.asCullVisitor()->getCurrentCamera()->getProjectionMatrix();

	//osg::Matrixd mv   = nv.getCurrentCamera()->getViewMatrix();
    
    //nv.getEyePoint
    //nv.getDistanceToEyePoint

	//osg::Matrixd temp = proj * mv;
    
	osg::Matrixd inv;
    inv.invert_4x4(proj); // compute inverse of matrix
	 
	const osg::Vec4 fr[8]= {
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
	    tfr[i] = inv * fr[i];
	    tfr[i][0] /= tfr[i][3];
	    tfr[i][1] /= tfr[i][3];
	    tfr[i][2] /= tfr[i][3];
	    tfr[i][3] = 1.0f;
    }
    //    osgUtil::CullVisitor *cv = nv.asCullVisitor();
	if ( cv ) {
	    osg::Camera* cam = cv->getCurrentCamera();
	    if (cam) {
		fprintf(stderr," ######## obteve camera\n");
		osg::Vec3d c_eye, c_center, c_up;
		cam->getViewMatrixAsLookAt (c_eye, c_center, c_up, 1.0);
    		fprintf(stderr, "         camera with eye %.2f %.2f %.2f view %.2f %.2f %.2f\n",
		    c_eye[0],c_eye[1],c_eye[2], c_center[0],c_center[1],c_center[2]);
		// ok. 
		double fovy, aRatio, zNear, zFar;
		cam->getProjectionMatrixAsPerspective(fovy,aRatio,zNear,zFar);
		double fovx = fovy*aRatio;
		printf("          fovx %.4f fovy%.4f, aRatio %.4f, Near %.3f Far %.2f\n",
				fovx, fovy, aRatio, zNear, zFar);
		
		// Viewing direction
		osg::Vec3d vdir = c_center - c_eye;

		// Screen side vector
		osg::Vec3d c_side = vdir ^ c_up;

		// 4 frustum edges 
		osg::Matrixd rMat, lMat, tMat, bMat;
		rMat.makeRotate( fovx/2., c_up);
		lMat.makeRotate(-fovx/2., c_up);
                tMat.makeRotate( fovy/2., c_side);
                bMat.makeRotate(-fovy/2., c_side);

		osg::Matrixd matVertex[4];
		matVertex[0] = rMat * tMat;
		matVertex[1] = rMat * bMat;
                matVertex[2] = lMat * bMat;
                matVertex[3] = lMat * tMat;

		// 4 rays
		osg::Vec3d frustRay[4];
		double kk = dotProductNormal( c_eye );
		for( i=0 ; i<4 ;i++ ) {
		    frustRay[i] = osg::Matrix::transform3x3( vdir, matVertex[i] );
		    double rr = dotProductNormal( frustRay[i] );
		    if ( abs(rr)>0.0000001 ) {
			double iDist = kk/rr;
			if ( iDist>zFar ) {
			//	iDist = zFar;
			}
			else if ( iDist<zNear ) {
			//	iDist = zNear;
			}
		        // calc intersection point
		        osg::Vec3d iPoint = eye + frustRay[i]*iDist;
		        fprintf(stderr, "          kk=%f, ipoint[%d]=%.2f %.2f %.2f \n",
					kk,i,iPoint[0],iPoint[1],iPoint[2]);
		        (*planodeVerts)[i].set( iPoint );
		    }
		}

		// Old implementation
		// ONE ray and 4 points in a rectangle
		//fprintf(stderr, "          plano=%.2f %.2f %.2f, d=%f \n",_fv[0],_fv[1],_fv[2],_fv[3]);
		//fprintf(stderr, "          vdir=%.2f %.2f %.2f, len=%f \n",vdir[0],vdir[1],vdir[2],vdir.length());
		// dot product between  Normal and view vector
		double r = dotProductNormal( vdir );
		//double r = _fv[0]*vdir[0]+_fv[1]*vdir[1]+_fv[2]*vdir[2];
		if ( abs(r)>0.00000001 ) {
			double k = dotProductNormal( c_eye );
			//double k = -(_fv[0]*c_eye[0]+_fv[1]*c_eye[1]+_fv[2]*c_eye[2]+_fv[3]) / r; 
			osg::Vec3d iPoint = eye + vdir*k;
			fprintf(stderr, "          k=%f, ipoint=%.2f %.2f %.2f \n",k, iPoint[0],iPoint[1],iPoint[2]);
			    (*planodeVerts)[0].set( iPoint + (osg::Vec3( 1,  1, 0.) *dist) );
			    (*planodeVerts)[1].set( iPoint + (osg::Vec3( 1, -1, 0.) *dist) );
			    (*planodeVerts)[2].set( iPoint + (osg::Vec3(-1, -1, 0.) *dist) );
			    (*planodeVerts)[3].set( iPoint + (osg::Vec3(-1,  1, 0.) *dist) );
		}
		osg::Matrixd c_proj = cam->getProjectionMatrix();
	    }
	}

    planodeVerts->dirty();

    for( unsigned int i=0 ; i<_children.size() ; i++ ) {
        getDrawable(i)->dirtyDisplayList();
        getDrawable(i)->dirtyBound();
    }
    
    dirtyBound();
    computeBound();

    fprintf(stderr,"end\n");


    //getParent(0)->dirtyBound();

   //setCullingActive(false);
    
}

bool Planode::computeMatrix(Matrix& modelview, const Vec3& eye_local, const Vec3& pos_local) const
{
    //Vec3 up_local(matrix(0,1),matrix(1,1),matrix(2,1));
fprintf(stderr, "#### computing matrix Planode eye %f %f %f pos %f %f %f\n",eye_local[0],eye_local[1],eye_local[2], pos_local[0],pos_local[1],pos_local[2]);

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

    /*
    fprintf(stderr,"    OLD sphere center %f %f %f radius %f\n", 
	_boundingSphere._center[0], _boundingSphere._center[1], _boundingSphere._center[2], _boundingSphere._radius);
    */

    // Compute poligon center
    osg::Vec3 total = osg::Vec3(0,0,0);
    for( int i=0 ; i<4 ; i++ )
	total += (*planodeVerts)[i];
    fprintf(stderr,"    total  %f %f %f \n", total[0], total[1], total[2]);

    BoundingSphere bsphere;
    bsphere._center.set(total / 4.);

    //bsphere._center /= (float)(ngsets);

    // Compute max distace from center to vertex
    float maxDist = 0.f;    
    for( int i=0   ; i<4 ; i++ ) {
	osg::Vec3 ij = (*planodeVerts)[i]-bsphere._center;
	float dij = ij.length2();
	if ( dij>maxDist ) maxDist = dij;
	//fprintf(stderr,"dist %d-center = %f\n", i,dij);
    }
    bsphere._radius = sqrt(maxDist);

    //bsphere._radius = 4*getDrawable(1)->getGLObjectSizeHint();

    fprintf(stderr,"    sphere center %f %f %f radius %f\n", bsphere._center[0], bsphere._center[1], bsphere._center[2], bsphere._radius);

    return bsphere;
}




