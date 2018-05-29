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

using namespace osg;

#define square(x)   ((x)*(x))

/*
Planode(Plane pl) : Plane(pl)
{

}
*/
Planode::Planode()
{
fprintf(stderr,"init planode\n");
    osg::Shape *sp = new osg::Box( osg::Vec3(0.,0.,0.), 10., 10., 10. );
    if ( sp ) {
          osg::ShapeDrawable *sd = new osg::ShapeDrawable(sp);
          if ( sd ) {
		bool res = addDrawable(sd);
		if (res) fprintf(stderr,"added planode\n");
	  }
          else fprintf(stderr,"Error creating osg::Shape\n");
    } else fprintf(stderr,"Error creating osg::Shape\n");
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
    // Planode can only have ONE gset.
    if ( _children.size() == 0 ) 
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

    _children[0]->accept(nv);

    //fprintf(stderr, "traversing Planode\n");
}

bool Planode::computeMatrix(Matrix& modelview, const Vec3& eye_local, const Vec3& pos_local) const
{
    //Vec3 up_local(matrix(0,1),matrix(1,1),matrix(2,1));

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

    BoundingSphere bsphere;
    bsphere._center.set(0.0f,0.0f,0.0f);

    bsphere._center /= (float)(ngsets);

    bsphere._radius = 20;

    return bsphere;
}
