/**********************************************************************
 *
 *    FILE:            Viewport.cpp
 *
 *    DESCRIPTION:    Read/Write osg::Viewport in binary format to disk.
 *
 *    CREATED BY:        Auto generated by iveGenerated
 *                    and later modified by Rune Schmidt Jensen.
 *
 *    HISTORY:        Created 21.3.2003
 *
 *    Copyright 2003 VR-C
 **********************************************************************/

#include "Exception.h"
#include "Viewport.h"
#include "Object.h"

using namespace ive;

void Viewport::write(DataOutputStream* out){
    // Write Viewport's identification.
    out->writeInt(IVEVIEWPORT);
    // If the osg class is inherited by any other class we should also write this to file.
    osg::Object*  obj = dynamic_cast<osg::Object*>(this);
    if(obj){
        ((ive::Object*)(obj))->write(out);
    }
    else
        throw Exception("Viewport::write(): Could not cast this osg::Viewport to an osg::Object.");
    // Write Viewport's properties.

    out->writeInt(static_cast<int>(x()));
    out->writeInt(static_cast<int>(y()));
    out->writeInt(static_cast<int>(width()));
    out->writeInt(static_cast<int>(height()));
}

void Viewport::read(DataInputStream* in){
    // Peek on Viewport's identification.
    int id = in->peekInt();
    if(id == IVEVIEWPORT){
        // Read Viewport's identification.
        id = in->readInt();
        // If the osg class is inherited by any other class we should also read this from file.
        osg::Object*  obj = dynamic_cast<osg::Object*>(this);
        if(obj){
            ((ive::Object*)(obj))->read(in);
        }
        else
            throw Exception("Viewport::read(): Could not cast this osg::Viewport to an osg::Object.");

        // Read Viewport's properties
        x() = in->readInt();
        y() = in->readInt();
        width() = in->readInt();
        height() = in->readInt();

    }
    else{
        throw Exception("Viewport::read(): Expected Viewport identification.");
    }
}
