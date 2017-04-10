/*
Copyright (c) 2007 Ilya Baran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef DEFMESH_H_INCLUDED
#define DEFMESH_H_INCLUDED

#include "../Pinocchio/attachment.h"
#include "filter.h"
#include "DisplayMesh.h"

class Motion;

class DefMesh : public DisplayMesh
{
public:
    DefMesh(const Mesh inMesh, const Skeleton &inOrigSkel, const vector<Vector3> &inMatch, const Attachment &inAttachment, Motion *inMotion = NULL)
      : origSkel(inOrigSkel), match(inMatch), attachment(inAttachment),
        origMesh(inMesh), motion(inMotion), filter(match, origSkel.fPrev())
    {
      transforms.resize(origSkel.fPrev().size() - 1); // tranform's size is bones' size
    }

    void setMotion(Motion *inMotion) { motion = inMotion; }
    Motion *getMotion() const { return motion; }
    void updateIfHasMotion() const { if(motion) updateMesh(); } // never activated

    vector<Vector3> getSkel() const;
    const Skeleton &getOrigSkel() const { return origSkel; }

    const Attachment &getAttachment() const { return attachment; }

    const Mesh &getMesh(bool useMyOwnTransform = false) { if (!useMyOwnTransform) updateMesh(); else updateMesh2(); return curMesh; }

private:
    double getLegRatio() const;
    vector<Transform<> > computeTransforms() const;
    void updateMesh() const;
    void updateMesh2();

    Skeleton origSkel;
    vector<Vector3> match; // vector<Vector3> o.embedding, size is 18
    Attachment attachment;
    Mesh origMesh;
    Motion *motion;
    mutable Mesh curMesh; // const function could change this
    vector<Quaternion<> > transforms;

    vector<double> footOffsets; // footOffsets[0]:left foot, footOffsets[1]:right foot
    mutable MotionFilter filter; // const function getSkel() could change it, TODO: 这里好奇怪，filter并没有给参数，但是motionfilter class中是不带默认构造函数的。。。
};

#endif //DEFMESH_H_INCLUDED
