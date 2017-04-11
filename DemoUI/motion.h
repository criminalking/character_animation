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

#ifndef MOTION_H
#define MOTION_H

#include "../Pinocchio/transform.h"

class Motion
{
public:
  Motion(const string &filename, bool flag = false);
    void get_size() {std::vector<Vector3> p = poses[0];}

    bool empty() const { return data.empty(); }

    vector<Transform<> > get() const { return data[getFrameIdx()]; }
    vector<Vector3> getPose() const { return poses[getFrameIdx()]; }
    vector<Vector3> getRefPose() const { return refPose; }
    double getLegLength() const { return legLength; }
    double getLegWidth() const { return legWidth; }
    bool getUseMyOwnMotion() const { return useMyOwnMotion; }
    Vector3 getRoot() const { return root; }

    const vector<vector<Transform<> > > &getData() const { return data; } // not activated
    void setFixedFrame(int inFrame) { fixedFrame = inFrame < 0 ? -1 : (int)(inFrame % data.size()); } // not activated
private:
    int getFrameIdx() const;
    void readH(istream &strm); // orginal read motion
    void readMotion(istream &strm); // my own read motion
    vector<vector<Transform<> > > data; // data.size is number of row in motion.txt(except 0 and #), data.size = 17
    vector<vector<Vector3> > poses;
    vector<Vector3> refPose; // 19 points, TODO: why?
    double legLength;
    double legWidth;
    int fixedFrame;
    bool useMyOwnMotion;
    Vector3 root;
};

#endif
