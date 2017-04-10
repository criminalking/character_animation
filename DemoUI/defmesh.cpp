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

#include "defmesh.h"
#include "motion.h"
#include "../Pinocchio/intersector.h"

vector<Transform<> > DefMesh::computeTransforms() const
{
  vector<Transform<> > out;
  int i;

  if(motion) {
    vector<Transform<> > ts;
    ts = motion->get(); // data[getFrameIdx()], size is 17, should be ref trans

    double legRatio = getLegRatio();
    Vector3 trans = ts[0].getTrans() * legRatio; // get root's translation

    for(int times = 0; times < 2; ++times) {

      if(times == 1)
        trans += (out[0] * match[0] - out[1] * match[2]); // TODO: why???

      out.clear();
      vector<Vector3> tm;
      tm.push_back(match[0]); // root joint's embedding
      ts[0] = ts[0].linearComponent(); // discard ts[0]'s trans, keep rotation and scale(because 'trans' already get ts[0]'s trans)
      for(i = 0; i < (int)ts.size(); ++i) { // 0-16
        int prevV = origSkel.fPrev()[i + 1];
        out.push_back(Transform<>(tm[prevV]) * ts[i] * Transform<>(-match[prevV]));
        tm.push_back(out.back() * match[i + 1]);
      }

      for(i = 0; i < (int)out.size(); ++i) // 0-16
        out[i] = Transform<>(trans + Vector3(0.5, 0, 0.5)) * out[i];
    }

    return out;
  }

  out.push_back(Transform<>(Vector3(0.5, 0, 0.5))); // only have trans

  for(i = 1; i < (int)origSkel.fPrev().size(); ++i) { // 1-17
    int prevV = origSkel.fPrev()[i]; // fPrev here is index
    Transform<> cur = out[prevV];
    cur = cur * Transform<>(match[prevV]) * Transform<>(transforms[i - 1]) * Transform<>(-match[prevV]); //TODO: ??? here transforms has no value

    out.push_back(cur);
  }

  out.erase(out.begin());
  return out;
}

bool reallyDeform = true;

void DefMesh::updateMesh2() // update for attachment
{
  vector<Vector3> pose = motion->getPose(); // joints of one frame
  vector<Transform<> > t; // t.size() = joints' size - 1
  joints.clear();
  // compute every bone's transforms
  // if this is the first frame, should remember its root
  Vector3 root = motion->getRoot();
  vector<Vector3> trans; // translation of joints
  trans.push_back((root - pose[0])*1); // trans[0]
  joints.push_back(match[0] + root - pose[0]); // joints[0]
  for (int i = 1; i < (int)origSkel.fPrev().size(); ++i)
    {
      int prevV = origSkel.fPrev()[i];
      Quaternion<> rot(match[i] - match[prevV], pose[i] - pose[prevV]);
      t.push_back(Transform<>(rot, 1, trans[prevV])); // t[i-1]
      joints.push_back(t.back() * match[i]); // child's new position
      trans.push_back(joints[i] - match[i]); // child's translation
    }
  curMesh = attachment.deform(origMesh, t); // normal LBS
}

void DefMesh::updateMesh() const // every frame should update mesh
{
  vector<Transform<> > t = computeTransforms(); // size is 17, 17 bones' transforms(without R0)

  if(motion) {
    if(footOffsets.empty()) { // only once
      Intersector s(origMesh, Vector3(0, 1, 0)); // projection plane is (x,z)

      vector<Vector3> sects;
      double offs;

      sects = s.intersect(match[7]); // left foot, size is 4
      offs = 0;
      for(int i = 0; i < (int)sects.size(); ++i)
        offs = max(offs, match[7][1] - sects[i][1]);
      const_cast<vector<double> *>(&footOffsets)->push_back(offs); // y offset of left foot

      sects = s.intersect(match[11]); // right foot, size is 2
      offs = 0;
      for(int i = 0; i < (int)sects.size(); ++i)
        offs = max(offs, match[11][1] - sects[i][1]);
      const_cast<vector<double> *>(&footOffsets)->push_back(offs); // y offset of right foot
    }

    vector<Vector3> pose = motion->getPose(); // 36, only used here
    vector<Vector3> refPose = motion->getRefPose(); // 19, only used here
    vector<Vector3> feet;

    double legRatio = getLegRatio();
    feet.push_back(pose[15] * legRatio); // rthign?
    feet.push_back(pose[7] * legRatio); // lthign?

    double widthDiff = 0.3 * ((refPose[8][0] - refPose[4][0]) * legRatio - (match[7][0] - match[11][0])); // 0.3 * (refpose's foot - o.embedding's foot)
    Vector3 offs1 = t[6].getRot() * Vector3(-widthDiff, 0, 0);
    Vector3 offs2 = t[10].getRot() * Vector3(widthDiff, 0, 0);
    offs1[1] = offs2[1] = 0;
    offs1 += /*t[6].getRot() */ Vector3(0, footOffsets[0], 0);
    offs2 += /*t[10].getRot() */ Vector3(0, footOffsets[1], 0);
    feet[0] += offs1;
    feet[1] += offs2;

    Vector3 pelvisVec = (refPose[0] - 0.5 * (refPose[4] + refPose[8])) * legRatio;
    Vector3 mpelvisVec = (match[2] - 0.5 * (match[7] + match[11]));
    mpelvisVec += Vector3(0, min(footOffsets[0], footOffsets[1]), 0);
    Vector3 v(0, 1, 0);
    feet.push_back(pose[0] * legRatio + v * (v * (mpelvisVec - pelvisVec)));
    //feet.back()[1] = 0.;

#if 0
    Debugging::clear();
    for(int i = 0; i < (int)feet.size(); ++i)"?{}"
                                               }/////
      Debugging::drawCircle(feet[i], 0.01, QPen(Qt::blue));

    Debugging::drawLine(feet[0], feet[0] - offs1, QPen(Qt::red));
    Debugging::drawLine(feet[1], feet[1] - offs2, QPen(Qt::red));
#endif

    filter.step(t, feet); // do online motion retargeting, get new transforms
    if(reallyDeform) // always true
      curMesh = attachment.deform(origMesh, filter.getTransforms()); // the second parameter is 17 bones' transforms

#if 0
    static int period = 1;
    if(--period == 0) {
      period = 3;
      if(rand() % 40 == 0)
        Debugging::clear();
      PtGraph skelGraph = origSkel.fGraph();
      skelGraph.verts = getSkel();
      Debugging::drawGraph(skelGraph, QPen(Qt::red, 5));
    }
#endif
  }
  else // no motion
    curMesh = attachment.deform(origMesh, t); // normal LBS
}

vector<Vector3> DefMesh::getSkel(bool useMyOwnTransform) const // final joints position
{
  if (!useMyOwnTransform)
    {
      vector<Vector3> out = match; // o.embedding

      vector<Transform<> > t;
      if(motion)
        t = filter.getTransforms(); // 17
      else // static
        t = computeTransforms(); // 17

      for(int i = 0; i < (int)out.size(); ++i) { // 0-17
        out[i] = t[max(0, i - 1)] * out[i]; // transform to get the new joints
      }
      return out; // location of joints, 18
    }
  else
    {
      //vector<Vector3> pose = motion->getPose(); // joints of one frame
      return joints;
    }
}

double DefMesh::getLegRatio() const
{
  // (lfoot.y - hips.y) / (lthigh.y - shoulder.y)
  double lengthRatio = fabs(match[7][1] - match[2][1]) / motion->getLegLength();
  // (lfoot.x - rfoot.x) / (lthign.x - rthign.x)
  double widthRatio = fabs(match[7][0] - match[11][0]) / motion->getLegWidth();

  return lengthRatio;
  return max(lengthRatio, min(lengthRatio * 1.4, sqrt(lengthRatio * widthRatio)));
}
