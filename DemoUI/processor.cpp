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

#include <FL/fl_ask.H>
#include <fstream>

#include "processor.h"
#include "../Pinocchio/skeleton.h"
#include "../Pinocchio/utils.h"
#include "../Pinocchio/debugging.h"
#include "../Pinocchio/attachment.h"
#include "../Pinocchio/pinocchioApi.h"
#include "defmesh.h"
#include "motion.h"

struct ArgData
{
  ArgData() :
    stopAtMesh(false), stopAfterCircles(false), skelScale(1.), noFit(false),
    skeleton(HumanSkeleton())
  {
  }

  bool stopAtMesh; // if true, only displays the mesh and doesn't do any analysis
  bool stopAfterCircles;
  string filename; // mesh's name, e.g. "xxx.obj"
  string motionname; // motion's file, e.g. "xxx.txt"
  Quaternion<> meshTransform; // rotation this mesh (x,y,z,deg)
  double skelScale; // scaling this mesh
  bool noFit; //  If the skeleton has already been embedded, and you just want Pinocchio to generate bone weights, use the -nofit option.
  Skeleton skeleton; // type of skeleton
  string skeletonname; // "human" or "horse" or ...
};

void printUsageAndExit()
{
  cout << "Usage: DemoUI filename.{obj | ply | off | gts | stl}" << endl;
  cout << "              [-skel skelname] [-rot x y z deg]* [-scale s]" << endl;
  cout << "              [-meshonly | -mo] [-circlesonly | -co]" << endl;
  cout << "              [-motion motionname] [-nofit]" << endl;

  exit(0);
}

ArgData processArgs(const vector<string> &args)
{
  ArgData out;
  int cur = 2;
  int num = args.size();
  if(num < 2)
    printUsageAndExit();

  out.filename = args[1];

  while(cur < num) {
    string curStr = args[cur++];
    if(curStr == string("-skel")) {
      if(cur == num) {
        cout << "No skeleton specified; ignoring." << endl;
        continue;
      }
      curStr = args[cur++];
      if(curStr == string("human"))
        out.skeleton = HumanSkeleton();
      else if(curStr == string("horse"))
        out.skeleton = HorseSkeleton();
      else if(curStr == string("quad"))
        out.skeleton = QuadSkeleton();
      else if(curStr == string("centaur"))
        out.skeleton = CentaurSkeleton();
      else
        out.skeleton = FileSkeleton(curStr);
      out.skeletonname = curStr;
      continue;
    }
    if(curStr == string("-rot")) {
      if(cur + 3 >= num) {
        cout << "Too few rotation arguments; exiting." << endl;
        printUsageAndExit();
      }
      double x, y, z, deg;
      sscanf(args[cur++].c_str(), "%lf", &x);
      sscanf(args[cur++].c_str(), "%lf", &y);
      sscanf(args[cur++].c_str(), "%lf", &z);
      sscanf(args[cur++].c_str(), "%lf", &deg);

      out.meshTransform = Quaternion<>(Vector3(x, y, z), deg * M_PI / 180.) * out.meshTransform; // (x,y,z) is the axis
      continue;
    }
    if(curStr == string("-scale")) {
      if(cur >= num) {
        cout << "No scale provided; exiting." << endl;
        printUsageAndExit();
      }
      sscanf(args[cur++].c_str(), "%lf", &out.skelScale);
      continue;
    }
    if(curStr == string("-meshonly") || curStr == string("-mo")) {
      out.stopAtMesh = true;
      continue;
    }
    if(curStr == string("-circlesonly") || curStr == string("-co")) {
      out.stopAfterCircles = true;
      continue;
    }
    if(curStr == string("-nofit")) {
      out.noFit = true;
      continue;
    }
    if(curStr == string("-motion")) {
      if(cur == num) {
        cout << "No motion filename specified; ignoring." << endl;
        continue;
      }
      out.motionname = args[cur++];
      continue;
    }
    cout << "Unrecognized option: " << curStr << endl;
    printUsageAndExit();
  }

  return out;
}

void process(const vector<string> &args, MyWindow *w)
{
  int i;
  ArgData a = processArgs(args);

  Debugging::setOutStream(cout);

  Mesh m(a.filename); // read this file, create vertices and edges
  if(m.vertices.size() == 0) {
    cout << "Error reading file.  Aborting." << endl;
    exit(0);
    return;
  }

  for(i = 0; i < (int)m.vertices.size(); ++i)
    m.vertices[i].pos = a.meshTransform * m.vertices[i].pos; // rotate every vertex(if angle should add 180)
  m.normalizeBoundingBox();
  m.computeVertexNormals();

  Skeleton given = a.skeleton;
  given.scale(a.skelScale * 0.5); // if remove this operate, no big difference

  if(a.stopAtMesh) { //if early bailout
    w->addMesh(new StaticDisplayMesh(m));
    return;
  }

  PinocchioOutput o;
  if(!a.noFit) { //do everything about embedding
    o = autorig(given, m); // in "pinocchioApi.h", given is skeleton, m is mesh
  }
  else { //skip the fitting step--assume the skeleton is already correct for the mesh
    TreeType *distanceField = constructDistanceField(m);
    VisTester<TreeType> *tester = new VisTester<TreeType>(distanceField);

    o.embedding = a.skeleton.fGraph().verts; // fGraph is full graph
    for(i = 0; i < (int)o.embedding.size(); ++i)
      o.embedding[i] = m.toAdd + o.embedding[i] * m.scale;

    o.attachment = new Attachment(m, a.skeleton, o.embedding, tester);

    delete tester;
    delete distanceField;
  }

  if(o.embedding.size() == 0) { // size is joints' number
    cout << "Error embedding" << endl;
    exit(0);
  }

  if(a.motionname.size() > 0) {
    bool useMyOwnMotion = true;
    //Motion *p = new Motion(a.motionname); // input is frames*114, readH processes motion
    Motion *p = new Motion(a.motionname, useMyOwnMotion); // input is 3frames*17
    w->addMesh(new DefMesh(m, given, o.embedding, *(o.attachment), p));
    p->get_size();
  }
  else {
    w->addMesh(new StaticDisplayMesh(m));
    // draw skeleton
    for(i = 1; i < (int)o.embedding.size(); ++i)
      {
        w->addLine(LineSegment(o.embedding[i], o.embedding[given.fPrev()[i]], Vector3(.25, .5, 0), 4.)); // p1, p2, color
      }
  }

  //output skeleton embedding
  //for(i = 0; i < (int)o.embedding.size(); ++i)
  //  o.embedding[i] = (o.embedding[i] - m.toAdd) / m.scale;
  ofstream os("skeleton.out");
  for(i = 0; i < (int)o.embedding.size(); ++i) {
    os << i << " " << o.embedding[i][0] << " " << o.embedding[i][1] <<
      " " << o.embedding[i][2] << " " << a.skeleton.fPrev()[i] << endl;
  }

  //output attachment
  std::ofstream astrm("attachment.out");
  for(i = 0; i < (int)m.vertices.size(); ++i) {
    Vector<double, -1> v = o.attachment->getWeights(i);
    for(int j = 0; j < v.size(); ++j) { // bones' number
      double d = floor(0.5 + v[j] * 10000.) / 10000.; // four decimal places
      astrm << d << " ";
    }
    astrm << endl;
  }
  delete o.attachment;
}
