#ifndef VOXEL_GRID_HPP
#define VOXEL_GRID_HPP

#include <vector>
#include <list>
#include "algebra.hpp"

class VoxelGrid {
public:
  VoxelGrid(double voxelSize, Point3D minP, int xLen, int yLen, int zLen) :
              mVoxelSize(voxelSize), mMinP(minP) { dimensions[0] = xLen; dimensions[1] = yLen; dimensions[2] = zLen;}

private:
  vector<vector<list<int*> > > mVoxels;

  double mVoxelSize;
  Point3D mMinP;
  int dimensions[3];

};


#endif



