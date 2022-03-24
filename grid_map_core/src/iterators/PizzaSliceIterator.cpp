/*
 * PizzaSliceIterator.cpp
 *
 *  Created on: Sep 21, 2021
 *      Author: Sergio García Vergara
 *   Institute: RIF Robotics, Atlanta, USA
 */

// debug
#include <iostream>

#include "grid_map_core/iterators/PizzaSliceIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

// PizzaSliceIterator::PizzaSliceIterator(const GridMap& gridMap, const Position center,
//                                        const Position p1, const Position p2)
//   : center_(center)
PizzaSliceIterator::PizzaSliceIterator(const GridMap& gridMap,
                                       const Position center,
                                       const float radius,
                                       const float yaw_rad,
                                       const float fov_rad)
    : center_(center),
      radius_(radius)
{
  // // Sanity check: distance between center and both points has to be the same
  // float d1 = (center - p1).norm();
  // float d2 = (center - p2).norm();
  // if (std::abs(d1 - d2) < 0.001) {
  //   radiusSquare_ = std::pow(d1, 2);
  // }
  // else {
  //   throw std::invalid_argument("Failed to construct PizzaSliceIterator: radius not consistent.");
  // }

  // Compute the coordinates of the polygon that encapsulate the pizza slice
  const float x1 = center_.x() + radius_ * std::cos(yaw_rad + fov_rad / 2);
  const float y1 = center_.y() + radius_ * std::sin(yaw_rad + fov_rad / 2);
  const float x2 = center_.x() + radius_ * std::cos(yaw_rad);
  const float y2 = center_.y() + radius_ * std::sin(yaw_rad);
  const float x3 = center_.x() + radius_ * std::cos(yaw_rad - fov_rad / 2);
  const float y3 = center_.y() + radius_ * std::sin(yaw_rad - fov_rad / 2);

  const Position p1({ x1, y1 });
  const Position p2({ x2, y2 });
  const Position p3({ x3, y3 });

  // Build the polygon
  polygon_.setFrameId(gridMap.getFrameId());
  polygon_.addVertex(center_);
  polygon_.addVertex(p1);
  polygon_.addVertex(p2);
  polygon_.addVertex(p3);

  radiusSquare_ = pow(radius_, 2);
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Size submapBufferSize;
  findSubmapParameters(submapStartIndex, submapBufferSize);
  internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
  if(!isInside()) ++(*this);
}

PizzaSliceIterator& PizzaSliceIterator::operator =(const PizzaSliceIterator& other)
{
  // TODO: make sure that these are a copy of the global variables

  polygon_ = other.polygon_;
  center_ = other.center_;
  radius_ = other.radius_;
  radiusSquare_ = other.radiusSquare_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool PizzaSliceIterator::operator !=(const PizzaSliceIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& PizzaSliceIterator::operator *() const
{
  return *(*internalIterator_);
}

PizzaSliceIterator& PizzaSliceIterator::operator ++()
{
  // TODO: make sure this logic remains the same

  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return *this;

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) break;
  }

  return *this;
}

bool PizzaSliceIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool PizzaSliceIterator::isInside() const
{
  // TODO: missing the check based on the pizza slice limits

  // Position position;
  // getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  // double squareNorm = (position - center_).array().square().sum();
  // return (squareNorm <= radiusSquare_);

  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  return polygon_.isInside(position);
}

void PizzaSliceIterator::findSubmapParameters(Index& startIndex, Size& bufferSize) const
{
  Position topLeft = polygon_.getVertices()[0];
  Position bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices()) {
    topLeft = topLeft.array().max(vertex.array());
    bottomRight = bottomRight.array().min(vertex.array());
  }
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = getSubmapSizeFromCornerIndeces(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace grid_map */
