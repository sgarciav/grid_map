/*
 * PizzaSliceIterator.hpp
 *
 *  Created on: Sep 21, 2021
 *      Author: Sergio García Vergara
 *   Institute: RIF Robotics, Atlanta, USA
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

#include <memory>

namespace grid_map {

/*!
 * Iterator class to iterate through a polygonal area of the map.
 */
class PizzaSliceIterator
{
public:

  // /*!
  //  * Constructor.
  //  * @param gridMap the grid map to iterate on.
  //  * @param center the position of the pizza slice starting vertex.
  //  * @param p1 the position of the first pizza slice vertiex.
  //  * @param p2 the position of the second pizza slice vertex.
  //  */
  // PizzaSliceIterator(const grid_map::GridMap& gridMap, const Position center,
  //                    const Position p1, const Position p2);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param center the position of the pizza slice starting vertex.
   * @param radius the radius of the pizza slice.
   * @param yaw_rad the direction of the vector that goes through the middle of the pizza slice [in radians].
   * @param fov_rad the angle of the pizza slice [in radians].
   */
    PizzaSliceIterator(const grid_map::GridMap& gridMap, const Position center,
                       const float radius, const float yaw_rad, const float fov_rad);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  PizzaSliceIterator& operator =(const PizzaSliceIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const PizzaSliceIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  PizzaSliceIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  /*!
   * Check if current index is inside polygon.
   * @return true if inside, false otherwise.
   */
  bool isInside() const;

  // TODO: update this section

  /*!
   * Finds the submap that fully contains the pizza slice and returns the parameters.
   * @param[out] startIndex the start index of the submap.
   * @param[out] bufferSize the buffer size of the submap.
   */
  void findSubmapParameters(Index& startIndex,Size& bufferSize) const;

  //! Polygon that encapsulates the pizza slice.
  grid_map::Polygon polygon_;

  //! Position of the pizza slice starting vertex.
  Position center_;

  //! Radius of the pizza slice.
  double radius_;

  //! Square of the radius (for efficiency).
  double radiusSquare_;

  //! Grid submap iterator.
  std::shared_ptr<SubmapIterator> internalIterator_;

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */
