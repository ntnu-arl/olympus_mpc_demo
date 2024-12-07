#ifndef DRAKE_HELPERS
#define DRAKE_HELPERS

#include <iostream>
#include <fstream>
#include <drake/systems/framework/diagram.h>
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/math/rigid_transform.h> 



#define MAX(a,b) ((a>b)?a:b)

using drake_tfd       = drake::math::RigidTransform<double>;
using drake_rotMat    = drake::math::RotationMatrix<double>;
using drake_rigidBody = drake::multibody::RigidBody<double> ; 
using drake_plant     = drake::multibody::MultibodyPlant<double>;
using drake_builder   = drake::systems::DiagramBuilder<double>;

/*!
 * @brief Returns the indices that would sort the input vector in ascending order.
 * 
 * This function takes a vector of elements and returns a vector of indices that
 * represent the order of the elements in ascending order. The original vector
 * remains unchanged.
 * 
 * @tparam T The type of elements in the input vector.
 * @param vec The input vector whose elements' indices are to be sorted.
 * @return A vector of indices that would sort the input vector in ascending order.
 */
template<typename T>
std::vector<int> ascending_indices(const std::vector<T> & vec){

  std::vector<std::pair<T,int>> paired_vec;
  std::vector<int> ind_vec;

  paired_vec.reserve(vec.size());
  ind_vec.reserve(vec.size());

  for (int i=0;i<vec.size();i++){
      paired_vec.emplace_back(std::pair<T,int>(vec[i],i));
  }
  
  std::sort(paired_vec.begin(),paired_vec.end());

  for (int i=0;i<vec.size();i++){
      ind_vec.emplace_back(paired_vec[i].second );
  }

  return ind_vec;
}

/*!
 * @brief Generates a Graphviz representation of the system diagram and writes it to a file.
 * 
 * This function generates a Graphviz string representation of the given Drake system diagram
 * and writes it to the specified file. The resulting file can be used to visualize the system
 * diagram using Graphviz tools.
 * 
 * @param diagram Pointer to the Drake system diagram.
 * @param filePath The path to the output file where the Graphviz string will be written.
 * 
 * @note View the generated file using the command: `dot -Tpng <filePath> -o diagram.png`
 */
void get_system_graph(const drake::systems::Diagram<double> * diagram,const std::string & filePath = "graph.dot" );

#endif // DRAKE_HELPERS_HPP
