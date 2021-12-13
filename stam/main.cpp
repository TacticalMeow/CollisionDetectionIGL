
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

#include <igl/circulation.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Welcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  std::ifstream infile("C:/Users/wisam/Desktop/EngineForAnimationCourse-master/configuration.txt");
  std::string line;
  while (std::getline(infile, line)) {
	  /*viewer.load_mesh_from_file("C:/Users/wisam/Desktop/EngineForAnimationCourse-master/tutorial/data/sphere.obj");
	  viewer.load_mesh_from_file("C:/Users/wisam/Desktop/EngineForAnimationCourse-master/tutorial/data/cube.obj");
	  viewer.load_mesh_from_file("C:/Users/wisam/Desktop/EngineForAnimationCourse-master/tutorial/data/bunny.off");*/
	  std::cout << line << std::endl;
	  viewer.load_mesh_from_file(line);
	  std::istringstream iss(line);
  }
  viewer.reset();
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  
  delete disp;
}
