// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

#define INIT_VELOCITY 0.005f
// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
	  

  }

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;
	animation_speed = INIT_VELOCITY;
  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

  //Collision detection using AABB for two objects
  //Eden Mansdorf

  //Adds a new collision box to the mesh of both objects.
  void Viewer::addBox(Eigen::AlignedBox<double, 3> box1, Eigen::AlignedBox<double, 3> box2, Eigen::RowVector3d color) {
	  for (int i = 0; i < 2; i++) {
		  Eigen::MatrixXd V_box(8, 3);
		  Eigen::MatrixXi E_box(12, 2);
		  if (i == 0) {
			  V_box <<
				  box1.corner(box1.TopLeftCeil)[0], box1.corner(box1.TopLeftCeil)[1], box1.corner(box1.TopLeftCeil)[2],
				  box1.corner(box1.TopRightCeil)[0], box1.corner(box1.TopRightCeil)[1], box1.corner(box1.TopRightCeil)[2],
				  box1.corner(box1.TopRightFloor)[0], box1.corner(box1.TopRightFloor)[1], box1.corner(box1.TopRightFloor)[2],
				  box1.corner(box1.TopLeftFloor)[0], box1.corner(box1.TopLeftFloor)[1], box1.corner(box1.TopLeftFloor)[2],
				  box1.corner(box1.BottomLeftCeil)[0], box1.corner(box1.BottomLeftCeil)[1], box1.corner(box1.BottomLeftCeil)[2],
				  box1.corner(box1.BottomRightCeil)[0], box1.corner(box1.BottomRightCeil)[1], box1.corner(box1.BottomRightCeil)[2],
				  box1.corner(box1.BottomRightFloor)[0], box1.corner(box1.BottomRightFloor)[1], box1.corner(box1.BottomRightFloor)[2],
				  box1.corner(box1.BottomLeftFloor)[0], box1.corner(box1.BottomLeftFloor)[1], box1.corner(box1.BottomLeftFloor)[2];
		  }
		  else {
			  V_box <<
				  box2.corner(box2.TopLeftCeil)[0], box2.corner(box2.TopLeftCeil)[1], box2.corner(box2.TopLeftCeil)[2],
				  box2.corner(box2.TopRightCeil)[0], box2.corner(box2.TopRightCeil)[1], box2.corner(box2.TopRightCeil)[2],
				  box2.corner(box2.TopRightFloor)[0], box2.corner(box2.TopRightFloor)[1], box2.corner(box2.TopRightFloor)[2],
				  box2.corner(box2.TopLeftFloor)[0], box2.corner(box2.TopLeftFloor)[1], box2.corner(box2.TopLeftFloor)[2],
				  box2.corner(box2.BottomLeftCeil)[0], box2.corner(box2.BottomLeftCeil)[1], box2.corner(box2.BottomLeftCeil)[2],
				  box2.corner(box2.BottomRightCeil)[0], box2.corner(box2.BottomRightCeil)[1], box2.corner(box2.BottomRightCeil)[2],
				  box2.corner(box2.BottomRightFloor)[0], box2.corner(box2.BottomRightFloor)[1], box2.corner(box2.BottomRightFloor)[2],
				  box2.corner(box2.BottomLeftFloor)[0], box2.corner(box2.BottomLeftFloor)[1], box2.corner(box2.BottomLeftFloor)[2];
		  }
		  E_box <<
			  0, 1,
			  1, 2,
			  2, 3,
			  3, 0,
			  4, 5,
			  5, 6,
			  6, 7,
			  7, 4,
			  0, 4,
			  1, 5,
			  2, 6,
			  7, 3;
		  for (unsigned j = 0; j < E_box.rows(); ++j)
			  data_list[i].add_edges
			  (
				  V_box.row(E_box(j, 0)),
				  V_box.row(E_box(j, 1)),
				  color
			  );
	  }
  }

  /*
	Recursive function that traverses the kdtrees of the objects, if we hit both leaves , then we have collision ,
	and we add the small collision box to the objects as overlay lines
  */
  bool Viewer::check_rec_intersection(igl::AABB<Eigen::MatrixXd, 3> &t1, igl::AABB<Eigen::MatrixXd, 3> &t2, Eigen::Matrix4f &A, Eigen::Matrix4f &B)
  {
	  bool f = check_intersection(t1.m_box, t2.m_box, A, B);
	  if (!f)
	  {
		  return false;
	  }
	  if (t1.is_leaf() && t2.is_leaf())
	  {
		  addBox(t1.m_box, t2.m_box, Eigen::RowVector3d(1.0, 0.0, 1.0));
		  return true;
	  }
	  if (t1.is_leaf())
	  {
		  data_list[0].lastTree = t1;
		  return (f && (check_rec_intersection(t1, *t2.m_right, A, B) || check_rec_intersection(t1, *t2.m_left, A, B)));
	  }

	  if (t2.is_leaf())
	  {
		  data_list[1].lastTree = t2;
		  return (f && (check_rec_intersection(*t1.m_right, t2, A, B) || check_rec_intersection(*t1.m_left, t2, A, B)));
	  }
	  data_list[1].lastTree = t2;
	  data_list[0].lastTree = t1;
	  if (check_rec_intersection(*t1.m_right, *t2.m_right, A, B))
		  return f;
	  if (check_rec_intersection(*t1.m_right, *t2.m_left, A, B))
		  return f;
	  if (check_rec_intersection(*t1.m_left, *t2.m_left, A, B))
		  return f;
	  if (check_rec_intersection(*t1.m_left, *t2.m_right, A, B))
		  return f;


	  return false;
  }
  IGL_INLINE void Viewer::animate() {
	  if (isMoving) {
		  Eigen::Matrix4f A = MakeTransScale() * data_list[0].MakeTransScale();
		  Eigen::Matrix4f B = MakeTransScale() * data_list[1].MakeTransScale();
		  if
			  (check_rec_intersection(data_list[0].tree, data_list[1].tree,A ,B)) {
			  isMoving = false;
			  std::cout << "collision detected!" << std::endl;
			  return;
		  }
		  if (selected_data_index == 0) {
			  data().MyTranslate(Eigen::Vector3d(animation_speed, 0, 0), true);
		  }
		  else {
			  data().MyTranslate(Eigen::Vector3d(-1.0f * animation_speed, 0, 0), true);
		  }
	  }
  }

  //given 2 collision boxes , we use the 15 checks to determine if there is no collision 
  bool Viewer::check_intersection(Eigen::AlignedBox<double, 3>& Box1, Eigen::AlignedBox<double, 3>& Box2, Eigen::Matrix4f& A, Eigen::Matrix4f& B)
  {
	  Eigen::Matrix4f c = A.inverse() * B;
	  Eigen::Vector4f c_ = Eigen::Vector4f(Box1.center()(0), Box1.center()(1), Box1.center()(2), 1);
	  c_ = A * c_;
	  Eigen::Vector3d c0 = Eigen::Vector3d(c_.x(), c_.y(), c_.z());
	  c_ = Eigen::Vector4f(Box2.center()(0), Box2.center()(1), Box2.center()(2), 1);
	  c_ = B * c_;
	  Eigen::Vector3d c1 = Eigen::Vector3d(c_.x(), c_.y(), c_.z());
	  Eigen::Vector3d d = c1 - c0;
	  Eigen::Vector3d Ax = Eigen::Vector3d(A(0, 0), A(1, 0), A(2, 0));
	  Eigen::Vector3d Ay = Eigen::Vector3d(A(0, 1), A(1, 1), A(2, 1));
	  Eigen::Vector3d Az = Eigen::Vector3d(A(0, 2), A(1, 2), A(2, 2));
	  Eigen::Vector3d Bx = Eigen::Vector3d(B(0, 0), B(1, 0), B(2, 0));
	  Eigen::Vector3d By = Eigen::Vector3d(B(0, 1), B(1, 1), B(2, 1));
	  Eigen::Vector3d Bz = Eigen::Vector3d(B(0, 2), B(1, 2), B(2, 2));
	  double a0, a1, a2, b1, b2, b0, R0, R1, R;
	  a0 = (Box1.sizes()(0) / 2) + 0.0001; a1 = (Box1.sizes()(1) / 2) + 0.0001; a2 = (Box1.sizes()(2) / 2) + 0.0001;
	  b0 = (Box2.sizes()(0) / 2) + 0.0001; b1 = (Box2.sizes()(1) / 2) + 0.0001; b2 = (Box2.sizes()(2) / 2) + 0.0001;
	  R = std::abs(Ax.dot(d));
	  R0 = a0;
	  R1 = (b0 * std::abs(c(0, 0))) + (b1 * std::abs(c(0, 1))) + (b2 * std::abs(c(0, 2)));
	  if (R > R1 + R0)
		  return false;
	  R = std::abs(Ay.dot(d));
	  R0 = a1;
	  R1 = (b0 * std::abs(c(1, 0))) + (b1 * std::abs(c(1, 1))) + (b2 * std::abs(c(1, 2)));
	  if (R > R1 + R0)
		  return false;
	  R = std::abs(Az.dot(d));
	  R0 = a2;
	  R1 = (b0 * std::abs(c(2, 0))) + (b1 * std::abs(c(2, 1))) + (b2 * std::abs(c(2, 2)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(0, 0))) + (a1 * std::abs(c(1, 0))) + (a2 * std::abs(c(2, 0)));
	  R1 = b0;
	  R = std::abs(Bx.dot(d));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(0, 1))) + (a1 * std::abs(c(1, 1))) + (a2 * std::abs(c(2, 1)));
	  R1 = b1;
	  R = std::abs(By.dot(d));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(0, 2))) + (a1 * std::abs(c(1, 2))) + (a2 * std::abs(c(2, 2)));
	  R1 = b2;
	  R = std::abs(Bz.dot(d));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a1 * std::abs(c(2, 0))) + (a2 * std::abs(c(1, 0)));
	  R1 = (b1 * std::abs(c(0, 2))) + (b2 * std::abs(c(0, 1)));
	  R = std::abs((d.dot(c(1, 0) * Az)) - (d.dot(c(2, 0) * Ay)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a1 * std::abs(c(2, 1))) + (a2 * std::abs(c(1, 1)));
	  R1 = (b0 * std::abs(c(0, 2))) + (b2 * std::abs(c(0, 0)));
	  R = std::abs((d.dot(c(1, 1) * Az)) - (d.dot(c(2, 1) * Ay)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a1 * std::abs(c(2, 2))) + (a2 * std::abs(c(1, 2)));
	  R1 = (b0 * std::abs(c(0, 1))) + (b1 * std::abs(c(0, 0)));
	  R = std::abs((d.dot(c(1, 2) * Az)) - (d.dot(c(2, 2) * Ay)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(2, 0))) + (a2 * std::abs(c(0, 0)));
	  R1 = (b1 * std::abs(c(1, 2))) + (b2 * std::abs(c(1, 1)));
	  R = std::abs((d.dot(c(2, 0) * Ax)) - (d.dot(c(0, 0) * Az)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(2, 1))) + (a2 * std::abs(c(0, 1)));
	  R1 = (b0 * std::abs(c(1, 2))) + (b2 * std::abs(c(1, 0)));
	  R = std::abs((d.dot(c(2, 1) * Ax)) - (d.dot(c(0, 1) * Az)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(2, 2))) + (a2 * std::abs(c(0, 2)));
	  R1 = (b0 * std::abs(c(1, 1))) + (b1 * std::abs(c(1, 0)));
	  R = std::abs((d.dot(c(2, 2) * Ax)) - (d.dot(c(0, 2) * Az)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(1, 0))) + (a1 * std::abs(c(0, 0)));
	  R1 = (b1 * std::abs(c(2, 2))) + (b2 * std::abs(c(2, 1)));
	  R = std::abs((d.dot(c(0, 0) * Ay)) - (d.dot(c(1, 0) * Ax)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(1, 1))) + (a1 * std::abs(c(0, 1)));
	  R1 = (b0 * std::abs(c(2, 2))) + (b2 * std::abs(c(2, 0)));
	  R = std::abs((d.dot(c(0, 1) * Ay)) - (d.dot(c(1, 1) * Ax)));
	  if (R > R1 + R0)
		  return false;
	  R0 = (a0 * std::abs(c(1, 2))) + (a1 * std::abs(c(0, 2)));
	  R1 = (b0 * std::abs(c(2, 1))) + (b1 * std::abs(c(2, 0)));
	  R = std::abs((d.dot(c(0, 2) * Ay)) - (d.dot(c(1, 2) * Ax)));
	  if (R > R1 + R0)
		  return false;
	  return true;
  }

  void Viewer::addAnimationSpeed(float animspeed)
  {
	  if (animspeed <= 0.0f)
	  {
		  if (animation_speed + animspeed >= INIT_VELOCITY)
			  animation_speed = animation_speed + animspeed;
		  else
			  animation_speed = INIT_VELOCITY;
	  }
	  else
	  {
		  if (animation_speed + animspeed <= 0.1f)
			  animation_speed = animation_speed + animspeed;
		  else
			  animation_speed = 0.1f;
	  }

  }

} // end namespace
} // end namespace
}
