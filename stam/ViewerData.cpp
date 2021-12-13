#include "ViewerData.h"
#include "ViewerData.h"
// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "ViewerData.h"
#include "ViewerCore.h"

#include "../per_face_normals.h"
#include "../material_colors.h"
#include "../parula.h"
#include "../per_vertex_normals.h"

#include <iostream>

#include <igl/collapse_edge.h>
#include <igl/circulation.h>
#include "igl/edge_collapse_is_valid.h"



//Collapse Edge
IGL_INLINE bool collapse_assignment2(
	const int e,
	const Eigen::RowVectorXd& p,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	int& a_e1,
	int& a_e2,
	int& a_f1,
	int& a_f2,
	std::vector<Eigen::Matrix4d>& Qs)
{
	// Assign this to 0 rather than, say, -1 so that deleted elements will get
	// draw as degenerate elements at vertex 0 (which should always exist and
	// never get collapsed to anything else since it is the smallest index)
	using namespace Eigen;
	using namespace std;
	const int eflip = E(e, 0) > E(e, 1);
	// source and destination
	const int s = eflip ? E(e, 1) : E(e, 0);
	const int d = eflip ? E(e, 0) : E(e, 1);

	if (!igl::edge_collapse_is_valid(e, F, E, EMAP, EF, EI))
	{
		return false;
	}

	// Important to grab neighbors of d before monkeying with edges
	const std::vector<int> nV2Fd = igl::circulation(e, !eflip, EMAP, EF, EI);

	// The following implementation strongly relies on s<d
	assert(s < d && "s should be less than d");
	// move source and destination to midpoint
	V.row(s) = p;
	V.row(d) = p;
	Qs[E(e, 0)] = Qs[E(e, 0)] + Qs[E(e, 1)];
	Qs[E(e, 1)] = Qs[E(e, 0)];

	// Helper function to replace edge and associate information with NULL
	const auto& kill_edge = [&E, &EI, &EF](const int e)
	{
		E(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		E(e, 1) = IGL_COLLAPSE_EDGE_NULL;
		EF(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		EF(e, 1) = IGL_COLLAPSE_EDGE_NULL;
		EI(e, 0) = IGL_COLLAPSE_EDGE_NULL;
		EI(e, 1) = IGL_COLLAPSE_EDGE_NULL;
	};

	// update edge info
	// for each flap
	const int m = F.rows();
	for (int side = 0; side < 2; side++)
	{
		const int f = EF(e, side);
		const int v = EI(e, side);
		const int sign = (eflip == 0 ? 1 : -1) * (1 - 2 * side);
		// next edge emanating from d
		const int e1 = EMAP(f + m * ((v + sign * 1 + 3) % 3));
		// prev edge pointing to s
		const int e2 = EMAP(f + m * ((v + sign * 2 + 3) % 3));
		assert(E(e1, 0) == d || E(e1, 1) == d);
		assert(E(e2, 0) == s || E(e2, 1) == s);
		// face adjacent to f on e1, also incident on d
		const bool flip1 = EF(e1, 1) == f;
		const int f1 = flip1 ? EF(e1, 0) : EF(e1, 1);
		assert(f1 != f);
		assert(F(f1, 0) == d || F(f1, 1) == d || F(f1, 2) == d);
		// across from which vertex of f1 does e1 appear?
		const int v1 = flip1 ? EI(e1, 0) : EI(e1, 1);
		// Kill e1
		kill_edge(e1);
		// Kill f
		F(f, 0) = IGL_COLLAPSE_EDGE_NULL;
		F(f, 1) = IGL_COLLAPSE_EDGE_NULL;
		F(f, 2) = IGL_COLLAPSE_EDGE_NULL;
		// map f1's edge on e1 to e2
		assert(EMAP(f1 + m * v1) == e1);
		EMAP(f1 + m * v1) = e2;
		// side opposite f2, the face adjacent to f on e2, also incident on s
		const int opp2 = (EF(e2, 0) == f ? 0 : 1);
		assert(EF(e2, opp2) == f);
		EF(e2, opp2) = f1;
		EI(e2, opp2) = v1;
		// remap e2 from d to s
		E(e2, 0) = E(e2, 0) == d ? s : E(e2, 0);
		E(e2, 1) = E(e2, 1) == d ? s : E(e2, 1);
		if (side == 0)
		{
			a_e1 = e1;
			a_f1 = f;
		}
		else
		{
			a_e2 = e1;
			a_f2 = f;
		}
	}

	// finally, reindex faces and edges incident on d. Do this last so asserts
	// make sense.
	//
	// Could actually skip first and last, since those are always the two
	// collpased faces.
	for (auto f : nV2Fd)
	{
		for (int v = 0; v < 3; v++)
		{
			if (F(f, v) == d)
			{
				const int flip1 = (EF(EMAP(f + m * ((v + 1) % 3)), 0) == f) ? 1 : 0;
				const int flip2 = (EF(EMAP(f + m * ((v + 2) % 3)), 0) == f) ? 0 : 1;
				assert(
					E(EMAP(f + m * ((v + 1) % 3)), flip1) == d ||
					E(EMAP(f + m * ((v + 1) % 3)), flip1) == s);
				E(EMAP(f + m * ((v + 1) % 3)), flip1) = s;
				assert(
					E(EMAP(f + m * ((v + 2) % 3)), flip2) == d ||
					E(EMAP(f + m * ((v + 2) % 3)), flip2) == s);
				E(EMAP(f + m * ((v + 2) % 3)), flip2) = s;
				F(f, v) = s;
				break;
			}
		}
	}
	// Finally, "remove" this edge and its information
	kill_edge(e);

	return true;
}

IGL_INLINE bool collapse_assignment2(
	const int e,
	const Eigen::RowVectorXd& p,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::vector<Eigen::Matrix4d>& Qs)
{
	int e1, e2, f1, f2;
	return collapse_assignment2(e, p, V, F, E, EMAP, EF, EI, e1, e2, f1, f2, Qs);
}


IGL_INLINE bool collapse_assignment2(
	const std::function<void(
		const int,
		const Eigen::MatrixXd&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		const Eigen::VectorXi&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		double&,
		Eigen::RowVectorXd&,
		std::vector<Eigen::Matrix4d>&)>& cost_and_placement,
	const std::function<bool(
		const Eigen::MatrixXd&,/*V*/
		const Eigen::MatrixXi&,/*F*/
		const Eigen::MatrixXi&,/*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,/*EF*/
		const Eigen::MatrixXi&,/*EI*/
		const std::set<std::pair<double, int> >&,/*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
		const Eigen::MatrixXd&,/*C*/
		const int                                                        /*e*/
		)>& pre_collapse,
	const std::function<void(
		const Eigen::MatrixXd&,   /*V*/
		const Eigen::MatrixXi&,   /*F*/
		const Eigen::MatrixXi&,   /*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,  /*EF*/
		const Eigen::MatrixXi&,  /*EI*/
		const std::set<std::pair<double, int> >&,   /*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
		const Eigen::MatrixXd&,   /*C*/
		const int,   /*e*/
		const int,  /*e1*/
		const int,  /*e2*/
		const int,  /*f1*/
		const int,  /*f2*/
		const bool                                                  /*collapsed*/
		)>& post_collapse,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::set<std::pair<double, int> >& Q,
	std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
	Eigen::MatrixXd& C,
	int& e,
	int& e1,
	int& e2,
	int& f1,
	int& f2,
	std::vector<Eigen::Matrix4d>& Qs)
{
	using namespace Eigen;
	int counter = 0;
	if (Q.empty())
	{
		// no edges to collapse
		return false;
	}
	std::pair<double, int> p = *(Q.begin());
	if (p.first == std::numeric_limits<double>::infinity())
	{
		// min cost edge is infinite cost
		return false;
	}
	Q.erase(Q.begin());
	e = p.second;
	Qit[e] = Q.end();
	Eigen::Matrix4d q1 = Qs[E(e, 0)];
	Eigen::Matrix4d q2 = Qs[E(e, 1)];
	std::vector<int> N = igl::circulation(e, true, EMAP, EF, EI);
	std::vector<int> Nd = igl::circulation(e, false, EMAP, EF, EI);
	N.insert(N.begin(), Nd.begin(), Nd.end());
	bool collapsed = true;
	if (pre_collapse(V, F, E, EMAP, EF, EI, Q, Qit, C, e))
	{
		collapsed = collapse_assignment2(e, C.row(e), V, F, E, EMAP, EF, EI, e1, e2, f1, f2, Qs);
	}
	else
	{
		// Aborted by pre collapse callback
		collapsed = false;
	}
	post_collapse(V, F, E, EMAP, EF, EI, Q, Qit, C, e, e1, e2, f1, f2, collapsed);
	if (collapsed)
	{
		printf("edge %d, cost = %d, new v position (%f, %f, %f)\n", e, p.first, C.row(e)[0], C.row(e)[1], C.row(e)[2]);
		printf("edge %d, cost = %d, new v position (%f, %f, %f)\n", e1, Qit[e1], C.row(e1)[0], C.row(e1)[1], C.row(e1)[2]);
		printf("edge %d, cost = %d, new v position (%f, %f, %f)\n", e2, Qit[e2], C.row(e2)[0], C.row(e2)[1], C.row(e2)[2]);
		// Erase the two, other collapsed edges
		Q.erase(Qit[e1]);
		Qit[e1] = Q.end();
		Q.erase(Qit[e2]);
		Qit[e2] = Q.end();
		// update local neighbors
		// loop over original face neighbors
		for (auto n : N)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// get edge id
					const int ei = EMAP(v * F.rows() + n);
					// erase old entry
					Q.erase(Qit[ei]);
					// compute cost and potential placement
					double cost;
					RowVectorXd place;
					cost_and_placement(ei, V, F, E, EMAP, EF, EI, cost, place, Qs);
					// Replace in queue
					Qit[ei] = Q.insert(std::pair<double, int>(cost, ei)).first;
					C.row(ei) = place;
				}
			}
		}
		printf("faces : %d\n", Q.size());
	}
	else
	{
		// reinsert with infinite weight (the provided cost function must **not**
		// have given this un-collapsable edge inf cost already)
		p.first = std::numeric_limits<double>::infinity();
		Qit[e] = Q.insert(p).first;
	}
	return collapsed;
}

IGL_INLINE bool collapse_assignment2(
	const std::function<void(
		const int,
		const Eigen::MatrixXd&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		const Eigen::VectorXi&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		double&,
		Eigen::RowVectorXd&,
		std::vector<Eigen::Matrix4d> &)>& cost_and_placement,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::set<std::pair<double, int> >& Q,
	std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
	Eigen::MatrixXd& C,
	std::vector<Eigen::Matrix4d>& Qs)
{
	int e, e1, e2, f1, f2;
	const auto always_try = [](
		const Eigen::MatrixXd&,/*V*/
		const Eigen::MatrixXi&,/*F*/
		const Eigen::MatrixXi&,/*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,/*EF*/
		const Eigen::MatrixXi&,/*EI*/
		const std::set<std::pair<double, int> >&,/*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
		const Eigen::MatrixXd&,/*C*/
		const int                                                        /*e*/
		) -> bool { return true; };
	const auto never_care = [](
		const Eigen::MatrixXd&,   /*V*/
		const Eigen::MatrixXi&,   /*F*/
		const Eigen::MatrixXi&,   /*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,  /*EF*/
		const Eigen::MatrixXi&,  /*EI*/
		const std::set<std::pair<double, int> >&,   /*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
		const Eigen::MatrixXd&,   /*C*/
		const int,   /*e*/
		const int,  /*e1*/
		const int,  /*e2*/
		const int,  /*f1*/
		const int,  /*f2*/
		const bool                                                  /*collapsed*/
		)-> void {};
	return
		collapse_assignment2(
			cost_and_placement, always_try, never_care,
			V, F, E, EMAP, EF, EI, Q, Qit, C, e, e1, e2, f1, f2, Qs);
}

IGL_INLINE bool collapse_assignment2(
	const std::function<void(
		const int,
		const Eigen::MatrixXd&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		const Eigen::VectorXi&,
		const Eigen::MatrixXi&,
		const Eigen::MatrixXi&,
		double&,
		Eigen::RowVectorXd&,
		std::vector<Eigen::Matrix4d> &)>& cost_and_placement,
	const std::function<bool(
		const Eigen::MatrixXd&,/*V*/
		const Eigen::MatrixXi&,/*F*/
		const Eigen::MatrixXi&,/*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,/*EF*/
		const Eigen::MatrixXi&,/*EI*/
		const std::set<std::pair<double, int> >&,/*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&,/*Qit*/
		const Eigen::MatrixXd&,/*C*/
		const int                                                        /*e*/
		)>& pre_collapse,
	const std::function<void(
		const Eigen::MatrixXd&,   /*V*/
		const Eigen::MatrixXi&,   /*F*/
		const Eigen::MatrixXi&,   /*E*/
		const Eigen::VectorXi&,/*EMAP*/
		const Eigen::MatrixXi&,  /*EF*/
		const Eigen::MatrixXi&,  /*EI*/
		const std::set<std::pair<double, int> >&,   /*Q*/
		const std::vector<std::set<std::pair<double, int> >::iterator >&, /*Qit*/
		const Eigen::MatrixXd&,   /*C*/
		const int,   /*e*/
		const int,  /*e1*/
		const int,  /*e2*/
		const int,  /*f1*/
		const int,  /*f2*/
		const bool                                                  /*collapsed*/
		)>& post_collapse,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	std::set<std::pair<double, int> >& Q,
	std::vector<std::set<std::pair<double, int> >::iterator >& Qit,
	Eigen::MatrixXd& C,
	std::vector<Eigen::Matrix4d>& Qs)
{
	int e, e1, e2, f1, f2;
	return
		collapse_assignment2(
			cost_and_placement, pre_collapse, post_collapse,
			V, F, E, EMAP, EF, EI, Q, Qit, C, e, e1, e2, f1, f2, Qs);
}

//Viewer Data start
IGL_INLINE igl::opengl::ViewerData::ViewerData()
: dirty(MeshGL::DIRTY_ALL),
  show_faces(true),
  show_lines(true),
  invert_normals(false),
  show_overlay(true),
  show_overlay_depth(true),
  show_vertid(false),
  show_faceid(false),
  show_texture(false),
  point_size(30),
  line_width(0.5f),
  line_color(0,0,0,1),
  label_color(0,0,0.04,1),
  shininess(35.0f),
  id(-1),
  is_visible(1)
{
  clear();
};

IGL_INLINE void igl::opengl::ViewerData::set_face_based(bool newvalue)
{
  if (face_based != newvalue)
  {
    face_based = newvalue;
    dirty = MeshGL::DIRTY_ALL;
  }
}

//Assignment 2
void printMatrix(Eigen::Matrix4d m) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%f ", m(i, j));
		}
		printf("\n");
	}
}

bool point_on_l(Eigen::Vector4d p, Eigen::Vector3d p2, Eigen::Vector3d p1)
{
	double x, x1, x2, y1, y2, y, z, z1, z2;
	x = p[0]; y = p[1]; z = p[2];
	x1 = p1[0]; y1 = p1[1]; z1 = p[2];
	x2 = p2[0]; y2 = p2[1]; z2 = p[2];
	double AB = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
	double AP = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1) + (z - z1) * (z - z1));
	double PB = sqrt((x2 - x) * (x2 - x) + (y2 - y) * (y2 - y) + (z2 - z) * (z2 - z));
	if (AB == AP + PB)
		return true;
	return false;
}

IGL_INLINE void shortest(
	const int e,
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& /*F*/,
	const Eigen::MatrixXi& E,
	const Eigen::VectorXi& /*EMAP*/,
	const Eigen::MatrixXi& /*EF*/,
	const Eigen::MatrixXi& /*EI*/,
	double& cost,
	Eigen::RowVectorXd& p,
	std::vector<Eigen::Matrix4d> Qs)
{
	Eigen::Matrix4d q1andq2 = (Qs.at(E(e, 0)) + Qs.at(E(e, 1)));
	q1andq2.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
	p = q1andq2.inverse() * Eigen::Vector4d(0, 0, 0, 1);
		if ((p[0] >= V(E(e, 0), 0) && p[0] <= V(E(e, 1), 0)
			&& p[1] >= V(E(e, 0), 1) && p[1] <= V(E(e, 1), 1)
			&& p[2] >= V(E(e, 0), 2) && p[2] <= V(E(e, 1), 2)
			&& point_on_l(p, V.row(E(e, 1)) ,V.row(E(e, 0)))&& q1andq2.determinant()!=0)) {
			p = q1andq2.inverse() * Eigen::Vector4d(0, 0, 0, 1);
		}
		else {
			p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
			Eigen::Vector4d temp = { p[0], p[1], p[2], 1 };
			p = temp;
		}
	//Eigen::RowVector4d tempE1 = { V.row(E(e,0))[0], V.row(E(e,0))[1] , V.row(E(e,0))[2], 1 };
	//Eigen::RowVector4d tempE2 = {V.row(E(e,1))[0], V.row(E(e,1))[1] , V.row(E(e,1))[2], 1 };
	//cost = tempE1 * Qs.at(E(e, 0)) * tempE1.transpose();
	//cost = cost + tempE2 * Qs.at(E(e, 1)) * tempE2.transpose();
		/*Eigen::RowVector3d v0 = V.row(E(e, 0));
		Eigen::RowVector3d v1 = V.row(E(e, 1));
		Eigen::RowVector4d temp = { v0[0], v0[1], v0[2], 1 };
		cost = temp * Qs.at(E(e, 0)) * temp.transpose();
		 temp = { v1[0], v1[1], v1[2], 1 };
		cost += temp * Qs.at(E(e, 1)) * temp.transpose();*/
	cost = (p * (Qs.at(E(e, 0)) + Qs.at(E(e, 1))) * p.transpose());
}
Eigen::Matrix4d igl::opengl::ViewerData::kpCalculator(Eigen::Vector3d abc, Eigen::Vector3d v) {
	double d = -(abc[0] * v[0] + abc[1] * v[1] + abc[2] * v[2]);
	Eigen::Vector4d p = { abc[0], abc[1], abc[2], d };
	return p*p.transpose();
}
/*
IGL_INLINE void igl::calculateCostAndNewPoint(
	const int e,
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	const Eigen::MatrixXi& E,
	const Eigen::VectorXi& EMAP,
	const Eigen::MatrixXi& EF,
	const Eigen::MatrixXi& EI,
	double& cost,
	Eigen::RowVectorXd& p) {
	p = 0.5 * (V.row(E(e, 0)) + V.row(E(e, 1)));
	Eigen::RowVector4d temp = { p[0], p[1], p[2], 1 };
	p = temp;
	cost = 1;
}*/

void igl::opengl::ViewerData::reset() {
	Qs.resize(V.rows());
	for (int f = 0; f < F.rows(); f++) {
		Eigen::Vector3d v = Eigen::Vector3d(V(F(f, 0), 0), V(F(f, 0), 1), V(F(f, 0), 2));
		Eigen::Vector3d abc = F_normals.row(f).normalized();
		Eigen::MatrixXd res = kpCalculator(abc, v);
		Qs.at(F(f, 0)) += res;
		Qs.at(F(f, 1)) += res;
		Qs.at(F(f, 2)) += res;
	}
	OF = F;
	OV = V;
	igl::edge_flaps(OF, E, EMAP, EF, EI);
	Qit.resize(E.rows());

	C.resize(E.rows(), OV.cols());
	Eigen::VectorXd costs(E.rows());
	Q.clear();
	for (int e = 0; e < E.rows(); e++)
	{
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		shortest(e, OV, OF, E, EMAP, EF, EI, cost, p, Qs);
		C.row(e) = p;
		Qit[e] = Q.insert(std::pair<double, int>(cost, e)).first;
	}
	num_collapsed = 0;
	clear();
	set_mesh(OV, OF);
	set_face_based(true);
};

bool igl::opengl::ViewerData::simplify() {
	if (!Q.empty())
	{
		bool something_collapsed = false;
		// collapse edge
		const int max_iter = std::ceil(0.05 * Q.size());
		for (int j = 0; j < max_iter; j++)
		{
			if (!collapse_assignment2(shortest, OV, OF, E, EMAP, EF, EI, Q, Qit, C, Qs))
			{
				break;
			}
			something_collapsed = true;
			num_collapsed++;
		}

		if (something_collapsed)
		{
			clear();
			set_mesh(OV, OF);
			set_face_based(true);
		}
	}
	return false;
}

// Helpers that draws the most common meshes
IGL_INLINE void igl::opengl::ViewerData::set_mesh(
    const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F)
{
  using namespace std;

  Eigen::MatrixXd V_temp;

  // If V only has two columns, pad with a column of zeros
  if (_V.cols() == 2)
  {
    V_temp = Eigen::MatrixXd::Zero(_V.rows(),3);
    V_temp.block(0,0,_V.rows(),2) = _V;
  }
  else
    V_temp = _V;

  if (V.rows() == 0 && F.rows() == 0)
  {
    V = V_temp;
    F = _F;

    compute_normals();
    uniform_colors(
      Eigen::Vector3d(GOLD_AMBIENT[0], GOLD_AMBIENT[1], GOLD_AMBIENT[2]),
      Eigen::Vector3d(GOLD_DIFFUSE[0], GOLD_DIFFUSE[1], GOLD_DIFFUSE[2]),
      Eigen::Vector3d(GOLD_SPECULAR[0], GOLD_SPECULAR[1], GOLD_SPECULAR[2]));

    grid_texture();
  }
  else
  {
    if (_V.rows() == V.rows() && _F.rows() == F.rows())
    {
      V = V_temp;
      F = _F;
    }
    else
      cerr << "ERROR (set_mesh): The new mesh has a different number of vertices/faces. Please clear the mesh before plotting."<<endl;
  }
  dirty |= MeshGL::DIRTY_FACE | MeshGL::DIRTY_POSITION;
}

IGL_INLINE void igl::opengl::ViewerData::set_vertices(const Eigen::MatrixXd& _V)
{
  V = _V;
  assert(F.size() == 0 || F.maxCoeff() < V.rows());
  dirty |= MeshGL::DIRTY_POSITION;
}

IGL_INLINE void igl::opengl::ViewerData::set_normals(const Eigen::MatrixXd& N)
{
  using namespace std;
  if (N.rows() == V.rows())
  {
    set_face_based(false);
    V_normals = N;
  }
  else if (N.rows() == F.rows() || N.rows() == F.rows()*3)
  {
    set_face_based(true);
    F_normals = N;
  }
  else
    cerr << "ERROR (set_normals): Please provide a normal per face, per corner or per vertex."<<endl;
  dirty |= MeshGL::DIRTY_NORMAL;
}

IGL_INLINE void igl::opengl::ViewerData::set_visible(bool value, unsigned int core_id /*= 1*/)
{
  if (value)
    is_visible |= core_id;
  else
  is_visible &= ~core_id;
}

//IGL_INLINE void igl::opengl::ViewerData::copy_options(const ViewerCore &from, const ViewerCore &to)
//{
//  to.set(show_overlay      , from.is_set(show_overlay)      );
//  to.set(show_overlay_depth, from.is_set(show_overlay_depth));
//  to.set(show_texture      , from.is_set(show_texture)      );
//  to.set(show_faces        , from.is_set(show_faces)        );
//  to.set(show_lines        , from.is_set(show_lines)        );
//}

IGL_INLINE void igl::opengl::ViewerData::set_colors(const Eigen::MatrixXd &C)
{
  using namespace std;
  using namespace Eigen;
  if(C.rows()>0 && C.cols() == 1)
  {
    Eigen::MatrixXd C3;
    igl::parula(C,true,C3);
    return set_colors(C3);
  }
  // Ambient color should be darker color
  const auto ambient = [](const MatrixXd & C)->MatrixXd
  {
    MatrixXd T = 0.1*C;
    T.col(3) = C.col(3);
    return T;
  };
  // Specular color should be a less saturated and darker color: dampened
  // highlights
  const auto specular = [](const MatrixXd & C)->MatrixXd
  {
    const double grey = 0.3;
    MatrixXd T = grey+0.1*(C.array()-grey);
    T.col(3) = C.col(3);
    return T;
  };
  if (C.rows() == 1)
  {
    for (unsigned i=0;i<V_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        V_material_diffuse.row(i) << C.row(0),1;
      else if (C.cols() == 4)
        V_material_diffuse.row(i) << C.row(0);
    }
    V_material_ambient = ambient(V_material_diffuse);
    V_material_specular = specular(V_material_diffuse);

    for (unsigned i=0;i<F_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        F_material_diffuse.row(i) << C.row(0),1;
      else if (C.cols() == 4)
        F_material_diffuse.row(i) << C.row(0);
    }
    F_material_ambient = ambient(F_material_diffuse);
    F_material_specular = specular(F_material_diffuse);
  }
  else if (C.rows() == V.rows())
  {
    set_face_based(false);
    for (unsigned i=0;i<V_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        V_material_diffuse.row(i) << C.row(i), 1;
      else if (C.cols() == 4)
        V_material_diffuse.row(i) << C.row(i);
    }
    V_material_ambient = ambient(V_material_diffuse);
    V_material_specular = specular(V_material_diffuse);
  }
  else if (C.rows() == F.rows())
  {
    set_face_based(true);
    for (unsigned i=0;i<F_material_diffuse.rows();++i)
    {
      if (C.cols() == 3)
        F_material_diffuse.row(i) << C.row(i), 1;
      else if (C.cols() == 4)
        F_material_diffuse.row(i) << C.row(i);
    }
    F_material_ambient = ambient(F_material_diffuse);
    F_material_specular = specular(F_material_diffuse);
  }
  else
    cerr << "ERROR (set_colors): Please provide a single color, or a color per face or per vertex."<<endl;
  dirty |= MeshGL::DIRTY_DIFFUSE;

}

IGL_INLINE void igl::opengl::ViewerData::set_uv(const Eigen::MatrixXd& UV)
{
  using namespace std;
  if (UV.rows() == V.rows())
  {
    set_face_based(false);
    V_uv = UV;
  }
  else
    cerr << "ERROR (set_UV): Please provide uv per vertex."<<endl;;
  dirty |= MeshGL::DIRTY_UV;
}

IGL_INLINE void igl::opengl::ViewerData::set_uv(const Eigen::MatrixXd& UV_V, const Eigen::MatrixXi& UV_F)
{
  set_face_based(true);
  V_uv = UV_V.block(0,0,UV_V.rows(),2);
  F_uv = UV_F;
  dirty |= MeshGL::DIRTY_UV;
}

IGL_INLINE void igl::opengl::ViewerData::set_texture(
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B)
{
  texture_R = R;
  texture_G = G;
  texture_B = B;
  texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(R.rows(),R.cols(),255);
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::set_texture(
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& R,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& G,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& B,
  const Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>& A)
{
  texture_R = R;
  texture_G = G;
  texture_B = B;
  texture_A = A;
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::set_points(
  const Eigen::MatrixXd& P,
  const Eigen::MatrixXd& C)
{
  // clear existing points
  points.resize(0,0);
  add_points(P,C);
}

IGL_INLINE void igl::opengl::ViewerData::add_points(const Eigen::MatrixXd& P,  const Eigen::MatrixXd& C)
{
  Eigen::MatrixXd P_temp;

  // If P only has two columns, pad with a column of zeros
  if (P.cols() == 2)
  {
    P_temp = Eigen::MatrixXd::Zero(P.rows(),3);
    P_temp.block(0,0,P.rows(),2) = P;
  }
  else
    P_temp = P;

  int lastid = points.rows();
  points.conservativeResize(points.rows() + P_temp.rows(),6);
  for (unsigned i=0; i<P_temp.rows(); ++i)
    points.row(lastid+i) << P_temp.row(i), i<C.rows() ? C.row(i) : C.row(C.rows()-1);

  dirty |= MeshGL::DIRTY_OVERLAY_POINTS;
}

IGL_INLINE void igl::opengl::ViewerData::set_edges(
  const Eigen::MatrixXd& P,
  const Eigen::MatrixXi& E,
  const Eigen::MatrixXd& C)
{
  using namespace Eigen;
  lines.resize(E.rows(),9);
  assert(C.cols() == 3);
  for(int e = 0;e<E.rows();e++)
  {
    RowVector3d color;
    if(C.size() == 3)
    {
      color<<C;
    }else if(C.rows() == E.rows())
    {
      color<<C.row(e);
    }
    lines.row(e)<< P.row(E(e,0)), P.row(E(e,1)), color;
  }
  dirty |= MeshGL::DIRTY_OVERLAY_LINES;
}

IGL_INLINE void igl::opengl::ViewerData::add_edges(const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2, const Eigen::MatrixXd& C)
{
  Eigen::MatrixXd P1_temp,P2_temp;

  // If P1 only has two columns, pad with a column of zeros
  if (P1.cols() == 2)
  {
    P1_temp = Eigen::MatrixXd::Zero(P1.rows(),3);
    P1_temp.block(0,0,P1.rows(),2) = P1;
    P2_temp = Eigen::MatrixXd::Zero(P2.rows(),3);
    P2_temp.block(0,0,P2.rows(),2) = P2;
  }
  else
  {
    P1_temp = P1;
    P2_temp = P2;
  }

  int lastid = lines.rows();
  lines.conservativeResize(lines.rows() + P1_temp.rows(),9);
  for (unsigned i=0; i<P1_temp.rows(); ++i)
    lines.row(lastid+i) << P1_temp.row(i), P2_temp.row(i), i<C.rows() ? C.row(i) : C.row(C.rows()-1);

  dirty |= MeshGL::DIRTY_OVERLAY_LINES;
}

IGL_INLINE void igl::opengl::ViewerData::add_label(const Eigen::VectorXd& P,  const std::string& str)
{
  Eigen::RowVectorXd P_temp;

  // If P only has two columns, pad with a column of zeros
  if (P.size() == 2)
  {
    P_temp = Eigen::RowVectorXd::Zero(3);
    P_temp << P.transpose(), 0;
  }
  else
    P_temp = P;

  int lastid = labels_positions.rows();
  labels_positions.conservativeResize(lastid+1, 3);
  labels_positions.row(lastid) = P_temp;
  labels_strings.push_back(str);
}

IGL_INLINE void igl::opengl::ViewerData::clear_labels()
{
  labels_positions.resize(0,3);
  labels_strings.clear();
}

IGL_INLINE void igl::opengl::ViewerData::clear()
{
  V                       = Eigen::MatrixXd (0,3);
  F                       = Eigen::MatrixXi (0,3);

  F_material_ambient      = Eigen::MatrixXd (0,4);
  F_material_diffuse      = Eigen::MatrixXd (0,4);
  F_material_specular     = Eigen::MatrixXd (0,4);

  V_material_ambient      = Eigen::MatrixXd (0,4);
  V_material_diffuse      = Eigen::MatrixXd (0,4);
  V_material_specular     = Eigen::MatrixXd (0,4);

  F_normals               = Eigen::MatrixXd (0,3);
  V_normals               = Eigen::MatrixXd (0,3);

  V_uv                    = Eigen::MatrixXd (0,2);
  F_uv                    = Eigen::MatrixXi (0,3);

  lines                   = Eigen::MatrixXd (0,9);
  points                  = Eigen::MatrixXd (0,6);
  labels_positions        = Eigen::MatrixXd (0,3);
  labels_strings.clear();

  face_based = false;
}

IGL_INLINE void igl::opengl::ViewerData::compute_normals()
{
  igl::per_face_normals(V, F, F_normals);
  igl::per_vertex_normals(V, F, F_normals, V_normals);
  dirty |= MeshGL::DIRTY_NORMAL;
}

IGL_INLINE void igl::opengl::ViewerData::uniform_colors(
  const Eigen::Vector3d& ambient,
  const Eigen::Vector3d& diffuse,
  const Eigen::Vector3d& specular)
{
  Eigen::Vector4d ambient4;
  Eigen::Vector4d diffuse4;
  Eigen::Vector4d specular4;

  ambient4 << ambient, 1;
  diffuse4 << diffuse, 1;
  specular4 << specular, 1;

  uniform_colors(ambient4,diffuse4,specular4);
}

IGL_INLINE void igl::opengl::ViewerData::uniform_colors(
  const Eigen::Vector4d& ambient,
  const Eigen::Vector4d& diffuse,
  const Eigen::Vector4d& specular)
{
  V_material_ambient.resize(V.rows(),4);
  V_material_diffuse.resize(V.rows(),4);
  V_material_specular.resize(V.rows(),4);

  for (unsigned i=0; i<V.rows();++i)
  {
    V_material_ambient.row(i) = ambient;
    V_material_diffuse.row(i) = diffuse;
    V_material_specular.row(i) = specular;
  }

  F_material_ambient.resize(F.rows(),4);
  F_material_diffuse.resize(F.rows(),4);
  F_material_specular.resize(F.rows(),4);

  for (unsigned i=0; i<F.rows();++i)
  {
    F_material_ambient.row(i) = ambient;
    F_material_diffuse.row(i) = diffuse;
    F_material_specular.row(i) = specular;
  }
  dirty |= MeshGL::DIRTY_SPECULAR | MeshGL::DIRTY_DIFFUSE | MeshGL::DIRTY_AMBIENT;
}

IGL_INLINE void igl::opengl::ViewerData::grid_texture()
{
  // Don't do anything for an empty mesh
  if(V.rows() == 0)
  {
    V_uv.resize(V.rows(),2);
    return;
  }
  if (V_uv.rows() == 0)
  {
    V_uv = V.block(0, 0, V.rows(), 2);
    V_uv.col(0) = V_uv.col(0).array() - V_uv.col(0).minCoeff();
    V_uv.col(0) = V_uv.col(0).array() / V_uv.col(0).maxCoeff();
    V_uv.col(1) = V_uv.col(1).array() - V_uv.col(1).minCoeff();
    V_uv.col(1) = V_uv.col(1).array() / V_uv.col(1).maxCoeff();
    V_uv = V_uv.array() * 10;
    dirty |= MeshGL::DIRTY_TEXTURE;
  }

  unsigned size = 128;
  unsigned size2 = size/2;
  texture_R.resize(size, size);
  for (unsigned i=0; i<size; ++i)
  {
    for (unsigned j=0; j<size; ++j)
    {
      texture_R(i,j) = 0;
      if ((i<size2 && j<size2) || (i>=size2 && j>=size2))
        texture_R(i,j) = 255;
    }
  }

  texture_G = texture_R;
  texture_B = texture_R;
  texture_A = Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic>::Constant(texture_R.rows(),texture_R.cols(),255);
  dirty |= MeshGL::DIRTY_TEXTURE;
}

IGL_INLINE void igl::opengl::ViewerData::updateGL(
  const igl::opengl::ViewerData& data,
  const bool invert_normals,
  igl::opengl::MeshGL& meshgl
  )
{
  if (!meshgl.is_initialized)
  {
    meshgl.init();
  }

  bool per_corner_uv = (data.F_uv.rows() == data.F.rows());
  bool per_corner_normals = (data.F_normals.rows() == 3 * data.F.rows());

  meshgl.dirty |= data.dirty;

  // Input:
  //   X  #F by dim quantity
  // Output:
  //   X_vbo  #F*3 by dim scattering per corner
  const auto per_face = [&data](
      const Eigen::MatrixXd & X,
      MeshGL::RowMatrixXf & X_vbo)
  {
    assert(X.cols() == 4);
    X_vbo.resize(data.F.rows()*3,4);
    for (unsigned i=0; i<data.F.rows();++i)
      for (unsigned j=0;j<3;++j)
        X_vbo.row(i*3+j) = X.row(i).cast<float>();
  };

  // Input:
  //   X  #V by dim quantity
  // Output:
  //   X_vbo  #F*3 by dim scattering per corner
  const auto per_corner = [&data](
      const Eigen::MatrixXd & X,
      MeshGL::RowMatrixXf & X_vbo)
  {
    X_vbo.resize(data.F.rows()*3,X.cols());
    for (unsigned i=0; i<data.F.rows();++i)
      for (unsigned j=0;j<3;++j)
        X_vbo.row(i*3+j) = X.row(data.F(i,j)).cast<float>();
  };

  if (!data.face_based)
  {
    if (!(per_corner_uv || per_corner_normals))
    {
      // Vertex positions
      if (meshgl.dirty & MeshGL::DIRTY_POSITION)
        meshgl.V_vbo = data.V.cast<float>();

      // Vertex normals
      if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
      {
        meshgl.V_normals_vbo = data.V_normals.cast<float>();
        if (invert_normals)
          meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
      }

      // Per-vertex material settings
      if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
        meshgl.V_ambient_vbo = data.V_material_ambient.cast<float>();
      if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
        meshgl.V_diffuse_vbo = data.V_material_diffuse.cast<float>();
      if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
        meshgl.V_specular_vbo = data.V_material_specular.cast<float>();

      // Face indices
      if (meshgl.dirty & MeshGL::DIRTY_FACE)
        meshgl.F_vbo = data.F.cast<unsigned>();

      // Texture coordinates
      if (meshgl.dirty & MeshGL::DIRTY_UV)
      {
        meshgl.V_uv_vbo = data.V_uv.cast<float>();
      }
    }
    else
    {

      // Per vertex properties with per corner UVs
      if (meshgl.dirty & MeshGL::DIRTY_POSITION)
      {
        per_corner(data.V,meshgl.V_vbo);
      }

      if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
      {
        meshgl.V_ambient_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_ambient_vbo.row(i*3+j) = data.V_material_ambient.row(data.F(i,j)).cast<float>();
      }
      if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
      {
        meshgl.V_diffuse_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_diffuse_vbo.row(i*3+j) = data.V_material_diffuse.row(data.F(i,j)).cast<float>();
      }
      if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
      {
        meshgl.V_specular_vbo.resize(data.F.rows()*3,4);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_specular_vbo.row(i*3+j) = data.V_material_specular.row(data.F(i,j)).cast<float>();
      }

      if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
      {
        meshgl.V_normals_vbo.resize(data.F.rows()*3,3);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_normals_vbo.row(i*3+j) =
                         per_corner_normals ?
               data.F_normals.row(i*3+j).cast<float>() :
               data.V_normals.row(data.F(i,j)).cast<float>();


        if (invert_normals)
          meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
      }

      if (meshgl.dirty & MeshGL::DIRTY_FACE)
      {
        meshgl.F_vbo.resize(data.F.rows(),3);
        for (unsigned i=0; i<data.F.rows();++i)
          meshgl.F_vbo.row(i) << i*3+0, i*3+1, i*3+2;
      }

      if (meshgl.dirty & MeshGL::DIRTY_UV)
      {
        meshgl.V_uv_vbo.resize(data.F.rows()*3,2);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_uv_vbo.row(i*3+j) =
              data.V_uv.row(per_corner_uv ?
                data.F_uv(i,j) : data.F(i,j)).cast<float>();
      }
    }
  }
  else
  {
    if (meshgl.dirty & MeshGL::DIRTY_POSITION)
    {
      per_corner(data.V,meshgl.V_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_AMBIENT)
    {
      per_face(data.F_material_ambient,meshgl.V_ambient_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_DIFFUSE)
    {
      per_face(data.F_material_diffuse,meshgl.V_diffuse_vbo);
    }
    if (meshgl.dirty & MeshGL::DIRTY_SPECULAR)
    {
      per_face(data.F_material_specular,meshgl.V_specular_vbo);
    }

    if (meshgl.dirty & MeshGL::DIRTY_NORMAL)
    {
      meshgl.V_normals_vbo.resize(data.F.rows()*3,3);
      for (unsigned i=0; i<data.F.rows();++i)
        for (unsigned j=0;j<3;++j)
          meshgl.V_normals_vbo.row(i*3+j) =
             per_corner_normals ?
               data.F_normals.row(i*3+j).cast<float>() :
               data.F_normals.row(i).cast<float>();

      if (invert_normals)
        meshgl.V_normals_vbo = -meshgl.V_normals_vbo;
    }

    if (meshgl.dirty & MeshGL::DIRTY_FACE)
    {
      meshgl.F_vbo.resize(data.F.rows(),3);
      for (unsigned i=0; i<data.F.rows();++i)
        meshgl.F_vbo.row(i) << i*3+0, i*3+1, i*3+2;
    }

    if (meshgl.dirty & MeshGL::DIRTY_UV)
    {
        meshgl.V_uv_vbo.resize(data.F.rows()*3,2);
        for (unsigned i=0; i<data.F.rows();++i)
          for (unsigned j=0;j<3;++j)
            meshgl.V_uv_vbo.row(i*3+j) = data.V_uv.row(per_corner_uv ? data.F_uv(i,j) : data.F(i,j)).cast<float>();
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_TEXTURE)
  {
    meshgl.tex_u = data.texture_R.rows();
    meshgl.tex_v = data.texture_R.cols();
    meshgl.tex.resize(data.texture_R.size()*4);
    for (unsigned i=0;i<data.texture_R.size();++i)
    {
      meshgl.tex(i*4+0) = data.texture_R(i);
      meshgl.tex(i*4+1) = data.texture_G(i);
      meshgl.tex(i*4+2) = data.texture_B(i);
      meshgl.tex(i*4+3) = data.texture_A(i);
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_OVERLAY_LINES)
  {
    meshgl.lines_V_vbo.resize(data.lines.rows()*2,3);
    meshgl.lines_V_colors_vbo.resize(data.lines.rows()*2,3);
    meshgl.lines_F_vbo.resize(data.lines.rows()*2,1);
    for (unsigned i=0; i<data.lines.rows();++i)
    {
      meshgl.lines_V_vbo.row(2*i+0) = data.lines.block<1, 3>(i, 0).cast<float>();
      meshgl.lines_V_vbo.row(2*i+1) = data.lines.block<1, 3>(i, 3).cast<float>();
      meshgl.lines_V_colors_vbo.row(2*i+0) = data.lines.block<1, 3>(i, 6).cast<float>();
      meshgl.lines_V_colors_vbo.row(2*i+1) = data.lines.block<1, 3>(i, 6).cast<float>();
      meshgl.lines_F_vbo(2*i+0) = 2*i+0;
      meshgl.lines_F_vbo(2*i+1) = 2*i+1;
    }
  }

  if (meshgl.dirty & MeshGL::DIRTY_OVERLAY_POINTS)
  {
    meshgl.points_V_vbo.resize(data.points.rows(),3);
    meshgl.points_V_colors_vbo.resize(data.points.rows(),3);
    meshgl.points_F_vbo.resize(data.points.rows(),1);
    for (unsigned i=0; i<data.points.rows();++i)
    {
      meshgl.points_V_vbo.row(i) = data.points.block<1, 3>(i, 0).cast<float>();
      meshgl.points_V_colors_vbo.row(i) = data.points.block<1, 3>(i, 3).cast<float>();
      meshgl.points_F_vbo(i) = i;
    }
  }
}
