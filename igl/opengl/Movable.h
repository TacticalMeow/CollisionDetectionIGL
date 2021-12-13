#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void MyRotate(const Eigen::Matrix3d &rot);
	void MyScale(Eigen::Vector3d amt);

	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);

	Eigen::Matrix3d GetRotation() const{ return Tout.rotation().matrix(); }

	virtual ~Movable() {}
private:
	Eigen::Affine3d Tout,Tin;
};

