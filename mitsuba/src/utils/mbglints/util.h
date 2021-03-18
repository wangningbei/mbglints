/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#if !defined(__UTIL_H_)
#define __UTIL_H_


#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

Eigen::Vector3f myreflect(const Eigen::Vector3f &wi, const Eigen::Vector3f  &n);
Eigen::Vector3f myrefract(const Eigen::Vector3f &wi, const Eigen::Vector3f  &n, float eta);
void coordinateSystem(const Eigen::Vector3f &a, Eigen::Vector3f &b, Eigen::Vector3f &c);
void mycoordinateSystem(const Eigen::Vector3f &a, Eigen::Vector3f &b, Eigen::Vector3f &c);
float myfresnelDielectricExt(float cosThetaI,
	float &cosThetaT, float eta);
float myfresnelDielectric(float cosThetaI,
	float cosThetaT, float eta);
Eigen::Vector3f myfresnelConductorExact(float cosThetaI,
	const Eigen::Vector3f &eta, const Eigen::Vector3f &k);

#endif /* __MITSUBA_CORE_UTIL_H_ */
