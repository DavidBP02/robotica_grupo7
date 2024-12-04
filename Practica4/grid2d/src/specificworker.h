/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <QGraphicsRectItem>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>


#include <genericworker.h>


#include "abstract_graphic_viewer/abstract_graphic_viewer.h"


class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target);
		SpecificWorker(TuplePrx tprx, bool startup_check);
		~SpecificWorker();

	public slots:
		void initialize();
		void compute();
		void emergency();
		void restore();
		int startup_check();
	private:

	bool startup_check_flag;
	enum class State {Occupied, Free, Unknown};
	struct TCell
	{
		State state;
		QGraphicsRectItem *item;
		/* other fields */
	};
	static constexpr int world_size = 10000;
	static constexpr int cell_size  = 100; // mm
	static constexpr int grid_size = world_size / cell_size;
	std::array<std::array<TCell, grid_size>, grid_size> grid;
	using position2d = std::pair<int, int>;
	AbstractGraphicViewer *viewer;

	position2d float_to_grid(Eigen::Vector2f x);
	Eigen::Vector2f grid_to_float(position2d x);


	bool setParams(RoboCompCommonBehavior::ParameterList params);
	std::vector<Eigen::Vector2f> read_lidar_bpearl();
};

#endif
