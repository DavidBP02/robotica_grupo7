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
	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1900; // mm/s
		float MAX_ROT_SPEED = 2; // rad/s
		float SEARCH_ROT_SPEED = 0.9; // rad/s
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// person
		float PERSON_MIN_DIST = 800; // mm
		int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
		// lidar
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
		// control track
		float acc_distance_factor = 2;
		float k1 = 1.1;  // proportional gain for the angle error;
		float k2 = 0.5; // proportional gain for derivative of the angle error;
	};
	Params params;
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

	// viewer
	AbstractGraphicViewer *viewer;
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
	QGraphicsPolygonItem *robot_draw;

	position2d float_to_grid(Eigen::Vector2f x);
	Eigen::Vector2f grid_to_float(position2d x);


	bool setParams(RoboCompCommonBehavior::ParameterList params);
	std::vector<Eigen::Vector2f> read_lidar_bpearl();
};

#endif