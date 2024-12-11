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
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <cppitertools/range.hpp>
#include <cppitertools/enumerate.hpp>
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	

	return true;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		/*In the initialize() method, run over all the elements of the array initializing each cell.
		 *Create a graphic element using the scene->rect(...) method and store the resulting point
		 *in the TCell field. Position each rectangle to its correct place in the canvas and set the color to light_grey.*/
		viewer = new AbstractGraphicViewer(this->frame, QRectF{-5000, 2500, 10000, -5000});
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		this->resize(800, 700);

		for (const auto &[i, row]: grid | iter::enumerate)
			for (const auto &[j, cell]: row | iter::enumerate)
			{
				cell.state = State::Unknown;
				cell.item  = viewer->scene.addRect(QRectF{0,0, cell_size, cell_size}, QPen(QColor("lightgray"), 15));
				auto p = grid_to_float({i, j});
				printf("%f, %f, %lu, %lu\n", p.x(), p.y(), i, j);
				cell.item->setPos(p.x(), p.y());
			}

		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);

	}
}


void SpecificWorker::compute()
{
	auto lidar_points = read_lidar_bpearl();  // Fetch filtered LiDAR points

	for (const auto& point : lidar_points)
	{
		float x_i = point.x();
		float y_i = point.y();

		// Calcular la distancia desde el origen (0, 0) al punto (x_i, y_i)
		float distance = std::sqrt(x_i * x_i + y_i * y_i);

		// Calcular el número de pasos S y delta
		int S = static_cast<int>(distance / 100);  // Número de celdas de 100mm
		float delta = 1.0f / S;  // Tamaño del paso en la parábola

		// Recorrer la línea en pasos de delta
		for (float k = 0; k <= 1.0f; k += delta)
		{
			// Calculamos el punto en la línea para el valor de k
			float x_k = k * x_i;
			float y_k = k * y_i;

			// Convertir las coordenadas flotantes a la cuadrícula
			if (x_k > 5000)
				x_k = 5000;
			if (x_k < -5000)
				x_k = -4900;
			if (y_k > 5000)
				y_k = 5000;
			if (y_k < -5000)
				y_k = -4900;
			position2d grid_pos = float_to_grid(Eigen::Vector2f(x_k, y_k));

			// Cambiar el color de las celdas en la cuadrícula
			auto& cell = grid[grid_pos.first][grid_pos.second];  // Obtener la celda en la posición de la cuadrícula
			if (k < 1.0f) {
				// Cambiar el color de la celda a blanco, excepto la última celda
				cell.item->setBrush(QColor("white"));
				cell.state = State::Free;
			} else {
				// La última celda (k = 1) se pinta de rojo
				cell.item->setBrush(QColor("red"));
				cell.state = State::Occupied;
			}
		}
		dijkstra();
		// Debug: Imprimir la ecuación de la línea
		//std::cout << "Line Equation for point (" << x_i << ", " << y_i << "):" << std::endl;
		//std::cout << "r(t) = (" << x_i << " * t, " << y_i << " * t), where t runs from 0 to 1 in steps of delta" << std::endl;
	}
	
}
// x in mm
// de lo que devuelvo cada "cuadrado" son de 100mm
//
//
// si el t.point esta entre 2? q hago x ejemplo 1000, 0
// -5000+ -> 0
// 0-     -> 49
// 0+     -> 50 [0, 99]
// 5000-  -> 99
SpecificWorker::position2d SpecificWorker::float_to_grid(Eigen::Vector2f x)
{
	SpecificWorker::position2d tmp;
	tmp.first = (5000 - x.x()) / 100;
	tmp.second = (5000 - x.y()) / 100;

	return tmp;
}

// 0,0 -> -4950,4950
// 0,1 -> -4850,4950
Eigen::Vector2f SpecificWorker::grid_to_float(SpecificWorker::position2d x){
	Eigen::Vector2f tmp;
	tmp.x() = -4950 + x.second * 100;
	tmp.y() =  4950 - x.first  * 100;
	return tmp;
}

std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
	try
	{
		auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f>  p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z < 500 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}
		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
#ifdef HIBERNATION_ENABLED
	hibernation = true;
#endif
//implementCODE

}



/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result

