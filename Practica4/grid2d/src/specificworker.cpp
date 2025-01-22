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

RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target) {
    RoboCompGrid2D::Result result;

    auto maybe_position2d = float_to_grid({target.x, target.y});
    if (!maybe_position2d)
        return {};

    path = dijkstra({50, 50}, {maybe_position2d.value().first, maybe_position2d.value().second});
        
    std::ranges::transform(path, std::back_inserter(result.path), [&](auto &p) {
        auto f = grid_to_float({p.first, p.second});
        return RoboCompGrid2D::TPoint{f.x(), f.y(), 0.f};
    });
    
    return result;
}

SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx){
	this->startup_check_flag = startup_check;
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
//		viewer = new AbstractGraphicViewer(this->frame, QRectF{-5000, 2500, 10000, -5000}0i//);
		viewer = new AbstractGraphicViewer(this->frame, QRectF{-5000, -5000, 10000, 10000});
        viewer->setStyleSheet("background-color: lightGray;");
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
        robot_draw = r;
		this->resize(800, 700);
        viewer->show();
        QPen pen(QColor("blue"), 15);
		for (const auto &[i, row]: grid | iter::enumerate)
			for (const auto &[j, cell]: row | iter::enumerate)
			{
				//cell.state = State::Unknown;
				auto p = grid_to_float({i, j});
				cell.item  = viewer->scene.addRect(QRectF{p.x() - 50, p.y() - 50, cell_size, cell_size}, QPen(QColor("lightgray"), 15));
				//printf("%f, %f, %lu, %lu\n", p.x(), p.y(), i, j);
			}

   		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)), this, SLOT(viewerSlot(QPointF)));
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		this->setPeriod(STATES::Compute, 100);

	}
}

void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(Qt::darkGreen);
	auto brush = QBrush(QColor(Qt::darkGreen));
	for(const auto &p : filtered_points)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}

}

char * SpecificWorker::state_to_string(SpecificWorker::State a){
switch(a){
		        case State::Unknown:
	        return "Unknown";
		        case State::Occupied:
		        return "Occupied";
   		            break;
		        case State::Free:
	        return "Free";
	        }
}
void SpecificWorker::draw_state(void){
	for (int i =0; i< grid.size(); i++) {
		for (int j=0; j<grid[i].size(); j++) {
		    switch(grid[i][j].state){
		        case State::Unknown:
		            grid[i][j].item->setBrush(QColor("grey"));
		            break;
		        case State::Occupied:
		            grid[i][j].item->setBrush(QColor("red"));
		            break;
		        case State::Free:
                    grid[i][j].item->setBrush(QColor("white"));
                    break;
		    }
		}
	}
}

static bool hacer_camino = false;
static QPointF p_;
void SpecificWorker::compute()
{
	auto lidar_points = read_lidar_bpearl();  // Fetch filtered LiDAR points
	draw_lidar(lidar_points, &viewer->scene);

	for (int i =0; i< grid.size(); i++) {
		for (int j=0; j<grid[i].size(); j++) {
			grid[i][j].state = State::Unknown;
		}
	}
	for (const auto& point : lidar_points){
		const float distance = std::hypot(point.x(), point.y());
		const float delta = 1.0f / (distance / 100);  // Tamaño del paso en la parábola
		std::optional<SpecificWorker::position2d> maybe_position2d;
		for (float k = 0; k <= 1.1f; k += delta){
			maybe_position2d = float_to_grid(point * k);
			if (!maybe_position2d)
				continue;
	        if(k > 0.9f){
	            grid[maybe_position2d.value().first][maybe_position2d.value().second].state = State::Occupied;
	        } else{
	        	grid[maybe_position2d.value().first][maybe_position2d.value().second].state = State::Free;
            }
	    }
	}
	// Cambiar el color de las celdas en la cuadrícula
	draw_state();
	if(hacer_camino){
	    hacer_camino = false;
	    viewerSlot_compute(p_);
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

std::optional<SpecificWorker::position2d> SpecificWorker::float_to_grid(Eigen::Vector2f x)
{
	SpecificWorker::position2d tmp;
	if (x.x() > 5000 || x.x() < -5000 || x.y() > 5000 || x.y() < -5000)
		return std::nullopt;
	tmp.first =  (5000 + x.x()) / 100;
	tmp.second = (5000 - x.y()) / 100;
	return tmp;
}

// 0,0 -> -4950,4950
// 0,1 -> -4850,4950
Eigen::Vector2f SpecificWorker::grid_to_float(SpecificWorker::position2d x){
	Eigen::Vector2f tmp;
	tmp.x() = -4950 + x.first * 100;
	tmp.y() =  4950 - x.second  * 100;
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



void SpecificWorker::draw_path(QGraphicsScene *scene, bool solo_limpiar)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations
	// remove all items drawn in the previous iteration

	for(auto i: items) {
		scene->removeItem(i);
        delete i;
	}
	items.clear();
    if(!solo_limpiar){
		const auto color = QColor(Qt::darkBlue);
		const auto brush = QBrush(QColor(Qt::darkBlue));
		for(const auto &p : path){
			auto pfloat = grid_to_float({p.first, p.second});
			auto item = scene->addRect(-50, -50, 100, 100, color, brush);
			item->setPos(pfloat.x(), pfloat.y());
			items.push_back(item);
		}

		printf("PINTANDO %d, %d\n", path.back().first, path.back().second);
	}
}

bool SpecificWorker::grid_index_valid(const SpecificWorker::position2d& index) {
	return index.first >= 0 && index.first < grid_size && index.second >= 0 && index.second < grid_size;
}

std::vector<SpecificWorker::position2d> SpecificWorker::dijkstra(SpecificWorker::position2d start, SpecificWorker::position2d goal){
    std::unordered_map<SpecificWorker::position2d, int, position2dHash> distance_map;
    std::unordered_map<SpecificWorker::position2d, SpecificWorker::position2d, position2dHash> previous_map;

    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> pq;
    pq.push({0, start});
    distance_map[start] = 0;

    std::vector<SpecificWorker::position2d> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

    while (!pq.empty()) {
        Cell current = pq.top();
        pq.pop();

        if (current.position == goal) {
        	std::vector<SpecificWorker::position2d> path;
        	while (previous_map.find(current.position) != previous_map.end()) {
        		path.push_back(current.position);
        		current.position = previous_map[current.position];
        	}
        	std::reverse(path.begin(), path.end());
        	return path;
        }

        for (const auto& dir : directions) {
            SpecificWorker::position2d neighbor(current.position.first + dir.first, current.position.second + dir.second);
            if (grid_index_valid(neighbor)) {
            	int neighbor_cost = 1;
            	if (grid[neighbor.first][neighbor.second].state == State::Occupied)
            		continue;
            	else
            		neighbor_cost = 1;
                int new_cost = current.cost + neighbor_cost;

                if (distance_map.find(neighbor) == distance_map.end() || new_cost < distance_map[neighbor]) {
                    distance_map[neighbor] = new_cost;
                    previous_map[neighbor] = current.position;
                    pq.push({new_cost, neighbor});
                }
            }
        }
    }

    return {};
}

void SpecificWorker::viewerSlot(QPointF p)
{
    hacer_camino = true;
    p_ = p;
}

void SpecificWorker::viewerSlot_compute(QPointF p)
{
    auto maybe_position2d = float_to_grid({p.x(), p.y()});
    if (!maybe_position2d){
        return;
    }
    QPoint index(maybe_position2d.value().first, maybe_position2d.value().second);

	if (grid[index.x()][index.y()].state == State::Occupied) {
		draw_path(&viewer->scene, true);
		return;
	}
    //const QPoint index = real_to_index(p.x(), p.y());
    int goalX = index.x();
    int goalY = index.y();

    if (goalX < 0 || goalX >= grid_size || goalY < 0 || goalY >= grid_size){
        return;
    }

    path = dijkstra({50, 50}, {goalX, goalY});

	if(path.empty()) {
		draw_path(&viewer->scene, true);
	} else {
		draw_path(&viewer->scene, false);
	}
}
