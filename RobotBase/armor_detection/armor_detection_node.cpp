/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <unistd.h>
#include "armor_detection_node.h"
#include "../gimbal.h"

ArmorDetectionNode::ArmorDetectionNode() :
	demensions_(3),
	initialized_(true),
	detected_enemy_(false),
	undetected_count_(0)
{

}

bool ArmorDetectionNode::Init() {
	//create the selected algorithms
	std::string selected_algorithm = "constraint_set";
	// create image receiver
	cv_toolbox_ = std::make_shared<CVToolbox>("camera");
	// create armor detection algorithm
	/*
	armor_detector_ = AlgorithmFactory<ArmorDetectionBase, std::shared_ptr<CVToolbox>>::CreateAlgorithm
	(selected_algorithm, cv_toolbox_);
	*/
	armor_detector_ = std::make_shared<ConstraintSet>(cv_toolbox_);

	undetected_armor_delay_ = 10;
	if (armor_detector_ == nullptr)
		return false;
	else
		return true;
}

void ArmorDetectionNode::ExecuteLoop() {
	undetected_count_ = undetected_armor_delay_;
	//while (running_) {
	////	usleep(1);
	cv::Point3f target_3d;
	cv::Mat depth;
	if (!armor_detector_->DetectArmor(detected_enemy_, target_3d, depth))
	{
		std::lock_guard<std::mutex> guard(mutex_);
		x_ = target_3d.x;
		y_ = target_3d.y;
		z_ = target_3d.z;
	}
	if (detected_enemy_)
	{
		//把shoot改成false,跟踪装甲不需要上层控制发射子弹
		gimbal->AimAtArmor(target_3d.x, target_3d.y, depth, true, true, true);
	}
	else if (undetected_count_ != 0) {
		//下面的代码实际上是没有作用的
		gimbal->AimAtArmor(0, 0, depth, true, true, false);
	}
	//}
}

void ArmorDetectionNode::StartThread() {
	running_ = true;
	armor_detector_->SetThreadState(true);
	//armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
	condition_var_.notify_one();
}

void ArmorDetectionNode::StopThread() {
	running_ = false;
	armor_detector_->SetThreadState(false);
	if (armor_detection_thread_.joinable()) {
		armor_detection_thread_.join();
	}
}

ArmorDetectionNode::~ArmorDetectionNode() {
	StopThread();
}
