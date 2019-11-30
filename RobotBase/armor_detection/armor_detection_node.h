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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>
#include "../msg.h"

#include "cv_toolbox.h"

#include "armor_detection_base.h"
#include "armor_detection_algorithms.h"

class ArmorDetectionNode {
public:
	explicit ArmorDetectionNode();
	/**
	 * @brief Initializing armor detection algorithm.
	 * @return Return the error information.
	 */
	bool Init();
	/**
	 * @brief Starting the armor detection thread.
	 */
	void StartThread();
	/**
	 * @brief Stopping armor detection thread.
	 */
	void StopThread();
	/**
	 * @brief Executing the armor detection algorithm.
	 */
	void ExecuteLoop();

	~ArmorDetectionNode();

	std::shared_ptr<CVToolbox> cv_toolbox_;
protected:
private:
	std::shared_ptr<ArmorDetectionBase> armor_detector_;
	std::thread armor_detection_thread_;
	unsigned int max_rotating_fps_;
	unsigned int min_rotating_detected_count_;
	unsigned int undetected_armor_delay_;

	//! state and error
	bool initialized_;
	bool running_;
	std::mutex mutex_;
	std::condition_variable condition_var_;
	unsigned int undetected_count_;

	//! enemy information
	double x_;
	double y_;
	double z_;
	bool detected_enemy_;
	unsigned long demensions_;

	//! control model
	GimbalAngle gimbal_angle_;
};

#endif //ROBORTS_DETECTION_ARMOR_DETECTION_NODE_H
