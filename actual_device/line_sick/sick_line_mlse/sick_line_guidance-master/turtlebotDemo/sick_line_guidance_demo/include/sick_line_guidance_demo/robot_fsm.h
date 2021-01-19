/*
 * RobotFSM implements the state machine to explore and follow a line
 * for sick_line_guidance_demo.
 * The following RobotStates are executed:
 * INITIAL -> EXPLORE_LINE -> FOLLOW_LINE [ -> WAIT_AT_BARCODE -> FOLLOW_LINE ] -> EXIT
 * Theses states are implemented in the following classes:
 * EXPLORE_LINE:    ExploreLineState
 * FOLLOW_LINE:     FollowLineState
 * WAIT_AT_BARCODE: WaitAtBarcodeState
 *
 * Copyright (C) 2019 Ing.-Buero Dr. Michael Lehning, Hildesheim
 * Copyright (C) 2019 SICK AG, Waldkirch
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SICK AG nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *     * Neither the name of Ing.-Buero Dr. Michael Lehning nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Michael Lehning <michael.lehning@lehning.de>
 *
 *  Copyright 2019 SICK AG
 *  Copyright 2019 Ing.-Buero Dr. Michael Lehning
 *
 */
#ifndef __SICK_LINE_GUIDANCE_DEMO_ROBOT_FSM_H_INCLUDED
#define __SICK_LINE_GUIDANCE_DEMO_ROBOT_FSM_H_INCLUDED

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include "sick_line_guidance/OLS_Measurement.h"
#include "sick_line_guidance_demo/explore_line_state.h"
#include "sick_line_guidance_demo/follow_line_state.h"
#include "sick_line_guidance_demo/robot_fsm_context.h"
#include "sick_line_guidance_demo/wait_at_barcode_state.h"

namespace sick_line_guidance_demo
{
  /*
   * class RobotFSM implements the state machine to explore and follow a line
   * for sick_line_guidance_demo.
   * Input: ols and odometry messages
   * Output: cmd_vel messages
   */
  class RobotFSM
  {
  public:

    /*
     * Constructor
     * @param[in] nh ros handle
     * @param[in] ros_topic_ols_messages ROS topic for OLS_Measurement messages (input)
     * @param[in] ros_topic_odometry ROS topic for odometry incl. robot positions (input)
     * @param[in] ros_topic_cmd_vel ROS topic for cmd_vel messages (output)
     */
    RobotFSM(ros::NodeHandle* nh=0, const std::string & ros_topic_ols_messages = "/ols", const std::string & ros_topic_odometry = "/odom", const std::string & ros_topic_cmd_vel = "/cmd_vel");
  
    /*
     * Destructor
     */
    ~RobotFSM();
  
    /*
     * Start thread to run the final state machine. Read messages form ols and odom topics, publish messages to cmd_vel
     */
    void startFSM(void);
  
    /*
     * Stops the thread to run the final state machine
     */
    void stopFSM(void);
  
    /*
     * message callback for odometry messages. This function is called automatically by the ros::Subscriber after subscription of topic "/odom".
     * It transforms the robots xy-positions from world/meter into map/pixel position, detect lines and barcodes in the map plus their distance to the robot,
     * and transform them invers into world coordinates.
     * @param[in] msg odometry message (input)
     */
    virtual void messageCallbackOdometry(const nav_msgs::Odometry::ConstPtr& msg);
  
    /*
     * message callback for OLS measurement messages. This function is called automatically by the ros::Subscriber after subscription of topic "/ols".
     * It displays the OLS measurement (line info and barcodes), if visualization is enabled.
     * @param[in] msg OLS measurement message (input)
     */
    virtual void messageCallbackOlsMeasurement(const boost::shared_ptr<sick_line_guidance::OLS_Measurement const>& msg);

  protected:

    /*
     * thread callback, runs the final state machine for sick_line_guidance_demo.
     * Input: ols and odometry messages
     * Output: cmd_vel messages
     */
    void runFSMthread(void);

    /*
     * member data: context and subscriber
     */

    RobotFSMContext m_fsm_context;        // shared context
    ros::Subscriber m_ols_subscriber;     // ros subscriber for ols messages (fsm input)
    ros::Subscriber m_odom_subscriber;    // ros subscriber for odom messages (fsm input)
    ros::Publisher m_cmd_vel_publisher;   // ros publisher for cmd_vel messages (fsm output)
  
    /*
     * member data: states and thread to run the state engine
     */
  
    boost::thread* m_fsm_thread;          // thread to run the state machine
    bool m_fsm_thread_run;                // true: m_fsm_thread is currently running, false: m_fsm_thread stopping
    sick_line_guidance_demo::ExploreLineState m_explore_line;  // state to explore a line in state RobotState::EXPLORE_LINE
    sick_line_guidance_demo::FollowLineState m_follow_line;  // state to follow a line in state RobotState::FOLLOW_LINE
    sick_line_guidance_demo::WaitAtBarcodeState wait_at_barcode_state;  // state to wait at a barcode in state RobotState::WAIT_AT_BARCODE
  
  }; // class RobotFSM

} // namespace sick_line_guidance_demo
#endif // __SICK_LINE_GUIDANCE_DEMO_ROBOT_FSM_H_INCLUDED

