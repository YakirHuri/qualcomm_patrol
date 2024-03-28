/*
 * enable_auto_docking_cmd_node.cpp
 *
 *  Created on: Jun 12, 2020
 *      Author: yakir huri
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */


#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/SensorState.h>

#include <actionlib_msgs/GoalID.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <geometry_msgs/Twist.h>



#include <iostream>
#include <random>

using namespace std;

class EnableAutoDocking {

public:
   
   EnableAutoDocking(){


      ros::NodeHandle node_;
      ros::NodeHandle nodePrivate("~");

      dockingState_ = false;


      /// subscrivers
      autoDockingRequestSub_ = node_.subscribe("/start_stop_auto_docking", 1,
         &EnableAutoDocking::startAutoDocking, this); 

      undockFromChargerSub_ = node_.subscribe("/undock_from_charger", 1,
         &EnableAutoDocking::undockFromChargerCallback, this); 


      autoDockingStatusSub_ = node_.subscribe("/dock_drive_action/feedback", 1,
         &EnableAutoDocking::autoDockingStatus, this); 

      batteryVoltSub_ = node_.subscribe("/mobile_base/sensors/core", 1,
         &EnableAutoDocking::batteryStatusCallback, this);       

         
      ///publishers    
      autoDockingCmdPub_ = node_.advertise<kobuki_msgs::AutoDockingActionGoal>("/dock_drive_action/goal", 1, true);

      cancleAutoDockingCmdPub_ = node_.advertise<actionlib_msgs::GoalID>("/dock_drive_action/cancel", 1, true);

      dockingFeedbackPub_= node_.advertise<std_msgs::String>("/docking_feedback_string", 1, true);

      batteryStatusPub_ = node_.advertise<std_msgs::Int32>("/battery_voltage", 1, true);

      dockedInStatusPub_ = node_.advertise<std_msgs::Bool>("/is_charging_in_docking_station", 1, true);

      // for undocking
      velocityPublisher_ = node_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1, true);

      undockFromChanrger_ = false;

      nodePrivate.param("undocking_duration_seconds", undockingDurationSeconds_, double(3.0));
   }
   ~EnableAutoDocking(){}

   void autoDockingStatus(const kobuki_msgs::AutoDockingActionFeedbackPtr &msg){ 
      
    
      std_msgs::String docking_feedback_msg;
      docking_feedback_msg.data = msg->feedback.state;

      dockingFeedbackPub_.publish(docking_feedback_msg);
     
   
   }

   void batteryStatusCallback(const kobuki_msgs::SensorStatePtr &msg) {


      int batteryVolt = msg->battery;
      int kobuki_base_max_charge = 160;

      // nimbus - publish if the kobuki inside thhe docking station or not

      std_msgs::Bool chargingMsg;

      if(int(msg->charger) == 0){

         chargingMsg.data = false;
         dockedInStatusPub_.publish(chargingMsg);

      } else {         
         
         chargingMsg.data = true;
         dockedInStatusPub_.publish(chargingMsg);

      }
			
      
     
      
       // nimbus - publish precnet volt

      int nimbusPercentBatteryVolt = (float(batteryVolt) / float(kobuki_base_max_charge) * 100);

      std_msgs::Int32 msg_battery;
      msg_battery.data = nimbusPercentBatteryVolt;
      batteryStatusPub_.publish(msg_battery);


   }

   void startAutoDocking(const std_msgs::BoolConstPtr &msg) {     
    
      
      int range = 100  - 1 + 1;
           
      if ( msg->data == true && dockingState_ == false) {
         
         num = rand() % range + 1;   	 
	   cerr<<" start docking with id "<<num<<endl; 	
         dockingState_ = true;

         //publish cmd to start docking

         kobuki_msgs::AutoDockingActionGoal command;
         command.goal_id.id = std::to_string(num);
         autoDockingCmdPub_.publish(command);

      } else if ( msg->data == false && dockingState_ == true) {
         
         cerr<<" cancel docking "<<endl; 	
         dockingState_ = false;

         actionlib_msgs::GoalID cancleMsg;
         cancleMsg.id = std::to_string(num);
         cancleAutoDockingCmdPub_.publish(cancleMsg);

         // the feedback is /dock_drive_action/feedback - > kobuki_msgs/AutoDockingActionFeedback
      }
   }

   void undockFromChargerCallback(const std_msgs::BoolConstPtr &msg){

         if( msg->data == true ){

            cerr<<"undockFromChargerCallback"<<msg->data<<endl;

            undockFromCharger();
            undockFromChanrger_ = true;

         }   
   }



   void undockFromCharger() {
      ROS_INFO(" - Undocking from charging station...");

      const ros::Duration undockDuration(undockingDurationSeconds_); // seconds
      ros::Time startTime = ros::Time::now();

      ros::Rate rate(20);

      geometry_msgs::Twist velocity;

      velocity.linear.x = -0.1;

      while ( (ros::Time::now() - startTime) < undockDuration) {
         cerr<<" publish undock inside while "<<endl;
         velocityPublisher_.publish(velocity);
         rate.sleep();
      }

      velocityPublisher_.publish(geometry_msgs::Twist());

      ros::Duration(2).sleep();

      ROS_INFO(" - Done Undocking !");

   }

private:
   
   int num = 1;	
   bool dockingState_; 

   bool undockFromChanrger_ = false;


   ros::Subscriber autoDockingRequestSub_; 

   ros::Subscriber undockFromChargerSub_;

   ros::Subscriber autoDockingStatusSub_;

   ros::Subscriber batteryVoltSub_;

   ros::Publisher autoDockingCmdPub_; 

   ros::Publisher cancleAutoDockingCmdPub_;

   ros::Publisher dockingFeedbackPub_;

   ros::Publisher batteryStatusPub_;

   ros::Publisher dockedInStatusPub_;

   ros::Publisher velocityPublisher_;

   double undockingDurationSeconds_;
};




int main(int argc, char** argv) {
    
   ros::init(argc, argv, "enable_auto_docking_cmd_node");
   EnableAutoDocking enableAutoDocking;   
   ros::spin(); 
   
   return 0;

}

