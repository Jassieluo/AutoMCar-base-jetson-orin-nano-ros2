#include "FFDirewolfButton_msgsTools.hpp"

void FFDirewolfAxisButtonInit(ffdirewolfbutton_msgs::msg::AxisButtonMsgs &FFDirewolfMsgs_)
      {
      	 FFDirewolfMsgs_.control_mode = 0;  // 0: hand 1:auto 2:detect
      
         // FFDirewolf axis_name
         FFDirewolfMsgs_.axis_name.push_back("LeftHor"); // 0
         FFDirewolfMsgs_.axis_name.push_back("LeftVer"); // 1
         FFDirewolfMsgs_.axis_name.push_back("RightHor"); // 2
         FFDirewolfMsgs_.axis_name.push_back("RightVer"); // 3
         FFDirewolfMsgs_.axis_name.push_back("RightT"); // 4
         FFDirewolfMsgs_.axis_name.push_back("LeftT"); // 5
         FFDirewolfMsgs_.axis_name.push_back("MiddleHor"); // 6
         FFDirewolfMsgs_.axis_name.push_back("MiddleVer"); // 7

         // FFDirewolf button_name
         FFDirewolfMsgs_.button_name.push_back("A"); // 0
         FFDirewolfMsgs_.button_name.push_back("B"); // 1
         FFDirewolfMsgs_.button_name.push_back(" "); // 2
         FFDirewolfMsgs_.button_name.push_back("X"); // 3
         FFDirewolfMsgs_.button_name.push_back("Y"); // 4
         FFDirewolfMsgs_.button_name.push_back(" "); // 5
         FFDirewolfMsgs_.button_name.push_back("LB"); // 6
         FFDirewolfMsgs_.button_name.push_back("RB"); // 7
         FFDirewolfMsgs_.button_name.push_back("LT"); // 8
         FFDirewolfMsgs_.button_name.push_back("RT"); // 9
         FFDirewolfMsgs_.button_name.push_back("SELECT"); // 10
         FFDirewolfMsgs_.button_name.push_back("START"); // 11
         FFDirewolfMsgs_.button_name.push_back("M1"); // 12
         FFDirewolfMsgs_.button_name.push_back("M2"); // 13

         //FFDirewolf axis
         FFDirewolfMsgs_.axis.push_back(0); // 0
         FFDirewolfMsgs_.axis.push_back(0); // 1
         FFDirewolfMsgs_.axis.push_back(0); // 2
         FFDirewolfMsgs_.axis.push_back(0); // 3
         FFDirewolfMsgs_.axis.push_back(0); // 4
         FFDirewolfMsgs_.axis.push_back(0); // 5
         FFDirewolfMsgs_.axis.push_back(0); // 6
         FFDirewolfMsgs_.axis.push_back(0); // 7

         //FFDirewolf button
         FFDirewolfMsgs_.button.push_back(0); // 0
         FFDirewolfMsgs_.button.push_back(0); // 1
         FFDirewolfMsgs_.button.push_back(0); // 2
         FFDirewolfMsgs_.button.push_back(0); // 3
         FFDirewolfMsgs_.button.push_back(0); // 4
         FFDirewolfMsgs_.button.push_back(0); // 5
         FFDirewolfMsgs_.button.push_back(0); // 6
         FFDirewolfMsgs_.button.push_back(0); // 7
         FFDirewolfMsgs_.button.push_back(0); // 8
         FFDirewolfMsgs_.button.push_back(0); // 9
         FFDirewolfMsgs_.button.push_back(0); // 10
         FFDirewolfMsgs_.button.push_back(0); // 11
         FFDirewolfMsgs_.button.push_back(0); // 12
         FFDirewolfMsgs_.button.push_back(0); // 13
      }
