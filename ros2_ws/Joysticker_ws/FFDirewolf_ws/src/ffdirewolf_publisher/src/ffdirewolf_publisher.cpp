#include <rclcpp/rclcpp.hpp>
#include <linux/joystick.h>
#include <fcntl.h>
#include <ffdirewolfbutton_msgs_tools/FFDirewolfButton_msgsTools.hpp>

class FFDirewolfPubNode : public rclcpp::Node
{
   private:
      ffdirewolfbutton_msgs::msg::AxisButtonMsgs FFDirewolfMsgs = ffdirewolfbutton_msgs::msg::AxisButtonMsgs();
      int fd, rc;
      char number_of_axes = 0;
      char number_of_btns = 0;
      char js_name_str[128];
      const char* JoystickDevice;
      rclcpp::TimerBase::SharedPtr time_;
      rclcpp::Publisher<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>::SharedPtr FFDmsgsPuber;

   public:
      FFDirewolfPubNode(const std::string NodeName,const std::string TopicName,const char* JoystickDevice_="/dev/input/js0", int cL=10):Node(NodeName)
      {
         FFDirewolfAxisButtonInit(FFDirewolfMsgs);
         FFDmsgsPuber = this->create_publisher<ffdirewolfbutton_msgs::msg::AxisButtonMsgs>(TopicName, cL);
         time_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&FFDirewolfPubNode::FFDMsgRecPuber, this));

         JoystickDevice = JoystickDevice_;
         int fd_open_count = 1;

         while (fd_open_count<=10)
         {
            fd = open(JoystickDevice, O_RDONLY);
            
            if (fd==-1)
            {
               RCLCPP_INFO(this->get_logger(), "Epochs:%d Unable to Open the Joystick, waiting for 3 seconds.\n", fd_open_count);
               sleep(3); //wait for 3 seconds
               fd_open_count++;
               fd = open(JoystickDevice, O_RDONLY);
            }
            else
            {
               break;
            }
         }
         if (fd_open_count>=11)
         {
            perror("Unable to Open the Joystick");
            exit(EXIT_FAILURE);
         }
         if (ioctl(fd, JSIOCGNAME(sizeof(js_name_str)), js_name_str) < 0)
         {
            RCLCPP_INFO(this->get_logger(),"%s Unknown %ld\n", js_name_str, sizeof(js_name_str));
         }
         RCLCPP_INFO(this->get_logger(), "joystick Name: %s\n", js_name_str);

         rc = ioctl(fd, JSIOCGAXES, &number_of_axes);
         if (rc != -1)
         {
            RCLCPP_INFO(this->get_logger(), "number_of_axes:%d\n", number_of_axes);
         }
         rc = ioctl(fd, JSIOCGBUTTONS, &number_of_btns);
         if (rc != -1)
         {
            RCLCPP_INFO(this->get_logger(), "number_of_btns:%d\n", number_of_btns);
         }
      }

      void FFDMsgRecPuber()
      {
         struct js_event jsEvent;
         ssize_t bytes = read(fd, &jsEvent, sizeof(struct js_event));
         int jsEvent_count = 1;
         if (bytes == -1)
         {
            while (jsEvent_count<=1)
            {
               bytes = read(fd, &jsEvent, sizeof(struct js_event));
               if (bytes == -1)
               {
                  RCLCPP_INFO(this->get_logger(), "Epochs:%d Error reading joystick, waiting for 3 seconds.\n", jsEvent_count);
                  sleep(3); //wait for 3 seconds
                  jsEvent_count++;
                  fd = open(JoystickDevice, O_RDONLY);
               }
               else
               {
                  break;
               }
            }
         }
         if (jsEvent_count>=2) exit(EXIT_FAILURE);
         if (bytes == sizeof(struct js_event))
         {
               switch (jsEvent.type)
               {
               case JS_EVENT_BUTTON:
                  FFDirewolfMsgs.button[jsEvent.number] = jsEvent.value;
                  // printf("Button %d %d\n", jsEvent.number, jsEvent.value);
                  break;
               case JS_EVENT_AXIS:
                  FFDirewolfMsgs.axis[jsEvent.number] = jsEvent.value;
                  // printf("Axis %d value %d\n", jsEvent.number, jsEvent.value);
               default:
                     //printf("Unkown event type %d\n", js0Event.type);
                  break;
               }
         }
         FFDirewolfMsgs.control_mode = 0;
         FFDmsgsPuber->publish(FFDirewolfMsgs);
   }

   ffdirewolfbutton_msgs::msg::AxisButtonMsgs get_FFDirewolfMsgs()
   {
      return FFDirewolfMsgs;
   }

   int get_fd()
   {
      return fd;
   }

   int get_rc()
   {
      return rc;
   }

   char get_number_of_axes()
   {
      return number_of_axes;
   }

   char get_number_of_btns()
   {
      return number_of_btns;
   }

   char* get_js_name_str()
   {
      return js_name_str;
   }
};

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   auto Node = std::make_shared<FFDirewolfPubNode>("FFDirewolfPubNode", "FFDirewolfMsgsPub", "/dev/input/js0", 10);
   rclcpp::spin(Node);
   close(Node->get_fd());
   exit(EXIT_FAILURE);
   rclcpp::shutdown();
   return 0;
}
