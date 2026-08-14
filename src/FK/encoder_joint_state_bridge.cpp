/***********************************************************************************
Project: Lab Powder Manipulation
File: encoder_joint_state_bridge.cpp
Author: Liang Yan
Purpose: read encoder readings for each joint and pass that to RViz model to compute and visualze
         effector trajectory
Description: this code has two event pipelines: a 50Hz serial_timer drives reception of encoder readings,
             and a subcription to joint_states_gui messages that drives guiJointCallback.
             
             Upon node initialization, the code attempts to open serial port /dev/ttyACM0. 

                Every time the serial_timer fires, the serial printouts from Arduino serial is parsed, and using
                "Angle in rad", the code locates and extracts the numerical angle reading. This is then stored in
                private variable latest_encoder_joint_angle_rad_.
                
                Readings from all joints form a sensor_msgs::msg::JointState. Every time such a new msg arrives, 
                guiJointCallbacks is called, and it publishes the latest joint angles to /joint_states, which is 
                then visualized as manipulator movements in RViz, enabling end effector point trajectory tracking.
**********************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <fstream>
#include <string> 
#include <vector>
#include <functional>
#include <memory>
#include <chrono>

#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

class EncoderJointStateBridge : public rclcpp::Node
{
public:
    //constructor for the class EncoderJointStateBridge, which is a subclass of rclcpp::Node
    EncoderJointStateBridge()
    : Node("encoder_joint_state_bridge")
    {
        //subscriber listens to /joint_states_gui
        //whenever a type JointState msg arrives, call guiJointStateCallback(this msg!)
        //prior: joint_state_publisher_gui -> /joint_state_gui -> gui_joint_state_sub_ -> guiJointStateCallback() 
        gui_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states_gui",
            10,
            std::bind(&EncoderJointStateBridge::guiJointStateCallback, this, std::placeholders::_1)
            //When ROS receives a message, pass that incoming message as the first argument to my callback.
        );

        final_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        //Create a publisher that publishes JointState messages to /joint_states.
        //Store that publisher in final_joint_state_pub_.
            "/joint_states",
            10
        );

        RCLCPP_INFO(this->get_logger(), "encoder_joint_state_bridge started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /joint_states_gui");
        RCLCPP_INFO(this->get_logger(), "Publishing to /joint_states");

        //-----------------------------------------------------------------------------------------------------
        //initialize joint names, encoder_sign to 1.0, encoder_offset to 0.0
        this->joint_names_from_encoder_ = this->declare_parameter<std::vector<std::string>>("physical_joint_names",
            std::vector<std::string>{"base_joint","low_joint_A","low_joint_B","low_joint_C","high_joint_A","high_joint_B","high_joint_C",
            "wrist_joint_1","wrist_joint_2","wrist_joint_3"});

        this->encoder_sign_ = this->declare_parameter<double>("encoder_sign",1.0);
        this->encoder_offset_rad_= this->declare_parameter<double>("encoder_offset_rad",0.0);

        //-----------------------------------------------------------------------------------------------------
        //specify the COM port used by the Arduino R3 MCU
        this->serial_port_ = this->declare_parameter<std::string>("serial_port","/dev/ttyACM0");

        //setting serial communication baud rate to match with that of the Arduino Uno R3
        this->baud_rate_ = this->declare_parameter<int>("baud_rate",115200);

        //setting the angle_extraction_identifier
        this->angle_extraction_identifiers_ =  this->declare_parameter<std::vector<std::string>>
        ("angle_extraction_identifiers", std::vector<std::string>{"Angle 1 rad:","Angle 2 rad:","Angle 3 rad:","Angle 4 rad:","Angle 5 rad:","Angle 6 rad:"});

        //setting the maximum and minimum thresholds of valid angle readings to += 180 deg / +- pi rad
        this->max_angle_rad_ = this->declare_parameter<double>("max_angle_rad", 3.14159265359);
        this->min_angle_rad_ = this->declare_parameter<double>("min_angle_rad", -3.14159265359);
       
        //setting the serial sampling period to 0.0025s, meaning a sampling freq of 400Hz
        // prev: this->serial_sampling_period_ = this->declare_parameter<double> ("serial_sampling_period",0.005);
         this->serial_sampling_period_ = this->declare_parameter<double> ("serial_sampling_period",0.0025);

        this->serial_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(this->serial_sampling_period_),
            //EncoderJointStateBridge: the class name, this specifies that receiveEncoderAngleFromArduinoSerial
            //belongs to the class EncoderJointStateBridge

            //this: run receiveEncoderAngleFromArduinoSerial() on this specific object every time the timer fires
            std::bind(&EncoderJointStateBridge::receiveEncoderAngleFromArduinoSerial,this)
        );

        //-----------------------------------------------------------------------------------------------------
        //recording start time, used so saved timestamps are relative to node/simulation start
        //this->recording_start_time_ = this->now();

        
        //create a ROS2 publisher. The message type is geometry_msgs::msg::PoseStamped. The topic name is /FK_pose
        //queue size is 10
        this->joint_angles_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_angles",10);


        //opening the joint_angles csv file
        this->joint_angles_file_.open(joint_angles_file_path_);

        if (!this->joint_angles_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joint angles CSV file: %s", joint_angles_file_path_.c_str());
        } 
        else 
        {
            this->joint_angles_file_ << "timestamp,angle1_rad,angle2_rad,angle3_rad,angle4_rad,angle5_rad,angle6_rad\n";

            RCLCPP_INFO(this->get_logger(), "Recording joint angles to %s", joint_angles_file_path_.c_str());
        }

        //-----------------------------------------------------------------------------------------------------
        //runtime execution, not private variable definitions anymore!
        //runs the openSerialPort upon creation of this object
        this->openSerialPort();
    }

private:
    //-----------------------------------------------------------------------------------------------------

    //a ROS 2 subscription object whose message type is sensor_msgs::msg::JointState, stored using a sharedPtr
    //subscribes to joint_state_gui
    //with a remapping, the joint_state_publisher_gui publishes to joint_state_gui first
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gui_joint_state_sub_;

    //a ROS 2 publisher object whose message type is sensor_msgs::msg::JointState, stored using a sharedPtr
    //publishes the final joint_state to /joint_states
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_joint_state_pub_;

    //private variable for the latest raw encoder angles, initialized at 0.0
    std::vector<double> latest_encoder_joint_angles_rad_ = {0.0,0.0,0.0,0.0,0.0,0.0};

    //at startup, there might not be valid encoder angle
    bool if_encoder_angles_valid_ = false;

    //correct angle = encoder_sign * latest_encoder_joint_angle_rad + encoder_offset_rad
    //used for joint angle calibration
    double encoder_offset_rad_;
    double encoder_sign_;

    //name of the joint whose angle is to be replaced by physical encoder reading
    std::vector<std::string> joint_names_from_encoder_;

    //-----------------------------------------------------------------------------------------------------

    //serial port for the arduino COM3/COM4 in Ubuntu
    std::string serial_port_;
    
    //baud rate for serial communication
    int baud_rate_;
    
    //a buffer to store incomplete angle feedback text until a full line is available
    std::string serial_buffer_;

    // if -1, serial port not opened
    // otherwise, Ubuntu returns a non-negative integer
    int serial_fd_ = -1;

    //an identifier with which the correct angle reading in rad can be extracted from
    //the output text of the Arduino serial prints, e.g.
    /*
    "Raw angle: 2247 | Angle deg: -85.0 | Angle rad: -1.483700"
    Here, the extraction identifier should be "Angle rad:"
    */
    std::vector<std::string> angle_extraction_identifiers_;
    
    //max and min limits of valid angles
    double max_angle_rad_;
    double min_angle_rad_; 

    //sampling period for extracting angle reading with serial communication
    double serial_sampling_period_;

    rclcpp::TimerBase::SharedPtr serial_timer_;

    //private variable to store the latest time when encoder readings are received
    //initialized to 0,0, RCL_ROS_TIME
    //This timestamp is assigned to both /joint_angles and /joint_states.
    //robot_state_publisher propagates the /joint_states timestamp into TF,
    //so record_tf_path ultimately records the same acquisition time.
    
    rclcpp::Time latest_encoder_receive_time_{0, 0, RCL_ROS_TIME};
     
    std::ofstream joint_angles_file_;
    std::string joint_angles_file_path_ = "joint_angles.csv";

    //private publisher variable that publishes the current joint angles to /joint_angles topic for sensor synchronization
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angles_publisher_;

  
    //helper functon to read angle using the angle value extraction layer from an identified line in serial_buffer_
    //line is passed as a constant reference as it cannot be changed, while angle_rad will be updated 
    //to the actual parsed angle value
    bool parseAngleRadFromLine(const std::string& line, std::vector<double>& angles_rad)
    {
        //find the position/index in the line where the the angle_extraction_identifier_ that correspond to each read angle starts
        for (size_t i = 0; i<this->angle_extraction_identifiers_.size();i++)
        {
            size_t pos_extraction_identifier = line.find(this->angle_extraction_identifiers_[i]);

            if (pos_extraction_identifier == std::string::npos)
            {
                //if this position could not be found, exit
                return false;
            }
            
            //the position/index where the angle value starts is that where the angle_extraction_identifier_ + length of itself
            size_t pos_angle_value_start = pos_extraction_identifier + this->angle_extraction_identifiers_[i].length();
            
            //the angle reading is therefore the previous position to the end of the line
            std::string angle_reading = line.substr(pos_angle_value_start);

            //convert this angle reading from string to double, catch exceptions accordingly
            try
            {
                angles_rad[i] = std::stod(angle_reading);

            }catch(const std::exception&) {
        
                return false;
            }
        }
 
        
        //if there is no exception, return true
        return true;

    }
    

    //helper function for opening an Arduino serial port
    //it directly modifies the private member variable serial_fd_ if the serial port is successfully opened
    //this valid serial_fd_ is then passed to the following function receiveEncoderAngleFromArduinoSerial()
    void openSerialPort()
    {
        this->serial_fd_ = open(this->serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        if (this->serial_fd_ <0)
        {
           RCLCPP_ERROR(this->get_logger(),"Failed to open serial port %s, error msg is %s", 
           this->serial_port_.c_str(),std::strerror(errno));
        }

        else
        {
            RCLCPP_INFO(this->get_logger(),"Opened serial port %s", this->serial_port_.c_str());
        }
    }

    //everytime the serial timer fires, attempt to read a message from Arduino serial
    void receiveEncoderAngleFromArduinoSerial()
    {   
        //if the serial port failed to open, exit this function immediately
        if (this->serial_fd_<0)
        {
            return;
        }

        else
        {
            char read_buffer[256]; //why not string?

            //size_t read(int fd, void buf[count], size_t count);
            //read() attempts to read up to count bytes from file descriptor fd into the buffer starting at buf
            //why minus 1? -> later, when the read_buffer is prefilled with the terminating 0, there are only 255 
            //available slots, so we should read up to sizeof(read_buffer)-1 bytes
            ssize_t bytes_read = read(this->serial_fd_, read_buffer,sizeof(read_buffer)-1); 
 
            //if the bytes read is less than or equal to 0, it means the previous read() function failed to read 
            //any valid data
            if (bytes_read <=0)
            {
                return;

            }

            else
            {
                this->latest_encoder_receive_time_ = this->now();
                int64_t timestamp_ns = this->latest_encoder_receive_time_.nanoseconds();

                /*
                read() reads raw bytes from Arduino
                read_buffer temporarily stores these raw bytes
                serial_buffer_ accumulates these text segments until a line appears //how to detect if a complete line appears?
                */

                //intentionally add a C-string terminator of 0
                //without this, there is no way to tell where the "string" ends
                read_buffer[bytes_read] = '\0';
                //accumulate this most recently read segment to the private member serial_buffer_ until a full line appears
                this->serial_buffer_+= std::string(read_buffer);

                //in the serial_buffer_, find the newline character
                size_t newline_pos = this->serial_buffer_.find('\n');
                
                //if the newline is not found, a complete line has not been printed, skip this timer cycle
                if (newline_pos == std::string::npos)
                {
                    return;
                }

                else
                {
                    //extract the full line
                    std::string line = this->serial_buffer_.substr(0,newline_pos);

                    //erase the full line from serial_buffer_ up to the newline character
                    this->serial_buffer_.erase(0,newline_pos+1); 

                    std::vector<double> parsed_angles_rad(6,0.0);
                    if (!this->parseAngleRadFromLine(line,parsed_angles_rad))
                    {
                        return;
                    }
                    
                    for (size_t i = 0; i<this->angle_extraction_identifiers_.size();i++)
                    {

                        //check if the parse angle rad is in range
                        if (parsed_angles_rad[i] < this->min_angle_rad_ || parsed_angles_rad[i] > this->max_angle_rad_)
                        {
                            RCLCPP_WARN(this->get_logger(), "Recently read angle value %f out of range, rejected",parsed_angles_rad[i]);
                            return;
                        }

                        //if the parsed angle rad is indeed in range, update the private member variables latest_encoder_joint_angle_rad_
                        //and if_encoder_angle_valid_
                        else
                        {   //initialized to 0.0 in private:
                            this->latest_encoder_joint_angles_rad_[i] = parsed_angles_rad[i];
                            //initialized to false in private:
                        }
                    }
                    this->if_encoder_angles_valid_ = true;

                    //publish the latest encoder joint angles to /joint_angles topic for sensor synchronization
                    sensor_msgs::msg::JointState encoder_msg;

                    encoder_msg.header.stamp = this->latest_encoder_receive_time_;

                    encoder_msg.name = {
                        "joint_1",
                        "joint_2",
                        "joint_3",
                        "joint_4",
                        "joint_5",
                        "joint_6"
                    };

                    encoder_msg.position = latest_encoder_joint_angles_rad_;

                    this->joint_angles_publisher_->publish(encoder_msg);

                    //record the latest encoder joint angles to the CSV file
                    writeAnglestoCSV(this->latest_encoder_joint_angles_rad_,timestamp_ns);
   
                }    

            }
      
        }
    }

    
    //assume for now that the latest encoder reading is already stored in the private variable
    //latest_encoder_joint_angle_rad
    //joint_state_msg passed by reference so that 
    void replaceWithPhysicalJoints(sensor_msgs::msg::JointState& joint_state_msg)
    {
        //if the encoder angle is not yet valid, skip the replacement for now and wait for Arduino to get ready
        if (!this->if_encoder_angles_valid_)
        {
            return;
        }

        //iterate through all joint names in the input msg from /joint_states_gui
        for (size_t i=0;i<joint_state_msg.name.size();++i)
        {
            //find the joint name that matches our target joint angle to be replaced by physical encoder reading
            for (size_t j=0;j<joint_names_from_encoder_.size();++j)
            {
                if (joint_state_msg.name[i] == this->joint_names_from_encoder_[j])
                {
                    double corrected_angle = 0.0;

                    if (this->joint_names_from_encoder_[j] == "base_joint")
                    {
                    
                        //compute the corrected angle after joint calibration
                        corrected_angle = this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[0] + this->encoder_offset_rad_;

                        /*
                        sensor_msgs/JointState.msg:

                        Header header

                        string[] name
                        float64[] position
                        float64[] velocity
                        float64[] effort
                        */

                        //replace the angle in the corresponding position
                        joint_state_msg.position[i] = corrected_angle;
                    }
                    
                    else if (this->joint_names_from_encoder_[j] == "low_joint_A" || this->joint_names_from_encoder_[j] == "low_joint_B" )
                    {
                        
                        //compute the corrected angle after joint calibration
                        corrected_angle = this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[1] + this->encoder_offset_rad_;

                        joint_state_msg.position[i] = corrected_angle;

                    }
                                        
                    else if (this->joint_names_from_encoder_[j] == "low_joint_C" )
                    {
                        
                        //compute the corrected angle after joint calibration
                        corrected_angle = - this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[1] + this->encoder_offset_rad_;

                        joint_state_msg.position[i] = corrected_angle;

                    }

                    else if (this->joint_names_from_encoder_[j] == "high_joint_A" || this->joint_names_from_encoder_[j] == "high_joint_B" 
                    ||this->joint_names_from_encoder_[j] == "high_joint_C")  
                    {
                        
                        //compute the corrected angle after joint calibration
                        corrected_angle = - this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[2] + this->encoder_offset_rad_;

                        joint_state_msg.position[i] = corrected_angle;

                    }

                    else if (this->joint_names_from_encoder_[j] == "wrist_joint_1")                    
                    {
                        
                        //compute the corrected angle after joint calibration
                        corrected_angle = - this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[3] + this->encoder_offset_rad_;

                        joint_state_msg.position[i] = corrected_angle;

                    }

                    else if (this->joint_names_from_encoder_[j] == "wrist_joint_2")                    
                    {
                        
                        //compute the corrected angle after joint calibration
                        corrected_angle = - this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[4] + this->encoder_offset_rad_;

                        joint_state_msg.position[i] = corrected_angle;

                    }

                    else
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = - this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[5] + this->encoder_offset_rad_;

                        joint_state_msg.position[i] = corrected_angle;
                    }
                    

                }
                
            } 
        }
        


        return;

    }


    void guiJointStateCallback(const std::shared_ptr<const sensor_msgs::msg::JointState> msg)
    {
        //ROS sends GUI message msg, the two consts mean the pointer cannot be reassigned
        //the msg cannot be modifed either 

        //copy the msg into the output_msg
        //this keeps the original msg untouched -> safer
        sensor_msgs::msg::JointState output_msg = *msg;
        
        //set header stamp of this output_msg
        //this is using the ROS clock, which starts at 1970-01-01 00:00:00 UTC
        if (!this->if_encoder_angles_valid_)
        {
            RCLCPP_WARN(this->get_logger(),"Encoder angles not yet valid, using GUI joint angles for now");
            output_msg.header.stamp = this->now();
        }
        else
        {
            output_msg.header.stamp = this->latest_encoder_receive_time_;
        }
       
        //pass this output_msg into the replaceWithPhysicalJoints() function
        //where physical joint angles in this output_msg are replaced by 
        //encoders readings from [functional block 2!]
        replaceWithPhysicalJoints(output_msg);

        //publish this output_msg to final_joint_state_pub_ 
        this->final_joint_state_pub_->publish(output_msg);
    }

    //-----------------------------------------------------------------------------------------------------
    void writeAnglestoCSV(const std::vector<double>&angles_rad,const int64_t callback_time_ns)
    {
        if(!this->joint_angles_file_.is_open())
        {
            return;
        }
        
        
        //write the timestamp and angles to the CSV file
        this->joint_angles_file_ << callback_time_ns;

        for (const auto& angle : angles_rad)
        {
            //write a comma before each angle value
            this->joint_angles_file_ << "," << angle;
        }

        this->joint_angles_file_ << "\n";
        this->joint_angles_file_.flush();
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EncoderJointStateBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
