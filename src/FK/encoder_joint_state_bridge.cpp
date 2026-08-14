/***********************************************************************************
Project: Lab Powder Manipulation
File: encoder_joint_state_bridge.cpp
Author: Liang Yan
Contributor: Lang Yun

Purpose: read encoder readings for each joint and pass that to RViz model to compute and visualize
         effector trajectory
Description:
    Encoder-only, event-driven serial reception.

    Linux poll() waits for serial bytes from the MCU. Every complete valid six-angle
    packet publishes exactly one /joint_angles and one /joint_states message.
    There is no /joint_states_gui path and no fixed-frequency serial wall timer.

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
#include <poll.h>
#include <thread>
#include <atomic>

class EncoderJointStateBridge : public rclcpp::Node
{
public:
    //constructor for the class EncoderJointStateBridge, which is a subclass of rclcpp::Node
    EncoderJointStateBridge()
    : Node("encoder_joint_state_bridge")
    {
        //publisher for the six direct encoder readings as a /joint_states message used by RViz
        final_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            //Create a publisher that publishes JointState messages to /joint_states.
            //Store that publisher in final_joint_state_pub_.
            "/joint_states",
            1
        );

        //publisher for the six direct encoder readings used by sensor synchronization
        this->joint_angles_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_angles",
            1
        );

        RCLCPP_INFO(this->get_logger(), "encoder_joint_state_bridge started");
        RCLCPP_INFO(this->get_logger(), "encoder mode: no /joint_states_gui subscription");
        RCLCPP_INFO(this->get_logger(), "Publishing to /joint_angles");
        RCLCPP_INFO(this->get_logger(), "Publishing to /joint_states");

        //-----------------------------------------------------------------------------------------------------
        this->joint_names_from_encoder_ = this->declare_parameter<std::vector<std::string>>(
            "physical_joint_names",
            std::vector<std::string>{
                "base_joint",
                "low_joint_A",
                "low_joint_B",
                "low_joint_C",
                "high_joint_A",
                "high_joint_B",
                "high_joint_C",
                "wrist_joint_1",
                "wrist_joint_2",
                "wrist_joint_3"
            }
        );

        this->encoder_sign_ = this->declare_parameter<double>("encoder_sign",1.0);
        this->encoder_offset_rad_= this->declare_parameter<double>("encoder_offset_rad",0.0);

        //-----------------------------------------------------------------------------------------------------
        //specify the COM port used by the MCU
        this->serial_port_ = this->declare_parameter<std::string>("serial_port","/dev/ttyACM0");

        //setting serial communication baud rate to match with that of the MCU
        this->baud_rate_ = this->declare_parameter<int>("baud_rate",460800);

        //setting the angle_extraction_identifier
        this->angle_extraction_identifiers_ = this->declare_parameter<std::vector<std::string>>(
            "angle_extraction_identifiers",
            std::vector<std::string>{
                "1 ",
                "| 2 ",
                "| 3 ",
                "| 4 ",
                "| 5 ",
                "| 6 "
            }
        );

        //setting the maximum and minimum thresholds of valid angle readings to += 180 deg / +- pi rad
        this->max_angle_rad_ = this->declare_parameter<double>("max_angle_rad", 3.14159265359);
        this->min_angle_rad_ = this->declare_parameter<double>("min_angle_rad", -3.14159265359);

        /**********************************************************************************************
        Event Driven Serial Encoder Reading:

        This is not driven by a ROS timer.

        The serial_read_thread_ blocks in poll() until Linux reports that serial bytes are
        available on serial_fd_. When bytes arrive, receiveEncoderAngleFromArduinoSerial()
        is called immediately.

        serial_buffer_ is still retained because read() returns arbitrary byte chunks and
        one read() is not guaranteed to contain exactly one complete newline-terminated packet.
        **********************************************************************************************/

        //-----------------------------------------------------------------------------------------------------
        //runtime execution, not private variable definitions anymore!
        //runs the openSerialPort upon creation of this object
        this->openSerialPort();

        //start the event-driven serial read thread only if the serial port opened successfully
        //it starts the serialReadLoop() function in a separate thread
        if (this->serial_fd_ >= 0)
        {
            //runs the serialReadLoop() function in a separate thread every time serial bytes are available
            //until the node is destroyed
            this->serial_read_thread_ = std::thread(
                &EncoderJointStateBridge::serialReadLoop,
                this
            );
        }
    }

    //destructor: stop the serial event thread cleanly before the node is destroyed
    ~EncoderJointStateBridge() override
    {
        //stop the serial event thread cleanly before the node is destroyed
        this->serial_thread_running_.store(false);

        //wait for the serial event thread to finish before closing the serial port
        if (this->serial_read_thread_.joinable())
        {
            this->serial_read_thread_.join();
        }

        if (this->serial_fd_ >= 0)
        {
            close(this->serial_fd_);
            this->serial_fd_ = -1;
        }
    }

private:
    //-----------------------------------------------------------------------------------------------------
    //a ROS2 publisher object whose message type is sensor_msgs::msg::JointState, stored using a sharedPtr
    //publishes the final joint_state to /joint_states
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_joint_state_pub_;

    //private ROS2 publisher variable that publishes the current joint angles to /joint_angles topic for sensor synchronization
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angles_pub_;

    //private variable for the latest raw encoder angles, initialized at 0.0
    std::vector<double> latest_encoder_joint_angles_rad_ = {0.0,0.0,0.0,0.0,0.0,0.0};

    //boolean flags to indicate whether the latest encoder angles are valid and whether the first valid packet has been logged
    bool if_encoder_angles_valid_ = false;
    bool if_first_valid_packet_logged_ = false;

    //correct angle = encoder_sign * latest_encoder_joint_angle_rad + encoder_offset_rad
    //used for joint angle calibration
    double encoder_offset_rad_;
    double encoder_sign_;

    //name of the joint whose angle is to be replaced by physical encoder reading
    std::vector<std::string> joint_names_from_encoder_;

    //-----------------------------------------------------------------------------------------------------
    //serial port for the MCU in Ubuntu
    std::string serial_port_;

    //baud rate for serial communication
    int baud_rate_;

    //a buffer to store incomplete angle feedback text until a full line is available
    std::string serial_buffer_;

    // if -1, serial port not opened
    // otherwise, Ubuntu returns a non-negative integer
    int serial_fd_ = -1;

    //an identifier with which the correct angle reading in rad can be extracted from
    //the output text of the MCU serial prints
    std::vector<std::string> angle_extraction_identifiers_;

    //max and min limits of valid angles
    double max_angle_rad_;
    double min_angle_rad_;

    //event-driven serial reader thread
    std::thread serial_read_thread_;

    //the boolean trigger is used to stop the serial event thread cleanly before the node is destroyed
    //this is an atomic variable to ensure thread safety when accessed from multiple threads, ensuring 
    //a consistent read/write order throughout
    std::atomic<bool> serial_thread_running_{true};

    //private variable to store the latest time when a valid encoder packet is accepted
    //This timestamp is assigned to both /joint_angles and /joint_states.
    //robot_state_publisher propagates the /joint_states timestamp into TF,
    //so record_tf_path ultimately records the same timestamp
    rclcpp::Time latest_encoder_receive_time_{0, 0, RCL_ROS_TIME};

    // std::ofstream joint_angles_file_;
    // std::string joint_angles_file_path_ = "joint_angles.csv";

    //-----------------------------------------------------------------------------------------------------
    //helper functon to read angle using the angle value extraction layer from an identified line in serial_buffer_
    //line is passed as a constant reference as it cannot be changed, while angle_rad will be updated
    //to the actual parsed angle value
    bool parseAngleRadFromLine(const std::string& line, std::vector<double>& angles_rad)
    {
        //find the position/index in the line where the the angle_extraction_identifier_ that correspond to each read angle starts
        for (size_t i = 0; i < this->angle_extraction_identifiers_.size(); i++)
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
            //this works because the stod function automatically stops parsing at the first non-numeric character
            //so it will not parse the trailing text after the angle value, e.g. " | Angle deg: -85.0 | Angle rad: -1.483700"
            std::string angle_reading = line.substr(pos_angle_value_start);

            //convert this angle reading from string to double, catch exceptions accordingly
            try
            {
                angles_rad[i] = std::stod(angle_reading);
            }
            catch(const std::exception&)
            {
                return false;
            }
        }

        //if there is no exception, return true
        return true;
    }

    //-----------------------------------------------------------------------------------------------------
    //helper function for opening the MCU serial port
    //it directly modifies the private member variable serial_fd_ if the serial port is successfully opened
    //this valid serial_fd_ is then passed to the following function receiveEncoderAngleFromArduinoSerial()
    void openSerialPort()
    {
        this->serial_fd_ = open(this->serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (this->serial_fd_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open serial port %s, error msg is %s",
                this->serial_port_.c_str(),
                std::strerror(errno)
            );
            return;
        }

        //configure the serial port settings using termios
        termios tty{};
        if (tcgetattr(this->serial_fd_, &tty) != 0)
        {
            RCLCPP_ERROR
            (
                this->get_logger(),
                "tcgetattr failed for %s: %s",
                this->serial_port_.c_str(),
                std::strerror(errno)
            );
            close(this->serial_fd_);
            this->serial_fd_ = -1;
            return;
        }

        speed_t serial_speed;

        //automatically set the serial speed based on the baud_rate_ parameter
        switch (this->baud_rate_)
        {
            case 9600: serial_speed = B9600; break;
            case 19200: serial_speed = B19200; break;
            case 38400: serial_speed = B38400; break;
            case 57600: serial_speed = B57600; break;
            case 115200: serial_speed = B115200; break;
            case 230400: serial_speed = B230400; break;
            case 460800: serial_speed = B460800; break;
            default:
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Unsupported baud_rate parameter: %d",
                    this->baud_rate_
                );
                close(this->serial_fd_);
                this->serial_fd_ = -1;
                return;
        }

        cfmakeraw(&tty);
        cfsetispeed(&tty, serial_speed);
        cfsetospeed(&tty, serial_speed);

        //8 data bits, no parity, one stop bit, no hardware flow control.
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CLOCAL | CREAD;

        //Nonblocking reads: return immediately if no serial bytes are available.
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(this->serial_fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "tcsetattr failed for %s: %s",
                this->serial_port_.c_str(),
                std::strerror(errno)
            );
            close(this->serial_fd_);
            this->serial_fd_ = -1;
            return;
        }

        tcflush(this->serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(
            this->get_logger(),
            "Opened serial port %s at %d baud",
            this->serial_port_.c_str(),
            this->baud_rate_
        );
    }

    //-----------------------------------------------------------------------------------------------------

    //event-driven serial wait loop
    //poll() sleeps efficiently until Linux reports that serial data is available.
    //the timeout does NOT set the encoder sampling or publication frequency.

    void serialReadLoop()
    {
        //creates one poll descriptor record 
        pollfd serial_poll_fd{};

        //set the file descriptor to the serial port file descriptor
        //tell it to monitor the serial port for input events, i.e. when serial bytes are available
        serial_poll_fd.fd = this->serial_fd_;

        //watch for input events on the serial port, i.e. when serial bytes are available
        //later, the revents field will be checked to see if the POLLIN event occurred, indicating that there is data to read
        serial_poll_fd.events = POLLIN;

        //Combined: watch this serial port, and wake me when there are bytes available to read
        
        //keeps looping as long as the serial thread is running and ROS is still OK
        while (this->serial_thread_running_.load() && rclcpp::ok())
        {
            //poll() waits for serial bytes to arrive on the MCU serial port
            //once the serial_fd_ turns true, i.e. input is ready, poll_result is positive
            //poll() returns 0 if the timeout expires, and -1 if an error occurs
            int poll_result = poll(
                &serial_poll_fd, 
                1,
                100
            );

            //timeout: if no bytes, poll_result is 0, continue this loop until the next poll() call
            if (poll_result == 0)
            {
                continue;
            }

            //failed: 
            if (poll_result < 0)
            {
                //if the code was interrupted by a signal, continue the loop and call poll() again
                if (errno == EINTR)
                {
                    continue;
                }

                //otherwise, log the error and break the loop 
                RCLCPP_ERROR(
                    this->get_logger(),
                    "poll() failed for %s: %s",
                    this->serial_port_.c_str(),
                    std::strerror(errno)
                );

                break;
            }

           
            //handle serial-device errors or disconnects
            if (serial_poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL))
            {
                //POLERR: an error occured on the file descriptor
                //POLLHUP: the device was disconnected
                //POLLNVAL: the file descriptor is not open

                RCLCPP_ERROR
                (
                    this->get_logger(),
                    "Serial device %s reported poll error/hangup",
                    this->serial_port_.c_str()
                );

                break;
            }

             //serial bytes are available now
            //revents: set of bits that indicate the events that actually occurred on the file descriptor
            //POLLIN: the specific flag that indicates that there is data to read on the file descriptor
            //POLLIN: a bit mask where the bit position corresponding to POLLIN is set to 1, indicating that there is data to read
            if (serial_poll_fd.revents & POLLIN)
            {
                //this condition is true when the serial port has data available to read, so call the function to read the data
                //unlike serial_fd_, this event is driven by successful sending of a complete six-angle packet from the MCU, so it is not guaranteed to be periodic
                //if serial bytes are available, call the function to read them from the MCU serial port
                this->receiveEncoderAngleFromArduinoSerial();
            }
        }
    }

    //-----------------------------------------------------------------------------------------------------
    //every time Linux reports serial bytes are available, attempt to read them from MCU serial
    void receiveEncoderAngleFromArduinoSerial()
    {
        //if the serial port failed to open, exit this function immediately
        //this buffer has to used because a single set of 6 joint angles might arrive in a full line, or it
        //might be split into multiple segments until the newline is received, so we need to accumulate the segments until a full line is available
        if (this->serial_fd_ < 0)
        {
            return;
        }

        char read_buffer[256];

        //read() returns whatever bytes are available, up to the size of read_buffer - 1, and stores them in read_buffer
        //the -1 is to leave space for the C-string terminator of 0, which is added later
        //read() returns the number of bytes read, or -1 if an error occurred 
        ssize_t bytes_read = read(this->serial_fd_, read_buffer, sizeof(read_buffer)-1);

        //if the bytes read is less than or equal to 0, no valid serial data is available
        if (bytes_read <= 0)
        {
            return;
        }

        /*
        read() reads raw bytes from MCU
        read_buffer temporarily stores these raw bytes
        serial_buffer_ accumulates these text segments until a line appears
        */

        //intentionally add a C-string terminator of 0
        //without this, there is no way to tell where the "string" ends
        //this is the end of the string just read
        read_buffer[bytes_read] = '\0';

        //accumulate this most recently read string (delimited by the C-string terminator \0) 
        //to the private member serial_buffer_ until a full line appears
        //because the previous line inserts a C-string terminator, the std::string constructor will stop at that point
        this->serial_buffer_ += std::string(read_buffer);

        //-----------------------------------------------------------------------------------------------------
        
        //in the serial buffer, keep looping and keep looking for a newline character
        while (true)
        {
            //in the serial_buffer_, find the newline character
            //find the position of the newline character in the accumulated serial buffer
            size_t newline_pos = this->serial_buffer_.find('\n');

            //if the newline is not found, a complete line has not been printed
            //if there is a complete line, there is at least one newline character
            if (newline_pos == std::string::npos)
            {
                break;
            }

            //extract the full line from the serial_buffer_ up to the newline character
            //the next line erases the full line from serial buffer up to the new line character
            //so that the next iteration of the loop can look for the next complete line starting from position 0
            std::string line = this->serial_buffer_.substr(0,newline_pos);

            //erase the full line from serial_buffer_ up to the newline character
            this->serial_buffer_.erase(0,newline_pos+1);

            //initialize a vector of 6 double values to store the parsed angles in rad
            std::vector<double> parsed_angles_rad(6,0.0);

            //parse the angles from the line, if it fails, continue to the next iteration of the loop
            if (!this->parseAngleRadFromLine(line,parsed_angles_rad))
            {
                continue;
            }

            //validate the complete packet before updating the stored encoder values
            bool packet_valid = true;
            
            //iterate through all angle extraction identifiers to check if the parsed angles are within the valid range
            for (size_t i = 0; i < this->angle_extraction_identifiers_.size(); i++)
            {
                //check if the parsed angle rad is in range
                if (parsed_angles_rad[i] < this->min_angle_rad_ || parsed_angles_rad[i] > this->max_angle_rad_)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Recently read angle value %f out of range, rejected",
                        parsed_angles_rad[i]
                    );
                    packet_valid = false;
                    break;
                }
            }

            if (!packet_valid)
            {
                //if the packet is invalid, skip the rest of the loop and continue to the next iteration
                continue;
            }

            /**********************************************************************************************
            This is the only path that drives both ROS outputs.

            complete valid encoder packet
                    -> update encoder values
                    -> timestamp packet
                    -> publish /joint_angles
                    -> publish /joint_states

            There is no GUI callback anywhere in this path.
            **********************************************************************************************/

            this->latest_encoder_joint_angles_rad_ = parsed_angles_rad;
            this->if_encoder_angles_valid_ = true;

            //timestamp the complete valid encoder packet when it is accepted
            this->latest_encoder_receive_time_ = this->now();
            int64_t timestamp_ns = this->latest_encoder_receive_time_.nanoseconds();

            if (!this->if_first_valid_packet_logged_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Received first valid six-angle encoder packet"
                );
                this->if_first_valid_packet_logged_ = true;
            }

            //-------------------------------------------------------------------------------------------------
            //publish the latest six direct encoder readings to /joint_angles topic for sensor synchronization
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

            encoder_msg.position = this->latest_encoder_joint_angles_rad_;

            this->joint_angles_pub_->publish(encoder_msg);

            //-------------------------------------------------------------------------------------------------
            //Drive RViz directly from the encoder reception pipeline itself.
            publishFinalJointState();

            //record the latest encoder joint angles to the CSV file
            //writeAnglestoCSV(this->latest_encoder_joint_angles_rad_,timestamp_ns);
        }
    }

    //-----------------------------------------------------------------------------------------------------
    //assume for now that the latest encoder reading is already stored in the private variable
    //latest_encoder_joint_angles_rad_
    //joint_state_msg passed by reference so that the positions can be replaced directly
    //this function takes the latest encoder readings and replaces the corresponding joint angles in the joint_state_msg
    //and enforces the mimics as defined in the URDF, so that the final joint_state_msg can be published to /joint_states

    void replaceWithPhysicalJoints(sensor_msgs::msg::JointState& joint_state_msg)
    {
        //if the encoder angle is not yet valid, skip the replacement for now and wait for MCU to get ready
        if (!this->if_encoder_angles_valid_)
        {
            return;
        }

        //iterate through all joint names in the output /joint_states message
        for (size_t i = 0; i < joint_state_msg.name.size(); ++i)
        {
            //find the joint name that matches our target joint angle to be replaced by physical encoder reading
            for (size_t j = 0; j < joint_names_from_encoder_.size(); ++j)
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

                    else if (this->joint_names_from_encoder_[j] == "low_joint_A" || this->joint_names_from_encoder_[j] == "low_joint_B")
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[1] + this->encoder_offset_rad_;
                        joint_state_msg.position[i] = corrected_angle;
                    }

                    else if (this->joint_names_from_encoder_[j] == "low_joint_C")
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = -this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[1] + this->encoder_offset_rad_;
                        joint_state_msg.position[i] = corrected_angle;
                    }

                    else if (this->joint_names_from_encoder_[j] == "high_joint_A" ||
                             this->joint_names_from_encoder_[j] == "high_joint_B" ||
                             this->joint_names_from_encoder_[j] == "high_joint_C")
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = -this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[2] + this->encoder_offset_rad_;
                        joint_state_msg.position[i] = corrected_angle;
                    }

                    else if (this->joint_names_from_encoder_[j] == "wrist_joint_1")
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = -this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[3] + this->encoder_offset_rad_;
                        joint_state_msg.position[i] = corrected_angle;
                    }

                    else if (this->joint_names_from_encoder_[j] == "wrist_joint_2")
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = -this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[4] + this->encoder_offset_rad_;
                        joint_state_msg.position[i] = corrected_angle;
                    }

                    else if (this->joint_names_from_encoder_[j] == "wrist_joint_3")
                    {
                        //compute the corrected angle after joint calibration
                        corrected_angle = -this->encoder_sign_ * this->latest_encoder_joint_angles_rad_[5] + this->encoder_offset_rad_;
                        joint_state_msg.position[i] = corrected_angle;
                    }
                }
            }
        }
    }

    //-----------------------------------------------------------------------------------------------------
    /**********************************************************************************************
    Build and publish the final /joint_states message entirely from encoder data.

    There is no GUI template anymore.

    Every valid encoder packet creates a new JointState:
        name     = full encoder-controlled URDF joint-name list
        position = initialized to 0.0, then filled by replaceWithPhysicalJoints()
        stamp    = same timestamp as /joint_angles
    **********************************************************************************************/
    void publishFinalJointState()
    {
        if (!this->if_encoder_angles_valid_)
        {
            return;
        }

        sensor_msgs::msg::JointState output_msg;

        //assign the complete encoder-controlled URDF joint-name list
        output_msg.name = this->joint_names_from_encoder_;

        //initialize all joint angles to 0.0 before replacing them with encoder-derived values
        output_msg.position.assign(output_msg.name.size(), 0.0);

        //use the exact same accepted encoder-packet timestamp as /joint_angles
        output_msg.header.stamp = this->latest_encoder_receive_time_;

        //replace all physical/coupled joints using the six encoder readings
        replaceWithPhysicalJoints(output_msg);

        this->final_joint_state_pub_->publish(output_msg);
    }

    //-----------------------------------------------------------------------------------------------------
    // void writeAnglestoCSV(const std::vector<double>& angles_rad, const int64_t callback_time_ns)
    // {
    //     if (!this->joint_angles_file_.is_open())
    //     {
    //         return;
    //     }

    //     //write the timestamp and angles to the CSV file
    //     this->joint_angles_file_ << callback_time_ns;

    //     for (const auto& angle : angles_rad)
    //     {
    //         //write a comma before each angle value
    //         this->joint_angles_file_ << "," << angle;
    //     }

    //     this->joint_angles_file_ << "\n";
    //     this->joint_angles_file_.flush();
    // }
};


//this is the main C++ thread
//it starts the ROS2 node, creates an instance of the EncoderJointStateBridge class,
//and spins the node to process callbacks
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<EncoderJointStateBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
