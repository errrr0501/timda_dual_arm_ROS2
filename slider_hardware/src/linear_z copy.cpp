#include "linear_motion/slide.h"
#include <ctime>
void slide_callback(const manipulator_h_base_module_msgs::SlideCommand::ConstPtr& msg)
{
    goal_pos = (double)100000.0*(msg->pos+0.8);
    // goal_pos = -1*(double)100000.0*(msg->pos);
    if(goal_pos > 80000.0) goal_pos = 80000;
    if(goal_pos < 0.0) goal_pos = 0;
}

void read_feedback()
{
    int rc = modbus_read_registers(ct, ADDRESS_FDB, FDB_LENGTH, fdb_val);
    if (rc != FDB_LENGTH)
    {
        fprintf(stderr, "modbus read failed: %d %s\n", errno, modbus_strerror(errno));
        errno = 0;
    }

    curr_pos   = fdb_val[0]<<16 | fdb_val[1];
    curr_speed = fdb_val[4]<<16 | fdb_val[5];
}

void write_command()
{
    int rc = modbus_write_registers(ct, ADDRESS_CMD, CMD_LENGTH, cmd_arr);
    if (rc != CMD_LENGTH)
    {
        fprintf(stderr, "modbus write failed: %d %s\n", errno, modbus_strerror(errno));
        errno = 0;
    }
}

modbus_t* init_modbus_rtu(int id, std::string port, int baud_rate)
{
    modbus_t* ct = modbus_new_rtu(port.c_str(), baud_rate, 'E', 8, id);
    modbus_set_slave(ct, id);
    if (modbus_connect(ct) == -1)
    {
        fprintf(stderr, "Connection failed: %s\n",
            modbus_strerror(errno));
        std::cout << "Error connect" << std::endl;
        modbus_free(ct);
        return nullptr;
    }
    std::cout << "Init success" << std::endl;
    // modbus_set_debug(ct, true);

    return ct;
}

void send_cmd()
// for communication with driver
{
    if (goal_pos != curr_pos)
    {
        int diff_pos = goal_pos - curr_pos;
        int speed_tmp = float(diff_pos) / (smp_time*20);
        // speed_tmp = (speed_tmp < MAX_SPEED) ? speed_tmp : MAX_SPEED;
        if(speed_tmp > MAX_SPEED) speed_tmp = MAX_SPEED;
        if(speed_tmp < MIN_SPEED) speed_tmp = MIN_SPEED;
        // cmd_speed = (speed_tmp < cmd_speed) ? (
        //     ((cmd_speed - speed_tmp) < smp_deleration) ? speed_tmp : cmd_speed - smp_deleration
        //     ) : speed_tmp;
        // cmd_speed = (speed_tmp < cmd_speed) ? cmd_speed : speed_tmp;
        // cmd_speed = speed_tmp;
        // cmd_arr[6] = cmd_speed>>16;
        // if(diff_pos > 3) goal_pos = goal_pos + (diff_pos - 3) * 2;
        // if(diff_pos < -3) goal_pos = goal_pos + (diff_pos + 3) * 2;
        // if(goal_pos > 80000.0) goal_pos = 80000;
        // if(goal_pos < 0.0) goal_pos = 0;
        if(abs(diff_pos) < float(abs(curr_pos))/MAX_SPEED * 100)
        {
            cmd_arr[3] = 1;
            cmd_speed = abs(speed_tmp);
        }
        else
        {
            cmd_arr[3] = 7;
            cmd_speed = speed_tmp;
        }
        int diff_speed = abs(cmd_speed - curr_speed);
        int acc = 3 * diff_speed / (smp_time) + 1;
        // cmd_speed = abs(speed_tmp);
        // cmd_arr[3] = 1;
        cmd_arr[4] = goal_pos>>16;
        cmd_arr[5] = goal_pos;
        cmd_arr[6] = cmd_speed>>16;
        cmd_arr[7] = cmd_speed;
        // cmd_arr[9]  = exp((cmd_speed / MAX_SPEED)*4 - 2) * 5410;
        // cmd_arr[9] = (3*abs(cmd_speed) > ACCELERATION) ? ACCELERATION : 3*abs(cmd_speed);
        // cmd_arr[11] = (3*abs(cmd_speed) > DECELERATION) ? DECELERATION : 3*abs(cmd_speed);
        cmd_arr[9] = (acc > ACCELERATION) ? ACCELERATION : acc;
        cmd_arr[11] = (acc > DECELERATION) ? DECELERATION : acc;
        write_command();
    }
    else
        cmd_speed = 0;
}

int main(int argc, char **argv)
{
    //========================= Initialize ROS =============================
    ros::init(argc, argv, "linear_z");

    int baud_rate;
    std::string side_str;

    ros::NodeHandle nh_param("~");
    nh_param.param<int>("baud", baud_rate, 9600);
    nh_param.param<std::string>("side", side_str, "");

    //========================= Initialize Modbus_RTU ============================= 
    std::cout << "Preparing connection slide" << std::endl;

    int id = side_str == "right" ? 3 : 2;
    ct = init_modbus_rtu(id, "/dev/wrs/slide_" + side_str, baud_rate);
    if (!ct)
    {
        std::cout << "Connect " + side_str + " fail!!!" << std::endl;
        return -1;
    }
    std::cout << side_str + " slide connect ok" << std::endl;

    read_feedback();
    goal_pos = curr_pos;
    std::cout << side_str + " slide connect ok " << goal_pos<<std::endl;

    smp_deleration = DECELERATION * smp_time;

    // generate thread to communicate with driver
    // com_driver_thread_ = new boost::thread(boost::bind( &send_cmd ));

    // ============================= Subscribe message =============================
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("slide_command_msg", 1, slide_callback);
    ros::Publisher  pub = n.advertise<linear_motion::Slide_Feedback>("slide_feedback_msg", 1);
    ros::Rate loop_rate(1/smp_time);

    // ============================= ROS Loop =============================
    // main thread to communicate with other node
    while (ros::ok())
    {
        send_cmd();
        read_feedback();
        msg_fdb.curr_pos = curr_pos;
        msg_fdb.is_busy  = abs(curr_speed) > 10;
        pub.publish(msg_fdb);

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete com_driver_thread_;
    return 0;
}
