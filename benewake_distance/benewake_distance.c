#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int benewake_distance_main(int argc, char *argv[]);
int benewake_distance_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
orb_advert_t mavlink_log_pub = NULL;


int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;


    tcgetattr(fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [param]\n\n");
    exit(1);
}

int benewake_distance_main(int argc, char *argv[])
{

mavlink_log_critical(&mavlink_log_pub,"[inav] benewake_distance_main on init");
 mavlink_log_critical(mavlink_log_pub, "test>>>>>>> %f",9.98989);
			
    if (argc < 2) 
    {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("benewake_distance",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2000,
                         benewake_distance_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        thread_running = false;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

       return 0;
    }

    usage("unrecognized command");
    return 1;
}

int benewake_distance_thread_main(int argc, char *argv[])
{
    mavlink_log_critical(&mavlink_log_pub,"benewake_distance run ");
    if (argc < 2) {
        errx(1, "[YCM]need a serial port name as argument");
        usage("eg:");
    }
    char data = '0';
    //const char *uart_name = argv[1];
    char buffer[9] = "";
    float distance = 0;
    int strength = 0;
    //int sequence;
    long checksum = 0; 
    long check = 0;
    int uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read)
    {
         mavlink_log_critical(&mavlink_log_pub,"[YCM]uart init is failed\n");
         return -1;
    }
    if(false == set_uart_baudrate(uart_read,115200)){
        mavlink_log_critical(&mavlink_log_pub,"[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_critical(&mavlink_log_pub,"[YCM]uart init is successful\n");

    thread_running = true;
    struct distance_sensor_s _range;
    memset(&_range, 0 , sizeof(_range));

    //thread_should_exit = true;
   // struct benewake_distance_s b_distance;
   
   // memset(&b_distance, 0 , sizeof(b_distance));
   
    orb_advert_t _distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &_range);//公告这个主题

    
    while(thread_running)
   {
        warnx("warnxxxxxxxxxxxx!!!!");
    	read(uart_read,&data,1);
        if((data == 0x59))
        {
           buffer[0] = 0x59;
           data = '0';
           read(uart_read,&data,1);
           if((data == 0x59))
           {
               buffer[1] = 0x59;
               for(int k = 2;k < 9;k++)
	       {
		  data = '0';
		  read(uart_read,&data,1);
                  buffer[k] = data;

	       }

           }

        }
        
       distance = (uint16_t)buffer[2] + (uint16_t)buffer[3]*256;
       strength = (uint16_t)buffer[4] + (uint16_t)buffer[5]*256;
      // sequence = (uint16_t)buffer[6] + (uint16_t)buffer[7]*256;
       checksum = buffer[8];
       for(int k = 0; k<8; k++)
       {
           check = check + buffer[k];
       }
       if((checksum == (check&0xff)) && (strength > 3) && (checksum > 0))
       {
        
          warnx("benewake_distance = %f\n",(double)distance);    

        _range.timestamp = hrt_absolute_time();
	_range.type = 0;
	_range.max_distance = 25;
	_range.min_distance = 0.05;
	_range.current_distance = distance / 100;   /* 10 usec = 1 cm distance for LIDAR-Lite */
	_range.covariance = 0.0f;
	_range.orientation = 8;
	/* TODO: set proper ID */
	_range.id = 0;

	
	if (_distance_sensor_pub != NULL) {
		orb_publish(ORB_ID(distance_sensor),_distance_sensor_pub, &_range);
	}


       }
       memset(&buffer, 0 , sizeof(buffer));
       memset(&_range,0,sizeof(_range));
       check = 0;
       checksum = 0;
	
   }
    
   

   

     mavlink_log_critical(&mavlink_log_pub,"[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}
