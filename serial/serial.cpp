#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

#include <string>

#include "serial.h"

SerialPort::SerialPort(std::string port_name)
{
    this->_name = port_name;
    this->_fd = -1;

    this->open();
}
SerialPort::SerialPort(const char *port_name)
{
    this->_name = (std::string)port_name;
    this->_fd = -1;

    this->open();
}

SerialPort::~SerialPort()
{
    if (this->_fd != -1)
    {
        close(this->_fd);
        this->_fd = -1;
    }
}

int
SerialPort::open(void)
{
    const char * port_name = this->_name.c_str();
    int fd;

    if (this->_fd != -1) return 0;
    
    fd = ::open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        fprintf(stderr, "SerialPort %s: Unable to open\n", port_name);
        return -1;
    }
   
    fcntl(fd, F_SETFL, 0);
    this->_fd = fd;

    if (this->set_default_values() != 0)
    {
        fprintf(stderr, "SerialPort %s: Error setting default values\n", port_name);
        return -1;
    }

    return 0;
}

int
SerialPort::write(const char * data, size_t len)
{
    const char * port_name = this->_name.c_str();
    ssize_t n;

    n = ::write(this->_fd, data, len);
    if (n == -1)
    {
        fprintf(stderr, "SerialPort %s: Error writing to port\n", port_name);
        return -1;
    }

    if (n < (ssize_t)len)
    {
        fprintf(stderr, "SerialPort %s: Incomplete write\n", port_name);
        return -1;
    }

    return n;
}

int SerialPort::update_term_options(void)
{
    int res;
    struct termios term_options;
    const char * port_name = this->_name.c_str();

    res = tcgetattr(this->_fd, &term_options);
    if (res != 0)
    {
        fprintf(stderr, "SerialPort %s: Error updating termios options\n", port_name);
        return -1;
    }

    this->_term_options = term_options;
    return 0;
}

int SerialPort::set_term_options(struct termios term_options)
{
    int res;
    const char * port_name = this->_name.c_str();

    res = tcsetattr(this->_fd, TCSANOW, &term_options);
    if (res != 0)
    {
        fprintf(stderr, "SerialPort %s: Error setting termios options\n", port_name);
        return -1;
    }

    this->_term_options = term_options;
    return 0;
}

int SerialPort::set_baud_rate(int baud_rate, bool force)
{
    int res;
    struct termios term_options;
    speed_t termios_baud_rate;
    const char * port_name = this->_name.c_str();

    if (force)
    {
        res = this->update_term_options();
        if (res != 0) return res;
    }

    term_options = this->_term_options;
    switch(baud_rate)
    {
        case 50: termios_baud_rate = B50; break;
        case 75: termios_baud_rate = B75; break;
        case 110: termios_baud_rate = B110; break;
        case 134: termios_baud_rate = B134; break;
        case 150: termios_baud_rate = B150; break;
        case 200: termios_baud_rate = B200; break;
        case 300: termios_baud_rate = B300; break;
        case 600: termios_baud_rate = B600; break;
        case 1200: termios_baud_rate = B1200; break;
        case 1800: termios_baud_rate = B1800; break;
        case 2400: termios_baud_rate = B2400; break;
        case 4800: termios_baud_rate = B4800; break;
        case 9600: termios_baud_rate = B9600; break;
        case 19200: termios_baud_rate = B19200; break;
        case 38400: termios_baud_rate = B38400; break;
        case 57600: termios_baud_rate = B57600; break;
        case 115200: termios_baud_rate = B115200; break;
        default: 
            fprintf(stderr, "SerialPort %s: %d is an invalid baud rate\n", port_name, baud_rate);
            return -1;
    }

    res = cfsetispeed(&term_options, termios_baud_rate);
    if (res != 0)
    {
        fprintf(stderr, "SerialPort %s: Error setting input baud rate\n", port_name);
        return -1;
    }

    res = cfsetospeed(&term_options, termios_baud_rate);
    if (res != 0)
    {
        fprintf(stderr, "SerialPort %s: Error setting output baud rate\n", port_name);
        return -1;
    }

    if (force)
    {
        res = this->set_term_options(term_options);
        if (res != 0) return res;
    }

    return 0;
}

int SerialPort::query_baud_rate(int *baud_rate, bool force)
{
    int res;
    struct termios term_options;
    const char * port_name = this->_name.c_str();

    if (force)
    {
        res = this->update_term_options();
        if (res != 0) return res;
    }

    term_options = this->_term_options;
    
    speed_t input_baud_rate = cfgetispeed(&term_options);
    speed_t output_baud_rate = cfgetospeed(&term_options);
    if (input_baud_rate != output_baud_rate)
    {
        fprintf(stderr, "SerialPort %s: Input and output baud rate differ\n", port_name);
        return -1;
    }

    *baud_rate = 0;
    switch(input_baud_rate)
    {
        case B50: *baud_rate = 50; break;
        case B75: *baud_rate = 75; break;
        case B110: *baud_rate = 110; break;
        case B134: *baud_rate = 134; break;
        case B150: *baud_rate = 150; break;
        case B200: *baud_rate = 200; break;
        case B300: *baud_rate = 300; break;
        case B600: *baud_rate = 600; break;
        case B1200: *baud_rate = 1200; break;
        case B1800: *baud_rate = 1800; break;
        case B2400: *baud_rate = 2400; break;
        case B4800: *baud_rate = 4800; break;
        case B9600: *baud_rate = 9600; break;
        case B19200: *baud_rate = 19200; break;
        case B38400: *baud_rate = 38400; break;
        case B57600: *baud_rate = 57600; break;
        case B115200: *baud_rate = 115200; break;
        default:
            fprintf(stderr, "SerialPort %s: Unexpected value for baud rate (%d)\n", port_name, input_baud_rate); 
            return -1;
    }

    return 0;
}

int SerialPort::get_baud_rate(bool force)
{
    int baud_rate;

    if (this->query_baud_rate(&baud_rate, force) != 0)
    {
        baud_rate = -1;
    }
    return baud_rate;
}

int SerialPort::set_parity(parity_t parity, bool force)
{
    int res;
    struct termios term_options;
    const char * port_name = this->_name.c_str();

    if (force)
    {
        res = this->update_term_options();
        if (res != 0) return res;
    }

    term_options = this->_term_options;
    switch(parity)
    {
        case NO_PARITY: 
            term_options.c_cflag &= ~PARENB;
            break;
        case ODD_PARITY: 
            term_options.c_cflag |= PARENB;
            term_options.c_cflag |= PARODD;
            break;
        case EVEN_PARITY: 
            term_options.c_cflag |= PARENB;
            term_options.c_cflag &= ~PARODD;
            break;
        default: 
            fprintf(stderr, "SerialPort %s: %d is an invalid parity\n", port_name, parity);
            return -1;
    }

    if (force)
    {
        res = this->set_term_options(term_options);
        if (res != 0) return res;
    }

    return 0;
}

int SerialPort::query_parity(parity_t *parity, bool force)
{
    int res;
    struct termios term_options;

    if (force)
    {
        res = this->update_term_options();
        if (res != 0) return res;
    }

    term_options = this->_term_options;
    if (term_options.c_cflag & PARENB)
    {
        if (term_options.c_cflag & PARODD)
        {
            *parity = ODD_PARITY;
        }
        else
        {
            *parity = EVEN_PARITY;
        }
    }
    else
    {
        *parity = NO_PARITY;
    }

    return 0;
}

int SerialPort::set_char_size(int char_size, bool force)
{
    int res;
    struct termios term_options;
    const char * port_name = this->_name.c_str();

    if (force)
    {
        res = this->update_term_options();
        if (res != 0) return res;
    }

    term_options = this->_term_options;
    term_options.c_cflag &= ~CSIZE;
    switch(char_size)
    {
        case 5: term_options.c_cflag |= CS5; break;
        case 6: term_options.c_cflag |= CS6; break;
        case 7: term_options.c_cflag |= CS7; break;
        case 8: term_options.c_cflag |= CS8; break;
        default: 
            fprintf(stderr, "SerialPort %s: %d is an invalid character size\n", port_name, char_size);
            return -1;
    }

    if (force)
    {
        res = this->set_term_options(term_options);
        if (res != 0) return res;
    }

    return 0;
}

int SerialPort::query_char_size(int *char_size, bool force)
{
    int res;
    struct termios term_options;

    if (force)
    {
        res = this->update_term_options();
        if (res != 0) return res;
    }

    term_options = this->_term_options;
    switch(term_options.c_cflag & CSIZE)
    {
        case CS5: *char_size = 5; break;
        case CS6: *char_size = 6; break;
        case CS7: *char_size = 7; break;
        case CS8: *char_size = 8; break;
        default: break;
            
    }

    return 0;
}


int SerialPort::get_char_size(bool force)
{
    int char_size;

    if (this->query_char_size(&char_size, force) != 0)
    {
        char_size = -1;
    }
    return char_size;
}

int SerialPort::set_default_values()
{
    int res;
    struct termios term_options;

    res = this->update_term_options();
    if (res != 0) return res;

    term_options = this->_term_options;
    // 1 stop bit
    term_options.c_cflag &= ~CSIZE;
    // Raw input;
    term_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // Enable the receiver and set the local mode
    term_options.c_cflag |= (CLOCAL | CREAD);
    // Disable software flow control
    term_options.c_iflag &= ~(IXON | IXOFF | IXANY);
    // Raw output
    term_options.c_oflag &= ~OPOST;

    res = this->set_term_options(term_options);
    if (res != 0) return res;

    return 0;
}

int SerialPort::update_term_status(void)
{
    int res;
    int term_status;
    const char * port_name = this->_name.c_str();

    res = ioctl(this->_fd, TIOCMGET, &term_status);
    if (res != 0)
    {
        fprintf(stderr, "SerialPort %s: Error on TIOCMGET ioctl (errno %d)\n", port_name, errno);
        return -1;
    }

    this->_term_status = term_status;
    return 0;
}

int SerialPort::set_term_status(int term_status)
{
    int res;
    const char * port_name = this->_name.c_str();

    res = ioctl(this->_fd, TIOCMSET, &term_status);
    if (res != 0)
    {
        fprintf(stderr, "SerialPort %s: Error on TIOCMSET ioctl (errno %d)\n", port_name, errno);
        return -1;
    }

    this->_term_status = term_status;
    return 0;
}

// 
// Helper functions for pin functions
//
int SerialPort::set_pin_status(int pin_id, bool status, bool force)
{
    int res;
    int term_status;

    if (force)
    {
        res = this->update_term_status();
        if (res != 0) return res;
    }

    term_status = this->_term_status;
    term_status &= ~pin_id;
    if (status)
    {
        term_status |= pin_id;
    }

    if (force)
    {
        res = this->set_term_status(term_status);
        if (res != 0) return res;
    }

    return 0;
}
int SerialPort::query_pin_status(int pin_id, bool *status, bool force)
{
    int res;
    int term_status;

    if (force)
    {
        res = this->update_term_status();
        if (res != 0) return res;
    }

    term_status = this->_term_status;
    term_status &= pin_id;
    *status = term_status ? true : false;

    return 0;
}
int SerialPort::get_pin_status(int pin_id, bool force)
{
    bool status;
    int res;
    
    res = this->query_pin_status(pin_id, &status, force);
    if (res != 0) return res;

    return status ? 1 : 0;
}

//
// DTR functions
//
int SerialPort::set_dtr(bool status, bool force)
{
    return set_pin_status(TIOCM_DTR, status, force);
}
int SerialPort::query_dtr(bool *status, bool force)
{
    return query_pin_status(TIOCM_DTR, status, force);
}
int SerialPort::get_dtr(bool force)
{
    return get_pin_status(TIOCM_DTR, force);
}

//
// RTS functions
//
int SerialPort::set_rts(bool status, bool force)
{
    return set_pin_status(TIOCM_RTS, status, force);
}
int SerialPort::query_rts(bool *status, bool force)
{
    return query_pin_status(TIOCM_RTS, status, force);
}
int SerialPort::get_rts(bool force)
{
    return get_pin_status(TIOCM_RTS, force);
}

//
// DCD functions
//
int SerialPort::query_dcd(bool *status, bool force)
{
    return query_pin_status(TIOCM_CAR, status, force);
}
int SerialPort::get_dcd(bool force)
{
    return get_pin_status(TIOCM_CAR, force);
}

//
// DSR functions
//
int SerialPort::query_dsr(bool *status, bool force)
{
    return query_pin_status(TIOCM_LE, status, force);
}
int SerialPort::get_dsr(bool force)
{
    return get_pin_status(TIOCM_LE, force);
}

//
// CTS functions
//
int SerialPort::query_cts(bool *status, bool force)
{
    return query_pin_status(TIOCM_CTS, status, force);
}
int SerialPort::get_cts(bool force)
{
    return get_pin_status(TIOCM_CTS, force);
}

//
// RI functions
//
int SerialPort::query_ri(bool *status, bool force)
{
    return query_pin_status(TIOCM_RI, status, force);
}
int SerialPort::get_ri(bool force)
{
    return get_pin_status(TIOCM_RI, force);
}

