#include <termios.h> /* POSIX terminal control definitions */

#include <string>


class SerialPort
{
public:
    enum parity_t
    {
        NO_PARITY = 0,
        ODD_PARITY = 1,
        EVEN_PARITY = 2
    };

public:
    SerialPort(std::string port_name);
    SerialPort(const char *port_name);

    ~SerialPort();

    int open();
    int write(const char *data, size_t len);

    //
    // Config functions
    //
    // Baud Rate
    int set_baud_rate(int baud_rate, bool force=false);
    int query_baud_rate(int *baud_rate, bool force=false);
    int get_baud_rate(bool force=false);
    // Parity
    int set_parity(parity_t parity, bool force=false);
    int query_parity(parity_t *parity, bool force=false);
    // Char size
    int set_char_size(int char_size, bool force=false);
    int query_char_size(int *char_size, bool force=false);
    int get_char_size(bool force=false);

    //
    // Pin functions
    //
    // DTR
    int set_dtr(bool status, bool force=false);
    int query_dtr(bool *status, bool force=false);
    int get_dtr(bool force=false);
    // RTS
    int set_rts(bool status, bool force=false);
    int query_rts(bool *status, bool force=false);
    int get_rts(bool force=false);
    // DCD
    int query_dcd(bool *status, bool force=false);
    int get_dcd(bool force=false);
    // DSR
    int query_dsr(bool *status, bool force=false);
    int get_dsr(bool force=false);
    // CTS
    int query_cts(bool *status, bool force=false);
    int get_cts(bool force=false);
    // RI
    int query_ri(bool *status, bool force=false);
    int get_ri(bool force=false);

    //
    // Termios option/status functions
    //
    int update_term_options();
    int set_term_options(struct termios term_options);
    int update_term_status();
    int set_term_status(int term_status);
private:
    std::string _name;
    int _fd;
    struct termios _term_options;
    int _term_status;

private:
    int set_default_values();

    int set_pin_status(int pin_id, bool status, bool force);
    int query_pin_status(int pin_id, bool *status, bool force);
    int get_pin_status(int pin_id, bool force);

};
