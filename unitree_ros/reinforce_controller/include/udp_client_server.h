#ifndef SNAP_UDP_CLIENT_SERVER_H
#define SNAP_UDP_CLIENT_SERVER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>

namespace udp_client_server
{
    class udp_client_server_runtime_error : public std::runtime_error
    {
    public:
        udp_client_server_runtime_error(const char *w) : std::runtime_error(w) {}
    };

    class udp_client
    {
    public:
        udp_client(const std::string& addr, int port);
        ~udp_client();
        int get_socket() const;
        int get_port() const;
        std::string get_addr() const;
        int send(const char *msg, size_t size);

    private:
        int f_socket;
        int f_port;
        std::string f_addr;
        struct addrinfo* f_addrinfo;
};

    class udp_server
    {
    public:
        udp_server(const std::string& addr, int port);
        ~udp_server();

        int get_socket() const;
        int get_port() const;
        std::string get_addr() const;

        int recv(char *msg, size_t max_size);
        int timed_recv(char *msg, size_t max_size, int max_wait_ms);

    private:
        int f_socket;
        int f_port;
        std::string f_addr;
        struct addrinfo* f_addrinfo;
};

} // namespace udp_client_server
#endif
