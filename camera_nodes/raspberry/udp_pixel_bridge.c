#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <ctype.h>
#include <sys/types.h>
#include <stdint.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
 * In the included file <sys/un.h> a sockaddr_un is defined as follows
 * struct sockaddr_un {
 *  short   sun_family;
 *  char    sun_path[108];
 * };
 */


#include <stdio.h>
#define NAME "udp_bridge_process"
#define MAX_BRIGHT_PIXELS 4096
#define IP_PROTOCOL 0

struct
pixel_data{
 int16_t row;
 int16_t col;
 int32_t val;
};

union{
 	struct{
		uint32_t seq_length;
		uint64_t timestamp;
		uint32_t status;
		struct pixel_data pixels[MAX_BRIGHT_PIXELS];
	}Data;
	char buffer[MAX_BRIGHT_PIXELS*8+16];
}UDP_Packet;

/*
 * This program creates a UNIX domain datagram socket, binds a name to it,
 * then reads from the socket.
 */
main(int argc, char** argv)
{
    int port;
    char ip_addr[64];
    if(argc<3)
    {
      perror("Not enough input arguments!, udp_bridge remote_ip remote_port");
      exit(1);
    }
    sscanf(argv[2],"%d",&port);
    strcpy(ip_addr,argv[1]);

    //UDP Socket
    int transmit_socket;
    struct sockaddr_in addr_con;
    int addrlen = sizeof(addr_con);
    addr_con.sin_family = AF_INET;
    addr_con.sin_port = htons(port);
    addr_con.sin_addr.s_addr = inet_addr(ip_addr);
    transmit_socket = socket(AF_INET, SOCK_DGRAM,
                IP_PROTOCOL);
    if (transmit_socket < 0)
      printf("\nCould not open the transmit socket!!\n");
    //IPC Socket
    unlink(NAME);
    int sock, length;
    struct sockaddr_un name;
    /* Create socket from which to read. */
    sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("opening datagram socket");
        exit(1);
    }

    /* Create name. */
    name.sun_family = AF_UNIX;
    strcpy(name.sun_path, NAME);

    /* Bind the UNIX domain address to the created socket */
    if (bind(sock, (struct sockaddr *) &name, sizeof(struct sockaddr_un))) {
        perror("binding name to datagram socket");
        exit(1);
    }
    printf("socket -->%s\n", NAME);

    /* Read from the socket */
    while(1)
    {
      if (read(sock, UDP_Packet.buffer, sizeof(UDP_Packet.buffer)) < 0)
          perror("receiving datagram packet");
      sendto(transmit_socket, UDP_Packet.buffer, sizeof(UDP_Packet.buffer),
                           0,(struct sockaddr*)&addr_con, sizeof(addr_con));
      printf("message arrived!\n");
    }
    unlink(NAME);
}

