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

#define NAME "marker_bridge_process"
#define IP_PROTOCOL 0
#define CLUSTER_CNT 4

typedef struct{
	float px,py;
	float cx[CLUSTER_CNT];
	float cy[CLUSTER_CNT];
	float Rx[CLUSTER_CNT];
	float Ry[CLUSTER_CNT];
	int cnt[CLUSTER_CNT];
}clustering_result_t;

typedef struct{
	int num_frames; //we only have one frame per raspberry pi this is just for compatiblity with the ps3eye nodes
	int markers_per_frame; //we only have one frame per raspberry pi so no need to set it as a list
	double cx1;
	double cy1;
	double cx2;
	double cy2;
	double cx3;
	double cy3;
	double cx4;
	double cy4;
}output_data_t;

union{
	clustering_result_t data;
	char buffer[sizeof(clustering_result_t)];
} clustering_result_packet;

union{
	output_data_t data;
	char buffer[sizeof(output_data_t)];
} transmit_packet;
/*
 * This program creates a UNIX domain datagram socket, binds a name to it,
 * then reads from the socket.
 */
main(int argc, char** argv)
{
    int port;
    char ip_addr[64];
    transmit_packet.data.num_frames=1;    
    transmit_packet.data.markers_per_frame=1;
    if(argc<3)
    {
      perror("Not enough input arguments!: udp_bridge remote_ip remote_port");
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
    struct sockaddr_un name; //unix socket
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
    printf("%s --> UDP\n", NAME);

    /* Read from the socket */
    while(1)
    {
      if (read(sock, clustering_result_packet.buffer, sizeof(clustering_result_packet.buffer)) < 0)
          perror("receiving datagram packet");

      transmit_packet.data.cx1=clustering_result_packet.data.cx[0];
      transmit_packet.data.cy1=clustering_result_packet.data.cy[0];

      transmit_packet.data.cx2=clustering_result_packet.data.cx[1];
      transmit_packet.data.cy2=clustering_result_packet.data.cy[1];

      transmit_packet.data.cx3=clustering_result_packet.data.cx[2];
      transmit_packet.data.cy3=clustering_result_packet.data.cy[2];

      transmit_packet.data.cx4=clustering_result_packet.data.cx[3];
      transmit_packet.data.cy4=clustering_result_packet.data.cy[3];

      sendto(transmit_socket, transmit_packet.buffer, sizeof(transmit_packet.buffer),
                           0,(struct sockaddr*)&addr_con, sizeof(addr_con));
//      printf("message arrived!\n");
    }
    unlink(NAME);
}
