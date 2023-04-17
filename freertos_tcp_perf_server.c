/*
 * Copyright (C) 2018 - 2019 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#include "freertos_tcp_perf_server.h"
#include "dma_app.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdlib.h>

extern struct netif server_netif;
static struct perf_stats server;
extern u8 *RxBufferPtr0;
extern u8 *RxBufferPtr1;
extern SemaphoreHandle_t command_signal;
extern u8 dma_recv_coming_flag;
extern u8 pingpang_flag;
u8 send_data_flag;
//define tcp variable
struct tcp_msg tcp_ack_p;
struct tcp_msg tcp_ack_op;
struct tcp_msg tcp_handle_p;
struct tcp_msg tcp_release_p;
struct tcp_msg tcp_ctlACK_p;


u16 crc_16();
u32_t convert_end32();
u16_t convert_end16();



/* Interval time in seconds */
#define REPORT_INTERVAL_TIME (INTERIM_REPORT_INTERVAL * 1000)

void print_app_header(void)
{

	xil_printf("Server listening on port %d\r\n",
			TCP_CONN_PORT);
#if LWIP_IPV6==1
	xil_printf("On Host: Run $iperf -V -c %s%%<interface> -i %d -t 300 -w 2M\r\n",
			inet6_ntoa(server_netif.ip6_addr[0]),
			INTERIM_REPORT_INTERVAL);
#else
	xil_printf("On Host: Run $iperf -c %s -i %d -t 300 -w 2M\r\n",
			inet_ntoa(server_netif.ip_addr),
			INTERIM_REPORT_INTERVAL);
#endif /* LWIP_IPV6 */

}

static void print_tcp_conn_stats(int sock)
{
#if LWIP_IPV6==1
	struct sockaddr_in6 local, remote;
#else
	struct sockaddr_in local, remote;
#endif /* LWIP_IPV6 */
	int size;

	size = sizeof(local);
	getsockname(sock, (struct sockaddr *)&local, (socklen_t *)&size);
	getpeername(sock, (struct sockaddr *)&remote, (socklen_t *)&size);
#if LWIP_IPV6==1
	xil_printf("[%3d] local %s port %d connected with ", server.client_id,
			inet6_ntoa(local.sin6_addr), ntohs(local.sin6_port));
	xil_printf("%s port %d\r\n", inet6_ntoa(remote.sin6_addr),
			ntohs(local.sin6_port));
#else
	xil_printf("[%3d] local %s port %d connected with ", server.client_id,
			inet_ntoa(local.sin_addr), ntohs(local.sin_port));
	xil_printf("%s port %d\r\n", inet_ntoa(remote.sin_addr),
			ntohs(local.sin_port));
#endif /* LWIP_IPV6 */
	xil_printf("[ ID] Interval    Transfer     Bandwidth\n\r");
}

void UDPbroadcast_thread(void)
{
	//SEND THREAD INCLUDE UDP THREAD AND TCP THREAD
	//USE semaphore to determine which thread to use
	int sock;
#if LWIP_IPV6==1
	struct sockaddr_in6 address;
#else
	struct sockaddr_in address;
#endif /* LWIP_IPV6 */

	/* set up address to connect to */
    memset(&address, 0, sizeof(address));
#if LWIP_IPV6==1
	if ((sock = lwip_socket(AF_INET6, SOCK_DGRAM, 0)) < 0) {
		xil_printf("UDP broadcast: Error creating Socket\r\n");
		return;
	}
	address.sin6_family = AF_INET6;
	address.sin6_port = htons(UDP_CAST_PORT);
	address.sin6_len = sizeof(address);
#else
	if ((sock = lwip_socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		xil_printf("UDP broadcast: Error creating Socket\r\n");
		return;
	}
	address.sin_family = AF_INET;
	address.sin_port = htons(UDP_CAST_PORT);
	address.sin_addr.s_addr = inet_addr("192.168.1.255");
	//192.168.1.255
#endif /* LWIP_IPV6 */
	struct udp_broadcast *udp_cast;
	udp_cast = mem_malloc(sizeof(struct udp_broadcast));
	//to form the packet
	udp_cast->frameheader = convert_end32(0x484f5354);
	udp_cast->length = convert_end32(0x1C);
	udp_cast->command =convert_end32(0x00000001);
	udp_cast->seq =convert_end32(0x00000001);
//		crc_16((u8*)udp_broadcast,4);
	udp_cast->u1.hardversion =convert_end32(0x12345678);
	udp_cast->u1.firmware_version=convert_end32(0x22345678);
	udp_cast->u1.sub_version =convert_end32(0x32345678);
	udp_cast->u1.tcp_port=htons(TCP_CONN_PORT);
	udp_cast->u1.udp_port=htons(UDP_CAST_PORT);
	udp_cast->crc=convert_end16(crc_16((u8*)udp_cast,44)); //TODO the length of crc might be wrong, need to be checked later
		while (1)
		{
	    	sendto(sock,udp_cast,46,1,(struct sockaddr *)&address, sizeof (address));
			vTaskDelay(1000);

		}
		vTaskDelete(NULL);
}

struct tcp_msg *tcp_start_ack(struct tcp_msg * tcp_ack_p ,u16 crct, u32 seq1)
{
	//2.应答包 start
	tcp_ack_p->frameheader = convert_end32(0x484f5354);
	tcp_ack_p->length = convert_end32(0x00000001);
	tcp_ack_p->command = convert_end32(0x00000011);
	tcp_ack_p->seq =convert_end32(seq1);
	if(crct != 0)
	{
		tcp_ack_p->content=0x01;
	}
	else
	{
		tcp_ack_p->content=0x00;
	}
	tcp_ack_p->crc=convert_end16(crc_16((u8*)tcp_ack_p,17));
//	//the send out buffer should be: 48 4F 53 54 00 00 00 01 00 00 00 11 00 00 00 01 00 34 23

//		xil_printf(" ack___start: %x \n\r",0);
	return tcp_ack_p;
}

struct tcp_msg *tcp_stop_ack(struct tcp_msg * tcp_ack_op ,u16 crct, u32 seq1)
{
	//2.应答包 stop
	tcp_ack_op->frameheader = convert_end32(0x484f5354);
	tcp_ack_op->length = convert_end32(1);
	tcp_ack_op->command = convert_end32(0x00000012);
	tcp_ack_op->seq =convert_end32(seq1);
	if(crct != 0)
	{
		tcp_ack_op->content=0x01;
	}
	else
	{
		tcp_ack_op->content=0x00;
	}
	tcp_ack_op->crc=convert_end16(crc_16((u8*)tcp_ack_op,17));
	return tcp_ack_op;

}


struct tcp_msg *tcp_handle(struct tcp_msg *tcp_handle_p,u16 crct, u32 seq1)
{
	//4.获取句柄
	tcp_handle_p->frameheader = convert_end32(0x484f5354);
	tcp_handle_p->length = convert_end32(1);
	tcp_handle_p->command = convert_end32(0x0000001b);
	tcp_handle_p->seq =convert_end32(seq1);
	if(crct != 0)
		{
			tcp_handle_p->content=0x01;
		}
	else
		{
			tcp_handle_p->content=0x00;
		}
	tcp_handle_p->crc=convert_end16(crc_16((u8*)tcp_handle_p,17));
		return tcp_handle_p;
}

struct tcp_msg *tcp_release(struct tcp_msg *tcp_release_p, u16 crct, u32 seq1)
{
	//5.释放句柄
	tcp_release_p->frameheader = convert_end32(0x484f5354);
	tcp_release_p->length = convert_end32(1);
	tcp_release_p->command = convert_end32(0x0000001e);
	tcp_release_p->seq =convert_end32(seq1);
	if(crct != 0)
		{
			tcp_release_p->content=0x01;
		}
	else
		{
			tcp_release_p->content=0x00;
		}
	tcp_release_p->crc=convert_end16(crc_16((u8*)tcp_release_p,17));
		return tcp_release_p;

}

struct tcp_msg *tcp_ctlACK(struct tcp_msg *tcp_ctlACK_p, u16 crct, u32 seq1)
{
	//6.控制参数
	tcp_ctlACK_p->frameheader = convert_end32(0x484f5354);
	tcp_ctlACK_p->length = convert_end32(1);
	tcp_ctlACK_p->command = convert_end32(0x00000020);
	tcp_ctlACK_p->seq =convert_end32(seq1);
	if(crct != 0)
	{
		tcp_ctlACK_p->content=0x01;
	}
	else
	{
		tcp_ctlACK_p->content=0x00;
	}
	tcp_ctlACK_p->crc=convert_end16(crc_16((u8*)tcp_ctlACK_p,17));
		return tcp_ctlACK_p;

}

static void stats_buffer(char* outString, double data, enum measure_t type)
{
	int conv = KCONV_UNIT;
	const char *format;
	double unit = 1024.0;

	if (type == SPEED)
		unit = 1000.0;

	while (data >= unit && conv <= KCONV_GIGA) {
		data /= unit;
		conv++;
	}

	/* Fit data in 4 places */
	if (data < 9.995) { /* 9.995 rounded to 10.0 */
		format = "%4.2f %c"; /* #.## */
	} else if (data < 99.95) { /* 99.95 rounded to 100 */
		format = "%4.1f %c"; /* ##.# */
	} else {
		format = "%4.0f %c"; /* #### */
	}
	sprintf(outString, format, data, kLabel[conv]);
}

/* The report function of a TCP server session */
static void tcp_conn_report(u64_t diff, enum report_type report_type)
{
	u64_t total_len;
	double duration, bandwidth = 0;
	char data[16], perf[16], time[64];

	if (report_type == INTER_REPORT) {
		total_len = server.i_report.total_bytes;
	} else {
		server.i_report.last_report_time = 0;
		total_len = server.total_bytes;
	}

	/* Converting duration from milliseconds to secs,
	 * and bandwidth to bits/sec .
	 */
	duration = diff / 1000.0; /* secs */
	if (duration)
		bandwidth = (total_len / duration) * 8.0;

	stats_buffer(data, total_len, BYTES);
	stats_buffer(perf, bandwidth, SPEED);
	/* On 32-bit platforms, xil_printf is not able to print
	 * u64_t values, so converting these values in strings and
	 * displaying results
	 */
	sprintf(time, "%4.1f-%4.1f sec",
			(double)server.i_report.last_report_time,
			(double)(server.i_report.last_report_time + duration));
	xil_printf("[%3d] %s  %sBytes  %sbits/sec\n\r", server.client_id,
			time, data, perf);

	if (report_type == INTER_REPORT)
		server.i_report.last_report_time += duration;
}



void UDP_application(void)
{
	int sock;
	send_data_flag=0;
	#if LWIP_IPV6==1
		struct sockaddr_in6 address;
	#else
		struct sockaddr_in address;
	#endif /* LWIP_IPV6 */

		/* set up address to connect to */
	        memset(&address, 0, sizeof(address));
	#if LWIP_IPV6==1
		if ((sock = lwip_socket(AF_INET6, SOCK_DGRAM, 0)) < 0) {
			xil_printf("UDP server: Error creating Socket\r\n");
			return;
		}
		address.sin6_family = AF_INET6;
		address.sin6_port = htons(TCP_CONN_PORT);
		address.sin6_len = sizeof(address);
	#else
		if ((sock = lwip_socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			xil_printf("UDP server: Error creating Socket\r\n");
			return;
		}
		address.sin_family = AF_INET;
		address.sin_port = htons(TCP_CONN_PORT);
		address.sin_addr.s_addr = INADDR_ANY;
	#endif /* LWIP_IPV6 */

		if (bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0) {
			xil_printf("UDP server: Unable to bind to port %d\r\n",
					TCP_CONN_PORT);
			close(sock);
			return;
		}

		struct sockaddr_in address_host;
        memset(&address_host, 0, sizeof(address_host));
		address_host.sin_family = AF_INET;
		address_host.sin_port = htons(9000);
		address_host.sin_addr.s_addr = inet_addr("192.168.1.3");
		while (1)
		{
			if(send_data_flag==1)
			{
				if(dma_recv_coming_flag == 1)

					{
						dma_recv_coming_flag = 0;

						if(pingpang_flag == 0)
						{
							sendto(sock,RxBufferPtr0,POINTCLOUD_PKT_LEN,0,(struct sockaddr *)&address_host, sizeof (address_host));
						}
						else if(pingpang_flag == 1)
						{
							sendto(sock,RxBufferPtr1,POINTCLOUD_PKT_LEN,0,(struct sockaddr *)&address_host, sizeof (address_host));
						}

					}
			}
				vTaskDelay(1);
		}

}

/* thread spawned for each connection */
void tcp_recv_perf_traffic(void *p)
{
	u8 recv_buf[RECV_BUF_SIZE];
	int read_bytes;
	u16 data_length;
	int sock = *((int *)p);
	static u8 heartBeat[]="No heart beat from the PC. TCP is disconnected.";
	static u8 INVALIDCOMMAND[]="------ERROR:INVALID COMMAND FROM CLIENT-------";

	server.start_time = sys_now() * portTICK_RATE_MS;
	server.client_id++;
	server.i_report.last_report_time = 0;
	server.i_report.start_time = 0;
	server.i_report.total_bytes = 0;
	server.total_bytes = 0;

	print_tcp_conn_stats(sock);


	while (1) {
		/* read a max of RECV_BUF_SIZE bytes from socket */
		if ((read_bytes = lwip_recvfrom(sock, recv_buf, RECV_BUF_SIZE,
						0, NULL, NULL)) < 0) {
			u64_t now = sys_now() * portTICK_RATE_MS;
			u64_t diff_ms = now - server.start_time;
			tcp_conn_report(diff_ms, TCP_ABORTED_REMOTE);
			break;
		}

		/* break if client closed connection */
		if (read_bytes == 0) {
			u64_t now = sys_now() * portTICK_RATE_MS;
			u64_t diff_ms = now - server.start_time;
			tcp_conn_report(diff_ms, TCP_DONE_SERVER);
			xil_printf("TCP test passed Successfully\n\r");
			break;
		}

		if (REPORT_INTERVAL_TIME) {
			u64_t now = sys_now() * portTICK_RATE_MS;
			server.i_report.total_bytes += read_bytes;
			if (server.i_report.start_time) {
				u64_t diff_ms = now - server.i_report.start_time;

				if (diff_ms >= REPORT_INTERVAL_TIME) {
					tcp_conn_report(diff_ms, INTER_REPORT);
					server.i_report.start_time = 0;
					server.i_report.total_bytes = 0;
				}
			} else {
				server.i_report.start_time = now;
			}
		}
		/* Record total bytes for final report */
		server.total_bytes += read_bytes;
		data_length = recv_buf[6]*256+recv_buf[7];
		//determine the total length of receive buffer: 4+4+4+4+2 + length of data (byte)
//		u16 total = data_length + 18 ;
		xil_printf("length of data: %x   total length: %d \n\r",data_length, read_bytes);
		u32 seqs = (recv_buf[12] << 24) | (recv_buf[13] << 16) | (recv_buf[14] << 8) | recv_buf[15];
		u16 recv_crc =  convert_end16((recv_buf[read_bytes-2]<<8)| recv_buf[read_bytes-1]);
		u16 crc_t =crc_16(recv_buf,read_bytes-2);
		u16 diff = crc_t-recv_crc;

		if (recv_buf[11]==0x02)
		{
			char temp=NULL;
			temp = recv_buf[12];
			if(temp!=0x00)
			{
				send(sock,heartBeat,sizeof(heartBeat),0);
				break;
			}
		}

		else if (recv_buf[11]==0x11)
		{
		/* xil_printf(" sequence: %d  ",seqs);
			 * u32 ttttt =(recv_buf[0]<<24) |(recv_buf[1] << 16) | (recv_buf[2]<<8) | recv_buf[3];
			 * TESE CASE1: 4C 73 44 52 00 00 00 01 00 00 00 11 00 00 00 01 00 69 AD
			 * TEST CASE2: 4C 73 44 52 00 00 00 01 00 00 00 12 00 00 00 01 00 14 A1
//			xil_printf("Is crc start coming?%x\n\r", crc_t);
			CRC little endian */
			send(sock,tcp_start_ack(&tcp_ack_p,diff, seqs), 19,0);
			send_data_flag=1;
		}
		else if(recv_buf[11]== 0x12)
		{
			send(sock, tcp_stop_ack(&tcp_ack_op,diff, seqs), 19,0);

		}
		else if(recv_buf[11]== 0x1B)
		{

			send(sock, tcp_handle(&tcp_handle_p,diff, seqs), 19,0);
		}
		else if(recv_buf[11]== 0x1E)
		{
			send(sock,tcp_release(&tcp_release_p,diff, seqs), 19,0);
			send_data_flag=0;
			break;

		}
		else if(recv_buf[11]== 0x20)
		{

			send(sock,tcp_ctlACK(&tcp_ctlACK_p,diff, seqs), 19,0);
		}
		else
		{
			send(sock,INVALIDCOMMAND,sizeof(INVALIDCOMMAND),0);
		}
		vTaskDelay(1);
	}

	/* close connection */
	close(sock);
	vTaskDelete(NULL);
}



void TCP_application(void)
{
	int sock, new_sd;
#if LWIP_IPV6==1
	struct sockaddr_in6 address, remote;
#else
	struct sockaddr_in address, remote;
#endif /* LWIP_IPV6 */
	int size;

	/* set up address to connect to */
        memset(&address, 0, sizeof(address));
#if LWIP_IPV6==1
	if ((sock = lwip_socket(AF_INET6, SOCK_STREAM, 0)) < 0) {
		xil_printf("TCP server: Error creating Socket\r\n");
		return;
	}
	address.sin6_family = AF_INET6;
	address.sin6_port = htons(TCP_CONN_PORT);
	address.sin6_len = sizeof(address);
#else
	if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		xil_printf("TCP server: Error creating Socket\r\n");
		return;
	}
	address.sin_family = AF_INET;
	address.sin_port = htons(TCP_CONN_PORT);
	address.sin_addr.s_addr = INADDR_ANY;

#endif /* LWIP_IPV6 */

	if (bind(sock, (struct sockaddr *)&address, sizeof (address)) < 0) {
		xil_printf("TCP server: Unable to bind to port %d\r\n",
				TCP_CONN_PORT);
		close(sock);
		return;
	}

	if (listen(sock, 1) < 0) {
		xil_printf("TCP server: tcp_listen failed\r\n");
		close(sock);
		return;
	}

	size = sizeof(remote);

	while (1) {
		if ((new_sd = accept(sock, (struct sockaddr *)&remote,
						(socklen_t *)&size)) > 0)
			sys_thread_new("TCP_recv_perf thread",
				tcp_recv_perf_traffic, (void*)&new_sd,
				TCP_SERVER_THREAD_STACKSIZE,
				DEFAULT_THREAD_PRIO);

	}
}
