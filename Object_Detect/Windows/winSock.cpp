#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <WinSock2.h>
#include <time.h>
#include "opencv2/opencv.hpp"

#define PORT_NUM	4000
#define SEND_SIZE	1024 //58368
#define DEBUG

int main()
{
	SOCKET server_socket, client_socket;
	WSADATA wsa_data;
	SOCKADDR_IN server_addr, client_addr;
	int client_addr_size;

	int recv_info[3];
	char send_buff[3];
	unsigned char* recv_data;
	int ret, total_size, loc = 0;
	int h, w, c;
	int i, x, y, z;
	int count = 0;

	// winsock �ʱ�ȭ, ���� 2.2
	if (0 < WSAStartup(MAKEWORD(2, 2), &wsa_data)) {
		std::cerr << "WSAStartup() error\n";
		return -1;
	}
	
	// server socket ����
	server_socket = socket(PF_INET, SOCK_STREAM, 0);
	if (INVALID_SOCKET == server_socket) {
		std::cerr << "socket() error\n";
		return -1;
	}

	// server addr �ʱ�ȭ
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT_NUM);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	// server socket �� addr ������ Ŀ�ο� ���
	if (SOCKET_ERROR == bind(server_socket, (SOCKADDR*)&server_addr, sizeof(server_addr))) {
		std::cerr << "bind() error\n";
		return -1;
	}
#ifdef DEBUG
	std::cerr << "bind ok\n";
#endif
	// client �� connect ���
	if (SOCKET_ERROR == listen(server_socket, 5)) {
		std::cerr << "listen() error\n";
		return -1;
	}
	std::cerr << "wait for client connection...\n";
#ifdef DEBUG
	std::cerr << "listen ok\n";
#endif
	// client connect ����
	client_addr_size = sizeof(client_addr);
	client_socket = accept(server_socket, (SOCKADDR*)&client_addr, &client_addr_size);

	if (INVALID_SOCKET == client_socket) {
		std::cerr << "accept() error\n";
		return -1;
	}
	std::cerr << "connected!\n\n";
#ifdef DEBUG
	std::cerr << "accept ok\n";
#endif

	// client �� ���� send buff �ʱ�ȭ
	memset(send_buff, 0, sizeof(send_buff));
	
	while (1)
	{
#ifdef DEBUG
		clock_t start, end;
		double result;

		start = clock();	// �ð� ����
#endif
		// image ���� ����
		ret = 0;
		while (ret != sizeof(recv_info)) { // socket ���� ������ ���� �� ��� ���� ���� �����Ƿ�
			ret += recv(client_socket, (char*)recv_info + ret, sizeof(recv_info) - ret, 0);
			if (ret < 0) {
				std::cerr << "image info receive fail\n";
				return -1;
			}
		}
		total_size = recv_info[0];	// ���� image�� ũ��

		// image data ���� buff ����
		recv_data = (unsigned char*)calloc(total_size, sizeof(unsigned char));
		if (recv_data == NULL) {
			std::cerr << "recv_data malloc() fail\n";
			return -1;
		}

		// image ������ ����
		ret = 0;
		while (ret != total_size) {	// socket ���� ������ ���� �� ��� ���� ���� �����Ƿ�
			ret += recv(client_socket, (char*)recv_data + loc + ret, total_size - ret, 0);
			if (ret < 0) {
				std::cerr << "image data receive fail\n";
				return -1;
			}
		}
		
#ifdef DEBUG
		std::cout << "read ret : " << ret << std::endl;
		std::cerr << "[receive img]\n";

#endif
		// ���Ź��� image ������ Mat ��ü ���� �� �� ����
		std::vector<uchar> decoding(recv_data, recv_data + total_size);
		cv::Mat img = cv::imdecode(decoding, cv::IMREAD_COLOR);

		cv::namedWindow("recv", cv::WINDOW_AUTOSIZE);
		cv::imshow("recv", img);
		int c = cv::waitKey(1);
		if (c == 27) {
			// ESC �Է� �� ����
			send_buff[0] = 'c';
			ret = send(client_socket, send_buff, sizeof(send_buff), 0);
			if (ret < 0) {
				std::cerr << "[tcpSocket.c] message write fail\n";
				return -1;
			}
			break;
		}
		send_buff[0] = 'o';
		ret = send(client_socket, send_buff, sizeof(send_buff), 0);
		if (ret < 0) {
			std::cerr << "[tcpSocket.c] message write fail\n";
			return -1;
		}

		free(recv_data);

#ifdef DEBUG
		std::cout << "write ret : " << ret << std::endl;
		std::cout << "loop count : " << ++count << std::endl;

		end = clock();
		result = (double)(end - start) / 1000;	// �ҿ�ð� ����
		std::cout << "time: " << result << std::endl;
#endif
	}
	std::cerr << "Quit the program!\n";

	cv::destroyWindow("recv");
	//free(recv_data);
	closesocket(client_socket);
	closesocket(server_socket);
	WSACleanup();

	return 0;
}
