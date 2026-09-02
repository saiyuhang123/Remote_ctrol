#include "ConnectDriver.h"
#include <error.h>
ConnectDriver::ConnectDriver(bool udpenable)
{
	sock = -1;
	port = 0;
	address = "";
	this->udpenable = udpenable;
} 
ConnectDriver::~ConnectDriver()
{
	Close();
}

bool ConnectDriver::setup(string address, int port)
{
	addrlen = sizeof(struct sockaddr_in);
	if (sock == -1)
	{
		if(udpenable)
		{
			sock = socket(AF_INET, SOCK_DGRAM, 0);
			if (sock < 0)
			{
				std::cout << "udp socket error" << std::endl;
			}
			memset(&server, 0, sizeof(server));
			server.sin_family = AF_INET;
			// 需要调用主机转网络
			server.sin_port = htons(port);
			server.sin_addr.s_addr = ::inet_addr(address.c_str());
		 int opt = 1;
		 setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

		}
		else
			sock = socket(AF_INET, SOCK_STREAM, 0);
		if (sock == -1)
		{
			cout << "Could not create socket" << endl;
			return false;
		}
	}
	

	if(udpenable)
	{
		if(sock > 0)
			udpopenenable = true;
	}
	else
	{
		if ((signed)inet_addr(address.c_str()) == -1)
		{
			struct hostent *he;
			struct in_addr **addr_list;
			if ((he = gethostbyname(address.c_str())) == NULL)
			{
				herror("gethostbyname");
				cout << "Failed to resolve hostname\n";
				return false;
			}
			addr_list = (struct in_addr **)he->h_addr_list;
			for (int i = 0; addr_list[i] != NULL; i++)
			{
				server.sin_addr = *addr_list[i];
				break;
			}
		}
		else
		{
			server.sin_addr.s_addr = inet_addr(address.c_str());
		}
		server.sin_family = AF_INET;
		server.sin_port = htons(port);
			int error = -1, len;
			len = sizeof(int);
			timeval tm;
			fd_set set;
			unsigned long ul = 1;
			ioctl(sock, FIONBIO, &ul); //设置为非阻塞模式
			bool ret = false;

			if (connect(sock, (struct sockaddr *)&server, sizeof(server)) == -1)
			{
				tm.tv_sec = 1;
				tm.tv_usec = 0;
				FD_ZERO(&set);
				FD_SET(sock, &set);
				if (select(sock + 1, NULL, &set, NULL, &tm) > 0)
				{
					getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, (socklen_t *)&len);
					if (error == 0)
						ret = true;
					else
						ret = false;
				}
				else
					ret = false;
			}
			else
				ret = true;

			ul = 0;
			ioctl(sock, FIONBIO, &ul); //设置为阻塞模式
			if (!ret)
			{
				close(sock);
				return false;
			}
				openenable = true;
				  struct timeval timeout = {1, 0};
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));
	}

	listen_thread = std::thread(&ConnectDriver::Listen, this);
	listen_thread.detach();
	return true;
}
void ConnectDriver::Listen()
{
	int get = 0;
    std::vector<unsigned char> RX_buf(2048);
	while (openenable || udpopenenable)
	{
		try
		{
			if(sock > 0)
			{
				if(udpenable)
				{
						struct sockaddr_in temp;
						get =  recvfrom(sock,&RX_buf[0],2048,0, (struct sockaddr *)&server,(socklen_t *)&addrlen);
				}
			else
				get = recv(sock, &RX_buf[0], sizeof(RX_buf), 0);
			}
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
		if (get > 0)
		{
			if(recv_data_callback != NULL)

				recv_data_callback(RX_buf,get);
			count = 0;
		}
		else if (get < 0)
		{
			//ROS_INFO("get data false");
		}
	}
	Close();
}

bool ConnectDriver::Send(string data)
{
	if (sock != -1)
	{
		if(udpenable)
		{
			sendto(sock,data.c_str(),strlen(data.c_str())+1,0,(sockaddr*)&server,sizeof(server));
			cout << "udpenable send" << endl;
		}
		else
		{
			if (send(sock, data.c_str(), strlen(data.c_str()), 0) < 0)
			{
				cout << "Send failed : " << data << endl;
				return false;
			}
		}
	}
	else
		return false;
	return true;
}

bool ConnectDriver::Send(std::vector<unsigned char> data)
{
	if (sock != -1)
	{
		if(udpenable)
		{
			sendto(sock,(void *)&data[0],data.size()+1,0,(sockaddr*)&server,sizeof(server));
			cout << "udpenable send2" << endl;
		}
		else
		{
		if (send(sock, (void *)&data[0], data.size(), 0) < 0)
		{
			cout << "Send failed : " << endl;
			return false;
		}
		}
	}
	else
		return false;
	return true;
}

bool ConnectDriver::Send(char *data, unsigned int len)
{
	if (sock != -1)
	{
		if(udpenable)
		{
			int sendsize = sendto(sock,(void *)&data[0],len,0,(sockaddr*)&server,sizeof(server));

		}
		else
		{

		if (send(sock, (void *)&data[0], len, 0) < 0)
		{
			return false;
		}
		}
	}
	else
		return false;
	return true;
}

string ConnectDriver::receive(int size)
{
	char buffer[size]; 
	memset(&buffer[0], 0, sizeof(buffer));

	string reply;
	if (recv(sock, buffer, size, 0) < 0)
	{
		cout << "receive failed!" << endl;
		return nullptr;
	}
	buffer[size - 1] = '\0';
	reply = buffer;
	return reply;
}

string ConnectDriver::read()
{
	char buffer[1] = {};
	string reply;
	while (buffer[0] != '\n')
	{
		if (recv(sock, buffer, sizeof(buffer), 0) < 0)
		{
			cout << "receive failed!" << endl;
			return nullptr;
		}
		reply += buffer[0];
	}
	return reply;
}

void ConnectDriver::Close()
{
	openenable = false;
	udpopenenable = false;

	if(sock > 0)
	{

		shutdown(sock,SHUT_RD);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
		close(sock);
	}
}

bool ConnectDriver::is_nodata(int rate)
{
	if (count++ > (rate / 10))
	{
		count = 0;
		return true;
	}
	else
		return false;
}

bool ConnectDriver::clear_cnt()
{
    count = 0;
}


bool ConnectDriver::is_open()
{
	if(udpenable)
	{
		return udpopenenable;
	}
	return openenable;
}

void ConnectDriver::setcallback(RecvDataCallback callback)
{
	recv_data_callback = callback;
}

