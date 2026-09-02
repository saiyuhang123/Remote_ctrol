#include "ConnectDriveUDP.h"

ConnectDriverUDP::ConnectDriverUDP(bool udpenable)
{
	sock = -1;
	port = 0;
	address = "";
	this->udpenable = udpenable;
} 
ConnectDriverUDP::~ConnectDriverUDP()
{
	Close();
}

bool ConnectDriverUDP::setup(string address, int port)
{
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
		}
		
		if (sock == -1)
		{
			cout << "Could not create socket" << endl;
			return false;
		}
	}
	
	if(sock > 0)
			udpopenenable = true;
	listen_thread = std::thread(&ConnectDriverUDP::Listen, this);

	listen_thread.detach();

	return true;
}

//监听线程　读取的数据存放在容器
void ConnectDriverUDP::Listen()
{
	int get = 0;
    std::vector<unsigned char> RX_buf(2048);
	while (openenable || udpenable)
	{
		try
		{
			if(sock > 0)
			{
				if(udpenable)
				{
						struct sockaddr_in temp;
						socklen_t len = sizeof(temp);

						get =  recvfrom(sock,&RX_buf[0],2048,0, NULL,NULL);
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
			udpopenenable = true;
			
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

bool ConnectDriverUDP::Send(string data)
{
	if (sock != -1)
	{

		if(udpenable)
		{
			sendto(sock,data.c_str(),strlen(data.c_str())+1,0,(sockaddr*)&server,sizeof(server));
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

bool ConnectDriverUDP::Send(std::vector<unsigned char> data)
{
	if (sock != -1)
	{
		if(udpenable)
		{
			sendto(sock,(void *)&data[0],data.size()+1,0,(sockaddr*)&server,sizeof(server));
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

bool ConnectDriverUDP::Send(char *data, unsigned int len)
{
	if (sock != -1)
	{
		if(udpenable)
		{
			sendto(sock,(void *)&data[0],len+1,0,(sockaddr*)&server,sizeof(server));
		}
		else
		{

		if (send(sock, (void *)&data[0], len, 0) < 0)
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

string ConnectDriverUDP::receive(int size)
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

string ConnectDriverUDP::read()
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

void ConnectDriverUDP::Close()
{
	close(sock);
	udpopenenable = false;
}

void ConnectDriverUDP::Connect(string address, int port)
{
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	// 1. 创建套接字

	if (sock < 0)
	{
		std::cout << "udp socket error" << std::endl;
		
	}

	// 1.1 填充server信息

	memset(&server, 0, sizeof(server));
	server.sin_family = AF_INET;
	// 需要调用主机转网络
	server.sin_port = htons(port);
	server.sin_addr.s_addr = ::inet_addr(address.c_str());
	udpopenenable = true;
}

bool ConnectDriverUDP::is_nodata(int rate)
{
	if (count++ > (rate / 10))
	{
		count = 0;
		return true;
	}
	else
		return false;
}

bool ConnectDriverUDP::clear_cnt()
{
    count = 0;
}


bool ConnectDriverUDP::is_open()
{
	if(udpenable)
	{
		return udpopenenable;
	}
	return openenable;
}

void ConnectDriverUDP::setcallback(RecvDataCallback callback)
{
	recv_data_callback = callback;
}

