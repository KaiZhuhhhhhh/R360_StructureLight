
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include<string>

#include "config.h"
#include "Calibrate.h"
#include "PairAlign.h"

int config_num[50];

void ArgvConfig()
{
	Get_ConfigNum();
	board_size.width = config_num[0];
	board_size.height = config_num[1];
	square_size.width = config_num[2];
	square_size.height = config_num[3];
	total_clude = config_num[4];
	Registration_flag = config_num[5];
}

void Get_ConfigNum()
{
	std::string a[50];              //采用 string 类型，存100行的文本，不要用数组 

	int i = 0;
	std::ifstream infile;

	infile.open("./config/config.txt", std::ios::in);

	while (!infile.eof())            // 若未到文件结束一直循环 
	{
		getline(infile, a[i], '\n');//读取一行，以换行符结束，存入 a[] 中
		i++;                    //下一行
	}
	for (int ii = 0; ii<i; ii++)        // 显示读取的txt内容 
	{
		std::cout << a[ii] << std::endl;
		std::string::size_type position = a[ii].find(":");
		a[ii]=a[ii].substr(position+1);
		config_num[ii] = std::atoi(a[ii].c_str());
	}
	infile.close();
}