#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>
#include <map>
#include <ctime>
#include "json11.hpp"

using namespace std;

void printTime(string name)
{
	cout << time(0) << " : " << name << endl;
}

void newFunc()
{
	printTime(__FUNCTION__);
	map<int, int> test;
	for (int i = 0; i < 10; i++)
	{
		int key = rand() % 5;
		test[key] = test[key] + 1;
	}

	for (map<int, int>::iterator ii = test.begin(); ii != test.end(); ii++)
	{
		cout << ii->first << "\t:" << ii->second << endl;
	}
}

int main()
{
	std::ifstream t("info.json");
	std::string str((std::istreambuf_iterator<char>(t)),
		std::istreambuf_iterator<char>());
	string err;
	json11::Json ret = json11::Json::parse(str, err);
	cout << "hello wordl!" << endl;
	cout << ret["Supine"]["Directory"].string_value() << endl;
	string str1 = ret["Supine"]["Directory"].string_value();
	ret = json11::Json::object{ {"tets", "" } };
	str1 += ret["Supine"]["SegmentedDirectory"].string_value();
	cout << ret.dump() << endl;

	json11::Json temp = json11::Json::object{ {"Hello", "there"} };

	newFunc();

	return 0;
}