#include <iostream>
//#include <Windows.h>
#include "astar.h"

//#define DEBUG
#define WIDTH	10
#define HEIGHT	10

int main()
{
	Map mapp(10, 10); // 10x10 ¸Ê »ý¼º
	mapp.setObject(4, 3);
	mapp.setObject(4, 4);
	mapp.setObject(4, 5);
	mapp.setObject(4, 6);
	mapp.setObject(4, 7);
	mapp.setObject(2, 5);
	mapp.setObject(3, 5);
	mapp.setObject(5, 5);

	mapp.printMap();

	int** map;
	map = new int* [HEIGHT];
	for (int i = 0; i < HEIGHT; ++i) {
		map[i] = new int[WIDTH];
		memset(map[i], LOAD, sizeof(int) * WIDTH);
	}

	Astar as;
	as.setMap(mapp);

	Node* fin;
	fin = as.findRoute(0, 0, 9, 9);
	as.setPathToMap(fin);
	as.printMapAll();

}