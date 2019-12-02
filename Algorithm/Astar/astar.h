#pragma once
#include <vector>

#define LOAD	0
#define OBJECT	1
#define PATH	2
#define DIAGONAL	true
#define CROSSCORNER	false

class Node {
public:
	unsigned xPos;
	unsigned yPos;
	int fScore;
	int gScore;
	int hScore;
	Node* parent;

	Node();
	Node(unsigned _x, unsigned _y);
	Node(unsigned _x, unsigned _y, Node* _parent);

	bool compNode(Node* node);
	void initNode();
};


class Map {
	unsigned xSize;
	unsigned ySize;
	int** data;

public:
	Map();
	Map(unsigned width, unsigned height);
	Map(const Map& map);
	~Map();

	Map& operator=(const Map& map);
	void setMap(unsigned width, unsigned height, int** _map_data);
	int getMapData(unsigned _x, unsigned _y);
	void setMapData(unsigned _x, unsigned _y, int val);
	void setObject(unsigned _x, unsigned _y);
	void printMap();
	bool isWalkable(unsigned _x, unsigned _y);
	void setPath(Node* fin);
};


class pqueue {
	std::vector<Node*> que;
	unsigned int size;

public:
	pqueue();
	pqueue(int _reserve);
	~pqueue();

	void push(Node* const node);
	void sorting();
	void pushNode(Node* const newNode);
	bool isInQueue(Node* node); // 안쓰넹..
	void pop();
	Node* top();
	bool empty();
	unsigned int getSize();
};


class Astar {
	Map map;
	Node start;
	Node finish;

	pqueue openList;
	std::vector<Node*> closeList;
	bool hasRoute;

public:
	Astar();
	Astar(const Map& _map, Node _start, Node _finish);
	~Astar();

	void init();
	void setStart(unsigned int _x, unsigned int _y);
	void setFinish(unsigned int _x, unsigned int _y);
	void setMap(int _x, int _y, int** _map_data);
	void setMap(const Map& _map);
	int calcH(Node* const from, Node* const to);
	int calcG(Node* const from, int diag);
	void calcF(Node* temp, int diag);
	void setObjectToMap(unsigned int _x, unsigned int _y);
	void printMapAll();
	void setPathToMap(Node* fin);
	bool isInOpenList(Node* node); // 안쓰네?
	bool isInCloseList(Node* node);
	bool checkAround(Node* node, bool allowDiagonal, bool crossCorner);
	Node* findRoute(unsigned int fromX, unsigned int fromY, unsigned int toX, unsigned int toY);
};
