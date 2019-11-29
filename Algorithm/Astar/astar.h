#pragma once
#include <vector>

#define LOAD	0
#define OBJECT	1
#define PATH	2
#define DIAGONAL	true
#define CROSSCORNER	false

class Node {
public:
	unsigned int xPos, yPos;
	int fScore;
	int gScore;
	int hScore;
	Node* parent;

	Node();
	Node(unsigned int _x, unsigned int _y);
	Node(unsigned int _x, unsigned int _y, Node* _parent);

	bool compNode(Node* node);
	void initNode();
};


class Map {
public:
	unsigned xSize;
	unsigned ySize;
	int** data;

//public:
	Map();
	Map(unsigned _x, unsigned _y);
	Map(const Map& map);
	~Map();

	Map& operator=(const Map& map);
	void setMapData(unsigned width, unsigned height, int** _map_data);
	void setObject(int _x, int _y);
	void printMap();
	bool isWalkable(int _x, int _y);
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


//struct comp {	// 우선순위 큐 정렬 기준 정의(fScore 기준 오름차순)
//	bool operator()(Node* a, Node* b){
//		return a->fScore > b->fScore;	
//	}
//};

class Astar {
public:
	Map map;
	Node start;
	Node finish;

	//std::priority_queue<Node*, std::vector<Node*>, comp> openList;
	//std::vector<Node> openList;
	pqueue openList;
	std::vector<Node*> closeList;
	bool hasRoute;

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
