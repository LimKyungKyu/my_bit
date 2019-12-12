#include <iostream>
#include <algorithm>
#include "astar.h"


/* 
   ###################################################################################
   ################################  Node Ŭ���� ����  ################################
   ###################################################################################
*/

myNode::myNode() 
	: xPos(0), yPos(0), fScore(0), gScore(0), hScore(0), parent(nullptr)
{

}

myNode::myNode(unsigned _x, unsigned _y) 
	: xPos(_x), yPos(_y), fScore(0), gScore(0), hScore(0), parent(nullptr)
{

}

myNode::myNode(unsigned _x, unsigned _y, myNode* _parent) 
	: xPos(_x), yPos(_y), fScore(0), gScore(0), hScore(0), parent(_parent)
{

}

// Node ��ǥ�� ��, ������ 1 �ƴϸ� 0
bool myNode::compNode(myNode* node) {
	return (this->xPos == node->xPos && this->yPos == node->yPos);
}

// Node �ʱ�ȭ
void myNode::initNode() {
	xPos = 0;
	yPos = 0;
	fScore = 0;
	gScore = 0;
	hScore = 0;
	parent = nullptr;
}

/*
   ###################################################################################
   ################################   myMap Ŭ���� ����  ################################
   ###################################################################################
*/
myMap::myMap()
	: width(0), height(0), data(nullptr)
{

}

myMap::myMap(int _width, int _height)
	: width(_width), height(_height), data(nullptr)
{

}

// ���� ������ (�̰� ���ϸ� Map Ŭ������ ���ڷ� �޾Ƽ� ó���� �� ���� ���� �Ͼ�� ���߿� �Ҹ��� 2�� �θ�)
myMap::myMap(const myMap& map)
	: width(map.width), height(map.height)
{
	int size = width * height;
	data = new unsigned char[size];

	for (int i = 0; i != size; ++i)
		data[i] = map.data[i];
}

myMap::myMap(int _width, int _height, unsigned char* _mapData)
	: width(_width), height(_height)//, data(_mapData)
{
	int size = width * height;
	data = new unsigned char[size];

	for (int i = 0; i != size; ++i)
		data[i] = _mapData[i];
}

myMap::~myMap() {
	delete data;
}

 //���� ���� ������ ������
myMap& myMap::operator=(const myMap& map) {
	width = map.width;
	height = map.height;
	int size = width * height;
	data = new unsigned char[size];

	for (int i = 0; i != size; ++i)
		data[i] = map.data[i];

	return *this;
}


// �ܺ� �޸� �����ؼ� Map ����.
void myMap::setMap(int _width, int _height, unsigned char* _mapData) {
	width = _width;
	height = _height;
	int size = width * height;
	data = new unsigned char[size];
	for (int i = 0; i != size; ++i)
		data[i] = _mapData[i];
}

int myMap::getMapWidth() {
	return width;
}
int myMap::getMapHeight() {
	return height;
}
unsigned char* myMap::getMapAddr() {
	return data;
}

// �ʿ� �ش��ϴ� ��ǥ �� ��ȯ
int myMap::getMapData(int _x, int _y) {
	return data[_y * width + _x];
}

// �ʿ� �ش��ϴ� ��ǥ ���� val �� ����
void myMap::setMapData(int _x, int _y, unsigned char val) {
	data[_y * width + _x] = val;
}



// �ʿ� ��ֹ� ��ġ
void myMap::setObject(int _x, int _y) {
	data[_y * width + _x] = OBJECT;
}


// ��ü �� ���
void myMap::printMap() {
	printf("��ü �� ���: %d x %d\n\n", width, height);
	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			printf("%3d ", data[j * width + i]);
		}
		puts("");
	}
}

// �� �� �ִ� ��ǥ���� �ƴ���. �� �� ������ 1, �ƴϸ� 0
bool myMap::isWalkable(int _x, int _y) {
	if (_x >= width || _y >= height)
		return false;

	return (data[_y * width + _x] != OBJECT) ? true : false;
}


// ���ڷ� ���� Node �� parent�� �����ؼ� PATH �׷���.
void myMap::setPath(myNode* fin) {
	while (fin->parent != nullptr) {
		//printf("��ǥ: [%d, %d], F: %d, G: %d, H: %d\n", fin->xPos, fin->yPos, fin->fScore, fin->gScore, fin->hScore);
		setMapData(fin->xPos, fin->yPos, PATH);
		fin = fin->parent;
	}
	//printf("��ǥ: [%d, %d], F: %d, G: %d, H: %d\n", fin->xPos, fin->yPos, fin->fScore, fin->gScore, fin->hScore);
	setMapData(fin->xPos, fin->yPos, PATH);
}


/*
   ###################################################################################
   ###############################  pqueue Ŭ���� ����  ###############################
   ###################################################################################
*/

pqueue::~pqueue() {
	for (std::list<myNode*>::iterator iter = que.begin(); iter != que.end();) {
		delete (*iter);
		iter = que.erase(iter);
	}
}

// ����
void pqueue::push(myNode* const node) {
	que.push_back(node);
}

bool myComp(myNode* first, myNode* second) {
	return (first->fScore > second->fScore);
}

// ���� (fScore ����, ��������)
void pqueue::sorting() {
	que.sort(myComp);
}

// �ش� ��ǥ�� �ߺ� ��� �ִ��� Ȯ��, �ִٸ� g �� ���ؼ� �� ������ g �� �� f �� ����
void pqueue::pushNode(myNode* const newNode) {
	for (auto riter = que.rbegin(); riter != que.rend(); ++riter) {
		if ((*riter)->compNode(newNode)) { // �ߺ��Ǵ� ��尡 �ִ���, �ִٸ�
			if ((*riter)->gScore > newNode->gScore) { // �� ��尡 g���� �� �۴ٸ� g�� ���� �� f�� �ٽ� ���
				(*riter)->gScore = newNode->gScore;
				(*riter)->fScore = (*riter)->gScore + (*riter)->fScore;
				(*riter)->parent = newNode->parent;
			}
			else {	// �� ��尡 g���� ���ų� �� ������ ����
				delete newNode;
				return;
			}
		}
	}	// �ߺ���� ������
	push(newNode);
}

// �� �ڿ� (f���� ���� ���� ���) ����
void pqueue::pop() {
	if (!empty()) {
		que.pop_back();
	}
	else {
		std::cerr << "pqueue is empty\n";
	}
}

// �� �ڿ� (f���� ���� ���� ���) ��� ����
myNode* pqueue::top() {
	if (!empty())
		return que.back();
	else
		return nullptr;
}

// �������?
bool pqueue::empty() {
	return que.empty();
}

// ť �ȿ� ����.
size_t pqueue::getSize() {
	return que.size();
}


/*
   ###################################################################################
   ###############################  Astar Ŭ���� ����  ################################
   ###################################################################################
*/
Astar::Astar()
	: map(), hasRoute(false)
{

}

Astar::Astar(const myMap& _map, myNode _start, myNode _finish)
	: map(_map), start(_start), finish(_finish), hasRoute(false)
{

}

Astar::~Astar() {

}

// �ɹ������� �ʱ�ȭ
void Astar::init() {
	while (!openList.empty()) {
		openList.pop();
	}
	closeList.clear();
	start.initNode();
	finish.initNode();
	hasRoute = false;
}

// ���� ��� ����
void Astar::setStart(unsigned int _x, unsigned int _y) {
	//start->initNode();
	if (!map.isWalkable(_x, _y)) {
		std::cerr << "This point is object\n";
		return;
	}
	start.xPos = _x;
	start.yPos = _y;
}

// ��ǥ ��� ����
void Astar::setFinish(unsigned int _x, unsigned int _y) {
	//start->initNode();
	if (!map.isWalkable(_x, _y)) {
		std::cerr << "This point is object\n";
		return;
	}
	finish.xPos = _x;
	finish.yPos = _y;
}

// �� ����
void Astar::setMap(int _x, int _y, unsigned char* _map_data) {
	map.setMap(_x, _y, _map_data);
}
void Astar::setMap(const myMap& _map) {
	map = _map;
}

// H �� ���
int Astar::calcH(myNode* const from, myNode* const to) {
	int temp = 0;
	temp += std::abs((int)(to->xPos - from->xPos));
	temp += std::abs((int)(to->yPos - from->yPos));
	return temp * 10;
}

// G �� ���
int Astar::calcG(myNode* const from, int diag) {
	if (diag) //�밢���� ��
		return from->parent->gScore + 14;
	else
		return from->parent->gScore + 10;
}

// F �� ���
void Astar::calcF(myNode* temp, int diag) {
	//if (diag)	// �밢���� ��
	//	temp->gScore = temp->parent->gScore + 14;
	//else		// ����, ������ ��
	//	temp->gScore = temp->parent->gScore + 10;
	temp->gScore = calcG(temp, diag);
	temp->hScore = calcH(temp, &finish);
	temp->fScore = temp->gScore + temp->hScore;
}

void Astar::setObjectToMap(unsigned int _x, unsigned int _y) {
	map.setObject(_x, _y);
}

void Astar::printMapAll() {
	map.printMap();
}

void Astar::setPathToMap(myNode* fin) {
	map.setPath(fin);
}


// ���� ��Ͽ� ��尡 �ִ���? ������ 1, ������ 0
bool Astar::isInCloseList(myNode* node) {
	for (auto riter = closeList.rbegin(); riter != closeList.rend(); ++riter) {
		if ((*riter)->compNode(node))
			return true;
	}
	return false;
}

// ���� ��忡 ������ ��带 �˻��� ���� ��Ͽ� ����
bool Astar::checkAround(myNode* now, bool allowDiagonal, bool crossCorner) {
	bool isThere = false;
	bool s0 = false, s1 = false, s2 = false, s3 = false;
	bool d0 = false, d1 = false, d2 = false, d3 = false;

	if (map.isWalkable(now->xPos, now->yPos - 1)) { // ��(��) üũ
		myNode* tmp = new myNode(now->xPos, now->yPos - 1, now); // ���� ��� ����
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s0 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos + 1, now->yPos)) { // ��(��) üũ
		myNode* tmp = new myNode(now->xPos + 1, now->yPos, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s1 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos, now->yPos + 1)) { // ��(��) üũ
		myNode* tmp = new myNode(now->xPos, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s2 = true;
			isThere = true;
		}
		else
			delete tmp;
	}

	if (map.isWalkable(now->xPos - 1, now->yPos)) { // ��(��) üũ
		myNode* tmp = new myNode(now->xPos - 1, now->yPos, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 0);
			openList.pushNode(tmp);
			s3 = true;
			isThere = true;
		}
		else
			delete tmp;
	}
	if (!allowDiagonal)	// �밢 �̵� �Ұ��� ��ȯ
		return isThere;

	if (!crossCorner) {	// �ڳʿ��� �밢 �̵� x
		d0 = s3 && s0;
		d1 = s0 && s1;
		d2 = s1 && s2;
		d3 = s2 && s3;
	}
	else {	// �ڳʿ��� �밢 �̵� o
		d0 = s3 || s0;
		d1 = s0 || s1;
		d2 = s1 || s2;
		d3 = s2 || s3;
	}

	if (d0 && map.isWalkable(now->xPos - 1, now->yPos - 1)) { // �ϼ�(��) üũ
		myNode* tmp = new myNode(now->xPos - 1, now->yPos - 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d1 && map.isWalkable(now->xPos + 1, now->yPos - 1)) { // �ϵ�(��) üũ
		myNode* tmp = new myNode(now->xPos + 1, now->yPos - 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d2 && map.isWalkable(now->xPos + 1, now->yPos + 1)) { // ����(��) üũ
		myNode* tmp = new myNode(now->xPos + 1, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	if (d3 && map.isWalkable(now->xPos - 1, now->yPos + 1)) { // ����(��) üũ
		myNode* tmp = new myNode(now->xPos - 1, now->yPos + 1, now);
		if (!isInCloseList(tmp)) {
			calcF(tmp, 1);
			openList.pushNode(tmp);
			isThere = true;
		}
		else
			delete tmp;
	}

	return isThere;
}

// ���۳��� ��ǥ��� �����ؼ� �ִܰŸ� ��� ã��
myNode* Astar::findRoute(unsigned int fromX, unsigned int fromY, unsigned int toX, unsigned int toY) {
	// 0. �ʱ�ȭ
	init();
	setStart(fromX, fromY);
	setFinish(toX, toY);

	// 1. ���۳�忡�� ������ & �����ִ� ���� ������Ͽ� �ֱ�
	closeList.push_back(&start);
	checkAround(&start, DIAGONAL, CROSSCORNER);

	myNode* now;
	while (1) {
		// 2. ������Ͽ��� ���� ���� F ��� ��带 ������� ���� �� ���� ��忡 ����
		openList.sorting();
		now = openList.top();
		if (now == nullptr) { // ������Ͽ� ���̻� ��尡 ���ٸ�
			hasRoute = false; // ��ã�� ����
			break;
		}
		openList.pop();
		closeList.push_back(now);
		if (isInCloseList(&finish)) { // ������Ͽ� ��ǥ ��尡 �ִٸ�
			hasRoute = true; // ��ã�� ����
			break;
		}

		// 3. �����忡�� ������ & �����ִ� ���� ������Ͽ� �ֱ�
		checkAround(now, DIAGONAL, CROSSCORNER);
	}
	return closeList.back();
}