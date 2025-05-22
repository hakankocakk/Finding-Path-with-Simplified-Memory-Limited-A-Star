#include "sm-astar.h"


int main(int argc, char **argv)
{

	unsigned char pMap[] =
		{
			1,1,1,1,0,1,1,0,1,1,1,1,0,1,1,1, // Satır 0
			1,1,1,0,0,0,0,1,1,0,0,1,1,1,1,1, // Satır 1
			1,1,1,1,0,1,1,1,1,1,1,0,1,1,1,1, // Satır 2
			1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1, // Satır 3
			1,0,1,1,0,1,1,1,0,1,1,0,1,1,1,1, // Satır 4
			1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1, // Satır 5
			1,1,1,1,0,1,0,1,0,0,1,0,1,1,1,1, // Satır 6
			1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1  // Satır 7
		}; // 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 (sütunlar)
		
	int width = 16;
	int height = 8;
	std::vector<int> res;
	res.resize(128);

	int start_x = 0;
	int start_y = 0;
	int tgt_x = 13;
	int tgt_y = 4;

	int rv = FindPath(start_x, start_y, tgt_x, tgt_y, pMap, width, height, res.data(), res.size());

	return rv;
}



