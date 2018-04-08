#ifndef ASTAR_TX2_H
#define ASTAR_TX2_H

#include <stdlib.h>  
#include <vector>  
#include <algorithm>

namespace ORB_SLAM2
{
	#define MAPM 61
	#define MAPN 61

	struct localGuide
	{
		int m;	//from 0-360
		int n;	//the distance of this way to next point
	};

	class AStarPoint
	{
	public:
		AStarPoint(int m, int n) :M(m), N(n), G(0), H(0), F(0), T(0), m_pParentPoint(NULL)	
		{
		}

		~AStarPoint()
		{
		}

		void calcF(){			
			this->F = this->H + this->G;
		}
	public:
		int M;
		int N;
		int G;
		int H;
		int F;
		int T;
		AStarPoint* m_pParentPoint;
	};

	typedef std::vector<AStarPoint*> AStarPointList;

	bool CompF(const AStarPoint* pl, const AStarPoint* pr);

	class AStar
	{
	public:
		AStar(int textureMap[MAPM][MAPN]);		

		~AStar();							

		std::vector<localGuide> FindPath(AStarPoint* start, AStarPoint* end);	

		bool CanReach(int m, int n);			

		AStarPoint* inCloseList(int m, int n);			

		AStarPoint* inOpenList(int m, int n);			

		bool CanReach(AStarPoint* start, int m, int n);	

		AStarPointList SurrroundPoints(AStarPoint* point);	

		void clearSurroundMem(AStarPointList tSurroundPoints);	

		AStarPoint* getMinFPoint();		

		void removeFromOpenList(AStarPoint* point);		

		void FoundPoint(AStarPoint* tempStart, AStarPoint* point);	

		void NotFoundPoint(AStarPoint* tempStart, AStarPoint* end, AStarPoint* point);	

		int CalcG(AStarPoint* start, AStarPoint* point);	

		int CalcH(AStarPoint* end, AStarPoint* point);	

	private:
		static const int STEP = 10;		
		static const int OBLIQUE = 14;	
		static const int PUNISH = 15;

		AStarPointList m_listOpen;
		AStarPointList m_listClose;

		int m_textureMap[MAPM][MAPN];		

		std::vector<localGuide>	localGuideList;
	};

}
#endif // ASTAR_H
