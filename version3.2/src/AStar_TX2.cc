#include "AStar_TX2.h"
#include <stdlib.h>   
#include <vector>  
#include <algorithm>
#include <string.h>

namespace ORB_SLAM2
{

bool CompF(const AStarPoint* pl, const AStarPoint* pr)	
{
	return pl->F < pr->F;
}

AStar::AStar(int textureMap[MAPM][MAPN])
{
	memcpy(m_textureMap, textureMap, sizeof(int)*MAPM*MAPN);
}

AStar::~AStar()
{
	AStarPointList::iterator _iter;

	for (_iter = m_listOpen.begin(); _iter != m_listOpen.end(); ++_iter)
	{
		AStarPoint *p = *_iter;

		delete p;			
	}

	for (_iter = m_listClose.begin(); _iter != m_listClose.end(); ++_iter)
	{
		AStarPoint *p = *_iter;

		delete p;			
	}
}

int AStar::CalcH(AStarPoint* end, AStarPoint* point)	
{
	int step = abs(abs(point->M - end->M) - abs(point->N - end->N))*STEP + std::min(abs(point->M - end->M), abs(point->N - end->N))*OBLIQUE;
	return step;
}

int AStar::CalcG(AStarPoint* start, AStarPoint* point)	
{
	int G = (abs(point->M - start->M) + abs(point->N - start->N)) == 2 ? OBLIQUE : STEP;	
	int parentG = start->G;				
	return G + parentG;	
}

void AStar::FoundPoint(AStarPoint* tempStart, AStarPoint* point)	
{
	int G = CalcG(tempStart, point);	

	int T = (point->M - tempStart->M) * 10 + (point->N - tempStart->N);
	if (T != tempStart->T)
		G = G + PUNISH;		

	if (G < point->G)					
	{
		point->m_pParentPoint = tempStart;	
		point->G = G;
		point->T = T;
	
		point->calcF();
	}
}

void AStar::NotFoundPoint(AStarPoint* tempStart, AStarPoint* end, AStarPoint* point)	
{
	point->m_pParentPoint = tempStart;
	point->G = CalcG(tempStart, point);

	point->T = (point->M - tempStart->M) * 10 + (point->N - tempStart->N);
	if (point->T != tempStart->T)
		point->G = point->G + PUNISH;		

	point->H = CalcH(end, point);
	point->calcF();
	m_listOpen.push_back(point);
}

void AStar::removeFromOpenList(AStarPoint* point)		
{
	AStarPointList::iterator _iter;	
	for (_iter = m_listOpen.begin(); _iter != m_listOpen.end(); ++_iter)			
	{
		if (point == *_iter)								
		{
			m_listOpen.erase(_iter);							
			break;
		}
	}
}

AStarPoint* AStar::getMinFPoint()		
{

	AStarPointList tempList;

	for (int i = 0; i < (int)m_listOpen.size(); i++)		
	{
		tempList.push_back(m_listOpen[i]);
	}
	sort(tempList.begin(), tempList.end(), CompF);	

	if (tempList.size())		
	{
		return tempList[0];
	}
	else
		return NULL;			
}

bool AStar::CanReach(int m, int n)			
{
	if (m < MAPM && m >= 0 && n < MAPN && n >= 0)
		return m_textureMap[m][n] == 0;
	else
		return false;
}

AStarPoint* AStar::inCloseList(int m, int n)			
{
	AStarPointList::iterator _iter;
	for (_iter = m_listClose.begin(); _iter != m_listClose.end(); ++_iter)	
	{
		AStarPoint* temp = *_iter;
		if (temp->M == m && temp->N == n)
			return temp;
	}
	return NULL;
}

AStarPoint* AStar::inOpenList(int m, int n)			
{
	AStarPointList::iterator _iter;
	for (_iter = m_listOpen.begin(); _iter != m_listOpen.end(); ++_iter)
	{
		AStarPoint* temp = *_iter;		
		if (temp->M == m && temp->N == n)
			return temp;
	}
	return NULL;
}

bool AStar::CanReach(AStarPoint* start, int m, int n)	
{
	if (!CanReach(m, n) || inCloseList(m, n))		
		return false;
	else                           
	{
		return true;
	}
}

AStarPointList AStar::SurrroundPoints(AStarPoint* point)	
{
	AStarPointList surroundPoints;

	for (int m = point->M - 1; m <= point->M + 1; m++)	
	for (int n = point->N - 1; n <= point->N + 1; n++)
	{
		if (CanReach(point, m, n))
		{
			AStarPoint *p = new AStarPoint(m, n);

			surroundPoints.push_back(p);			
		}
	}
	return surroundPoints;
}

void AStar::clearSurroundMem(AStarPointList tSurroundPoints)	
{
	AStarPointList::iterator _iter;

	for (_iter = tSurroundPoints.begin(); _iter != tSurroundPoints.end(); ++_iter)
	{
		AStarPoint *p = *_iter;

		delete p;			
	}
}

std::vector<localGuide> AStar::FindPath(AStarPoint* start, AStarPoint* end)	
{
	AStarPoint *tstart = new AStarPoint(start->M, start->N);

	m_listOpen.push_back(tstart); 

	while (m_listOpen.size())
	{
		AStarPoint* tempStart = getMinFPoint(); 

		removeFromOpenList(tempStart);  
		m_listClose.push_back(tempStart); 

		AStarPointList surroundPoints = SurrroundPoints(tempStart); 

		AStarPointList::iterator _iter;
 
		for (_iter = surroundPoints.begin(); _iter != surroundPoints.end(); ++_iter)	
		{
			AStarPoint *point = *_iter;
			AStarPoint *Opoint = inOpenList(point->M, point->N);

			if (Opoint != NULL)
			{
				FoundPoint(tempStart, Opoint);  
				delete point;	
			}
			else
				NotFoundPoint(tempStart, end, point); 
		}

		AStarPoint* tend = inOpenList(end->M, end->N);
		if (tend != NULL)
		{
			localGuide	tGuide;
			
			tGuide.m = tend->M;
			tGuide.n = tend->N;
			localGuideList.push_back(tGuide);

			while (tend->m_pParentPoint != NULL)	
			{
				if (tend->T != tend->m_pParentPoint->T)	
				{
					tGuide.m = tend->m_pParentPoint->M;
					tGuide.n = tend->m_pParentPoint->N;
					localGuideList.push_back(tGuide);
				}
				
				tend = tend->m_pParentPoint;
			}
			return localGuideList;
		}
	}
	return localGuideList;	
}

}
