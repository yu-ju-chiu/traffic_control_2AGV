#include <iostream>
#include <stdlib.h>
#include <vector>
#include <fstream>

#define METHOD 2

using namespace std;
#define X_MAP 42
#define Y_MAP 27

#define START_Y1 15
#define START_X1 18
#define END_Y1 10
#define END_X1 31

#define START_Y2 18
#define START_X2 21
#define END_Y2 12
#define END_X2 33

class CPoint
{
public:
    CPoint(int x, int y) : X(x), Y(y), G(0), H(0), F(0), m_parentPoint(NULL), time(0){};
    ~CPoint(){};

    void CalcF()
    {
        F = G + H;
    }
    int X;
    int Y;
    int G;
    int H;
    int F;
    int time;
    CPoint *m_parentPoint;
};

class CAStar
{
public:
    // 構造函數
    CAStar(int array[][X_MAP])
    {
        for (int i = 0; i < Y_MAP; ++i)
            for (int j = 0; j < X_MAP; ++j)
            {
                m_array[i][j] = array[i][j];
            }
    }

    CPoint *GetMinFPoint()
    {
        int idx = 0, valueF = -9999;
        for (int i = 0; i < m_openVec.size(); ++i)
        {
            if (m_openVec[i]->F < valueF)
            {
                valueF = m_openVec[i]->F;
                idx = i;
            }
        }
        return m_openVec[idx];
    }

    bool RemoveFromOpenVec(CPoint *point)
    {
        for (POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
        {
            if ((*it)->X == point->X && (*it)->Y == point->Y)
            {
                m_openVec.erase(it);
                return true;
            }
        }
        return false;
    }

    bool canReach(int x, int y)
    {
        return 0 == m_array[x][y];
    }

    bool IsAccessiblePoint(CPoint *point, int x, int y, bool isIgnoreCorner)
    {
        if (!canReach(x, y) || isInCloseVec(x, y))
            return false;
        else
        {
            //可達
            if (abs(x - point->X) + abs(y - point->Y) == 1) // 左右上下
                return true;
            else
            {
                if (canReach(abs(x - 1), y) && canReach(x, abs(y - 1))) // 對角
                    return true;
                else
                    return isIgnoreCorner; //牆角
            }
        }
    }

    std::vector<CPoint *> GetAdjacentPoints(CPoint *point, bool isIgnoreCorner)
    {
        POINTVEC adjacentPoints;
        for (int x = point->X - 1; x <= point->X + 1; ++x)
            for (int y = point->Y - 1; y <= point->Y + 1; ++y)
            {
                if (IsAccessiblePoint(point, x, y, isIgnoreCorner))
                {
                    CPoint *tmpPoint = new CPoint(x, y);
                    adjacentPoints.push_back(tmpPoint);
                }
            }

        return adjacentPoints;
    }

    bool isInOpenVec(int x, int y)
    {
        for (POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
        {
            if ((*it)->X == x && (*it)->Y == y)
                return true;
        }
        return false;
    }

    bool isInCloseVec(int x, int y)
    {
        for (POINTVEC::iterator it = m_closeVec.begin(); it != m_closeVec.end(); ++it)
        {
            if ((*it)->X == x && (*it)->Y == y)
                return true;
        }
        return false;
    }

    void RefreshPoint(CPoint *tmpStart, CPoint *point)
    {
        int valueG = CalcG(tmpStart, point);
        if (valueG < point->G)
        {
            point->m_parentPoint = tmpStart;
            point->G = valueG;
            point->CalcF();
        }
    }

    void NotFoundPoint(CPoint *tmpStart, CPoint *end, CPoint *point)
    {
        point->m_parentPoint = tmpStart;
        point->G = CalcG(tmpStart, point);
        point->G = CalcH(end, point);
        point->CalcF();
        m_openVec.push_back(point);
    }

    int CalcG(CPoint *start, CPoint *point)
    {
        int G = (abs(point->X - start->X) + abs(point->Y - start->Y)) == 2 ? STEP : OBLIQUE;
        int parentG = point->m_parentPoint != NULL ? point->m_parentPoint->G : 0;
        return G + parentG;
    }

    int CalcH(CPoint *end, CPoint *point)
    {
        int step = abs(point->X - end->X) + abs(point->Y - end->Y);
        return STEP * step;
    }

    // 搜索路徑
    CPoint *FindPath(CPoint *start, CPoint *end, bool isIgnoreCorner)
    {
        m_openVec.push_back(start);
        while (0 != m_openVec.size())
        {
            CPoint *tmpStart = GetMinFPoint(); // 取F最小值
            RemoveFromOpenVec(tmpStart);
            m_closeVec.push_back(tmpStart);

            POINTVEC adjacentPoints = GetAdjacentPoints(tmpStart, isIgnoreCorner);
            for (POINTVEC::iterator it = adjacentPoints.begin(); it != adjacentPoints.end(); ++it)
            {
                CPoint *point = *it;
                if (isInOpenVec(point->X, point->Y)) // 在開啟列表
                    RefreshPoint(tmpStart, point);
                //else if(inCloseVec(point))
                //{
                // 檢查節點的g值，如果新計算得到的路徑開銷比該g值低，那麼要重新打開該節點（即重新放入OPEN集）
                //}
                else
                    NotFoundPoint(tmpStart, end, point);
            }
            if (isInOpenVec(end->X, end->Y)) // 目標點已在開啟列表中
            {
                for (int i = 0; i < m_openVec.size(); ++i)
                {
                    if (end->X == m_openVec[i]->X && end->Y == m_openVec[i]->Y)
                        return m_openVec[i];
                }
            }
        }
        return end;
    }

private:
    int m_array[Y_MAP][X_MAP];
    static const int STEP = 10;
    static const int OBLIQUE = 14;

    typedef std::vector<CPoint *> POINTVEC;
    POINTVEC m_openVec;
    POINTVEC m_closeVec;
};

int main()
{
    int num_node = 0;
    // input the map
    ifstream file{"map.txt"};
    if (!file.is_open())
        return -1;

    int array[Y_MAP][X_MAP]{};
    for (int i{}; i != Y_MAP; ++i)
    {
        for (int j{}; j != X_MAP; ++j)
        {
            file >> array[i][j];
        }
    }
    for (int i = 0; i < Y_MAP; i++)
    {
        for (int j = 0; j < X_MAP; j++)
        {
            if (array[i][j] == 1)
                cout << "X";
            else
                cout << " ";
        }
        cout << endl;
    }

    // two AGV setting
    CAStar *pAStar_AGV1 = new CAStar(array);
    CPoint *start_AGV1 = new CPoint(START_Y1, START_X1);
    CPoint *end_AGV1 = new CPoint(END_Y1, END_X1);
    CPoint *point_AGV1 = pAStar_AGV1->FindPath(start_AGV1, end_AGV1, false);
    CPoint *head_AGV1 = new CPoint(START_Y1, START_X1);
    head_AGV1->m_parentPoint = point_AGV1;

    CPoint *start_AGV2 = new CPoint(START_Y2, START_X2);
    CPoint *end_AGV2 = new CPoint(END_Y2, END_X2);
    CAStar *pAStar_AGV2 = new CAStar(array);
    CPoint *point_AGV2 = pAStar_AGV2->FindPath(start_AGV2, end_AGV2, false);
    CPoint *head_AGV2 = new CPoint(START_Y2, START_X2);
    head_AGV2->m_parentPoint = point_AGV2;

    switch (METHOD)
    {
    case 1: //alternative way
    {
        num_node = 0;
        while (point_AGV1 != NULL)
        {
            cout << "AGV1:"
                 << "(" << point_AGV1->X << "," << point_AGV1->Y << ");" << std::endl;
            array[point_AGV1->X][point_AGV1->Y] = 2;
            point_AGV1->time++;
            point_AGV1 = point_AGV1->m_parentPoint;
            num_node++;
        }
        cout << "AGV2_num_node:" << num_node << endl;
        array[START_Y1][START_X1] = 0;
        array[END_Y1][END_X1] = 0;

        CAStar *pAStar_AGV2_alter = new CAStar(array);
        point_AGV2 = pAStar_AGV2_alter->FindPath(start_AGV2, end_AGV2, false);

        num_node = 0;
        while (point_AGV2 != NULL)
        {
            cout << "AGV2:"
                 << "(" << point_AGV2->X << "," << point_AGV2->Y << ");" << endl;
            array[point_AGV2->X][point_AGV2->Y] = 2;
            point_AGV2->time++;
            point_AGV2 = point_AGV2->m_parentPoint;
            num_node++;
        }
        cout << "AGV2_num_node:" << num_node << endl;
        array[START_Y2][START_X2] = 0;
        array[END_Y2][END_X2] = 0;

        for (int i = 0; i < Y_MAP; i++)
        {
            for (int j = 0; j < X_MAP; j++)
            {
                if (array[i][j] == 1)
                    cout << "X";
                else if (array[i][j] == 2)
                    cout << "-";
                else
                    cout << " ";
            }
            cout << endl;
        }

        break;
    }
    case 2: //waiting way
    {
        point_AGV1 = head_AGV1->m_parentPoint;
        point_AGV2 = head_AGV2->m_parentPoint;

        while (point_AGV2->m_parentPoint != NULL && point_AGV1->m_parentPoint != NULL)
        {
            if (point_AGV1->m_parentPoint->X == point_AGV2->m_parentPoint->X && point_AGV1->m_parentPoint->Y == point_AGV2->m_parentPoint->Y)
            {
                CPoint *temp_point = new CPoint(point_AGV2->X, point_AGV2->Y);
                temp_point->m_parentPoint = point_AGV2->m_parentPoint;
                point_AGV2->m_parentPoint = temp_point;
            }
            point_AGV1 = point_AGV1->m_parentPoint;
            point_AGV2 = point_AGV2->m_parentPoint;
        }

        point_AGV1 = head_AGV1->m_parentPoint;
        point_AGV2 = head_AGV2->m_parentPoint;

        num_node = 0;
        while (point_AGV2 != NULL)
        {
            cout << "AGV2:"
                 << "(" << point_AGV2->X << "," << point_AGV2->Y << ");" << endl;
            array[point_AGV2->X][point_AGV2->Y] = 2;
            point_AGV2 = point_AGV2->m_parentPoint;
            num_node++;
        }
        cout << "AGV2_num_node:" << num_node << endl;

        num_node = 0;
        while (point_AGV1 != NULL)
        {
            cout << "AGV1:"
                 << "(" << point_AGV1->X << "," << point_AGV1->Y << ");" << endl;
            array[point_AGV1->X][point_AGV1->Y] = 2;
            point_AGV1 = point_AGV1->m_parentPoint;
            num_node++;
        }
        cout << "AGV1_num_node:" << num_node << endl;

        for (int i = 0; i < Y_MAP; i++)
        {
            for (int j = 0; j < X_MAP; j++)
            {
                if (array[i][j] == 1)
                    cout << "X";
                else if (array[i][j] == 2)
                    cout << "-";
                else
                    cout << " ";
            }
            cout << endl;
        }

        break;
    }
    default: //waiting way
    {
        point_AGV1 = head_AGV1->m_parentPoint;
        point_AGV2 = head_AGV2->m_parentPoint;

        while (point_AGV2->m_parentPoint != NULL && point_AGV1->m_parentPoint != NULL)
        {
            if (point_AGV1->m_parentPoint->X == point_AGV2->m_parentPoint->X && point_AGV1->m_parentPoint->Y == point_AGV2->m_parentPoint->Y)
            {
                CPoint *temp_point = new CPoint(point_AGV2->X, point_AGV2->Y);
                temp_point->m_parentPoint = point_AGV2->m_parentPoint;
                point_AGV2->m_parentPoint = temp_point;
            }
            point_AGV1 = point_AGV1->m_parentPoint;
            point_AGV2 = point_AGV2->m_parentPoint;
        }

        point_AGV1 = head_AGV1->m_parentPoint;
        point_AGV2 = head_AGV2->m_parentPoint;

        num_node = 0;
        while (point_AGV2 != NULL)
        {
            cout << "AGV2:"
                 << "(" << point_AGV2->X << "," << point_AGV2->Y << ");" << endl;
            array[point_AGV2->X][point_AGV2->Y] = 2;
            point_AGV2 = point_AGV2->m_parentPoint;
            num_node++;
        }
        cout << "AGV2_num_node:" << num_node << endl;

        num_node = 0;
        while (point_AGV1 != NULL)
        {
            cout << "AGV1:"
                 << "(" << point_AGV1->X << "," << point_AGV1->Y << ");" << endl;
            array[point_AGV1->X][point_AGV1->Y] = 2;
            point_AGV1 = point_AGV1->m_parentPoint;
            num_node++;
        }
        cout << "AGV1_num_node:" << num_node << endl;
        break;

        for (int i = 0; i < Y_MAP; i++)
        {
            for (int j = 0; j < X_MAP; j++)
            {
                if (array[i][j] == 1)
                    cout << "X";
                else if (array[i][j] == 2)
                    cout << "-";
                else
                    cout << " ";
            }
            cout << endl;
        }
    }
    }
    system("pause");
    return 0;
}
