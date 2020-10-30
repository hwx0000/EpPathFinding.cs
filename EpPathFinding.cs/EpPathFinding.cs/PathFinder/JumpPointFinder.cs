/*! 
@file JumpPointFinder.cs
@author Woong Gyu La a.k.a Chris. <juhgiyo@gmail.com>
		<http://github.com/juhgiyo/eppathfinding.cs>
@date July 16, 2013
@brief Jump Point Search Algorithm Interface
@version 2.0

@section LICENSE

The MIT License (MIT)

Copyright (c) 2013 Woong Gyu La <juhgiyo@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

@section DESCRIPTION

An Interface for the Jump Point Search Algorithm Class.

*/
using C5;
using System;
using System.Collections.Generic;
using System.Collections;


namespace EpPathFinding.cs
{
    public enum IterationType
    {
        LOOP,
        RECURSIVE,
    };

    public enum EndNodeUnWalkableTreatment
    {
        ALLOW,
        DISALLOW
    };


    public class JumpPointParam : ParamBase
    {
        [System.Obsolete("This constructor is deprecated, please use the Constructor with EndNodeUnWalkableTreatment and DiagonalMovement instead.")]
        public JumpPointParam(BaseGrid iGrid, GridPos iStartPos, GridPos iEndPos, bool iAllowEndNodeUnWalkable = true, bool iCrossCorner = true, bool iCrossAdjacentPoint = true, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iStartPos, iEndPos, Util.GetDiagonalMovement(iCrossCorner, iCrossAdjacentPoint), iMode)
        {
            CurEndNodeUnWalkableTreatment = iAllowEndNodeUnWalkable ? EndNodeUnWalkableTreatment.ALLOW : EndNodeUnWalkableTreatment.DISALLOW;
            openList = new IntervalHeap<Node>();

            CurIterationType = IterationType.LOOP;
        }

        [System.Obsolete("This constructor is deprecated, please use the Constructor with EndNodeUnWalkableTreatment and DiagonalMovement instead.")]
        public JumpPointParam(BaseGrid iGrid, bool iAllowEndNodeUnWalkable = true, bool iCrossCorner = true, bool iCrossAdjacentPoint = true, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, Util.GetDiagonalMovement(iCrossCorner, iCrossAdjacentPoint), iMode)
        {
            CurEndNodeUnWalkableTreatment = iAllowEndNodeUnWalkable ? EndNodeUnWalkableTreatment.ALLOW : EndNodeUnWalkableTreatment.DISALLOW;

            openList = new IntervalHeap<Node>();
            CurIterationType = IterationType.LOOP;
        }

        [System.Obsolete("This constructor is deprecated, please use the Constructor with EndNodeUnWalkableTreatment and DiagonalMovement instead.")]
        public JumpPointParam(BaseGrid iGrid, GridPos iStartPos, GridPos iEndPos, bool iAllowEndNodeUnWalkable = true, DiagonalMovement iDiagonalMovement = DiagonalMovement.Always, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iStartPos, iEndPos, iDiagonalMovement, iMode)
        {

            CurEndNodeUnWalkableTreatment = iAllowEndNodeUnWalkable ? EndNodeUnWalkableTreatment.ALLOW : EndNodeUnWalkableTreatment.DISALLOW;
            openList = new IntervalHeap<Node>();

            CurIterationType = IterationType.LOOP;
        }

        [System.Obsolete("This constructor is deprecated, please use the Constructor with EndNodeUnWalkableTreatment and DiagonalMovement instead.")]
        public JumpPointParam(BaseGrid iGrid, bool iAllowEndNodeUnWalkable = true, DiagonalMovement iDiagonalMovement = DiagonalMovement.Always, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iDiagonalMovement, iMode)
        {
            CurEndNodeUnWalkableTreatment = iAllowEndNodeUnWalkable ? EndNodeUnWalkableTreatment.ALLOW : EndNodeUnWalkableTreatment.DISALLOW;

            openList = new IntervalHeap<Node>();
            CurIterationType = IterationType.LOOP;
        }


        public JumpPointParam(BaseGrid iGrid, GridPos iStartPos, GridPos iEndPos, EndNodeUnWalkableTreatment iAllowEndNodeUnWalkable = EndNodeUnWalkableTreatment.ALLOW, DiagonalMovement iDiagonalMovement = DiagonalMovement.Always, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iStartPos, iEndPos, iDiagonalMovement, iMode)
        {

            CurEndNodeUnWalkableTreatment = iAllowEndNodeUnWalkable;
            openList = new IntervalHeap<Node>();

            CurIterationType = IterationType.LOOP;
        }

        public JumpPointParam(BaseGrid iGrid, EndNodeUnWalkableTreatment iAllowEndNodeUnWalkable = EndNodeUnWalkableTreatment.ALLOW, DiagonalMovement iDiagonalMovement = DiagonalMovement.Always, HeuristicMode iMode = HeuristicMode.EUCLIDEAN)
            : base(iGrid, iDiagonalMovement, iMode)
        {
            CurEndNodeUnWalkableTreatment = iAllowEndNodeUnWalkable;

            openList = new IntervalHeap<Node>();
            CurIterationType = IterationType.LOOP;
        }


        public JumpPointParam(JumpPointParam b) : base(b)
        {
            m_heuristic = b.m_heuristic;
            CurEndNodeUnWalkableTreatment = b.CurEndNodeUnWalkableTreatment;

            openList = new IntervalHeap<Node>();
            openList.AddAll(b.openList);

            CurIterationType = b.CurIterationType;
        }


        internal override void _reset(GridPos iStartPos, GridPos iEndPos, BaseGrid iSearchGrid = null)
        {
            openList = new IntervalHeap<Node>();
            //openList.Clear();
        }

        [System.Obsolete("This property is deprecated, please use the CurEndNodeUnWalkableTreatment instead.")]
        public bool AllowEndNodeUnWalkable
        {
            get
            {
                return CurEndNodeUnWalkableTreatment == EndNodeUnWalkableTreatment.ALLOW;
            }
            set
            {
                CurEndNodeUnWalkableTreatment = value ? EndNodeUnWalkableTreatment.ALLOW : EndNodeUnWalkableTreatment.DISALLOW;
            }
        }

        [System.Obsolete("This property is deprecated, please use the CurIterationType instead.")]
        public bool UseRecursive
        {
            get
            {
                return CurIterationType == IterationType.RECURSIVE;
            }
            set
            {
                CurIterationType = value ? IterationType.RECURSIVE : IterationType.LOOP;
            }
        }

        public EndNodeUnWalkableTreatment CurEndNodeUnWalkableTreatment
        {
            get;
            set;
        }
        public IterationType CurIterationType
        {
            get;
            set;
        }

        //public List<Node> openList;
        public IntervalHeap<Node> openList;

    }
    public class JumpPointFinder
    {
        public static List<GridPos> GetFullPath(List<GridPos> routeFound)
        {
            if (routeFound == null)
                return null;
            List<GridPos> consecutiveGridList = new List<GridPos>();
            if (routeFound.Count > 1)
                consecutiveGridList.Add(new GridPos(routeFound[0]));
            for (int routeTrav = 0; routeTrav < routeFound.Count - 1; routeTrav++)
            {
                GridPos fromGrid = new GridPos(routeFound[routeTrav]);
                GridPos toGrid = routeFound[routeTrav + 1];
                int dX = toGrid.x - fromGrid.x;
                int dY = toGrid.y - fromGrid.y;

                int nDX = 0;
                int nDY = 0;
                if (dX != 0)
                {
                    nDX = (dX / Math.Abs(dX));
                }
                if (dY != 0)
                {
                    nDY = (dY / Math.Abs(dY));
                }

                while (fromGrid != toGrid)
                {
                    fromGrid.x += nDX;
                    fromGrid.y += nDY;
                    consecutiveGridList.Add(new GridPos(fromGrid));
                }

            }
            return consecutiveGridList;
        }
        public static List<GridPos> FindPath(JumpPointParam iParam)
        {

            IntervalHeap<Node> tOpenList = iParam.openList;
            Node tStartNode = iParam.StartNode;
            Node tEndNode = iParam.EndNode;
            Node tNode;
            bool revertEndNodeWalkable = false;

            // set the `g` and `f` value of the start node to be 0
            tStartNode.startToCurNodeLen = 0;
            tStartNode.heuristicStartToEndLen = 0;

            // push the start node into the open list
            tOpenList.Add(tStartNode);
            tStartNode.isOpened = true;

            if (iParam.CurEndNodeUnWalkableTreatment == EndNodeUnWalkableTreatment.ALLOW && !iParam.SearchGrid.IsWalkableAt(tEndNode.x, tEndNode.y))
            {
                iParam.SearchGrid.SetWalkableAt(tEndNode.x, tEndNode.y, true);
                revertEndNodeWalkable = true;
            }

            // while the open list is not empty
            while (tOpenList.Count > 0)
            {
                // pop the position of node which has the minimum `f` value.
                tNode = tOpenList.DeleteMin();
                tNode.isClosed = true;

                if (tNode.Equals(tEndNode))
                {
                    if (revertEndNodeWalkable)
                    {
                        iParam.SearchGrid.SetWalkableAt(tEndNode.x, tEndNode.y, false);
                    }
                    return Node.Backtrace(tNode); // rebuilding path
                }

                // 寻找当前节点的继任节点
                identifySuccessors(iParam, tNode);
            }

            if (revertEndNodeWalkable)
            {
                iParam.SearchGrid.SetWalkableAt(tEndNode.x, tEndNode.y, false);
            }

            // fail to find the path
            return new List<GridPos>();
        }

        // 根据父节点的方向，寻找跳点，找到了就加入OpenList里
        private static void identifySuccessors(JumpPointParam iParam, Node iNode)
        {
            HeuristicDelegate tHeuristic = iParam.HeuristicFunc;
            IntervalHeap<Node> tOpenList = iParam.openList;
            int tEndX = iParam.EndNode.x;
            int tEndY = iParam.EndNode.y;
            GridPos tNeighbor;
            GridPos tJumpPoint;
            Node tJumpNode;

            // 寻找iNode的邻居节点，包括自然邻居和强迫邻居
            List<GridPos> tNeighbors = findNeighbors(iParam, iNode);
            for (int i = 0; i < tNeighbors.Count; i++)
            {
                tNeighbor = tNeighbors[i];
                // 根据参数类型的不同，有两种方式的跳点寻找函数
                // 第一种是递归型，会沿着方向一直到寻找跳点，或者不存在跳点，才return
                if (iParam.CurIterationType == IterationType.RECURSIVE)
                    tJumpPoint = jump(iParam, tNeighbor.x, tNeighbor.y, iNode.x, iNode.y);
                // 第二种是循环型，默认好像是第二种
                else
                    tJumpPoint = jumpLoop(iParam, tNeighbor.x, tNeighbor.y, iNode.x, iNode.y);
                if (tJumpPoint != null)
                {
                    tJumpNode = iParam.SearchGrid.GetNodeAt(tJumpPoint.x, tJumpPoint.y);
                    if (tJumpNode == null)
                    {
                        if (iParam.EndNode.x == tJumpPoint.x && iParam.EndNode.y == tJumpPoint.y)
                            tJumpNode = iParam.SearchGrid.GetNodeAt(tJumpPoint);
                    }
                    if (tJumpNode.isClosed)
                    {
                        continue;
                    }
                    // include distance, as parent may not be immediately adjacent:
                    float tCurNodeToJumpNodeLen = tHeuristic(Math.Abs(tJumpPoint.x - iNode.x), Math.Abs(tJumpPoint.y - iNode.y));
                    float tStartToJumpNodeLen = iNode.startToCurNodeLen + tCurNodeToJumpNodeLen; // next `startToCurNodeLen` value

                    if (!tJumpNode.isOpened || tStartToJumpNodeLen < tJumpNode.startToCurNodeLen)
                    {
                        tJumpNode.startToCurNodeLen = tStartToJumpNodeLen;
                        tJumpNode.heuristicCurNodeToEndLen = (tJumpNode.heuristicCurNodeToEndLen == null ? tHeuristic(Math.Abs(tJumpPoint.x - tEndX), Math.Abs(tJumpPoint.y - tEndY)) : tJumpNode.heuristicCurNodeToEndLen);
                        tJumpNode.heuristicStartToEndLen = tJumpNode.startToCurNodeLen + tJumpNode.heuristicCurNodeToEndLen.Value;
                        tJumpNode.parent = iNode;

                        if (!tJumpNode.isOpened)
                        {
                            tOpenList.Add(tJumpNode);
                            tJumpNode.isOpened = true;
                        }
                    }
                }
            }
        }

        // Snapshot用来每一次查找路点的情况
        private class JumpSnapshot
        {
            public int iX;
            public int iY;
            public int iPx;
            public int iPy;
            public int tDx;
            public int tDy;
            public int stage;
            public JumpSnapshot()
            {

                iX = 0;
                iY = 0;
                iPx = 0;
                iPy = 0;
                tDx = 0;
                tDy = 0;
                stage = 0;
            }
        }

        // 由于jump函数可能会沿着一个方向递归很多次，所以这个函数进行了改进
        // 通过创建JumpSnapshot对象，把每一次节点间的寻路信息记录下来
        // 然后放到Stack中，不断循环
        private static GridPos jumpLoop(JumpPointParam iParam, int iX, int iY, int iPx, int iPy)
        {
            // 这里的retVal应该是作为记录值，保存斜向时，出现的第三种情况(也就是竖直水平方向存在跳点)的跳点
            GridPos retVal = null;
            // 用stack作为Jump过程中的OpenList
            Stack<JumpSnapshot> stack = new Stack<JumpSnapshot>();

            JumpSnapshot currentSnapshot = new JumpSnapshot();
            JumpSnapshot newSnapshot = null;
            currentSnapshot.iX = iX;
            currentSnapshot.iY = iY;
            currentSnapshot.iPx = iPx;
            currentSnapshot.iPy = iPy;
            currentSnapshot.stage = 0;

            stack.Push(currentSnapshot);
            while (stack.Count != 0)
            {
                currentSnapshot = stack.Pop();
                switch (currentSnapshot.stage)
                {
                    case 0:
                        // 如果当前位置能不能走
                        if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY))
                        {
                            retVal = null;
                            continue;
                        }
                        // 找到终点
                        else if (iParam.SearchGrid.GetNodeAt(currentSnapshot.iX, currentSnapshot.iY).Equals(iParam.EndNode))
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }

                        currentSnapshot.tDx = currentSnapshot.iX - currentSnapshot.iPx;
                        currentSnapshot.tDy = currentSnapshot.iY - currentSnapshot.iPy;
                        if (iParam.DiagonalMovement == DiagonalMovement.Always || iParam.DiagonalMovement == DiagonalMovement.IfAtLeastOneWalkable)
                        {
                            // 1.首先判断是否存在强迫邻居，若存在，说明找到跳点，给retVal赋值
                            // check for forced neighbors
                            // along the diagonal
                            
                            // 斜向判断强迫邻居
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                            {
                                if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - currentSnapshot.tDy) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - currentSnapshot.tDy)))
                                {
                                    retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                                    continue;
                                }
                            }
                            // horizontally/vertically
                            // 水平或竖直向判断强迫邻居
                            else
                            {
                                if (currentSnapshot.tDx != 0)
                                {
                                    // moving along x
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                                        continue;
                                    }
                                }
                                else
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY + currentSnapshot.tDy) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY + currentSnapshot.tDy) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                                        continue;
                                    }
                                }
                            }

                            // 2. 判断完强迫邻居后，则开始类似递归的处理

                            // when moving diagonally, must check for vertical/horizontal jump points
                            // 这里的斜向点记为Node, 则Node作为一个分支加进了stack里，斜向拓展为了三个分支
                            // stage0阶段下添加水平方向的snapshot
                            // stage1阶段下添加垂直方向的snapshot
                            // stage2阶段下添加斜方向的snapshot
                            // 如果在分支里找到了跳点，则用retVal记录下来
                            // 这里的Node属于第三种情况的跳点，所以要用Node去覆盖分支找到的retVal
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                            {
                                currentSnapshot.stage = 1;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }

                            // moving diagonally, must make sure one of the vertical/horizontal
                            // neighbors is open to allow the path
                            if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY) || iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy))
                            {
                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                // 对于垂直或水平向的查找，创建新的snapshot加入到stack中
                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                        }
                        else if (iParam.DiagonalMovement == DiagonalMovement.OnlyWhenNoObstacles)
                        {
                            // check for forced neighbors
                            // along the diagonal
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                            {
                                if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY)) ||
                                    (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY + currentSnapshot.tDy) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy)))
                                {
                                    retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                                    continue;
                                }
                            }
                            // horizontally/vertically
                            else
                            {
                                if (currentSnapshot.tDx != 0)
                                {
                                    // moving along x
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY + 1)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY - 1) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - currentSnapshot.tDx, currentSnapshot.iY - 1)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                                        continue;
                                    }
                                }
                                else
                                {
                                    if ((iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + 1, currentSnapshot.iY - currentSnapshot.tDy)) ||
                                        (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY) && !iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX - 1, currentSnapshot.iY - currentSnapshot.tDy)))
                                    {
                                        retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                                        continue;
                                    }
                                }
                            }


                            // when moving diagonally, must check for vertical/horizontal jump points
                            if (currentSnapshot.tDx != 0 && currentSnapshot.tDy != 0)
                            {
                                currentSnapshot.stage = 3;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }

                            // moving diagonally, must make sure both of the vertical/horizontal
                            // neighbors is open to allow the path
                            if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy))
                            {
                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                                newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                        }
                        else // if(iParam.DiagonalMovement == DiagonalMovement.Never)
                        {
                            if (currentSnapshot.tDx != 0)
                            {
                                // moving along x
                                if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY))
                                {
                                    retVal = new GridPos(iX, iY);
                                    continue;
                                }
                            }
                            else
                            {
                                if (!iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy))
                                {
                                    retVal = new GridPos(iX, iY);
                                    continue;
                                }
                            }

                            //  must check for perpendicular jump points
                            if (currentSnapshot.tDx != 0)
                            {
                                currentSnapshot.stage = 5;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX;
                                newSnapshot.iY = currentSnapshot.iY + 1;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;
                            }
                            else // tDy != 0
                            {
                                currentSnapshot.stage = 6;
                                stack.Push(currentSnapshot);

                                newSnapshot = new JumpSnapshot();
                                newSnapshot.iX = currentSnapshot.iX + 1;
                                newSnapshot.iY = currentSnapshot.iY;
                                newSnapshot.iPx = currentSnapshot.iX;
                                newSnapshot.iPy = currentSnapshot.iY;
                                newSnapshot.stage = 0;
                                stack.Push(newSnapshot);
                                continue;

                            }
                        }
                        retVal = null;
                        break;
                    case 1:
                        // case != 0 都说明这是斜向的拓展节点，这里是为了override第三种跳点的分解方向上得到的跳点
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }

                        currentSnapshot.stage = 2;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 2:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }

                        // moving diagonally, must make sure one of the vertical/horizontal
                        // neighbors is open to allow the path
                        if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY) || iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy))
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        retVal = null;
                        break;
                    case 3:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }

                        currentSnapshot.stage = 4;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 4:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }

                        // moving diagonally, must make sure both of the vertical/horizontal
                        // neighbors is open to allow the path
                        if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy))
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        retVal = null;
                        break;
                    case 5:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }
                        currentSnapshot.stage = 7;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX;
                        newSnapshot.iY = currentSnapshot.iY - 1;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 6:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }
                        currentSnapshot.stage = 7;
                        stack.Push(currentSnapshot);

                        newSnapshot = new JumpSnapshot();
                        newSnapshot.iX = currentSnapshot.iX - 1;
                        newSnapshot.iY = currentSnapshot.iY;
                        newSnapshot.iPx = currentSnapshot.iX;
                        newSnapshot.iPy = currentSnapshot.iY;
                        newSnapshot.stage = 0;
                        stack.Push(newSnapshot);
                        break;
                    case 7:
                        if (retVal != null)
                        {
                            retVal = new GridPos(currentSnapshot.iX, currentSnapshot.iY);
                            continue;
                        }
                        // keep going
                        if (iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX + currentSnapshot.tDx, currentSnapshot.iY) && iParam.SearchGrid.IsWalkableAt(currentSnapshot.iX, currentSnapshot.iY + currentSnapshot.tDy))
                        {
                            newSnapshot = new JumpSnapshot();
                            newSnapshot.iX = currentSnapshot.iX + currentSnapshot.tDx;
                            newSnapshot.iY = currentSnapshot.iY + currentSnapshot.tDy;
                            newSnapshot.iPx = currentSnapshot.iX;
                            newSnapshot.iPy = currentSnapshot.iY;
                            newSnapshot.stage = 0;
                            stack.Push(newSnapshot);
                            continue;
                        }
                        retVal = null;
                        break;
                }

            }

            return retVal;

        }
        // 从iX，iY位置出发，延[iPx, iPy]方向寻找跳点
        private static GridPos jump(JumpPointParam iParam, int iX, int iY, int iPx, int iPy)
        {
            // 如果遇到障碍物或者遇到边界
            if (!iParam.SearchGrid.IsWalkableAt(iX, iY))
            {
                return null;
            }
            // 如果寻找到终点
            else if (iParam.SearchGrid.GetNodeAt(iX, iY).Equals(iParam.EndNode))
            {
                return new GridPos(iX, iY);
            }

            int tDx = iX - iPx;
            int tDy = iY - iPy;
            if (iParam.DiagonalMovement == DiagonalMovement.Always || iParam.DiagonalMovement == DiagonalMovement.IfAtLeastOneWalkable)
            {
                // 1. 先判断此方向的节点是否有强迫节点
                // check for forced neighbors
                // along the diagonal
                if (tDx != 0 && tDy != 0)
                {
                    // 斜向方向，如果存在强迫邻居，则该点为跳点
                    if ((iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + tDy) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY)) ||
                        (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - tDy) && !iParam.SearchGrid.IsWalkableAt(iX, iY - tDy)))
                    {
                        return new GridPos(iX, iY);
                    }
                }
                // horizontally/vertically
                else
                {
                    if (tDx != 0)
                    {
                        // moving along x
                        if ((iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY + 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY - 1) && !iParam.SearchGrid.IsWalkableAt(iX, iY - 1)))
                        {
                            return new GridPos(iX, iY);
                        }
                    }
                    else
                    {
                        if ((iParam.SearchGrid.IsWalkableAt(iX + 1, iY + tDy) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY + tDy) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY)))
                        {
                            return new GridPos(iX, iY);
                        }
                    }
                }

                // 2. 检查完强迫邻居后，分解为两个方向寻找跳点
                // when moving diagonally, must check for vertical/horizontal jump points
                if (tDx != 0 && tDy != 0)
                {
                    // 从两个方向查找，只要存在跳点，证明这里是拐点，也是跳点
                    // 水平方向查找，找到了即返回
                    if (jump(iParam, iX + tDx, iY, iX, iY) != null)
                    {
                        return new GridPos(iX, iY);
                    }
                    // 竖直方向查找，找到了即返回
                    if (jump(iParam, iX, iY + tDy, iX, iY) != null)
                    {
                        return new GridPos(iX, iY);
                    }
                }

                // 如果分解方向没有找到，说明这个点不是跳点，则返回下一个斜向点
                // moving diagonally, must make sure one of the vertical/horizontal
                // neighbors is open to allow the path
                if (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY) || iParam.SearchGrid.IsWalkableAt(iX, iY + tDy))
                {
                    return jump(iParam, iX + tDx, iY + tDy, iX, iY);
                }
                else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                {
                    return jump(iParam, iX + tDx, iY + tDy, iX, iY);
                }
                else
                {
                    return null;
                }
            }
            else if (iParam.DiagonalMovement == DiagonalMovement.OnlyWhenNoObstacles)
            {
                // check for forced neighbors
                // along the diagonal
                if (tDx != 0 && tDy != 0)
                {
                    if (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY + tDy) && (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy) || !iParam.SearchGrid.IsWalkableAt(iX + tDx, iY)))
                    {
                        return new GridPos(iX, iY);
                    }
                }
                // horizontally/vertically
                else
                {
                    if (tDx != 0)
                    {
                        // moving along x
                        if ((iParam.SearchGrid.IsWalkableAt(iX, iY + 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY + 1)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX, iY - 1) && !iParam.SearchGrid.IsWalkableAt(iX - tDx, iY - 1)))
                        {
                            return new GridPos(iX, iY);
                        }
                    }
                    else
                    {
                        if ((iParam.SearchGrid.IsWalkableAt(iX + 1, iY) && !iParam.SearchGrid.IsWalkableAt(iX + 1, iY - tDy)) ||
                            (iParam.SearchGrid.IsWalkableAt(iX - 1, iY) && !iParam.SearchGrid.IsWalkableAt(iX - 1, iY - tDy)))
                        {
                            return new GridPos(iX, iY);
                        }
                    }
                }


                // when moving diagonally, must check for vertical/horizontal jump points
                if (tDx != 0 && tDy != 0)
                {
                    if (jump(iParam, iX + tDx, iY, iX, iY) != null) return new GridPos(iX, iY);
                    if (jump(iParam, iX, iY + tDy, iX, iY) != null) return new GridPos(iX, iY);
                }

                // moving diagonally, must make sure both of the vertical/horizontal
                // neighbors is open to allow the path
                if (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY) && iParam.SearchGrid.IsWalkableAt(iX, iY + tDy))
                {
                    return jump(iParam, iX + tDx, iY + tDy, iX, iY);
                }
                else
                {
                    return null;
                }
            }
            else // if(iParam.DiagonalMovement == DiagonalMovement.Never)
            {
                if (tDx != 0)
                {
                    // moving along x
                    if (!iParam.SearchGrid.IsWalkableAt(iX + tDx, iY))
                    {
                        return new GridPos(iX, iY);
                    }
                }
                else
                {
                    if (!iParam.SearchGrid.IsWalkableAt(iX, iY + tDy))
                    {
                        return new GridPos(iX, iY);
                    }
                }

                //  must check for perpendicular jump points
                if (tDx != 0)
                {
                    if (jump(iParam, iX, iY + 1, iX, iY) != null) return new GridPos(iX, iY);
                    if (jump(iParam, iX, iY - 1, iX, iY) != null) return new GridPos(iX, iY);
                }
                else // tDy != 0
                {
                    if (jump(iParam, iX + 1, iY, iX, iY) != null) return new GridPos(iX, iY);
                    if (jump(iParam, iX - 1, iY, iX, iY) != null) return new GridPos(iX, iY);
                }

                // keep going
                if (iParam.SearchGrid.IsWalkableAt(iX + tDx, iY) && iParam.SearchGrid.IsWalkableAt(iX, iY + tDy))
                {
                    return jump(iParam, iX + tDx, iY + tDy, iX, iY);
                }
                else
                {
                    return null;
                }
            }
        }

        // 寻找周围的邻居，这些邻居被修剪过的，只可能是自然邻居或强迫邻居
        private static List<GridPos> findNeighbors(JumpPointParam iParam, Node iNode)
        {
            Node tParent = (Node)iNode.parent;
            //var diagonalMovement = Util.GetDiagonalMovement(iParam.CrossCorner, iParam.CrossAdjacentPoint);
            int tX = iNode.x;
            int tY = iNode.y;
            int tPx, tPy, tDx, tDy;
            List<GridPos> tNeighbors = new List<GridPos>();
            List<Node> tNeighborNodes;
            Node tNeighborNode;

            // 如果存在父节点，正常情况下都会存在父节点，除了对起始节点的处理
            // directed pruning: can ignore most neighbors, unless forced.
            if (tParent != null)
            {
                tPx = tParent.x;
                tPy = tParent.y;
                // 归一化parten->cur的方向，因为parent也是跳点，这里只需要知道方向就行了
                tDx = (tX - tPx) / Math.Max(Math.Abs(tX - tPx), 1);
                tDy = (tY - tPy) / Math.Max(Math.Abs(tY - tPy), 1);

                // 如果允许斜向运动
                if (iParam.DiagonalMovement == DiagonalMovement.Always || iParam.DiagonalMovement == DiagonalMovement.IfAtLeastOneWalkable)
                {
                    // 斜向，斜向情况有三个自然节点
                    if (tDx != 0 && tDy != 0)
                    {
                        // 竖直方向
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + tDy));
                        }
                        // 水平方向
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY));
                        }
                        // 斜向
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy) || iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy));
                            }
                        }

                        // 如果左上角可以走，而且左边不可走，则左上角是一个强迫邻居
                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy) && !iParam.SearchGrid.IsWalkableAt(tX - tDx, tY))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy))
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy));
                            }
                        }

                        // 如果右下角可以走，而且下边不可走，则右下角是一个强迫邻居
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy) && !iParam.SearchGrid.IsWalkableAt(tX, tY - tDy))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy));
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy));
                            }
                        }
                    }
                    // search horizontally/vertically
                    else
                    {
                        // 水平向
                        if (tDx != 0)
                        {
                            // 如果存在自然邻居
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                            {
                                // 加入自然邻居
                                tNeighbors.Add(new GridPos(tX + tDx, tY));

                                // 再分别判断两边是否存在强迫邻居
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY + 1));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY - 1));
                                }
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                // 如果不存在自然邻居，则检查强迫邻居
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY + 1))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY + 1));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1) && !iParam.SearchGrid.IsWalkableAt(tX, tY - 1))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY - 1));
                                }
                            }
                        }
                        // 竖直向，与水平向是一样的
                        else
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy));

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY))
                                {
                                    tNeighbors.Add(new GridPos(tX + 1, tY + tDy));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY))
                                {
                                    tNeighbors.Add(new GridPos(tX - 1, tY + tDy));
                                }
                            }
                            else if (iParam.DiagonalMovement == DiagonalMovement.Always)
                            {
                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy) && !iParam.SearchGrid.IsWalkableAt(tX + 1, tY))
                                {
                                    tNeighbors.Add(new GridPos(tX + 1, tY + tDy));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy) && !iParam.SearchGrid.IsWalkableAt(tX - 1, tY))
                                {
                                    tNeighbors.Add(new GridPos(tX - 1, tY + tDy));
                                }
                            }
                        }
                    }
                }
                else if (iParam.DiagonalMovement == DiagonalMovement.OnlyWhenNoObstacles)
                {
                    // search diagonally
                    if (tDx != 0 && tDy != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + tDy));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + tDy))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                                tNeighbors.Add(new GridPos(tX + tDx, tY + tDy));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX - tDx, tY + tDy))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy) && iParam.SearchGrid.IsWalkableAt(tX - tDx, tY))
                                tNeighbors.Add(new GridPos(tX - tDx, tY + tDy));
                        }

                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - tDy))
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY - tDy) && iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                                tNeighbors.Add(new GridPos(tX + tDx, tY - tDy));
                        }


                    }
                    // search horizontally/vertically
                    else
                    {
                        if (tDx != 0)
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                            {

                                tNeighbors.Add(new GridPos(tX + tDx, tY));

                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY + 1) && iParam.SearchGrid.IsWalkableAt(tX, tY + 1))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY + 1));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY - 1) && iParam.SearchGrid.IsWalkableAt(tX, tY - 1))
                                {
                                    tNeighbors.Add(new GridPos(tX + tDx, tY - 1));
                                }
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1))
                                tNeighbors.Add(new GridPos(tX, tY + 1));
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1))
                                tNeighbors.Add(new GridPos(tX, tY - 1));
                        }
                        else
                        {
                            if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy))
                            {
                                tNeighbors.Add(new GridPos(tX, tY + tDy));

                                if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY + tDy) && iParam.SearchGrid.IsWalkableAt(tX + 1, tY))
                                {
                                    tNeighbors.Add(new GridPos(tX + 1, tY + tDy));
                                }
                                if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY + tDy) && iParam.SearchGrid.IsWalkableAt(tX - 1, tY))
                                {
                                    tNeighbors.Add(new GridPos(tX - 1, tY + tDy));
                                }
                            }
                            if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY))
                                tNeighbors.Add(new GridPos(tX + 1, tY));
                            if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY))
                                tNeighbors.Add(new GridPos(tX - 1, tY));
                        }
                    }
                }
                else // if(iParam.DiagonalMovement == DiagonalMovement.Never)
                {
                    if (tDx != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX + tDx, tY))
                        {
                            tNeighbors.Add(new GridPos(tX + tDx, tY));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + 1))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + 1));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY - 1))
                        {
                            tNeighbors.Add(new GridPos(tX, tY - 1));
                        }
                    }
                    else // if (tDy != 0)
                    {
                        if (iParam.SearchGrid.IsWalkableAt(tX, tY + tDy))
                        {
                            tNeighbors.Add(new GridPos(tX, tY + tDy));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX + 1, tY))
                        {
                            tNeighbors.Add(new GridPos(tX + 1, tY));
                        }
                        if (iParam.SearchGrid.IsWalkableAt(tX - 1, tY))
                        {
                            tNeighbors.Add(new GridPos(tX - 1, tY));
                        }
                    }
                }

            }
            // return all neighbors
            else
            {
                tNeighborNodes = iParam.SearchGrid.GetNeighbors(iNode, iParam.DiagonalMovement);
                for (int i = 0; i < tNeighborNodes.Count; i++)
                {
                    tNeighborNode = tNeighborNodes[i];
                    tNeighbors.Add(new GridPos(tNeighborNode.x, tNeighborNode.y));
                }
            }

            return tNeighbors;
        }
    }
}
