//
// Created by chrisliu on 2020/11/15.
//

#ifndef BEZIERTRAJGENERATOR_CORRIDORGENERATOR_H
#define BEZIERTRAJGENERATOR_CORRIDORGENERATOR_H

#include "eigen3/Eigen/Eigen"

#include "BezierTrajGenerator/Cube.h"

using namespace std;

class CorridorGenerator
{

private:
    vector<Cube> _corridor;

    Cube generateCube( Vector3d pt)
    {
/*
           P4------------P3
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /
        P5------------P6              / x
*/
        Cube cube;


        cube.center = pt;

        /**
         * 生成边长为0的cube
         */
        cube.vertex.row(0) = pt;
        cube.vertex.row(1) = pt;
        cube.vertex.row(2) = pt;
        cube.vertex.row(3) = pt;

        cube.vertex.row(4) = pt;
        cube.vertex.row(5) = pt;
        cube.vertex.row(6) = pt;
        cube.vertex.row(7) = pt;

        return cube;
    }

    bool isContains(Cube cube1, Cube cube2)
    {
        if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && cube1.vertex(0, 1) <= cube2.vertex(0, 1) && cube1.vertex(0, 2) >= cube2.vertex(0, 2) &&
            cube1.vertex(6, 0) <= cube2.vertex(6, 0) && cube1.vertex(6, 1) >= cube2.vertex(6, 1) && cube1.vertex(6, 2) <= cube2.vertex(6, 2)  )
            return true;
        else
            return false;
    }

    bool isSimilar(Cube cube1, Cube cube2)
    {
        double threshold = 0.3;
        if( cube1.vertex(0, 0)+threshold >= cube2.vertex(0, 0) && cube1.vertex(0, 1)-threshold <= cube2.vertex(0, 1) && cube1.vertex(0, 2)+threshold >= cube2.vertex(0, 2) &&
            cube1.vertex(6, 0)-threshold <= cube2.vertex(6, 0) && cube1.vertex(6, 1)+threshold >= cube2.vertex(6, 1) && cube1.vertex(6, 2)-threshold <= cube2.vertex(6, 2)  )
            return true;
        else
            return false;
    }

    pair<Cube, bool> inflateCube(Cube cube, Cube lstcube)
    {
        MapManager _MapManager = _AstarPathPlanner->_MapManager;

        Cube cubeMax = cube;

        // Inflate sequence: left, right, front, back, below, above
        MatrixXi vertex_idx(8, 3);//    对应八个点的xyz
        for (int i = 0; i < 8; i++)//   遍历八个点
        {
            double coord_x = cube.vertex(i, 0);
            double coord_y = cube.vertex(i, 1);
            double coord_z = cube.vertex(i, 2);
            Vector3d coord(coord_x, coord_y, coord_z);

            Vector3i pt_idx = _MapManager.coord2gridIndex(coord);

            if( _MapManager.isOccupied(pt_idx))
            {
                ROS_ERROR("[Planning Node] path has node in obstacles !");
                return make_pair(cubeMax, false);
            }

            vertex_idx.row(i) = pt_idx;
        }

        int id_x, id_y, id_z;

        /*
                   P4------------P3
                   /|           /|              ^
                  / |          / |              | z
                P1--|---------P2 |              |
                 |  P8--------|--p7             |
                 | /          | /               /--------> y
                 |/           |/               /
                P5------------P6              / x
        */

        // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face sweep
        // ############################################################################################################
        bool collide;

        MatrixXi vertex_idx_last = vertex_idx;

        int iter = 0;
        const int _max_inflate_iter = 100;
        const int _step_length = 2;
        const int _max_x_id = _MapManager.GLX_SIZE;
        const int _max_y_id = _MapManager.GLY_SIZE;
        const int _max_z_id = _MapManager.GLZ_SIZE;
        while(iter < _max_inflate_iter)
        {
            collide  = false;
            int y_lo = max(0, vertex_idx(0, 1) - _step_length);
            int y_up = min(_max_y_id, vertex_idx(1, 1) + _step_length);

            for(id_y = vertex_idx(0, 1); id_y >= y_lo; id_y-- )
            {
                if( collide == true)
                    break;

                //  遍历对应y长度与此时x(P4)和z(P5)组成的cube里是否有障碍物
                for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                {
                    if( collide == true)
                        break;

                    for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                    {
                        if(_MapManager.isOccupied( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z))// the voxel is occupied
                        {
                            collide = true;
                            break;
                        }
                    }
                }
            }

            if(collide)
            {
                vertex_idx(0, 1) = min(id_y+2, vertex_idx(0, 1));
                vertex_idx(3, 1) = min(id_y+2, vertex_idx(3, 1));
                vertex_idx(7, 1) = min(id_y+2, vertex_idx(7, 1));
                vertex_idx(4, 1) = min(id_y+2, vertex_idx(4, 1));
            }
            else
                vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + 1;

            // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
            // ############################################################################################################
            collide = false;
            for(id_y = vertex_idx(1, 1); id_y <= y_up; id_y++ )
            {
                if( collide == true)
                    break;

                for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x-- )
                {
                    if( collide == true)
                        break;

                    for(id_z = vertex_idx(1, 2); id_z >= vertex_idx(5, 2); id_z-- )
                    {
                        if(_MapManager.isOccupied( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z))// the voxel is occupied
                        {
                            collide = true;
                            break;
                        }
                    }
                }
            }

            if(collide)
            {
                vertex_idx(1, 1) = max(id_y-2, vertex_idx(1, 1));
                vertex_idx(2, 1) = max(id_y-2, vertex_idx(2, 1));
                vertex_idx(6, 1) = max(id_y-2, vertex_idx(6, 1));
                vertex_idx(5, 1) = max(id_y-2, vertex_idx(5, 1));
            }
            else
                vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - 1;

            // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
            // ############################################################################################################
            int x_lo = max(0, vertex_idx(3, 0) - _step_length);
            int x_up = min(_max_x_id, vertex_idx(0, 0) + _step_length);

            collide = false;
            for(id_x = vertex_idx(0, 0); id_x <= x_up; id_x++ )
            {
                if( collide == true)
                    break;

                for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
                {
                    if( collide == true)
                        break;

                    for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                    {
                        if(_MapManager.isOccupied( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z))// the voxel is occupied
                        {
                            collide = true;
                            break;
                        }
                    }
                }
            }

            if(collide)
            {
                vertex_idx(0, 0) = max(id_x-2, vertex_idx(0, 0));
                vertex_idx(1, 0) = max(id_x-2, vertex_idx(1, 0));
                vertex_idx(5, 0) = max(id_x-2, vertex_idx(5, 0));
                vertex_idx(4, 0) = max(id_x-2, vertex_idx(4, 0));
            }
            else
                vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - 1;

            // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
            // ############################################################################################################
            collide = false;
            for(id_x = vertex_idx(3, 0); id_x >= x_lo; id_x-- )
            {
                if( collide == true)
                    break;

                for(id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y++ )
                {
                    if( collide == true)
                        break;

                    for(id_z = vertex_idx(3, 2); id_z >= vertex_idx(7, 2); id_z-- )
                    {
                        if(_MapManager.isOccupied( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z))// the voxel is occupied
                        {
                            collide = true;
                            break;
                        }
                    }
                }
            }

            if(collide)
            {
                vertex_idx(3, 0) = min(id_x+2, vertex_idx(3, 0));
                vertex_idx(2, 0) = min(id_x+2, vertex_idx(2, 0));
                vertex_idx(6, 0) = min(id_x+2, vertex_idx(6, 0));
                vertex_idx(7, 0) = min(id_x+2, vertex_idx(7, 0));
            }
            else
                vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + 1;

            // Z+ now is the above side : (p1 -- p2 -- p3 -- p4) face
            // ############################################################################################################
            collide = false;
            int z_lo = max(0, vertex_idx(4, 2) - _step_length);
            int z_up = min(_max_z_id, vertex_idx(0, 2) + _step_length);
            for(id_z = vertex_idx(0, 2); id_z <= z_up; id_z++ )
            {
                if( collide == true)
                    break;

                for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
                {
                    if( collide == true)
                        break;

                    for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                    {
                        if(_MapManager.isOccupied( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z))// the voxel is occupied
                        {
                            collide = true;
                            break;
                        }
                    }
                }
            }

            if(collide)
            {
                vertex_idx(0, 2) = max(id_z-2, vertex_idx(0, 2));
                vertex_idx(1, 2) = max(id_z-2, vertex_idx(1, 2));
                vertex_idx(2, 2) = max(id_z-2, vertex_idx(2, 2));
                vertex_idx(3, 2) = max(id_z-2, vertex_idx(3, 2));
            }
            vertex_idx(0, 2) = vertex_idx(1, 2) = vertex_idx(2, 2) = vertex_idx(3, 2) = id_z - 1;

            // now is the below side : (p5 -- p6 -- p7 -- p8) face
            // ############################################################################################################
            collide = false;
            for(id_z = vertex_idx(4, 2); id_z >= z_lo; id_z-- )
            {
                if( collide == true)
                    break;

                for(id_y = vertex_idx(4, 1); id_y <= vertex_idx(5, 1); id_y++ )
                {
                    if( collide == true)
                        break;

                    for(id_x = vertex_idx(4, 0); id_x >= vertex_idx(7, 0); id_x-- )
                    {
                        if(_MapManager.isOccupied( (int64_t)id_x, (int64_t)id_y, (int64_t)id_z))// the voxel is occupied
                        {
                            collide = true;
                            break;
                        }
                    }
                }
            }

            if(collide)
            {
                vertex_idx(4, 2) = min(id_z+2, vertex_idx(4, 2));
                vertex_idx(5, 2) = min(id_z+2, vertex_idx(5, 2));
                vertex_idx(6, 2) = min(id_z+2, vertex_idx(6, 2));
                vertex_idx(7, 2) = min(id_z+2, vertex_idx(7, 2));
            }
            else
                vertex_idx(4, 2) = vertex_idx(5, 2) = vertex_idx(6, 2) = vertex_idx(7, 2) = id_z + 1;

            //  若边界点没有更新则提前退出
            if(vertex_idx_last == vertex_idx)
                break;

            vertex_idx_last = vertex_idx;

            MatrixXd vertex_coord(8, 3);
            for(int i = 0; i < 8; i++)
            {
                int index_x = max(min(vertex_idx(i, 0), _max_x_id - 1), 0);
                int index_y = max(min(vertex_idx(i, 1), _max_y_id - 1), 0);
                int index_z = max(min(vertex_idx(i, 2), _max_z_id - 1), 0);

                Vector3i index(index_x, index_y, index_z);
                Vector3d pos = _MapManager.gridIndex2coord(index);
                vertex_coord.row(i) = pos;
            }

            cubeMax.setVertex(vertex_coord, _MapManager.resolution);
            //  判断与前一个corridor有没有包含关系,若有则返回false,提前终止
            if( isContains(lstcube, cubeMax))
                return make_pair(lstcube, false);

            iter ++;
        }

        return make_pair(cubeMax, true);
    }

    vector<Cube> corridorGeneration(vector<Vector3d> path_coord)
    {
        vector<Cube> cubeList;
        Vector3d pt;

        Cube lastcube;

        for (int i = 0; i < (int)path_coord.size(); i++)
        {
            pt = path_coord[i];

            Cube cube = generateCube(pt);
            auto result = inflateCube(cube, lastcube);

            if(result.second == false)
                continue;

            cube = result.first;

            lastcube = cube;
            cubeList.push_back(cube);
        }
        return cubeList;
    }

    vector<Cube> simplifyCorridor(vector<Cube> corridor)
    {
        vector<Cube> simplifyCorridor;

        for(int j = (int)corridor.size() - 1; j >= 0; j--)
        {
            for(int k = j - 1; k >= 0; k--)
            {
                if(isSimilar(corridor[j], corridor[k]))
                    corridor[k].valid = false;
            }
        }

        for(auto cube:corridor)
            if(cube.valid == true)
                simplifyCorridor.push_back(cube);

        return simplifyCorridor;
    }


public:

    CorridorGenerator(){

    };

    ~CorridorGenerator(){};

    void visCorridor(vector<Cube> corridor)
    {
        for(auto & mk: cube_vis.markers)
            mk.action = visualization_msgs::Marker::DELETE;

        _corridor_vis_pub.publish(cube_vis);

        cube_vis.markers.clear();

        visualization_msgs::Marker mk;
        mk.header.frame_id = "map";
        mk.header.stamp = ros::Time::now();
        mk.ns = "corridor";
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;

        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.a = 0.4;
        mk.color.r = 1.0;
        mk.color.g = 1.0;
        mk.color.b = 1.0;

        int idx = 0;
        for(int i = 0; i < int(corridor.size()); i++)
        {
            mk.id = idx;

            mk.pose.position.x = (corridor[i].vertex(0, 0) + corridor[i].vertex(3, 0) ) / 2.0;
            mk.pose.position.y = (corridor[i].vertex(0, 1) + corridor[i].vertex(1, 1) ) / 2.0;
            mk.pose.position.z = 0.0;


            mk.scale.x = (corridor[i].vertex(0, 0) - corridor[i].vertex(3, 0) );
            mk.scale.y = (corridor[i].vertex(1, 1) - corridor[i].vertex(0, 1) );
            mk.scale.z = 0.02;

            idx ++;
            cube_vis.markers.push_back(mk);
        }

        _corridor_vis_pub.publish(cube_vis);
    }


};


#endif //BEZIERTRAJGENERATOR_CORRIDORGENERATOR_H
