// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <utility>
#include <algorithm>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector3f* _v)
{  
    Eigen::Vector3f edgeVector = _v[1]-_v[0];
    Eigen::Vector3f pointVector = Eigen::Vector3f(x+0.5-_v[0].x(),y+0.5-_v[0].y(),0.f);
    Eigen::Vector3f result1 = edgeVector.cross(pointVector);
    edgeVector = _v[2]-_v[1];
    pointVector = Eigen::Vector3f(x+0.5-_v[1].x(),y+0.5-_v[1].y(),0.f);
    Eigen::Vector3f result2 = edgeVector.cross(pointVector);
    edgeVector = _v[0]-_v[2];
    pointVector = Eigen::Vector3f(x+0.5-_v[2].x(),y+0.5-_v[2].y(),0.f);
    Eigen::Vector3f result3 = edgeVector.cross(pointVector);
    if((result1.z()>=0&&result2.z()>=0&&result3.z()>=0)||(result1.z()<=0&&result2.z()<=0&&result3.z()<=0))
    {
        return true;
    }
    return false;

    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static bool insideTriangle_(int x, int y, const Vector3f* _v)
{  
    Eigen::Vector3f edgeVector = _v[1]-_v[0];
    Eigen::Vector3f pointVector = Eigen::Vector3f(x+0.5-_v[0].x(),y+0.5-_v[0].y(),0.f);
    Eigen::Vector3f result1 = edgeVector.cross(pointVector);
    edgeVector = _v[2]-_v[1];
    pointVector = Eigen::Vector3f(x+0.5-_v[1].x(),y+0.5-_v[1].y(),0.f);
    Eigen::Vector3f result2 = edgeVector.cross(pointVector);
    edgeVector = _v[0]-_v[2];
    pointVector = Eigen::Vector3f(x+0.5-_v[2].x(),y+0.5-_v[2].y(),0.f);
    Eigen::Vector3f result3 = edgeVector.cross(pointVector);
    if((result1.z()>=0&&result2.z()>=0&&result3.z()>=0)||(result1.z()<=0&&result2.z()<=0&&result3.z()<=0))
    {
        return true;
    }
    return false;

    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle_msaa(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    std::pair<float,float> xRound;
    xRound.first = std::min({(t.v[0].x()),(t.v[1].x()),(t.v[2].x())});
    xRound.second = std::max({(t.v[0].x()),(t.v[1].x()),(t.v[2].x())});
    std::pair<float,float> yRound;
    yRound.first = std::min({(t.v[0].y()),(t.v[1].y()),(t.v[2].y())});
    yRound.second = std::max({(t.v[0].y()),(t.v[1].y()),(t.v[2].y())});
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    for(int i = static_cast<int>(std::floorf(xRound.first));i<=static_cast<int>(std::ceilf(xRound.second));i++)
    {
        for(int j = static_cast<int>(std::floorf(yRound.first));j<=static_cast<int>(std::ceilf(yRound.second));j++)
        {
            if(insideTriangle(i,j,t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if(z_interpolated<depth_buf[j*width+i])
                {
                    Eigen::Vector3f point(height-1-i,width-1-j,1.f);
                    set_pixel(point,t.getColor());
                    depth_buf[j*width+i] = z_interpolated;
                }
            }
        }
    }
    // If so, use the following code to get the interpolated z value.
    

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::rasterize_triangle_msaa(const Triangle& t)
{
    struct samplePoint
    {
        float m_x;
        float m_y;
        bool m_isInsideTriangle;
        float z_inter;

        samplePoint(float x,float y,const Triangle& p_t):m_x(x),m_y(y),m_isInsideTriangle(false)
        {
            z_inter = getZInterpolated(x,y,p_t);
        }
    };
    auto v = t.toVector4();
    std::pair<float,float> xRound;
    xRound.first = std::min({(t.v[0].x()),(t.v[1].x()),(t.v[2].x())});
    xRound.second = std::max({(t.v[0].x()),(t.v[1].x()),(t.v[2].x())});
    std::pair<float,float> yRound;
    yRound.first = std::min({(t.v[0].y()),(t.v[1].y()),(t.v[2].y())});
    yRound.second = std::max({(t.v[0].y()),(t.v[1].y()),(t.v[2].y())});
    
    
    for(int i = static_cast<int>(std::floorf(xRound.first));i<=static_cast<int>(std::ceilf(xRound.second));i++)
    {
        for(int j = static_cast<int>(std::floorf(yRound.first));j<=static_cast<int>(std::ceilf(yRound.second));j++)
        {
            std::array<samplePoint,4> sampleList{samplePoint{static_cast<float>(i)+0.25f,static_cast<float>(j)+0.25f,t},samplePoint{static_cast<float>(i)+0.75f,static_cast<float>(j)+0.25f,t},samplePoint{static_cast<float>(i)+0.25f,static_cast<float>(j)+0.75f,t},samplePoint{static_cast<float>(i)+0.75f,static_cast<float>(j)+0.75f,t}};
            for(int k=0;k<=3;k++)
            {
                float subX = k%2?i+0.25f:i-0.25f;
                float subY = k<2?j-0.25f:j+0.25f;
                sampleList[k].m_isInsideTriangle = insideTriangle(subX,subY,t.v);
            }
            // sampleList[0].m_isInsideTriangle = insideTriangle(i-0.25f,j-0.25f,t.v);
            // sampleList[1].m_isInsideTriangle = insideTriangle(i+0.25f,j-0.25f,t.v);
            // sampleList[2].m_isInsideTriangle = insideTriangle(i-0.25f,j+0.25f,t.v);
            // sampleList[3].m_isInsideTriangle = insideTriangle(i+0.25f,j+0.25f,t.v);
            int count = 0;
            for(int k=0;k<=3;k++)
            {
                if(!sampleList[k].m_isInsideTriangle)
                {
                    count++;
                }
            }
            for(int k = 0;k<=3;k++)
            {
                if(sampleList[k].m_isInsideTriangle)
                {
                    int msaa_index = (j * 2 + k / 2) * width * 2 + (i * 2 + k % 2);
                    if(sampleList[k].z_inter<depth_buf_msaa[msaa_index])
                    {
                        float point_x= height-1-i;
                        float point_y = width-1-j;
    
                        auto ind = (height-1-point_y)*width + point_x;
                        if(i==0&&count==4)
                        {
                            frame_buf[ind] = Eigen::Vector3f{0.0f,0.0f,0.0f};
                        }
                        frame_buf[ind] += (1.0f/4.0f)*t.getColor();
                        // frame_buf[ind] = frame_buf[ind].cwiseMin(1.0f);
                        depth_buf_msaa[msaa_index] = sampleList[k].z_inter;
                    }
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_msaa.begin(), depth_buf_msaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    depth_buf_msaa.resize(width*2*height*2);
    std::fill(depth_buf_msaa.begin(), depth_buf_msaa.end(), std::numeric_limits<float>::infinity());
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
