// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
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


static bool insideTriangle(double x, double y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]


    //前n个元素 vector.head(n),后n个：vector.tail(n)
    Eigen:Vector2f p;
    p<<x,y;

    Eigen::Vector2f ab=_v[1].head(2)-_v[0].head(2);
    Eigen::Vector2f bc=_v[2].head(2)-_v[1].head(2);
    Eigen::Vector2f ca=_v[0].head(2)-_v[2].head(2);

    Eigen::Vector2f ap=p-_v[0].head(2);
    Eigen::Vector2f bp=p-_v[1].head(2);
    Eigen::Vector2f cp=p-_v[2].head(2);

    bool isInside=false;
    if((ab[0]*ap[1]-ab[1]*ap[0]>0
    &&bc[0]*bp[1]-bc[1]*bp[0]>0
    &&ca[0]*cp[1]-ca[1]*cp[0]>0)||
    (ab[0]*ap[1]-ab[1]*ap[0]<0
    &&bc[0]*bp[1]-bc[1]*bp[0]<0
    &&ca[0]*cp[1]-ca[1]*cp[0]<0))
    isInside=true;

    return isInside;

    
    // Eigen:Vector2f p;
    // p<<x,y;
    // Eigen::Vector2f v0=_v[0].head(2)-_v[1].head(2);
    // Eigen::Vector2f v1=_v[2].head(2)-_v[1].head(2);
    // Eigen::Vector2f v2=p-_v[1].head(2);

    // float dot00=v0.dot(v0);
    // float dot01=v0.dot(v1);
    // float dot02=v0.dot(v2);
    // float dot11=v1.dot(v1);
    // float dot12=v0.dot(v2);

    // float inverDeno=1/(dot00*dot11-dot01*dot01);

    // float u=(dot11*dot02-dot01*dot12)*inverDeno;
    // if (u < 0 || u > 1) // if u out of range, return directly
    // {
    //     return false ;
    // }

    // float v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
    // if (v < 0 || v > 1) // if v out of range, return directly
    // {
    //     return false ;
    // }

    // return u + v <= 1 ;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
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

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    int min_x=(int)std::floor(std::min(v[0][0],std::min(v[1][0],v[2][0])));
    int min_y=(int)std::floor(std::min(v[0][1],std::min(v[1][1],v[2][1])));
    int max_x=(int)std::ceil(std::max(v[0][0],std::max(v[1][0],v[2][0])));
    int max_y=(int)std::ceil(std::max(v[0][1],std::max(v[1][1],v[2][1])));

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    bool MSAA=true;
    if(MSAA){
        std::vector<Eigen::Vector2f> pos{
            {0.25,0.25},
            {0.25,0.75},
            {0.75,0.25},
            {0.75,0.75}
        };
        for(int x=min_x;x<=max_x;x++){
            for(int y=min_y;y<=max_y;y++){
                int count=0;
                float min_depth=FLT_MAX;
                for(int i=0;i<4;i++){
                    if(insideTriangle(x+pos[i][0],y+pos[i][1],t.v)){
                        auto[alpha, beta, gamma] = computeBarycentric2D(x+pos[i][0], y+pos[i][1], t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        min_depth=std::min(min_depth,z_interpolated);
                        count++;
                    
                    }
                    
                }
                if(count){
                        if(min_depth<depth_buf[get_index(x,y)]){
                            Eigen::Vector3f color=t.getColor()*count/4.0;
                            depth_buf[get_index(x,y)]=min_depth;
                            Eigen::Vector3f point;
                            point<<(float)x,(float)y,min_depth;
                            set_pixel(point,color);
                        }
                    }
                
            }
        }
        

    }
    else{
        for(int x=min_x;x<=max_x;x++){
            for(int y=min_y;y<=max_y;y++){
                if(insideTriangle(x+0.5,y+0.5,t.v)){
                    auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if(z_interpolated<depth_buf[get_index(x,y)]){
                        Eigen::Vector3f color=t.getColor();
                        depth_buf[get_index(x,y)]=z_interpolated;
                        Eigen::Vector3f point;
                        point<<(float)x,(float)y,z_interpolated;
                        set_pixel(point,color);
                    }

                }
            }
        }
    }

}
// void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//     auto v = t.toVector4();
    
//     // TODO : Find out the bounding box of current triangle.
//     // iterate through the pixel and find if the current pixel is inside the triangle

//     // If so, use the following code to get the interpolated z value.
//     //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
//     //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//     //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//     //z_interpolated *= w_reciprocal;


//     // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.


// 	// bounding box
// 	float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
//     float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
// 	float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
// 	float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));

// 	min_x = (int)std::floor(min_x);
// 	max_x = (int)std::ceil(max_x);
// 	min_y = (int)std::floor(min_y);
// 	max_y = (int)std::ceil(max_y);

// 	bool MSAA = true;
// 	//MSAA 4X
// 	if (MSAA) {
// 		// 格子里的细分四个小点坐标
// 		std::vector<Eigen::Vector2f> pos
// 		{
// 			{0.25,0.25},
// 			{0.75,0.25},
// 			{0.25,0.75},
// 			{0.75,0.75},
// 		};
// 		for (int x = min_x; x <= max_x; x++) {
// 			for (int y = min_y; y <= max_y; y++) {
// 				// 记录最小深度
// 				float minDepth = FLT_MAX;
// 				// 四个小点中落入三角形中的点的个数
// 				int count = 0;
// 				// 对四个小点坐标进行判断 
// 				for (int i = 0; i < 4; i++) {
// 					// 小点是否在三角形内
// 					if (insideTriangle((float)x + pos[i][0], (float)y + pos[i][1], t.v)) {
// 						// 如果在，对深度z进行插值
// 						auto tup = computeBarycentric2D((float)x + pos[i][0], (float)y + pos[i][1], t.v);
// 						float alpha;
// 						float beta;
// 						float gamma;
// 						std::tie(alpha, beta, gamma) = tup;
// 						float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
// 						float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
// 						z_interpolated *= w_reciprocal;
// 						minDepth = std::min(minDepth, z_interpolated);
// 						count++;
// 					}
// 				}
// 				if (count != 0) {
// 					if (depth_buf[get_index(x, y)] > minDepth) {
// 						Vector3f color = t.getColor() * count / 4.0;
// 						Vector3f point(3);
// 						point << (float)x, (float)y, minDepth;
// 						// 替换深度
// 						depth_buf[get_index(x, y)] = minDepth;
// 						// 修改颜色
// 						set_pixel(point, color);
// 					}
// 				}
// 			}
// 		}
// 	}
// 	else {
// 		for (int x = min_x; x <= max_x; x++) {
// 			for (int y = min_y; y <= max_y; y++) {
// 				if (insideTriangle((float)x + 0.5, (float)y + 0.5, t.v)) {
// 					auto tup = computeBarycentric2D((float)x + 0.5, (float)y + 0.5, t.v);
// 					float alpha;
// 					float beta;
// 					float gamma;
// 					std::tie(alpha, beta, gamma) = tup; //std::tie会将变量的引用整合成一个tuple，从而实现批量赋值
// 					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
// 					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
// 					z_interpolated *= w_reciprocal;

// 					if (depth_buf[get_index(x, y)] > z_interpolated) {
// 						Vector3f color = t.getColor();
// 						Vector3f point(3);
// 						point << (float)x, (float)y, z_interpolated;
// 						depth_buf[get_index(x, y)] = z_interpolated;
// 						set_pixel(point, color);
// 					}
// 				}
// 			}
// 		}
// 	}

// }

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
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
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