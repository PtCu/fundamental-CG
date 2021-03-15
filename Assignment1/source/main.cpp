#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float radian = rotation_angle / 180.0 * MY_PI;
    Eigen::Matrix4f Rz;
    // Rz << cos(radian), -sin(radian), 0, 0,
    //     sin(radian), cos(radian)`, 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1;
    // Ry << cos(radian), 0, sin(radian), 0,
    //     0, 1, 0, 0,
    //     -sin(radian), 0, cos(radian), 0,
    //     0, 0, 0, 1;
    Rz << 1, 0, 0, 0,
        0, cos(radian), -sin(radian), 0,
        0, sin(radian), cos(radian), 0,
        0, 0, 0, 1;

    model = Rz * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    //将透视投影转化为Orthographic Projection

    Eigen::Matrix4f persp2ortho;
    persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -(zNear * zFar),
        0, 0, 1, 0;

    float radian = eye_fov / 180.0 * MY_PI;
    //t:top b:bottom  l:left r:right  n:near f:far
    auto t = abs(zNear) * tan(radian / 2);
    auto b = -t;
    auto r = t * aspect_ratio;
    auto l = -r;

    //平移矩阵
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;
    //缩放矩阵
    Eigen::Matrix4f scale;
    scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    projection = scale * translate * persp2ortho * projection;
    return projection;
}

//绕给定的axis旋转
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float radian = angle / 180.0 * MY_PI;

    float len = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    axis[0] /= len;
    axis[1] /= len;
    axis[2] /= len;
    Eigen::Matrix3f N;
    N << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;

    Eigen::Matrix3f component1 = Eigen::Matrix3f::Identity() * cos(radian);
    Eigen::Matrix3f component2 = axis * axis.transpose() * (1 - cos(radian));
    Eigen::Matrix3f component3 = sin(radian) * N;

    Eigen::Matrix3f rotate_m = component1 + component2 + component3;

    // Eigen 自带构造轴角旋转矩阵
    // 下列注释用于验证我们构造的轴角旋转矩阵是否和Eigen的构造的轴角旋转矩阵一致
    //Eigen::AngleAxisf rotation_vector(radian, Vector3f(axis[0], axis[1], axis[2]));
    //Eigen::Matrix3f rotation_matrix;
    //rotation_m = rotation_vector.toRotationMatrix();

    Eigen::Matrix4f rotate_martix = Eigen::Matrix4f::Identity();
    //提取块大小为(p,q),起始于(i,j)	matrix.block(i,j,p,q)
    rotate_martix.block(0, 0, 3, 3) = rotate_m; // 前三个维度为旋转矩阵

    model = rotate_martix * model;
    return model;
}
int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; //Vector3f  3个float

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}}; //Vector3i  3个int

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        //Mat是容器类，创建一个图像矩阵
        //CV_[The number of bits per item][Signed or Unsigned][Type Prefix]C[The channel number]
        //返回指向vector中第一个数据的指针或空vector之后的位置
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data()); //32位float，三通道
        image.convertTo(image, CV_8UC3, 1.0f);                      //CV_8UC3 表示使用8位的 unsigned char 型，每个像素由三个元素组成三通道。
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
