//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    int min_x = (int)std::floor(std::min(v[0][0], std::min(v[1][0], v[2][0])));
    int min_y = (int)std::floor(std::min(v[0][1], std::min(v[1][1], v[2][1])));
    int max_x = (int)std::ceil(std::max(v[0][0], std::max(v[1][0], v[2][0])));
    int max_y = (int)std::ceil(std::max(v[0][1], std::max(v[1][1], v[2][1])));

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    bool MSAA = true;
    if (MSAA)
    {
        std::vector<Eigen::Vector2f> pos{
            {0.25, 0.25},
            {0.25, 0.75},
            {0.75, 0.25},
            {0.75, 0.75}};
        for (int x = min_x; x <= max_x; x++)
        {
            for (int y = min_y; y <= max_y; y++)
            {
                int count = 0;
                float min_depth = FLT_MAX;
                for (int i = 0; i < 4; i++)
                {
                    if (insideTriangle(x + pos[i][0], y + pos[i][1], t.v))
                    {
                        auto [alpha, beta, gamma] = computeBarycentric2D(x + pos[i][0], y + pos[i][1], t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        min_depth = std::min(min_depth, z_interpolated);
                        count++;
                    }
                    if (count)
                    {
                        if (min_depth < depth_buf[get_index(x, y)])
                        {
                            Eigen::Vector3f color = t.getColor() * count / 4.0;
                            depth_buf[get_index(x, y)] = min_depth;
                            Eigen::Vector3f point;
                            point << (float)x, (float)y, min_depth;
                            set_pixel(point, color);
                        }
                    }
                }
            }
        }
    }
    else
    {
        for (int x = min_x; x <= max_x; x++)
        {
            for (int y = min_y; y <= max_y; y++)
            {
                if (insideTriangle(x + 0.5, y + 0.5, t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    if (z_interpolated < depth_buf[get_index(x, y)])
                    {
                        Eigen::Vector3f color = t.getColor();
                        depth_buf[get_index(x, y)] = z_interpolated;
                        Eigen::Vector3f point;
                        point << (float)x, (float)y, z_interpolated;
                        set_pixel(point, color);
                    }
                }
            }
        }
    }
}