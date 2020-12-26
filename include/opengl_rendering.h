#pragma once

//opengl
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Eigen/Dense"
#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>
#include "shader.h"
#include "camera.h"


#include <opencv2/imgproc.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>


class OpenglRendering{

private:
    std::vector<cv::Mat> imgs;


public:

    OpenglRendering(std::string window_name);
    ~OpenglRendering();

    Shader point_shader;
    Shader plane_shader;
    
    unsigned int VBO, VAO, EBO;

    std::string w_name;

    int screenWidth;
    int screenHeight;
    float lastX;
    float lastY;
    bool firstMouse;
    float deltaTime;
    float lastFrame;



    Camera * camera;


    typedef struct{
        GLfloat x, y, z;
        GLfloat r, g, b, a;
    } Vertex;

    void init_opengl();
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void processInput();
    void processInput_end();

    GLFWwindow * window;
    void clear_window();

    void draw_axis(float line_length, float line_width, Shader* shader);

    void draw_cameras(float line_length, float line_width, Shader* shader);

    void terminate();

    void draw_surfels(std::vector<std::vector<float>> & surfels);


    glm::mat4 eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4);


    static void mouse_callback(GLFWwindow * window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);

    void mouse_callback_function(double xpos, double ypos);
    void scroll_callback_function(double xoffset, double yoffset);

    Eigen::Matrix3f skew_symmetric(Eigen::Vector3f& vector);

};