#include "opengl_rendering.h"


OpenglRendering::OpenglRendering(std::string window_name){
    w_name = window_name;
    screenWidth = 1600;
    screenHeight = 900;

    camera = new Camera(glm::vec3(-2.0f, 0.0f, 1.0f), glm::vec3(0.0f,0.0f,-1.0f));

    lastX = (float)screenWidth/2.0f;
    lastY = (float)screenHeight/2.0f;
    firstMouse = true;

    deltaTime = 0.0f;
    lastFrame = 0.0f;
    

    imgs.clear();
}

OpenglRendering::~OpenglRendering(){

}

void OpenglRendering::init_opengl(){
    glfwInit(); //Initialize GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3); //OpenGL 3.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); //Use only core profiles of opengl

    
    window = glfwCreateWindow(screenWidth,screenHeight, w_name.c_str(), NULL, NULL);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetWindowUserPointer(window, this);


    if(window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    

    glfwMakeContextCurrent(window); //Tell GLFW to setup "window context" as primary context for current thread

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { //glfwGetProcAddress : Get correct functions regarding to OS or compile environment
        std::cout<< "Failed to initialize GLAD" << std::endl;
    }

    point_shader.InitShader("shaders/point/vertex_shader.vs", "shaders/point/fragment_shader.fs");
    plane_shader.InitShader("shaders/plane/with_texture/vertex_shader.vs", "shaders/plane/with_texture/fragment_shader.fs");

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenVertexArrays(1, &VAO);
    glEnable(GL_DEPTH_TEST);  

}

void OpenglRendering::framebuffer_size_callback(GLFWwindow* window, int width, int height) { // Change viewport if the user change the size of the window
    glViewport(0, 0, width, height);
}

void OpenglRendering::processInput() {
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }
}

glm::mat4 OpenglRendering::eigen_mat4_to_glm_mat4(Eigen::Matrix4f & e_mat4){

    glm::mat4 result;
    for(int i=0;i<4;++i){
        for(int j=0;j<4;++j){
            result[j][i] = e_mat4(i,j);
        }
    }
    return result;
}

void OpenglRendering::terminate(){
    glfwTerminate();
}

void OpenglRendering::clear_window(){
    processInput();
    glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
}


void OpenglRendering::draw_axis(float line_length, float line_width, Shader * shader){
    float vertices[]{
        0.0f, 0.0f, 0.0f,                   1.0f, 0.0f, 0.0f,//v0
        line_length*1.0f, 0.0f, 0.0f,       1.0f, 0.0f, 0.0f,//vx
        0.0f, 0.0f, 0.0f,                   0.0f, 1.0f, 0.0f,//v0
        0.0f, line_length*1.0f, 0.0f,       0.0f, 1.0f, 0.0f,//vy
        0.0f, 0.0f, 0.0f,                   0.0f, 0.0f, 1.0f,//v0
        0.0f, 0.0f, line_length*1.0f,       0.0f, 0.0f, 1.0f//vz
    };

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    glm::mat4 model = glm::mat4(1.0f);

    shader->setMat4("model", model);

    glLineWidth(line_width);
    // glDrawElements(GL_LINES, 6, GL_UNSIGNED_BYTE, 0);
    glDrawArrays(GL_LINES, 0, 6);
    
}

void OpenglRendering::processInput_end(){
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        camera->MovementSpeed = camera->FastSpeed;
    } else {
        camera->MovementSpeed = camera->OriginalSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera->ProcessKeyboard(FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        camera->ProcessKeyboard(BACKWARD, deltaTime);
    }    
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera->ProcessKeyboard(LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera->ProcessKeyboard(RIGHT, deltaTime);
    }

}

void OpenglRendering::mouse_callback(GLFWwindow * window, double xpos, double ypos) {

    OpenglRendering *ogl_pointer =
         static_cast<OpenglRendering*>(glfwGetWindowUserPointer(window));
    ogl_pointer->mouse_callback_function(xpos, ypos);

}

void OpenglRendering::scroll_callback(GLFWwindow * window, double xoffset, double yoffset) {
    OpenglRendering *ogl_pointer =
         static_cast<OpenglRendering*>(glfwGetWindowUserPointer(window));
    ogl_pointer->scroll_callback_function(xoffset, yoffset);
}

void OpenglRendering::mouse_callback_function(double xpos, double ypos) {
    if(firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos-lastX;
    float yoffset = lastY - ypos;
    lastX = xpos;
    lastY = ypos;

    camera->ProcessMouseMovement(xoffset, yoffset);
}

void OpenglRendering::scroll_callback_function(double xoffset, double yoffset){
    camera->ProcessMouseScroll(yoffset);
}


Eigen::Matrix3f OpenglRendering::skew_symmetric(Eigen::Vector3f& vector){
    Eigen::Matrix3f result;
    result(0, 0) = 0.0;
    result(0, 1) = -vector(2);
    result(0, 2) = vector(1);
    result(1, 0) = vector(2);
    result(1, 1) = 0;
    result(1, 2) = -vector(0);
    result(2, 0) = -vector(1);
    result(2, 1) = vector(0);
    result(2, 2) = 0;
    return result;
}


void OpenglRendering::draw_surfels(std::vector<std::vector<float>>& surfels){
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    std::vector<glm::mat4> circle_transformation;
    circle_transformation.resize(surfels.size());

    for(int i=0; i<surfels.size(); ++i) {
        Eigen::Vector3f a{-1.0f, 0.0f, 0.0f};
        Eigen::Vector3f b{surfels[i][0], surfels[i][1], surfels[i][2]};
        Eigen::Vector3f v = skew_symmetric(a)*b;

        float s2 = v.transpose()*v;
        Eigen::Matrix3f R;

        if(s2==0) {
            R = Eigen::Matrix3f::Identity(3,3);
        } else {
            float s = sqrt(v.transpose()*v);
            float c = a.transpose()*b;
            R = Eigen::Matrix3f::Identity(3,3) + 
                skew_symmetric(v) + 
                skew_symmetric(v)*skew_symmetric(v)*(1-c)/(s*s);
        }

        Eigen::Vector3f p{surfels[i][3], surfels[i][4], surfels[i][5]};
        Eigen::Matrix4f T_mat;
        T_mat<<R,p,0,0,0,1;
        std::cout<<T_mat<<std::endl;
        circle_transformation[i] = eigen_mat4_to_glm_mat4(T_mat);
    }

    int sides = 150;
    float vertices[6*(sides+2)];
    float r = 0.2f;

    for(int i = 0; i <(sides+2); ++i) {
        if(i==0){
            vertices[0] = 0.0f;
            vertices[1] = 0.0f;
            vertices[2] = 0.0f;
            vertices[3] = 1.0f;
            vertices[4] = 1.0f;
            vertices[5] = 0.0f;
            continue;
        }
        vertices[i*6+0] = 0.0f;
        vertices[i*6+1] = r*cos(2.0f/float(sides)*(i-1)*M_PI);
        vertices[i*6+2] = r*sin(2.0f/float(sides)*(i-1)*M_PI);
        vertices[i*6+3] = 1.0f;
        vertices[i*6+4] = 1.0f;
        vertices[i*6+5] = 0.0f;


    }

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::mat4(1.0f);
    glm::mat4 projection = glm::mat4(1.0f);

    std::cout<<(int)surfels.size()<<std::endl;

    point_shader.use();
    while(!glfwWindowShouldClose(window)){
        processInput_end();
        glClearColor(28.0/255.0, 40.0/255.0, 79.0/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  
        projection = glm::perspective(glm::radians(camera->Zoom), (float)screenWidth/(float)screenHeight, 0.1f, 100.0f);
        view = camera->GetViewMatrix();

        for(int i = 0; i<surfels.size(); ++i) {
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
            glEnableVertexAttribArray(1);

            //Model and camera;
            Eigen::Matrix4f cur_state;
            
            model = circle_transformation[i];

            point_shader.setMat4("model", model);
            point_shader.setMat4("view", view);
            point_shader.setMat4("projection", projection);

            glDrawArrays(GL_TRIANGLE_FAN, 0, sides+2);
            draw_axis(2.0f, 20.0f, &point_shader);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}